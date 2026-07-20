#!/usr/bin/env python3
"""Calibrate a rigid LiDAR-to-RUR-base transform from two odometry topics.

The node approximately synchronizes RUR wheel odometry and LiDAR-only
odometry (for example GenZ-ICP), selects informative keyframes, solves

    A_i X = X B_i

and writes X = T_base_lidar to YAML.  A_i is relative RUR/base motion and
B_i is relative LiDAR motion over the same synchronized time interval.

Planar mode is the default because ordinary ground-robot motion does not
observe z, roll, or pitch.  Those components are retained from --initial-*.
"""

import argparse
import math
import os
import threading
import time

import numpy as np
import rospy
import yaml
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def parse_arguments():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rur-topic", default="/odom",
                        help="RUR wheel/base nav_msgs/Odometry topic")
    parser.add_argument("--lidar-topic", default="/genz/odometry",
                        help="LiDAR-only nav_msgs/Odometry topic")
    parser.add_argument("--sync-slop", type=float, default=0.03,
                        help="Maximum header timestamp difference, seconds")
    parser.add_argument("--sync-queue", type=int, default=500)
    parser.add_argument("--samples", type=int, default=80,
                        help="Number of informative synchronized keyframes")
    parser.add_argument("--min-samples", type=int, default=20,
                        help="Minimum keyframes needed to solve on shutdown")
    parser.add_argument("--min-translation", type=float, default=0.15,
                        help="Keyframe translation threshold, metres")
    parser.add_argument("--min-rotation-deg", type=float, default=5.0,
                        help="Keyframe rotation threshold, degrees")
    parser.add_argument("--pair-gaps", default="1,2,4,8",
                        help="Keyframe index gaps used to form relative motions")
    parser.add_argument("--mode", choices=("planar", "se3"), default="planar")
    parser.add_argument("--initial-translation", default="0,0,0",
                        help="Initial base<-LiDAR x,y,z in metres")
    parser.add_argument("--initial-rpy-deg", default="0,0,0",
                        help="Initial base<-LiDAR roll,pitch,yaw in degrees")
    parser.add_argument("--base-frame", default="",
                        help="YAML parent frame; default: /odom child_frame_id")
    parser.add_argument("--lidar-frame", default="ouster/os_sensor",
                        help="YAML child frame (GenZ Odometry usually leaves it blank)")
    parser.add_argument("--loss", choices=("linear", "soft_l1", "huber",
                                           "cauchy", "arctan"),
                        default="soft_l1")
    parser.add_argument("--rotation-weight", type=float, default=1.0,
                        help="Metres-equivalent weight applied to rotation residuals")
    parser.add_argument("--output", default=os.path.join(
        script_dir, "lidar_to_rur.yaml"))
    return parser.parse_args(rospy.myargv()[1:])


def csv_floats(text, count, name):
    values = [float(value.strip()) for value in text.split(",")]
    if len(values) != count:
        raise ValueError("{} requires {} comma-separated values".format(name, count))
    return values


def csv_ints(text, name):
    values = sorted(set(int(value.strip()) for value in text.split(",")))
    if not values or values[0] < 1:
        raise ValueError("{} requires positive comma-separated integers".format(name))
    return values


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose_matrix(message):
    pose = message.pose.pose
    quaternion = np.array([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w
    ], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("odometry pose contains a zero quaternion")
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    transform[:3, 3] = [
        pose.position.x, pose.position.y, pose.position.z
    ]
    return transform


def transform_from_xyz_rpy(translation, rpy):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    transform[:3, 3] = translation
    return transform


def transform_to_xyz_rpy(transform):
    xyz = transform[:3, 3].copy()
    rpy = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")
    return xyz, rpy


def planar_pose(transform):
    yaw = Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")[2]
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    result[0, 3] = transform[0, 3]
    result[1, 3] = transform[1, 3]
    return result


def relative_pose(first, second):
    return np.linalg.inv(first) @ second


def planar_motion_size(transform):
    translation = np.linalg.norm(transform[:2, 3])
    yaw = abs(normalize_angle(
        Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")[2]))
    return translation, yaw


def se3_error(left, right, rotation_weight):
    error = np.linalg.inv(left) @ right
    return np.concatenate((
        error[:3, 3],
        rotation_weight * Rotation.from_matrix(error[:3, :3]).as_rotvec()
    ))


def planar_error(left, right, rotation_weight):
    error = np.linalg.inv(left) @ right
    yaw = Rotation.from_matrix(error[:3, :3]).as_euler("xyz")[2]
    return np.array([
        error[0, 3], error[1, 3], rotation_weight * normalize_angle(yaw)
    ])


class TrajectoryCollector:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.keyframes = []
        self.synchronized_count = 0
        self.rejected_stationary = 0
        self.latest_rur_stamp = None
        self.latest_lidar_stamp = None
        self.started_at = time.monotonic()
        self.base_frame = args.base_frame
        self.lidar_frame = args.lidar_frame

        self.rur_sub = Subscriber(args.rur_topic, Odometry, queue_size=args.sync_queue)
        self.lidar_sub = Subscriber(
            args.lidar_topic, Odometry, queue_size=args.sync_queue)
        self.rur_sub.registerCallback(self._rur_seen)
        self.lidar_sub.registerCallback(self._lidar_seen)
        self.sync = ApproximateTimeSynchronizer(
            [self.rur_sub, self.lidar_sub],
            queue_size=args.sync_queue,
            slop=args.sync_slop,
            allow_headerless=False)
        self.sync.registerCallback(self._synchronized)
        self.timer = rospy.Timer(rospy.Duration(5.0), self._diagnostics)

    def _rur_seen(self, message):
        self.latest_rur_stamp = message.header.stamp

    def _lidar_seen(self, message):
        self.latest_lidar_stamp = message.header.stamp

    def _synchronized(self, rur_message, lidar_message):
        if self.done.is_set():
            return
        try:
            rur_pose = pose_matrix(rur_message)
            lidar_pose = pose_matrix(lidar_message)
        except ValueError as error:
            rospy.logwarn("Rejected synchronized pair: %s", error)
            return

        stamp_delta = abs(
            (rur_message.header.stamp - lidar_message.header.stamp).to_sec())
        stamp = 0.5 * (
            rur_message.header.stamp.to_sec() +
            lidar_message.header.stamp.to_sec())

        with self.lock:
            self.synchronized_count += 1
            if not self.base_frame:
                self.base_frame = rur_message.child_frame_id or "base_footprint"
            if (not self.lidar_frame and lidar_message.child_frame_id):
                self.lidar_frame = lidar_message.child_frame_id

            if self.keyframes:
                rur_delta = relative_pose(self.keyframes[-1]["rur"], rur_pose)
                lidar_delta = relative_pose(
                    self.keyframes[-1]["lidar"], lidar_pose)
                if self.args.mode == "planar":
                    rur_delta = planar_pose(rur_delta)
                    lidar_delta = planar_pose(lidar_delta)
                rur_translation, rur_rotation = planar_motion_size(rur_delta)
                lidar_translation, lidar_rotation = planar_motion_size(lidar_delta)
                enough_motion = (
                    max(rur_translation, lidar_translation) >=
                    self.args.min_translation or
                    max(rur_rotation, lidar_rotation) >=
                    math.radians(self.args.min_rotation_deg))
                if not enough_motion:
                    self.rejected_stationary += 1
                    return

            self.keyframes.append({
                "stamp": stamp,
                "stamp_delta": stamp_delta,
                "rur": rur_pose,
                "lidar": lidar_pose,
            })
            rospy.loginfo(
                "Accepted keyframe %d/%d at %.6f (sync delta %.4f s)",
                len(self.keyframes), self.args.samples, stamp, stamp_delta)
            if len(self.keyframes) >= self.args.samples:
                self.done.set()

    def _diagnostics(self, _event):
        if self.done.is_set():
            return
        with self.lock:
            count = len(self.keyframes)
            synchronized = self.synchronized_count
            rejected = self.rejected_stationary
        if self.latest_rur_stamp is not None and self.latest_lidar_stamp is not None:
            latest_delta = abs(
                (self.latest_rur_stamp - self.latest_lidar_stamp).to_sec())
            delta_text = "{:.4f} s".format(latest_delta)
        else:
            delta_text = "unavailable"
        rospy.loginfo(
            "Collection: keyframes=%d/%d, synchronized=%d, low-motion rejected=%d, "
            "latest stamp delta=%s, elapsed=%.1f s",
            count, self.args.samples, synchronized, rejected, delta_text,
            time.monotonic() - self.started_at)

    def snapshot(self):
        with self.lock:
            return list(self.keyframes), self.base_frame, self.lidar_frame


def build_motion_pairs(keyframes, gaps, mode):
    motions = []
    for gap in gaps:
        for first_index in range(len(keyframes) - gap):
            second_index = first_index + gap
            rur_motion = relative_pose(
                keyframes[first_index]["rur"],
                keyframes[second_index]["rur"])
            lidar_motion = relative_pose(
                keyframes[first_index]["lidar"],
                keyframes[second_index]["lidar"])
            if mode == "planar":
                rur_motion = planar_pose(rur_motion)
                lidar_motion = planar_pose(lidar_motion)
            motions.append((rur_motion, lidar_motion, first_index, second_index))
    return motions


def motion_statistics(motions):
    base_translations = []
    lidar_translations = []
    base_rotations = []
    lidar_rotations = []
    for base_motion, lidar_motion, _, _ in motions:
        base_t, base_r = planar_motion_size(base_motion)
        lidar_t, lidar_r = planar_motion_size(lidar_motion)
        base_translations.append(base_t)
        lidar_translations.append(lidar_t)
        base_rotations.append(base_r)
        lidar_rotations.append(lidar_r)
    return {
        "base_translation_sum_m": float(np.sum(base_translations)),
        "lidar_translation_sum_m": float(np.sum(lidar_translations)),
        "base_abs_rotation_sum_deg": float(np.degrees(np.sum(base_rotations))),
        "lidar_abs_rotation_sum_deg": float(np.degrees(np.sum(lidar_rotations))),
        "base_max_translation_m": float(np.max(base_translations)),
        "lidar_max_translation_m": float(np.max(lidar_translations)),
        "base_max_rotation_deg": float(np.degrees(np.max(base_rotations))),
        "lidar_max_rotation_deg": float(np.degrees(np.max(lidar_rotations))),
    }


def solve_planar(motions, initial_transform, loss, rotation_weight):
    initial_xyz, initial_rpy = transform_to_xyz_rpy(initial_transform)

    def make_transform(parameters):
        xyz = initial_xyz.copy()
        rpy = initial_rpy.copy()
        xyz[:2] = parameters[:2]
        rpy[2] = parameters[2]
        return transform_from_xyz_rpy(xyz, rpy)

    def residual(parameters):
        transform = make_transform(parameters)
        errors = [
            planar_error(base @ transform, transform @ lidar, rotation_weight)
            for base, lidar, _, _ in motions
        ]
        return np.concatenate(errors)

    # Planar hand-eye yaw can have distant local minima. Try evenly spaced
    # yaw seeds and retain the best robust solution.
    solutions = []
    for yaw_seed in np.linspace(-math.pi, math.pi, 12, endpoint=False):
        seed = np.array([initial_xyz[0], initial_xyz[1], yaw_seed])
        result = least_squares(
            residual, seed, loss=loss, f_scale=0.05, max_nfev=3000)
        solutions.append(result)
    result = min(solutions, key=lambda candidate: np.sum(candidate.fun ** 2))
    return make_transform(result.x), result


def solve_se3(motions, initial_transform, loss, rotation_weight):
    initial_xyz, initial_rpy = transform_to_xyz_rpy(initial_transform)
    initial = np.concatenate((initial_xyz, Rotation.from_euler(
        "xyz", initial_rpy).as_rotvec()))

    def make_transform(parameters):
        transform = np.eye(4)
        transform[:3, 3] = parameters[:3]
        transform[:3, :3] = Rotation.from_rotvec(parameters[3:]).as_matrix()
        return transform

    def residual(parameters):
        transform = make_transform(parameters)
        errors = [
            se3_error(base @ transform, transform @ lidar, rotation_weight)
            for base, lidar, _, _ in motions
        ]
        return np.concatenate(errors)

    result = least_squares(
        residual, initial, loss=loss, f_scale=0.05, max_nfev=5000)
    return make_transform(result.x), result


def jacobian_diagnostics(result):
    singular_values = np.linalg.svd(result.jac, compute_uv=False)
    if len(singular_values) == 0:
        condition_number = float("inf")
    elif singular_values[-1] <= np.finfo(float).eps:
        condition_number = float("inf")
    else:
        condition_number = float(singular_values[0] / singular_values[-1])
    return singular_values, condition_number


def quaternion_xyzw(transform):
    return Rotation.from_matrix(transform[:3, :3]).as_quat()


def save_result(args, transform, result, keyframes, motions,
                base_frame, lidar_frame, statistics):
    xyz, rpy = transform_to_xyz_rpy(transform)
    quaternion = quaternion_xyzw(transform)
    residuals = result.fun.reshape((-1, 3 if args.mode == "planar" else 6))
    translation_rmse = float(np.sqrt(np.mean(
        np.sum(residuals[:, :2 if args.mode == "planar" else 3] ** 2, axis=1))))
    rotation_column = 2 if args.mode == "planar" else slice(3, 6)
    if args.mode == "planar":
        rotation_norms = np.abs(residuals[:, rotation_column]) / args.rotation_weight
    else:
        rotation_norms = np.linalg.norm(
            residuals[:, rotation_column], axis=1) / args.rotation_weight
    rotation_rmse_deg = float(np.degrees(
        np.sqrt(np.mean(rotation_norms ** 2))))
    singular_values, condition_number = jacobian_diagnostics(result)
    sync_deltas = [frame["stamp_delta"] for frame in keyframes]

    document = {
        "calibration": "motion_based_hand_eye",
        "equation": "A_base * T_base_lidar = T_base_lidar * B_lidar",
        "transform_direction": "lidar_to_base",
        "parent_frame": base_frame,
        "child_frame": lidar_frame,
        "rur_odometry_topic": args.rur_topic,
        "lidar_odometry_topic": args.lidar_topic,
        "mode": args.mode,
        "translation_xyz_m": [float(value) for value in xyz],
        "rpy_deg": [float(value) for value in np.degrees(rpy)],
        "quaternion_xyzw": [float(value) for value in quaternion],
        "transformation_matrix": [
            [float(value) for value in row] for row in transform
        ],
        "quality": {
            "optimizer_success": bool(result.success),
            "optimizer_message": str(result.message),
            "cost": float(result.cost),
            "translation_residual_rmse_m": translation_rmse,
            "rotation_residual_rmse_deg": rotation_rmse_deg,
            "jacobian_singular_values": [
                float(value) for value in singular_values
            ],
            "jacobian_condition_number": condition_number,
            "synchronized_keyframes": len(keyframes),
            "relative_motion_pairs": len(motions),
            "mean_sync_delta_s": float(np.mean(sync_deltas)),
            "max_sync_delta_s": float(np.max(sync_deltas)),
            "motion": statistics,
        },
        "notes": (
            "In planar mode z/roll/pitch are copied from the initial guess; "
            "planar ground motion cannot estimate them reliably."
            if args.mode == "planar" else
            "Full SE3 calibration requires genuine 6-DoF motion and good "
            "Jacobian conditioning."
        ),
    }

    output = os.path.abspath(os.path.expanduser(args.output))
    output_directory = os.path.dirname(output)
    if output_directory:
        os.makedirs(output_directory, exist_ok=True)
    with open(output, "w") as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
    return output, document


def validate_motion(args, statistics, result):
    warnings = []
    if statistics["base_abs_rotation_sum_deg"] < 90.0:
        warnings.append(
            "less than 90 degrees of accumulated base rotation; x/y/yaw may "
            "be weakly constrained")
    if statistics["base_translation_sum_m"] < 3.0:
        warnings.append(
            "less than 3 metres of accumulated base translation")
    singular_values, condition_number = jacobian_diagnostics(result)
    parameter_count = 3 if args.mode == "planar" else 6
    if len(singular_values) < parameter_count or not np.isfinite(condition_number):
        warnings.append("rank-deficient optimizer Jacobian")
    elif condition_number > 1e6:
        warnings.append(
            "poor optimizer conditioning ({:.3g})".format(condition_number))
    if args.mode == "se3":
        warnings.append(
            "SE3 mode is generally unobservable for flat-ground robot motion")
    return warnings


def main():
    args = parse_arguments()
    try:
        initial_translation = csv_floats(
            args.initial_translation, 3, "--initial-translation")
        initial_rpy = np.radians(csv_floats(
            args.initial_rpy_deg, 3, "--initial-rpy-deg"))
        gaps = csv_ints(args.pair_gaps, "--pair-gaps")
    except ValueError as error:
        raise SystemExit(str(error))

    if args.min_samples < 3 or args.samples < args.min_samples:
        raise SystemExit("--samples must be >= --min-samples >= 3")
    if args.rotation_weight <= 0:
        raise SystemExit("--rotation-weight must be positive")

    rospy.init_node("lidar_to_rur_handeye_calibration")
    collector = TrajectoryCollector(args)
    rospy.loginfo(
        "Collecting %d informative pairs: %s (base) <-> %s (LiDAR), "
        "mode=%s, slop=%.3f s",
        args.samples, args.rur_topic, args.lidar_topic,
        args.mode, args.sync_slop)
    rospy.loginfo(
        "Drive slowly with straight segments plus repeated left/right turns. "
        "Press Ctrl-C after at least %d keyframes to solve early.",
        args.min_samples)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not collector.done.is_set():
        rate.sleep()

    keyframes, base_frame, lidar_frame = collector.snapshot()
    if len(keyframes) < args.min_samples:
        rospy.logerr(
            "Only %d informative keyframes collected; need at least %d. "
            "No calibration file written.",
            len(keyframes), args.min_samples)
        return 2

    gaps = [gap for gap in gaps if gap < len(keyframes)]
    motions = build_motion_pairs(keyframes, gaps, args.mode)
    if len(motions) < 6:
        rospy.logerr("Only %d relative motions available; no result written.",
                     len(motions))
        return 2

    statistics = motion_statistics(motions)
    initial_transform = transform_from_xyz_rpy(
        initial_translation, initial_rpy)
    if args.mode == "planar":
        transform, result = solve_planar(
            motions, initial_transform, args.loss, args.rotation_weight)
    else:
        transform, result = solve_se3(
            motions, initial_transform, args.loss, args.rotation_weight)

    warnings = validate_motion(args, statistics, result)
    output, document = save_result(
        args, transform, result, keyframes, motions,
        base_frame or "base_footprint",
        lidar_frame or "lidar",
        statistics)

    rospy.loginfo("Calibration written to %s", output)
    rospy.loginfo(
        "T_%s_%s: xyz=[%.6f, %.6f, %.6f] m, rpy=[%.3f, %.3f, %.3f] deg",
        document["parent_frame"], document["child_frame"],
        *document["translation_xyz_m"], *document["rpy_deg"])
    rospy.loginfo(
        "Residual RMSE: %.4f m, %.3f deg; condition number %.3g",
        document["quality"]["translation_residual_rmse_m"],
        document["quality"]["rotation_residual_rmse_deg"],
        document["quality"]["jacobian_condition_number"])
    for warning in warnings:
        rospy.logwarn("Calibration quality warning: %s", warning)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

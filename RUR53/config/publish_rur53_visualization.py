#!/usr/bin/env python3
"""Publish the calibrated RUR53 sensor TF tree and centered CAD model."""

import argparse
import os

import numpy as np
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker


CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))
RVIZ_DIR = os.path.dirname(CONFIG_DIR)
AUTOSWEEP_ROOT = os.path.dirname(RVIZ_DIR)
EXTRINSIC_DIR = os.path.join(AUTOSWEEP_ROOT, "extrinsic_calib_RUR")

DEFAULT_RUR_CALIBRATION = os.path.join(
    EXTRINSIC_DIR, "lidar2rur", "lidar_to_rur.yaml")
DEFAULT_LIDAR_CALIBRATION = os.path.join(
    EXTRINSIC_DIR, "lidar2lidar", "dome_to_spinning.yaml")
DEFAULT_CAMERA_CALIBRATION = os.path.join(
    EXTRINSIC_DIR, "lidar2rgb", "lidar_in_cam_T",
    "lidar_in_cam_3_tuned.yaml")
DEFAULT_CAD_MODEL = os.path.join(RVIZ_DIR, "URDF", "RUR53.ply")


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rur-calibration", default=DEFAULT_RUR_CALIBRATION)
    parser.add_argument("--lidar-calibration", default=DEFAULT_LIDAR_CALIBRATION)
    parser.add_argument("--camera-calibration", default=DEFAULT_CAMERA_CALIBRATION)
    parser.add_argument("--cad-model", default=DEFAULT_CAD_MODEL)
    parser.add_argument("--include-camera", action="store_true",
                        help="Connect the Azure camera_base tree to the LiDAR")
    parser.add_argument("--camera-base-frame", default="camera_base",
                        help="Root frame published by the Azure driver")
    parser.add_argument("--camera-frame", default="rgb_camera_link",
                        help="Azure RGB frame used by the CA2LIB calibration")
    return parser.parse_args(rospy.myargv()[1:])


def load_yaml(path):
    with open(os.path.abspath(os.path.expanduser(path)), "r") as stream:
        return yaml.safe_load(stream)


def matrix_from_rows(rows, name):
    matrix = np.asarray(rows, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("{} must be a 4x4 matrix".format(name))
    return matrix


def matrix_from_flat_3x4(values, name):
    matrix_3x4 = np.asarray(values, dtype=np.float64)
    if matrix_3x4.size != 12:
        raise ValueError("{} must contain 12 values".format(name))
    matrix = np.eye(4)
    matrix[:3, :] = matrix_3x4.reshape((3, 4))
    return matrix


def validate_transform(matrix, name):
    if not np.all(np.isfinite(matrix)):
        raise ValueError("{} contains non-finite values".format(name))
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("{} has an invalid homogeneous last row".format(name))
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise ValueError("{} rotation is not orthonormal".format(name))
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-3):
        raise ValueError("{} rotation determinant is not +1".format(name))


def quaternion_from_matrix(matrix):
    """Return an x, y, z, w quaternion from a homogeneous transform."""
    rotation = matrix[:3, :3]
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quaternion = np.array([
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = 2.0 * np.sqrt(
            1.0 + rotation[index, index]
            - rotation[next_index, next_index]
            - rotation[last_index, last_index])
        quaternion = np.zeros(4)
        quaternion[index] = 0.25 * scale
        quaternion[3] = (
            rotation[last_index, next_index]
            - rotation[next_index, last_index]) / scale
        quaternion[next_index] = (
            rotation[next_index, index]
            + rotation[index, next_index]) / scale
        quaternion[last_index] = (
            rotation[last_index, index]
            + rotation[index, last_index]) / scale
    return quaternion / np.linalg.norm(quaternion)


def transform_message(parent, child, matrix):
    message = TransformStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = matrix[0, 3]
    message.transform.translation.y = matrix[1, 3]
    message.transform.translation.z = matrix[2, 3]
    quaternion = quaternion_from_matrix(matrix)
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


def matrix_from_transform_message(message):
    translation = message.transform.translation
    quaternion = message.transform.rotation
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("received a transform with a zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ]
    matrix[:3, 3] = [translation.x, translation.y, translation.z]
    return matrix


def calibrated_transforms(args):
    rur = load_yaml(args.rur_calibration)
    lidar = load_yaml(args.lidar_calibration)

    base_to_spinning = matrix_from_rows(
        rur["transformation_matrix"], "lidar_to_rur transformation_matrix")
    spinning_to_dome = matrix_from_rows(
        lidar["transformation_matrix"],
        "dome_to_spinning transformation_matrix")

    transforms = [
        (rur.get("parent_frame", "base_footprint"),
         rur.get("child_frame", "os_sensor"), base_to_spinning),
        (lidar.get("parent_frame", "os_sensor"),
         lidar.get("child_frame", "ousterDome/os_sensor"), spinning_to_dome),
    ]
    for parent, child, matrix in transforms:
        validate_transform(matrix, "{} -> {}".format(parent, child))
    return transforms


def calibrated_camera_base_transform(args, tf_buffer, spinning_frame):
    camera = load_yaml(args.camera_calibration)
    # CA2LIB's camera_in_lidar is T_spinning_rgb: it maps RGB-camera
    # coordinates into the spinning-LiDAR frame.
    spinning_to_rgb = matrix_from_flat_3x4(
        camera["camera_in_lidar"], "camera_in_lidar")

    rospy.loginfo(
        "Waiting for Azure transform %s -> %s",
        args.camera_base_frame, args.camera_frame)
    azure_transform = tf_buffer.lookup_transform(
        args.camera_base_frame,
        args.camera_frame,
        rospy.Time(0),
        rospy.Duration(10.0),
    )
    camera_base_to_rgb = matrix_from_transform_message(azure_transform)

    # Preserve the Azure driver's internal tree by solving
    # T_spinning_camera_base * T_camera_base_rgb = T_spinning_rgb.
    spinning_to_camera_base = (
        spinning_to_rgb @ np.linalg.inv(camera_base_to_rgb))
    validate_transform(
        spinning_to_camera_base,
        "{} -> {}".format(spinning_frame, args.camera_base_frame))
    return spinning_frame, args.camera_base_frame, spinning_to_camera_base


def cad_marker(model_path):
    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "rur53"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.pose.position.x = -0.223
    marker.pose.position.z = -0.255
    marker.pose.orientation.w = 1.0
    # RUR53.ply spans x=[-455, 455], y=[-305, 305], z=[0, 710] mm.
    # Move the CAD 223 mm backward along X relative to base_footprint. The
    # requested reference height is 510 mm, so shifting by half of it
    # (255 mm) puts that midpoint at z=0.
    marker.scale.x = 0.001
    marker.scale.y = 0.001
    marker.scale.z = 0.001
    marker.color.r = 0.72
    marker.color.g = 0.76
    marker.color.b = 0.82
    marker.color.a = 1.0
    marker.mesh_resource = "file://" + os.path.abspath(model_path)
    marker.mesh_use_embedded_materials = True
    return marker


def main():
    rospy.init_node("rur53_calibrated_visualization")
    args = parse_arguments()

    transforms = calibrated_transforms(args)
    if args.include_camera:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        try:
            transforms.append(calibrated_camera_base_transform(
                args, tf_buffer, transforms[0][1]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logfatal(
                "Cannot connect the Azure frame tree: %s. Ensure the Azure "
                "driver is running and publishes %s -> %s.",
                error, args.camera_base_frame, args.camera_frame)
            return 2

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform([
        transform_message(parent, child, matrix)
        for parent, child, matrix in transforms
    ])

    marker_publisher = rospy.Publisher(
        "/rur53/cad_model", Marker, queue_size=1, latch=True)
    marker_publisher.publish(cad_marker(args.cad_model))

    rospy.loginfo("Published calibrated RUR53 frame tree:")
    for parent, child, _ in transforms:
        rospy.loginfo("  %s -> %s", parent, child)
    rospy.loginfo("Published centered CAD model on /rur53/cad_model")
    rospy.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Estimate the dome-to-spinning LiDAR extrinsic with multi-scale ICP.

The returned transform maps points from ``dome_topic`` into ``spinning_topic``.
Collect data with the sensor rig and the surrounding scene stationary.
"""

import argparse
import copy
import math
import os
import threading
import time

import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2


_OPEN3D_APP_INITIALIZED = False


def open3d_application():
    """Return the GUI application, initializing it only once."""
    global _OPEN3D_APP_INITIALIZED
    app = o3d.visualization.gui.Application.instance
    if not _OPEN3D_APP_INITIALIZED:
        app.initialize()
        _OPEN3D_APP_INITIALIZED = True
    return app


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spinning-topic", default="/ouster/points")
    parser.add_argument("--dome-topic", default="/ousterDome/points")
    parser.add_argument("--sync-slop", type=float, default=0.06,
                        help="Maximum timestamp difference in seconds")
    parser.add_argument("--expected-hz", type=float, default=10.0,
                        help="Expected publication frequency for diagnostics")
    parser.add_argument("--expected-width", type=int, default=1024,
                        help="Expected PointCloud2 width for diagnostics")
    parser.add_argument("--min-range", type=float, default=1.0)
    parser.add_argument("--max-range", type=float, default=60.0)
    parser.add_argument("--collection-voxel", type=float, default=0.03,
                        help="Downsample each input cloud before ICP, metres")
    parser.add_argument("--collection-pairs", type=int, default=5,
                        help="Synchronized stationary pairs to aggregate")
    parser.add_argument("--min-points-per-cloud", type=int, default=100,
                        help="Minimum points after filtering/downsampling")
    parser.add_argument("--min-valid-ratio", type=float, default=0.02,
                        help="Minimum processed/nominal point ratio")
    parser.add_argument("--voxel-sizes", default="0.50,0.25,0.10,0.05",
                        help="Coarse-to-fine voxel sizes in metres")
    parser.add_argument("--max-correspondence",
                        default="1.50,0.75,0.30,0.12",
                        help="ICP correspondence distances in metres")
    parser.add_argument("--iterations", default="80,60,50,40")
    parser.add_argument("--initial-rpy-deg", default="0,0,0",
                        help="Initial dome-to-spinning roll,pitch,yaw in degrees")
    parser.add_argument("--initial-translation", default="0,0,0",
                        help="Initial dome origin in spinning frame, metres")
    parser.add_argument("--output", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "dome_to_spinning.yaml"))
    parser.add_argument("--publish-tf", action="store_true",
                        help="Keep node alive and publish the calibrated static TF")
    parser.add_argument("--no-visualization", action="store_true",
                        help="Skip the initial editor and final Open3D window")
    return parser.parse_args(rospy.myargv()[1:])


def csv_floats(text, count, name):
    values = [float(value.strip()) for value in text.split(",")]
    if len(values) != count:
        raise ValueError("{} requires {} comma-separated values".format(name, count))
    return values


def csv_float_list(text, name):
    values = [float(value.strip()) for value in text.split(",")]
    if not values:
        raise ValueError("{} requires at least one value".format(name))
    return values


def cloud_to_xyz(message, min_range, max_range):
    points = np.asarray(list(pc2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    points = points.reshape((-1, 3))
    ranges = np.linalg.norm(points, axis=1)
    return points[(ranges >= min_range) & (ranges <= max_range)]


def make_cloud(points):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


def voxel_downsample(points, voxel_size):
    if voxel_size <= 0:
        return points
    return np.asarray(make_cloud(points).voxel_down_sample(voxel_size).points)


def cloud_description(name, message, processed_points):
    nominal_points = message.width * message.height
    payload_mib = len(message.data) / (1024.0 * 1024.0)
    rospy.loginfo(
        "%s cloud: frame=%s, width=%d, height=%d, nominal_points=%d, "
        "payload=%.2f MiB, processed_points=%d",
        name, message.header.frame_id, message.width, message.height,
        nominal_points, payload_mib, len(processed_points))


def initial_transform(rpy_deg, translation):
    roll, pitch, yaw = np.deg2rad(rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rz @ ry @ rx
    transform[:3, 3] = translation
    return transform


def rotation_to_quaternion(rotation):
    # Open3D is not guaranteed to ship a quaternion conversion helper.
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
        w = 0.25 * s
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            x, y, z, w = 0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s, (rotation[0, 2] + rotation[2, 0]) / s, (rotation[2, 1] - rotation[1, 2]) / s
        elif index == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            x, y, z, w = (rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s, (rotation[1, 2] + rotation[2, 1]) / s, (rotation[0, 2] - rotation[2, 0]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            x, y, z, w = (rotation[0, 2] + rotation[2, 0]) / s, (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s, (rotation[1, 0] - rotation[0, 1]) / s
    return np.array([x, y, z, w])


class CloudCollector:
    def __init__(self, args):
        self.args = args
        self.spinning = None
        self.dome = None
        self.spinning_clouds = []
        self.dome_clouds = []
        self.accepted_pairs = 0
        self.spinning_frame = None
        self.dome_frame = None
        self.spinning_messages = 0
        self.dome_messages = 0
        self.synchronized_pairs = 0
        self.latest_spinning_stamp = None
        self.latest_dome_stamp = None
        self.started_at = time.monotonic()
        self.done = threading.Event()
        # Keep explicit references to the subscribers for the lifetime of the
        # collector. This also lets the diagnostics query ROS connections.
        self.spinning_sub = Subscriber(
            args.spinning_topic, PointCloud2, queue_size=10)
        self.dome_sub = Subscriber(
            args.dome_topic, PointCloud2, queue_size=10)
        self.spinning_sub.registerCallback(self.spinning_input_callback)
        self.dome_sub.registerCallback(self.dome_input_callback)
        self.sync = ApproximateTimeSynchronizer(
            [self.spinning_sub, self.dome_sub], queue_size=20,
            slop=args.sync_slop)
        self.sync.registerCallback(self.callback)
        self.diagnostic_timer = rospy.Timer(
            rospy.Duration(5.0), self.report_diagnostics)

    def spinning_input_callback(self, message):
        self.spinning_messages += 1
        self.latest_spinning_stamp = message.header.stamp

    def dome_input_callback(self, message):
        self.dome_messages += 1
        self.latest_dome_stamp = message.header.stamp

    def report_diagnostics(self, _event):
        if self.done.is_set():
            return
        spinning_connections = self.spinning_sub.sub.get_num_connections()
        dome_connections = self.dome_sub.sub.get_num_connections()
        if self.latest_spinning_stamp is not None and self.latest_dome_stamp is not None:
            stamp_delta = abs(
                (self.latest_spinning_stamp - self.latest_dome_stamp).to_sec())
            delta_text = "latest timestamp delta={:.3f} s".format(stamp_delta)
        else:
            delta_text = "timestamp delta unavailable"
        elapsed = max(time.monotonic() - self.started_at, 1e-6)
        spinning_hz = self.spinning_messages / elapsed
        dome_hz = self.dome_messages / elapsed
        rospy.logwarn(
            "Still waiting: spinning=%d messages/%.2f Hz/%d publishers, "
            "dome=%d messages/%.2f Hz/%d publishers, synchronized=%d; %s; "
            "expected=%.1f Hz, slop=%.3f s",
            self.spinning_messages, spinning_hz, spinning_connections,
            self.dome_messages, dome_hz, dome_connections,
            self.synchronized_pairs, delta_text, self.args.expected_hz,
            self.args.sync_slop)
        if self.spinning_messages and self.dome_messages and not self.synchronized_pairs:
            rospy.logwarn(
                "Both topics are active but timestamps are not matching. "
                "Increase --sync-slop or verify both drivers use the same clock.")

    def callback(self, spinning_message, dome_message):
        if self.done.is_set():
            return
        self.synchronized_pairs += 1
        spinning = cloud_to_xyz(spinning_message, self.args.min_range, self.args.max_range)
        dome = cloud_to_xyz(dome_message, self.args.min_range, self.args.max_range)
        spinning = voxel_downsample(spinning, self.args.collection_voxel)
        dome = voxel_downsample(dome, self.args.collection_voxel)
        cloud_description("Target/spinning", spinning_message, spinning)
        cloud_description("Source/dome", dome_message, dome)
        if spinning_message.width != self.args.expected_width:
            rospy.logwarn(
                "Target/spinning width is %d; expected %d for 1024x10 mode",
                spinning_message.width, self.args.expected_width)
        if dome_message.width != self.args.expected_width:
            rospy.logwarn(
                "Source/dome width is %d; expected %d for 1024x10 mode",
                dome_message.width, self.args.expected_width)
        spinning_minimum = max(
            self.args.min_points_per_cloud,
            int(spinning_message.width * spinning_message.height *
                self.args.min_valid_ratio))
        dome_minimum = max(
            self.args.min_points_per_cloud,
            int(dome_message.width * dome_message.height *
                self.args.min_valid_ratio))
        if (len(spinning) < spinning_minimum or len(dome) < dome_minimum):
            rospy.logwarn(
                "Rejecting incomplete pair: %d spinning, %d dome points "
                "(required %d and %d)", len(spinning), len(dome),
                spinning_minimum, dome_minimum)
            return
        self.spinning_clouds.append(spinning)
        self.dome_clouds.append(dome)
        self.accepted_pairs += 1
        self.spinning_frame = spinning_message.header.frame_id
        self.dome_frame = dome_message.header.frame_id
        rospy.loginfo(
            "Accepted synchronized pair %d/%d (%d spinning, %d dome points)",
            self.accepted_pairs, self.args.collection_pairs,
            len(spinning), len(dome))
        if self.accepted_pairs < self.args.collection_pairs:
            return
        # A second voxel pass averages repeated stationary measurements inside
        # each voxel, reducing range noise before registration.
        self.spinning = voxel_downsample(
            np.vstack(self.spinning_clouds), self.args.collection_voxel)
        self.dome = voxel_downsample(
            np.vstack(self.dome_clouds), self.args.collection_voxel)
        rospy.loginfo(
            "Aggregated %d pairs into %d spinning and %d dome points",
            self.accepted_pairs, len(self.spinning), len(self.dome))
        self.done.set()


def run_multiscale_icp(source_points, target_points, transform, voxel_sizes,
                       correspondence_distances, iterations):
    source_full = make_cloud(source_points)
    target_full = make_cloud(target_points)
    result = None
    for voxel, distance, iteration_count in zip(
            voxel_sizes, correspondence_distances, iterations):
        source = source_full.voxel_down_sample(voxel)
        target = target_full.voxel_down_sample(voxel)
        source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel * 3.0, max_nn=40))
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel * 3.0, max_nn=40))
        rospy.loginfo("ICP voxel %.3f m: %d source, %d target points",
                      voxel, len(source.points), len(target.points))
        estimator = (
            o3d.pipelines.registration.TransformationEstimationPointToPlane(
                o3d.pipelines.registration.HuberLoss(voxel)))
        result = o3d.pipelines.registration.registration_icp(
            source, target, distance, transform,
            estimator,
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-7, relative_rmse=1e-7,
                max_iteration=iteration_count))
        transform = result.transformation
        rospy.loginfo("  fitness=%.4f RMSE=%.4f m", result.fitness, result.inlier_rmse)
    return result


def save_result(path, transform, result, args, source_frame, target_frame):
    quaternion = rotation_to_quaternion(transform[:3, :3])
    data = {
        "transform_direction": "dome_to_spinning",
        "parent_frame": target_frame,
        "child_frame": source_frame,
        "source_topic": args.dome_topic,
        "target_topic": args.spinning_topic,
        "translation_xyz_m": transform[:3, 3].tolist(),
        "quaternion_xyzw": quaternion.tolist(),
        "transformation_matrix": transform.tolist(),
        "icp_fitness": float(result.fitness),
        "icp_inlier_rmse_m": float(result.inlier_rmse),
        "input_pairs": args.collection_pairs,
    }
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(path, "w") as output_file:
        yaml.safe_dump(data, output_file, sort_keys=False)
    rospy.loginfo("Saved dome-to-spinning calibration to %s", path)


def publish_static_tf(transform, source_frame, target_frame):
    quaternion = rotation_to_quaternion(transform[:3, :3])
    message = TransformStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = target_frame
    message.child_frame_id = source_frame
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = transform[:3, 3]
    message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = quaternion
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(message)
    rospy.loginfo("Publishing static TF %s -> %s", target_frame, source_frame)
    rospy.spin()


def make_coordinate_axes(transform, size):
    """Create RGB coordinate axes as point clouds for Open3D 0.13."""
    distances = np.linspace(0.0, size, 101)
    x_axis = np.column_stack((
        distances, np.zeros_like(distances), np.zeros_like(distances)))
    y_axis = np.column_stack((
        np.zeros_like(distances), distances, np.zeros_like(distances)))
    z_axis = np.column_stack((
        np.zeros_like(distances), np.zeros_like(distances), distances))
    local_points = np.vstack((x_axis, y_axis, z_axis))
    world_points = (
        local_points @ transform[:3, :3].T + transform[:3, 3])
    axes = make_cloud(world_points)
    axes.colors = o3d.utility.Vector3dVector(np.vstack((
        np.tile([1.0, 0.0, 0.0], (len(distances), 1)),
        np.tile([0.0, 1.0, 0.0], (len(distances), 1)),
        np.tile([0.0, 0.0, 1.0], (len(distances), 1)),
    )))
    return axes


def choose_initial_transform(source_points, target_points, rpy_deg,
                             translation, source_frame, target_frame):
    """Interactively edit and return the dome-to-spinning ICP initial guess."""
    if not hasattr(o3d.visualization, "gui"):
        rospy.logwarn(
            "Open3D GUI is unavailable; using the command-line initial guess.")
        return initial_transform(rpy_deg, translation)

    gui = o3d.visualization.gui
    rendering = o3d.visualization.rendering
    app = open3d_application()
    window = app.create_window("ICP initial-guess editor", 1280, 800)

    scene_widget = gui.SceneWidget()
    scene_widget.scene = rendering.Open3DScene(window.renderer)
    scene_widget.scene.set_background([0.05, 0.05, 0.05, 1.0])

    cloud_material = rendering.Material()
    cloud_material.shader = "defaultUnlit"
    cloud_material.point_size = 2.0
    axes_material = rendering.Material()
    axes_material.shader = "defaultUnlit"
    axes_material.point_size = 5.0

    spinning = make_cloud(target_points)
    spinning.paint_uniform_color([0.55, 0.55, 0.55])
    scene_widget.scene.add_geometry(
        "spinning", spinning, cloud_material)
    scene_widget.scene.add_geometry(
        "spinning_frame", make_coordinate_axes(np.eye(4), 1.0),
        axes_material)

    em = window.theme.font_size
    panel = gui.Vert(0.35 * em, gui.Margins(em, em, em, em))
    panel.add_child(gui.Label("ICP INITIAL GUESS"))
    panel.add_child(gui.Label(
        "Gray: {} (fixed)\nBlue: {} (interactive)".format(
            target_frame, source_frame)))

    controls = {}
    value_labels = {}
    state = {
        "running": True,
        "transform": initial_transform(rpy_deg, translation),
    }

    def current_transform():
        return initial_transform(
            [controls["roll"].double_value,
             controls["pitch"].double_value,
             controls["yaw"].double_value],
            [controls["x"].double_value,
             controls["y"].double_value,
             controls["z"].double_value])

    def update_geometry(_value=None):
        transform = current_transform()
        state["transform"] = transform.copy()
        for name in ("dome", "dome_frame"):
            if scene_widget.scene.has_geometry(name):
                scene_widget.scene.remove_geometry(name)
        dome = make_cloud(source_points)
        dome.transform(transform)
        dome.paint_uniform_color([0.0, 0.65, 1.0])
        scene_widget.scene.add_geometry("dome", dome, cloud_material)
        scene_widget.scene.add_geometry(
            "dome_frame", make_coordinate_axes(transform, 1.0),
            axes_material)
        for name in ("roll", "pitch", "yaw"):
            value_labels[name].text = "{:+.1f} deg".format(
                controls[name].double_value)
        for name in ("x", "y", "z"):
            value_labels[name].text = "{:+.3f} m".format(
                controls[name].double_value)
        scene_widget.force_redraw()

    def add_slider(name, title, minimum, maximum, value):
        panel.add_child(gui.Label(title))
        row = gui.Horiz(0.25 * em)
        slider = gui.Slider(gui.Slider.DOUBLE)
        slider.set_limits(minimum, maximum)
        slider.double_value = value
        label = gui.Label("")
        row.add_child(slider)
        row.add_fixed(0.5 * em)
        row.add_child(label)
        panel.add_child(row)
        controls[name] = slider
        value_labels[name] = label
        slider.set_on_value_changed(update_geometry)

    add_slider("roll", "Roll / X", -180.0, 180.0, rpy_deg[0])
    add_slider("pitch", "Pitch / Y", -180.0, 180.0, rpy_deg[1])
    add_slider("yaw", "Yaw / Z", -180.0, 180.0, rpy_deg[2])

    panel.add_child(gui.Label("Quick rotations"))
    rotation_buttons = gui.Horiz(0.2 * em)

    def rotate_90(axis, delta):
        value = controls[axis].double_value + delta
        controls[axis].double_value = ((value + 180.0) % 360.0) - 180.0
        update_geometry()

    for axis, caption in (("roll", "X"), ("pitch", "Y"), ("yaw", "Z")):
        minus = gui.Button("{} -90°".format(caption))
        minus.set_on_clicked(
            lambda a=axis: rotate_90(a, -90.0))
        plus = gui.Button("{} +90°".format(caption))
        plus.set_on_clicked(
            lambda a=axis: rotate_90(a, 90.0))
        rotation_buttons.add_child(minus)
        rotation_buttons.add_child(plus)
    panel.add_child(rotation_buttons)

    add_slider("x", "Translation X", -10.0, 10.0, translation[0])
    add_slider("y", "Translation Y", -10.0, 10.0, translation[1])
    add_slider("z", "Translation Z", -10.0, 10.0, translation[2])

    run_button = gui.Button("Use guess and run ICP")

    def accept_guess():
        state["transform"] = current_transform().copy()
        state["running"] = False
        window.close()

    run_button.set_on_clicked(accept_guess)
    panel.add_fixed(0.5 * em)
    panel.add_child(run_button)

    def on_close():
        state["transform"] = current_transform().copy()
        state["running"] = False
        return True

    window.set_on_close(on_close)

    def on_layout(_layout_context):
        rect = window.content_rect
        panel_width = int(24 * em)
        scene_widget.frame = gui.Rect(
            rect.x, rect.y, max(1, rect.width - panel_width), rect.height)
        panel.frame = gui.Rect(
            rect.get_right() - panel_width, rect.y,
            panel_width, rect.height)

    window.set_on_layout(on_layout)
    window.add_child(scene_widget)
    window.add_child(panel)
    update_geometry()
    bounds = scene_widget.scene.bounding_box
    scene_widget.setup_camera(60.0, bounds, bounds.get_center())

    rospy.loginfo(
        "Adjust the initial dome transform, then click "
        "'Use guess and run ICP'.")
    while state["running"] and app.run_one_tick():
        time.sleep(0.01)

    rospy.loginfo(
        "User-selected initial dome-to-spinning transform:\n%s",
        state["transform"])
    return state["transform"]


def visualize_alignment(source_points, target_points, final_transform,
                        source_frame, target_frame):
    """Show the aligned clouds and their final reference frames."""
    final_dome = copy.deepcopy(make_cloud(source_points))
    final_dome.transform(final_transform)
    final_dome.paint_uniform_color([1.0, 0.45, 0.0])

    spinning = make_cloud(target_points)
    spinning.paint_uniform_color([0.55, 0.55, 0.55])

    axis_size = 1.0
    spinning_frame = make_coordinate_axes(np.eye(4), axis_size)
    transformed_dome_frame = make_coordinate_axes(
        final_transform, axis_size)

    rospy.loginfo(
        "Opening final visualization: spinning %s=gray and aligned dome "
        "%s=orange, with the spinning and transformed dome frames. "
        "Close the window to continue.",
        target_frame, source_frame)

    # O3DVisualizer provides named geometry and labels. Retain compatibility
    # with older Open3D releases by falling back to draw_geometries.
    if hasattr(o3d.visualization, "O3DVisualizer"):
        app = open3d_application()

        viewer = o3d.visualization.O3DVisualizer(
            "LiDAR ICP calibration", 1280, 800)
        viewer.show_settings = True
        viewer.add_geometry("Spinning unchanged (gray)", spinning)
        viewer.add_geometry(
            "Dome transformed by final ICP (orange)", final_dome)
        viewer.add_geometry(
            "Spinning frame ({})".format(target_frame), spinning_frame)
        viewer.add_geometry(
            "Transformed dome frame ({})".format(source_frame),
            transformed_dome_frame)
        viewer.add_3d_label(
            [0.0, 0.0, axis_size * 0.15], "SPINNING FRAME")
        viewer.add_3d_label(
            final_transform[:3, 3] +
            np.array([0.0, 0.0, axis_size * 0.15]),
            "TRANSFORMED DOME FRAME")
        viewer.reset_camera_to_default()
        app.add_window(viewer)
        app.run()
    else:
        rospy.logwarn(
            "O3DVisualizer is unavailable; showing the aligned clouds in the "
            "legacy viewer.")
        o3d.visualization.draw_geometries(
            [spinning, final_dome, spinning_frame, transformed_dome_frame],
            window_name=(
                "Spinning gray | aligned dome orange | final frames"))


def main():
    rospy.init_node("lidar2lidar_icp_calibration")
    args = parse_arguments()
    if args.sync_slop <= 0:
        raise ValueError("--sync-slop must be positive")
    if args.expected_hz <= 0 or args.expected_width <= 0:
        raise ValueError("--expected-hz and --expected-width must be positive")
    if args.collection_voxel <= 0 or args.collection_pairs <= 0:
        raise ValueError(
            "--collection-voxel and --collection-pairs must be positive")
    if not 0.0 <= args.min_valid_ratio <= 1.0:
        raise ValueError("--min-valid-ratio must be between 0 and 1")
    voxel_sizes = csv_float_list(args.voxel_sizes, "--voxel-sizes")
    distances = csv_float_list(
        args.max_correspondence, "--max-correspondence")
    iteration_values = csv_float_list(args.iterations, "--iterations")
    if not (len(voxel_sizes) == len(distances) == len(iteration_values)):
        raise ValueError(
            "--voxel-sizes, --max-correspondence and --iterations must "
            "contain the same number of values")
    if (any(value <= 0 for value in voxel_sizes) or
            any(value <= 0 for value in distances) or
            any(value <= 0 for value in iteration_values)):
        raise ValueError("ICP scale values and iterations must be positive")
    iterations = [int(value) for value in iteration_values]
    rpy = csv_floats(args.initial_rpy_deg, 3, "--initial-rpy-deg")
    translation = csv_floats(args.initial_translation, 3, "--initial-translation")

    collector = CloudCollector(args)
    rospy.loginfo("Waiting for synchronized clouds on %s and %s", args.spinning_topic, args.dome_topic)
    while not rospy.is_shutdown() and not collector.done.wait(0.2):
        pass
    if rospy.is_shutdown():
        return

    source_points = collector.dome
    target_points = collector.spinning
    initial_guess = initial_transform(rpy, translation)
    rospy.loginfo("Initial dome-to-spinning transform:\n%s", initial_guess)
    if not args.no_visualization:
        initial_guess = choose_initial_transform(
            source_points, target_points, rpy, translation,
            collector.dome_frame, collector.spinning_frame)
    result = run_multiscale_icp(source_points, target_points,
                                initial_guess.copy(),
                                voxel_sizes, distances, iterations)
    if result.fitness < 0.05:
        rospy.logwarn("Very low ICP fitness (%.4f): check the initial rotation/translation and cloud overlap", result.fitness)
    source_frame = rospy.get_param("~dome_frame", collector.dome_frame)
    target_frame = rospy.get_param("~spinning_frame", collector.spinning_frame)
    save_result(args.output, result.transformation, result, args, source_frame, target_frame)
    rospy.loginfo("Final dome-to-spinning transform:\n%s", result.transformation)
    if not args.no_visualization:
        visualize_alignment(
            source_points, target_points, result.transformation,
            source_frame, target_frame)
    if args.publish_tf:
        publish_static_tf(result.transformation, source_frame, target_frame)


if __name__ == "__main__":
    main()

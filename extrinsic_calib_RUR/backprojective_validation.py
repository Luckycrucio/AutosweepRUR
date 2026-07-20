#!/usr/bin/env python3

import json
import rosbag
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import argparse
import yaml
from scipy.spatial.transform import Rotation as R
import os
import open3d as o3d


IMAGE_TOPIC = "/rgb/image_raw"
CAMERA_INFO_TOPIC = "/rgb/camera_info"
POINTS_TOPIC = "/ouster/points"

SSD_PATH = "/media/fsd/3127552a-0703-402f-ad8e-991656cc348c/autosweep/extrinsic/history"
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
REPO_PARENT = os.path.dirname(REPO_ROOT)


def parse_float_list(value, expected_len, name):
    values = [float(x) for x in value.replace(",", " ").split()]

    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"{name} must contain exactly {expected_len} values, got {len(values)}"
        )

    return values


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--mode",
        required=True,
        choices=["koide", "ca2lib"],
        help="Calibration modality to validate"
    )

    parser.add_argument(
        "--folder",
        required=True,
        help=(
            "Folder containing bags/. Relative paths are appended to SSD_PATH."
        )
    )

    parser.add_argument(
        "--yaml",
        default=None,
        help=(
            "ca2lib YAML calibration path, for example "
            "autosweep/extrinsic_calib_RUR/lidar_in_cam.yaml"
        )
    )

    parser.add_argument(
        "--K",
        type=lambda s: parse_float_list(s, 4, "K"),
        default=None,
        help="Camera intrinsics as 4 values: fx, fy, cx, cy"
    )

    parser.add_argument(
        "--D",
        type=lambda s: parse_float_list(s, len(s.replace(',', ' ').split()), "D"),
        default=None,
        help="Distortion coefficients, for example: k1 k2 p1 p2 k3"
    )

    parser.add_argument(
        "--output_overlay",
        default="lidar_camera_overlay.png",
        help="Output overlay image"
    )

    parser.add_argument(
        "--output_cloud",
        default="colored_projected_cloud.ply",
        help="Output colored pointcloud"
    )

    parser.add_argument(
        "--frame_size",
        type=float,
        default=0.5,
        help="Size of the Open3D coordinate frames drawn in the validation view"
    )

    return parser.parse_args()


def path_under_base(path):
    path = os.path.expanduser(path)

    if os.path.isabs(path):
        return path

    return os.path.join(SSD_PATH, path)


def resolve_local_path(path):
    path = os.path.expanduser(path)

    if os.path.isabs(path):
        return path

    candidates = [
        path,
        os.path.join(REPO_ROOT, path),
        os.path.join(REPO_PARENT, path),
    ]

    for candidate in candidates:
        if os.path.exists(candidate):
            return os.path.abspath(candidate)

    return os.path.abspath(path)


def first_bag_in_folder(dataset_dir):
    bags_dir = os.path.join(dataset_dir, "bags")

    if not os.path.isdir(bags_dir):
        raise RuntimeError(f"No bags folder found at {bags_dir}")

    bags = sorted(
        os.path.join(bags_dir, name)
        for name in os.listdir(bags_dir)
        if name.endswith(".bag")
    )

    if not bags:
        raise RuntimeError(f"No .bag files found in {bags_dir}")

    return bags[0]


def first_existing_file(paths, description):
    for path in paths:
        if os.path.isfile(path):
            return path

    raise RuntimeError(
        f"No {description} found. Checked:\n" +
        "\n".join(f"  {path}" for path in paths)
    )


def resolve_paths(args):
    if args.mode == "koide":
        dataset_dir = path_under_base(args.folder)
        bag_path = first_bag_in_folder(dataset_dir)
        calib_path = first_existing_file(
            [
                os.path.join(dataset_dir, "ouster_processed", "calib.json"),
                os.path.join(dataset_dir, "ouster_preprocessed", "calib.json"),
            ],
            "Koide calib.json"
        )
        output_dir = dataset_dir

    elif args.mode == "ca2lib":
        if args.yaml is None:
            raise RuntimeError("Provide --yaml when using --mode ca2lib")

        dataset_dir = path_under_base(args.folder)
        bag_path = first_bag_in_folder(dataset_dir)
        calib_path = resolve_local_path(args.yaml)
        output_dir = os.path.join(os.path.dirname(__file__), "validation")

    else:
        raise RuntimeError(f"Unsupported mode: {args.mode}")

    output_overlay = args.output_overlay
    if not os.path.isabs(output_overlay):
        output_overlay = os.path.join(output_dir, output_overlay)

    output_cloud = args.output_cloud
    if not os.path.isabs(output_cloud):
        output_cloud = os.path.join(output_dir, output_cloud)

    os.makedirs(os.path.dirname(output_overlay), exist_ok=True)
    os.makedirs(os.path.dirname(output_cloud), exist_ok=True)

    return dataset_dir, bag_path, calib_path, output_overlay, output_cloud


def matrix_from_flat_3x4(values, name):
    values = np.array(values, dtype=np.float64)

    if values.size != 12:
        raise RuntimeError(f"{name} must contain exactly 12 values, got {values.size}")

    T = np.eye(4)
    T[:3, :] = values.reshape(3, 4)

    return T


def load_json_transforms(calib_path):
    with open(calib_path, "r") as f:
        data = json.load(f)

    v = data["results"]["T_lidar_camera"]

    t = np.array(v[:3])
    q = np.array(v[3:])

    T_lidar_camera = np.eye(4)
    T_lidar_camera[:3, :3] = R.from_quat(q).as_matrix()
    T_lidar_camera[:3, 3] = t

    T_camera_lidar = np.linalg.inv(T_lidar_camera)

    return T_lidar_camera, T_camera_lidar


def load_yaml_transforms(calib_path):
    with open(calib_path, "r") as f:
        data = yaml.safe_load(f)

    if "lidar_in_camera" not in data:
        raise RuntimeError(
            f"{calib_path} is YAML, but it does not contain lidar_in_camera"
        )

    T_camera_lidar = matrix_from_flat_3x4(
        data["lidar_in_camera"],
        "lidar_in_camera"
    )

    if "camera_in_lidar" in data:
        T_lidar_camera = matrix_from_flat_3x4(
            data["camera_in_lidar"],
            "camera_in_lidar"
        )
    else:
        T_lidar_camera = np.linalg.inv(T_camera_lidar)

    return T_lidar_camera, T_camera_lidar


def load_transforms(calib_path):
    extension = os.path.splitext(calib_path)[1].lower()

    if extension in [".yaml", ".yml"]:
        return load_yaml_transforms(calib_path)

    return load_json_transforms(calib_path)


def read_camera_info(bag):
    for _, msg, _ in bag.read_messages(topics=[CAMERA_INFO_TOPIC]):
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.D, dtype=np.float64)
        return K, D

    raise RuntimeError("No camera_info message found")


def get_camera_parameters(args, bag):
    if args.K is not None and args.D is not None:
        fx, fy, cx, cy = args.K

        K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        D = np.array(args.D, dtype=np.float64)

        print("Using K and D from command-line arguments")
        return K, D

    if args.K is not None or args.D is not None:
        raise RuntimeError("You must provide both --K and --D, or neither of them")

    print("Reading K and D from rosbag camera_info topic")
    return read_camera_info(bag)


def cloud_to_xyz(cloud_msg):
    pts = []

    for p in pc2.read_points(
        cloud_msg,
        field_names=("x", "y", "z"),
        skip_nans=True
    ):
        pts.append([p[0], p[1], p[2]])

    return np.array(pts, dtype=np.float32)


def get_first_image_and_cloud(bag):
    bridge = CvBridge()

    first_image = None
    first_cloud = None

    for topic, msg, _ in bag.read_messages(topics=[IMAGE_TOPIC, POINTS_TOPIC]):
        if topic == IMAGE_TOPIC and first_image is None:
            first_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        elif topic == POINTS_TOPIC and first_cloud is None:
            first_cloud = msg

        if first_image is not None and first_cloud is not None:
            break

    if first_image is None:
        raise RuntimeError("No image message found")

    if first_cloud is None:
        raise RuntimeError("No pointcloud message found")

    return first_image, first_cloud


def project_lidar(points_lidar, T_camera_lidar, K, D):
    pts_h = np.hstack([
        points_lidar,
        np.ones((points_lidar.shape[0], 1), dtype=np.float32)
    ])

    pts_cam = (T_camera_lidar @ pts_h.T).T[:, :3]

    valid_z = pts_cam[:, 2] > 0.2
    pts_cam = pts_cam[valid_z]

    image_points, _ = cv2.projectPoints(
        pts_cam,
        np.zeros(3),
        np.zeros(3),
        K,
        D
    )

    uv = image_points.reshape(-1, 2)

    return uv, pts_cam


def filter_points_inside_image(uv, pts_cam, image):
    h, w = image.shape[:2]

    u = np.round(uv[:, 0]).astype(np.int32)
    v = np.round(uv[:, 1]).astype(np.int32)

    valid = (
        (u >= 0) &
        (u < w) &
        (v >= 0) &
        (v < h)
    )

    return uv[valid], pts_cam[valid], u[valid], v[valid]


def draw_points(image, uv, depth):
    overlay = image.copy()
    h, w = overlay.shape[:2]

    for (u, v), z in zip(uv, depth):
        u = int(round(u))
        v = int(round(v))

        if 0 <= u < w and 0 <= v < h:
            color = int(np.clip(255 - z * 10, 0, 255))
            cv2.circle(overlay, (u, v), 2, (0, color, 255), -1)

    return overlay


def backproject_pixels_to_3d(uv, depth, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    u = uv[:, 0]
    v = uv[:, 1]
    z = depth

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return np.column_stack((x, y, z)).astype(np.float32)


def make_colored_cloud(points_3d, image, u, v):
    colors_bgr = image[v, u]
    colors_rgb = colors_bgr[:, ::-1].astype(np.float32) / 255.0

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points_3d)
    cloud.colors = o3d.utility.Vector3dVector(colors_rgb)

    return cloud


def make_coordinate_frames(T_camera_lidar, frame_size):
    camera_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=frame_size,
        origin=[0.0, 0.0, 0.0]
    )

    os_sensor_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=frame_size,
        origin=[0.0, 0.0, 0.0]
    )
    os_sensor_frame.transform(T_camera_lidar)

    return camera_frame, os_sensor_frame


def main():
    args = parse_args()

    dataset_dir, bag_path, calib_path, output_overlay, output_cloud = resolve_paths(args)

    print("Mode:", args.mode)
    print("Dataset:", dataset_dir)
    print("Bag:", bag_path)
    print("Calibration:", calib_path)
    print("Output overlay:", output_overlay)
    print("Output cloud:", output_cloud)

    T_lidar_camera, T_camera_lidar = load_transforms(calib_path)

    with rosbag.Bag(bag_path, "r") as bag:
        K, D = get_camera_parameters(args, bag)
        image, cloud_msg = get_first_image_and_cloud(bag)

    print("T_lidar_camera:\n", T_lidar_camera)
    print("T_camera_lidar:\n", T_camera_lidar)
    print("K:\n", K)
    print("D:", D)

    points_lidar = cloud_to_xyz(cloud_msg)

    uv, pts_cam = project_lidar(
        points_lidar,
        T_camera_lidar,
        K,
        D
    )

    uv_inside, pts_cam_inside, u_inside, v_inside = filter_points_inside_image(
        uv,
        pts_cam,
        image
    )

    overlay = draw_points(
        image,
        uv_inside,
        pts_cam_inside[:, 2]
    )

    cv2.imwrite(output_overlay, overlay)
    print("Saved overlay:", output_overlay)

    points_3d_backprojected = backproject_pixels_to_3d(
        uv_inside,
        pts_cam_inside[:, 2],
        K
    )

    colored_cloud = make_colored_cloud(
        points_3d_backprojected,
        image,
        u_inside,
        v_inside
    )

    o3d.io.write_point_cloud(output_cloud, colored_cloud)
    print("Saved colored pointcloud:", output_cloud)

    camera_frame, os_sensor_frame = make_coordinate_frames(
        T_camera_lidar,
        args.frame_size
    )

    o3d.visualization.draw_geometries(
        [
            colored_cloud,
            camera_frame,
            os_sensor_frame
        ],
        window_name="Backprojected RGB-colored pointcloud"
    )


if __name__ == "__main__":
    main()

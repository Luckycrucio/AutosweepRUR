#!/usr/bin/env python3

import argparse
import os
import shlex
import shutil
import signal
import subprocess
import time


ROS_SETUP = """
source /opt/ros/noetic/setup.bash
source /home/fsd/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/fsd/autosweep:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://192.168.53.2:11311/
export ROS_IP=192.168.53.15
"""
DEFAULT_OUTPUT_DIR = "/media/fsd/3127552a-0703-402f-ad8e-991656cc348c/autosweep/visualOdometry"

SENSOR_TOPICS = [
    "/rgb/image_raw",
    "/rgb/camera_info",
    "/ouster/points",
    "/tf",
    "/tf_static",
    "/ouster/imu",
]

VISUAL_ODOMETRY_ALGORITHMS = {
    "1": {
        "name": "genz-ICP",
        "cmd": [
            "roslaunch",
            "genz_icp",
            "odometry.launch",
            "topic:=/ouster/points",
        ],
        "output_topics": [
            "/genz/odometry",
            "/genz/trajectory",
            "/genz/local_map",
            "/genz/planar_points",
            "/genz/non_planar_points",
        ],
        "ouster_point_type": "original",
        "title": "genz-ICP Odometry",
    },
    "2": {
        "name": "FAST-LIVO",
        "cmd": [
            "roslaunch",
            "fast_livo",
            "mapping_RUR.launch",
        ],
        "output_topics": [
            "/aft_mapped_to_init",
            "/path",
            "/cloud_registered",
            "/cloud_effected",
            "/cloud_visual_sub_map",
            "/rgb_img",
            "/Laser_map",
        ],
        "ouster_point_type": "original",
        "title": "FAST-LIVO Odometry",
    }
}
DEFAULT_ALREADY_RUNNING_ALGORITHM = "1"


def ensure_output_dir(path):
    try:
        os.makedirs(path, exist_ok=True)
    except PermissionError as exc:
        raise SystemExit(
            f"Cannot create output directory: {path}\n"
            "Check that the SSD is mounted and writable, or pass a different "
            "directory with --output-dir."
        ) from exc


def command_to_shell(cmd):
    return shlex.join(cmd)


def launch_terminal(cmd, title=None, quiet=False):
    if shutil.which("gnome-terminal") is None:
        raise RuntimeError("gnome-terminal was not found. Cannot launch ROS terminals.")

    launch_cmd = command_to_shell(cmd)
    if quiet:
        launch_cmd = f"{launch_cmd} >/dev/null 2>&1"

    full_cmd = f"""
    {ROS_SETUP}
    {launch_cmd}
    exec bash
    """

    if not quiet:
        print(f"Launching terminal: {command_to_shell(cmd)}")

    return subprocess.Popen([
        "gnome-terminal",
        "--title", title or "ROS",
        "--",
        "bash",
        "-c",
        full_cmd
    ])


def launch_process(cmd):
    full_cmd = f"""
    {ROS_SETUP}
    exec {command_to_shell(cmd)}
    """

    print(f"Starting: {command_to_shell(cmd)}")

    return subprocess.Popen(
        ["bash", "-c", full_cmd],
        start_new_session=True
    )


def terminate_process(proc, name):
    if proc is None:
        return

    if proc.poll() is not None:
        return

    print(f"Stopping {name}...")

    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=30)
    except subprocess.TimeoutExpired:
        print(f"{name} did not exit cleanly, killing...")
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=5)


def confirm(msg="Continue? [y/n]: "):
    while True:
        choice = input(msg).strip().lower()

        if choice == "y":
            return True

        if choice == "n":
            return False


def choose_visual_odometry_algorithm():
    print("\nChoose visual odometry algorithm:")

    for key, algorithm in VISUAL_ODOMETRY_ALGORITHMS.items():
        print(f"  {key}. {algorithm['name']}")

    while True:
        choice = (
            input(f"Selection [{DEFAULT_ALREADY_RUNNING_ALGORITHM}]: ").strip()
            or DEFAULT_ALREADY_RUNNING_ALGORITHM
        )

        if choice in VISUAL_ODOMETRY_ALGORITHMS:
            return VISUAL_ODOMETRY_ALGORITHMS[choice]

        print("Invalid selection.")


def choose_post_launch_action():
    print("\nSensor drivers and odometry are ready.")
    print("  r. Record a rosbag")
    print("  q. Quit without recording")

    while True:
        choice = input("Selection [r]: ").strip().lower() or "r"

        if choice in ("r", "record"):
            return "record"

        if choice in ("q", "quit", "exit"):
            return "quit"

        print("Invalid selection.")


def wait_for_bag_to_finalize(bag_path):
    active_bag_path = f"{bag_path}.active"

    while True:
        if os.path.exists(bag_path) and not os.path.exists(active_bag_path):
            return

        time.sleep(1)


def unique_topics(topics):
    unique = []

    for topic in topics:
        if topic not in unique:
            unique.append(topic)

    return unique


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        default="visual_odometry",
        help="Output rosbag file name"
    )

    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Directory where the rosbag will be saved"
    )

    parser.add_argument(
        "--fps-cam",
        type=int,
        choices=(5,15,30),
        default=15,
        help="Shared frame rate for Azure Kinect and Ouster drivers"
    )

    parser.add_argument(
        "--fps-lidar",
        type=int,
        choices=(5,10,20),
        default=10,
        help="Shared frame rate for Azure Kinect and Ouster drivers"
    )

    parser.add_argument(
        "--sensor-hostname",
        default="os-122220000120.local",
        help="Ouster hostname"
    )

    parser.add_argument(
        "--lidar_res",
        type=int,
        default=1024,
        help="Resolution for Ouster driver"
    )

    args = parser.parse_args()

    output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    ensure_output_dir(output_dir)
    bag_prefix = os.path.join(output_dir, args.bag)

    azure_proc = None
    bag_proc = None
    ouster_proc = None
    odometry_proc = None

    try:
        algorithm = choose_visual_odometry_algorithm()
        launch_odometry = confirm(f"Launch {algorithm['name']} odometry? [y/n]: ")

        launch_drivers = confirm("Launch sensor drivers? [y/n]: ")

        if launch_drivers:
            ouster_proc = launch_terminal([
                "roslaunch",
                "ouster_ros",
                "driver.launch",
                f"sensor_hostname:={args.sensor_hostname}",
                f"lidar_mode:={args.lidar_res}x{args.fps_lidar}",
                "timestamp_mode:=TIME_FROM_ROS_TIME",
                f"point_type:={algorithm['ouster_point_type']}",
                "viz:=false"
            ],
            title="Ouster OS1 Driver",
            quiet=False)

            time.sleep(5)

            azure_proc = launch_terminal(
                [
                    "roslaunch",
                    "azure_kinect_ros_driver",
                    "driver.launch",
                    f"fps:={args.fps_cam}"
                ],
                title="Azure Kinect Driver",
                quiet=False
            )

        else:
            print("\nDrivers are supposed to be already launched.")

        if launch_odometry:
            if launch_drivers and algorithm["name"] == "FAST-LIVO":
                time.sleep(5)

            odometry_proc = launch_terminal(algorithm["cmd"], title=algorithm["title"])
            time.sleep(5)
        else:
            print(f"\n{algorithm['name']} is supposed to be already launched.")

        if choose_post_launch_action() == "quit":
            print("\nQuitting without recording.")
            return

        input("\nPress ENTER when you want to start recording...\n")

        record_topics = unique_topics(SENSOR_TOPICS + algorithm["output_topics"])

        bag_proc = launch_process([
            "rosbag",
            "record",
            "-O",
            bag_prefix,
        ] + record_topics)

        time.sleep(4)

        input("\nPress ENTER when you want to stop recording...\n")

    finally:
        terminate_process(bag_proc, "rosbag")

    bag_path = f"{bag_prefix}.bag"
    wait_for_bag_to_finalize(bag_path)

    print("Bag recording completed and saved correctly.\n")

    info_cmd = f"""
    {ROS_SETUP}
    rosbag info {bag_path}
    """

    subprocess.run(
        ["bash", "-c", info_cmd]
    )

    print("\nDriver and odometry terminals were left open for inspection.")


if __name__ == "__main__":
    main()

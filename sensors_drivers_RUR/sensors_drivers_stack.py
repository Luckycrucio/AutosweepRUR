#!/usr/bin/env python3

import argparse
import shlex
import shutil
import signal
import subprocess
import time
import os


ROS_SETUP = """
source /opt/ros/noetic/setup.bash
source /home/fsd/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/fsd/autosweep:$ROS_PACKAGE_PATH
export ROS_MASTER_URI=http://192.168.53.2:11311/
export ROS_IP=192.168.53.15
"""
SSD_PATH = "../../../../media/fsd/3127552a-0703-402f-ad8e-991656cc348c/autosweep/DataAcquisition22Jul/bags"

SENSOR_TOPICS = [
    "/rgb/image_raw",
    "/rgb/camera_info",
    "/ouster/points",
    "/ousterDome/points",
    "/tf",
    "/tf_static",
    "/ouster/imu",
    "/ousterDome/imu",
    "/odom",
    "/joint_states",
    "/extended_fix",
    "/mavros/global_position/raw/gps_vel",
    "/mavros/global_position/compass_hdg",
    "/mavros/time_reference",
    "/summit_xl_controller/MotorsStatus"


]


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


def ros_master_is_running():
    check_cmd = f"""
    {ROS_SETUP}
    rosparam get /run_id
    """
    result = subprocess.run(
        ["bash", "-c", check_cmd],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def ensure_ros_master(timeout=10.0):
    """Return a roscore process if this script had to start one."""
    if ros_master_is_running():
        print("Using the existing ROS master.")
        return None

    print("No ROS master detected; starting roscore before sensor drivers.")
    master_proc = launch_process(["roscore"])
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if master_proc.poll() is not None:
            raise RuntimeError("roscore exited before becoming ready")
        if ros_master_is_running():
            print("ROS master is ready.")
            return master_proc
        time.sleep(0.25)

    terminate_process(master_proc, "roscore")
    raise RuntimeError("Timed out waiting for the ROS master")

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


def ask_sensor_mode():
    while True:
        choice = input(
            "Select sensor mode: [1] two LiDARs, [2] Azure and spinning, "
            "[3] Azure and dome: "
        ).strip().lower()

        if choice in ("1", "two lidars", "lidars"):
            return "two_lidars"

        if choice in ("2", "azure and spinning", "azure_spinning"):
            return "azure_spinning"

        if choice in ("3", "azure and dome", "azure_dome"):
            return "azure_dome"


def normalize_bag_prefix(bag_name):
    bag_name = os.path.expanduser(bag_name.strip())

    if bag_name.endswith(".bag"):
        bag_name = bag_name[:-4]

    if os.path.isabs(bag_name):
        return bag_name

    return os.path.join(SSD_PATH, bag_name)


def ask_bag_prefix(default_name=None):
    while True:
        prompt = "Bag name to record"
        if default_name:
            prompt += f" [{default_name}]"
        prompt += ": "

        bag_name = input(prompt).strip()

        if not bag_name and default_name:
            bag_name = default_name

        if bag_name:
            return normalize_bag_prefix(bag_name)


def wait_for_bag_saved(bag_prefix):
    bag_path = f"{bag_prefix}.bag"
    active_bag_path = f"{bag_path}.active"

    while True:
        if os.path.exists(bag_path) and not os.path.exists(active_bag_path):
            return bag_path

        time.sleep(1)


def print_bag_info(bag_path):
    info_cmd = f"""
    {ROS_SETUP}
    rosbag info {command_to_shell([bag_path])}
    """

    subprocess.run(
        ["bash", "-c", info_cmd]
    )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        default= "ext_calib",
        help="Output rosbag file"
    )

    parser.add_argument(
        "--fps_camera",
        "--fps-camera",
        dest="fps_camera",
        type=int,
        default=5,
        help="Frame per second for the Azure Kinect driver"
    )

    parser.add_argument(
        "--fps_lidar",
        "--fps-lidar",
        dest="fps_lidar",
        type=int,
        default=10,
        help="Scan frequency for the Ouster driver"
    )

    parser.add_argument(
        "--sensor-hostname",
        default="os-122220000120.local",
        help="Ouster hostname"
    )

    parser.add_argument(
        "--second-sensor-hostname",
        default="os-122331000255.local",
        help="Second Ouster hostname"
    )

    parser.add_argument(
        "--lidar_res",
        type=int,
        default=1024,
        help="resolution for Ouster driver"
    )

    args = parser.parse_args()
    azure_proc = None
    bag_proc = None
    ouster_proc = None
    second_ouster_proc = None
    ros_master_proc = None

    try:

        # Establish exactly one master before back-to-back roslaunch calls.
        # Otherwise both launches can race while auto-starting port 11311 and
        # create conflicting /run_id values.
        ros_master_proc = ensure_ros_master()

        # Launch drivers immediately after selecting the sensor mode.
        sensor_mode = ask_sensor_mode()

        if sensor_mode == "two_lidars":
            # Launch first Ouster
            ouster_proc = launch_terminal([
                "roslaunch",
                "ouster_ros",
                "driver.launch",
                f"sensor_hostname:={args.sensor_hostname}",
                f"lidar_mode:={args.lidar_res}x{args.fps_lidar}",
                f"ouster_ns:=ouster",
                f"tf_prefix:=ouster",  
                "lidar_port:=7502",
                "imu_port:=7503",
                "timestamp_mode:=TIME_FROM_ROS_TIME",
                "viz:=false",
                "point_type:=original"],
            title="Ouster OS1 Driver 1")

            # Launch second Ouster
            second_ouster_proc = launch_terminal([
                "roslaunch",
                "ouster_ros",
                "driver.launch",
                f"sensor_hostname:={args.second_sensor_hostname}",
                f"lidar_mode:={args.lidar_res}x{args.fps_lidar}",
                f"ouster_ns:=ousterDome",
                f"tf_prefix:=ousterDome",
                "lidar_port:=7504",
                "imu_port:=7505",
                "timestamp_mode:=TIME_FROM_ROS_TIME",
                "viz:=false",
                "point_type:=original"],
            title="Ouster DOME Driver 2")

            time.sleep(5)
        elif sensor_mode == "azure_spinning":
            # Launch the spinning Ouster
            ouster_proc = launch_terminal([
                "roslaunch",
                "ouster_ros",
                "driver.launch",
                f"sensor_hostname:={args.sensor_hostname}",
                f"lidar_mode:={args.lidar_res}x{args.fps_lidar}",
                "lidar_port:=7502",
                "imu_port:=7503",
                "timestamp_mode:=TIME_FROM_ROS_TIME",
                "viz:=false",
                "point_type:=original"],
            title="Ouster Spinning Driver")

            # Launch Azure Kinect immediately in its own terminal.
            azure_proc = launch_terminal(
                [
                    "roslaunch",
                    "azure_kinect_ros_driver",
                    "driver.launch",
                    f"fps:={args.fps_camera}"
                ],
                title="Azure Kinect Driver"
            )

            time.sleep(5)
        else:
            # Launch the dome Ouster
            second_ouster_proc = launch_terminal([
                "roslaunch",
                "ouster_ros",
                "driver.launch",
                f"sensor_hostname:={args.second_sensor_hostname}",
                f"lidar_mode:={args.lidar_res}x{args.fps_lidar}",
                "ouster_ns:=ousterDome",
                "tf_prefix:=ousterDome",
                "lidar_port:=7504",
                "imu_port:=7505",
                "timestamp_mode:=TIME_FROM_ROS_TIME",
                "viz:=false",
                "point_type:=original"],
            title="Ouster Dome Driver")

            # Launch Azure Kinect immediately in its own terminal.
            azure_proc = launch_terminal(
                [
                    "roslaunch",
                    "azure_kinect_ros_driver",
                    "driver.launch",
                    f"fps:={args.fps_camera}"
                ],
                title="Azure Kinect Driver"
            )

            time.sleep(5)

        next_bag_name = args.bag

        while True:
            bag_prefix = ask_bag_prefix(next_bag_name)
            next_bag_name = None

            input("\nPress ENTER when you want to start recording...\n")

            try:
                bag_proc = launch_process([
                    "rosbag",
                    "record",
                    "-O",
                    bag_prefix,
                ] + SENSOR_TOPICS)

                time.sleep(4)

                input("\nPress ENTER when you want to stop recording...\n")
            finally:
                terminate_process(bag_proc, "rosbag")
                bag_proc = None

            bag_path = wait_for_bag_saved(bag_prefix)

            print(f"Bag recording completed and saved correctly.\n")

            print_bag_info(bag_path)

            next_action = input(
                "\nEnter a new bag name to record another bag, or 'q' to quit: "
            ).strip()

            if next_action.lower() in ("q", "quit", "exit"):
                break

            next_bag_name = next_action

    finally:
        terminate_process(bag_proc, "rosbag")
        terminate_process(ros_master_proc, "roscore")



if __name__ == "__main__":
    main()












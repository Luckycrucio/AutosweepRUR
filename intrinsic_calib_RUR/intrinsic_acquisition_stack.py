#!/usr/bin/env python3

import argparse
import signal
import subprocess
import time
import os
import threading


ROS_SETUP = """
source /opt/ros/noetic/setup.bash
source ../../catkin_ws/devel/setup.bash
"""
SSD_PATH = "../../../../media/fsd/3127552a-0703-402f-ad8e-991656cc348c/autosweep/intrinsic/"

stop_monitor = threading.Event()

def monitor_sample_rate(sample_rate, stop_event):
    sample_period = 1.0 / sample_rate
    sample_idx = 0

    while not stop_event.wait(sample_period):
        sample_idx += 1
        print(f"[SAMPLE RATE HIT] sample {sample_idx}", flush=True)

def launch_terminal(cmd, title=None):
    full_cmd = f"""
    {ROS_SETUP}
    {' '.join(cmd)}
    exec bash
    """

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
    exec {' '.join(cmd)}
    """

    print(f"Starting: {' '.join(cmd)}")

    return subprocess.Popen(
        ["bash", "-c", full_cmd],
        start_new_session=True
    )

def terminate_process(proc, name):
    if proc is None:
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


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        default= "int_calib",
        help="Output bag name "
    )

    parser.add_argument(
        "--fps",
        type=int,
        default=5,
        help="Frame per second for the Azure Kinect driver"
    )

    parser.add_argument(
        "--sample-rate",
        type=float,
        default=0.2,
        help="Dataset extraction rate in Hz (e.g. 0.2 = one image every 5 seconds)"
    )

    parser.add_argument(
        "--size",
        type=float,
        default=0.027,
        help="Size of the checker"
    )

    args = parser.parse_args()
    args.bag = os.path.join(SSD_PATH, args.bag)
    azure_proc = None
    bag_proc = None
    rviz_proc = None

    try:

        # Launch driver ?
                
        confirm_recording = confirm("Launch sensor drivers? [y/n]: ")

        if confirm_recording:
            # Launch Azure Kinect in its own terminal
            azure_proc = launch_terminal(
                [
                    "roslaunch",
                    "azure_kinect_ros_driver",
                    "driver.launch",
                    f"fps:={args.fps}"
                ],
                title="Azure Kinect Driver"
            )

            time.sleep(5)

            rviz_proc = launch_terminal(
                [
                    "rviz",
                    "-d",
                    "azure_kinect.rviz"
                ],
                title="RViz"
            )

            time.sleep(3)
        else:
            print(f"\nDrivers are sopposed to be already launched.")

        input("\nPress ENTER when you want to start recording...\n")

        # Start recording
        bag_proc = launch_process([
            "rosbag",
            "record",
            "-O",
            args.bag,
            "/rgb/image_raw"
        ])

        monitor_thread = threading.Thread(
            target=monitor_sample_rate,
            args=(args.sample_rate, stop_monitor),
            daemon=True
        )
        monitor_thread.start()

        time.sleep(2)

        input("\nPress ENTER when you want to stop recording...\n")

    finally:
        terminate_process(bag_proc, "rosbag")

    bag_path = f"{args.bag}.bag"
    active_bag_path = f"{bag_path}.active"

    while True:
        if os.path.exists(bag_path) and not os.path.exists(active_bag_path):
            break

        time.sleep(1)

    print(f"Bag recording completed and saved correctly.\n")

    info_cmd = f"""
    {ROS_SETUP}
    rosbag info {bag_path}
    """

    subprocess.run(
        ["bash", "-c", info_cmd]
    )

    extract_cmd = f"""
    {ROS_SETUP}

    python3 dataset_loader.py \
        --bag {args.bag}.bag \
        --rate {args.sample_rate} \
        --output {args.bag}
    """

    subprocess.run(
        ["bash", "-c", extract_cmd],
        check=True
    )

    print("\nDataset extraction completed.")



if __name__ == "__main__":
    main()
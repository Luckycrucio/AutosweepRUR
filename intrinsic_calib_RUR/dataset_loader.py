#!/usr/bin/env python3

import os
import argparse

import cv2
import rosbag

from cv_bridge import CvBridge


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--bag",
        required=True,
        help="Path to rosbag"
    )

    parser.add_argument(
        "--rate",
        type=float,
        required=True,
        help="Desired output image rate (Hz)"
    )

    parser.add_argument(
        "--topic",
        default="/rgb/image_raw",
        help="Image topic"
    )

    parser.add_argument(
        "--output",
        default="../../../../media/fsd/data_SSD/autosweep/intrinsic_calib_data",
        help="Output folder"
    )

    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    bridge = CvBridge()

    bag = rosbag.Bag(args.bag)

    sample_period = 1.0 / args.rate

    last_saved_time = None
    saved_count = 0

    for topic, msg, t in bag.read_messages(topics=[args.topic]):

        timestamp = msg.header.stamp.to_sec()

        if last_saved_time is None:
            save_frame = True
        else:
            save_frame = (timestamp - last_saved_time) >= sample_period

        if not save_frame:
            continue

        image = bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="bgr8"
        )

        filename = os.path.join(
            args.output,
            f"image_{saved_count:06d}.png"
        )

        cv2.imwrite(filename, image)

        last_saved_time = timestamp
        saved_count += 1

    bag.close()

    print(f"Saved {saved_count} images to {args.output}")


if __name__ == "__main__":
    main()
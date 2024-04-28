"""
Script for collecting telemetry data alongside video capture from Intel RealSense.

usage: data_collector.py [-h] [--no-telem] [--no-save]

options:
  -h, --help  show this help message and exit
  --no-telem  do not gather telemetry data
  --no-save   do not store captured video
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import argparse
import os
import json

from pathlib import Path
from datetime import datetime

from drone_control.mavlink_telemetry import MavlinkTelemetry

FPS = 30
RESOLUTION = (1280, 720)

TELEMETRY_CONNECTION_STRING = "udpin:0.0.0.0:1455"

datetime_str = datetime.now().strftime("%m-%d-%Y %H-%M-%S")
DATA_STORE_DIR = Path(f"data/test_data/{datetime_str}")


def data_collector(no_save: bool = True, no_telem: bool = False):
    """Collects telemetry data alongside video capture.

    :param no_save: bool, defaults to True, determines if the video capture should be saved.
    :param no_telem: bool, defaults to False, determines if the telemetry data should be gathered.
    """
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    pipeline.start(config)

    if not no_telem:
        telemetry = MavlinkTelemetry()
        telemetry.connect(TELEMETRY_CONNECTION_STRING)
        telemetry.start()

    if not no_save:
        os.makedirs(DATA_STORE_DIR, exist_ok=True)
        out = cv2.VideoWriter(
            str(DATA_STORE_DIR / "out.mp4"), 0x7634706D, FPS, RESOLUTION
        )

    data = []

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
            )

            depth_colormap_dim = depth_colormap.shape
            color_colormap_dim = color_image.shape

            if depth_colormap_dim != color_colormap_dim:
                color_image = cv2.resize(
                    color_image,
                    dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                    interpolation=cv2.INTER_AREA,
                )

            images = np.hstack((color_image, depth_colormap))

            if not no_telem:
                telem_data = telemetry.get_telem_data()
                print(telem_data)

            if not no_save:
                out.write(images)
                if not no_telem:
                    data.append(telem_data)

            cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
            cv2.imshow("RealSense", images)
            cv2.waitKey(1)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

        if not no_save:
            with open(DATA_STORE_DIR / "data.json") as file:
                json.dump(data, file)
            out.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no-telem", action="store_true", help="do not gather telemetry data"
    )
    parser.add_argument(
        "--no-save", action="store_true", help="do not store captured video"
    )
    args = parser.parse_args()
    try:
        data_collector(no_save=args.no_save, no_telem=args.no_telem)
    except ValueError as err:
        print(err)

"""
@file example_tron2_camera.py

@brief Read the Tron2 top camera ("/camera/top/color/image_raw/compressed").

The robot publishes its top camera as sensor_msgs/CompressedImage (JPEG or PNG bytes).
The SDK has no mirror of that message, so CompressedImage below is a hand-written
one in the same shape as the generated limxsdk.msg classes, and robot.subscribe()
(the SDK's generic subscriber) delivers it. Read only: nothing is published.

What it does:
    - prints the frame rate, image format and size
    - --save N    writes the next N frames as camera_<time>_<n>.jpg (no extra library)
    - --show      live window (needs OpenCV: pip install opencv-python)

Use the frames in your own code:
    from example_tron2_camera import CompressedImage, to_numpy
    sub = robot.subscribe(CompressedImage, "/camera/top/color/image_raw/compressed", on_image)
    def on_image(msg):
        img = to_numpy(msg)        # H x W x 3 uint8, BGR (needs OpenCV)

Usage:
    python3 example_tron2_camera.py [--ip IP] [--topic TOPIC] [--save N] [--show] [--seconds S]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import struct
import sys
import threading
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
from limxsdk.msg import Header

TOPIC = "/camera/top/color/image_raw/compressed"
_u32 = struct.Struct("<I").unpack_from


class CompressedImage(object):
    """Mirror of the ROS / mros `sensor_msgs/CompressedImage` message (decode only).

    Fields, in wire order:
      header   std_msgs/Header
      format   str    e.g. "jpeg", "png", "rgb8; jpeg compressed bgr8"
      data     bytes  the compressed image
    """

    TYPE = "sensor_msgs/CompressedImage"
    MD5 = "8f7a12909da2c9d3332d540a0977563f"
    MIN_ENCODED_SIZE = 24

    __slots__ = ("header", "format", "data")

    @staticmethod
    def definition():
        return ("std_msgs/Header header\nstring format\nuint8[] data\n"
                "================================================================================\n"
                "MSG: std_msgs/Header\nuint32 seq\ntime stamp\nstring frame_id\n")

    def __init__(self):
        self.header = Header()
        self.format = ""
        self.data = b""

    @classmethod
    def decode(cls, buf, offset=0):
        self = cls.__new__(cls)
        self.header, o = Header.decode(buf, offset)
        n = _u32(buf, o)[0]
        o += 4
        if o + n > len(buf):
            raise ValueError("CompressedImage.format overruns the frame")
        self.format = str(buf[o:o + n], "utf-8", "replace")
        o += n
        n = _u32(buf, o)[0]
        o += 4
        if o + n > len(buf):
            raise ValueError("CompressedImage.data overruns the frame")
        self.data = bytes(buf[o:o + n])
        return self, o + n


def to_numpy(msg):
    """Decode a CompressedImage to an OpenCV BGR image (numpy array). Needs opencv-python."""
    import cv2
    import numpy as np
    return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)


def main():
    parser = argparse.ArgumentParser(description="Read the Tron2 top camera.")
    parser.add_argument("--ip", default="10.192.1.2")
    parser.add_argument("--topic", default=TOPIC)
    parser.add_argument("--save", type=int, default=0, help="save the next N frames as .jpg/.png")
    parser.add_argument("--show", action="store_true", help="live window (needs opencv-python)")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after S seconds (0 = until Ctrl+C)")
    args = parser.parse_args()
    if args.show:
        try:
            import cv2  # noqa: F401
        except ImportError:
            print("ERROR: --show needs OpenCV: pip install opencv-python")
            sys.exit(1)

    robot = Robot(RobotType.Tron2)
    if not robot.init(args.ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    lock = threading.Lock()
    state = {"n": 0, "last": None, "saved": 0}

    def on_image(msg):
        with lock:
            state["n"] += 1
            state["last"] = msg

    sub = robot.subscribe(CompressedImage, args.topic, on_image)
    print("Subscribed to {} (Ctrl+C to stop).".format(args.topic))

    stamp = time.strftime("%Y%m%d_%H%M%S")
    t0 = time.monotonic()
    last_n, last_t, seen = 0, t0, None
    try:
        while not args.seconds or time.monotonic() - t0 < args.seconds:
            time.sleep(0.03)
            with lock:
                msg, n = state["last"], state["n"]
            if msg is not None and msg is not seen:
                seen = msg
                if state["saved"] < args.save:
                    ext = "png" if "png" in msg.format.lower() else "jpg"
                    path = "camera_{}_{:03d}.{}".format(stamp, state["saved"], ext)
                    with open(path, "wb") as f:
                        f.write(msg.data)
                    state["saved"] += 1
                    print("\nsaved {}".format(path))
                if args.show:
                    import cv2
                    img = to_numpy(msg)
                    if img is not None:
                        cv2.imshow(args.topic, img)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            break
            now = time.monotonic()
            if now - last_t >= 1.0:
                fps = (n - last_n) / (now - last_t)
                last_n, last_t = n, now
                info = "no frames yet" if msg is None else "format '{}', {:.0f} kB, frame_id '{}'".format(
                    msg.format, len(msg.data) / 1024.0, msg.header.frame_id)
                sys.stdout.write("\r\033[K{:5.1f} fps | {} frames | {} | decode errors {}".format(
                    fps, n, info, sub.decode_errors))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        print()


if __name__ == "__main__":
    main()

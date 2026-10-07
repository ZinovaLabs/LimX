"""
@file example_tron2_ee_probe.py

@brief Read-only probe of the Tron2 arm / tele-operation channels, saved to a JSON file.

Collects what is needed to design end-effector keyboard tele-operation:
    1. every topic visible on the robot (name / type) and whether the SDK mirrors its type
    2. a recording, for --duration seconds, of
         - "/vr_cmd"            VR head-set / controller state   (subscribeVrState)
         - "/sdk_vr_cmd"        synthetic VR input, if anything publishes it (generic subscribe)
         - "/arm/ee_pose_state" measured end-effector poses       (subscribeArmEePose)
         - "TeleOperation"      tele-operation status diagnostic  (subscribeTeleopState)

It only subscribes; nothing is published, so the robot is never commanded.

For the most useful recording, run it while someone tele-operates the arms with the
VR controllers: hold grip, move each controller slowly along x, then y, then z,
release grip, move the controller, grip again, and press the trigger once.

Usage:
    python3 example_tron2_ee_probe.py [robot_ip] [--duration SECONDS]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import json
import sys
import threading
import time

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
from limxsdk.msg import VRState

MAX_RATE_HZ = 20.0   # per-channel recording rate cap, keeps the file small


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_ip", nargs="?", default="10.192.1.2")
    parser.add_argument("--duration", type=float, default=30.0)
    args = parser.parse_args()

    robot = Robot(RobotType.Tron2)
    if not robot.init(args.robot_ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    out = {"robot_ip": args.robot_ip, "topics": [], "support": [],
           "vr_cmd": [], "sdk_vr_cmd": [], "ee_pose": [], "teleop": []}

    print("Discovering topics (up to 5 s)...")
    out["topics"] = [{"name": t["name"], "type": t["type"]} for t in robot.get_topics(5)]
    out["support"] = [{"name": s["name"], "type": s["type"], "mirrored": s["mirrored"],
                       "md5_match": s["md5_match"]} for s in robot.query_topic_support(0)]
    print("Found {} topics.".format(len(out["topics"])))

    lock = threading.Lock()
    last = {}
    t0 = time.time()

    def record(channel, row):
        now = time.time()
        if now - last.get(channel, 0.0) < 1.0 / MAX_RATE_HZ:
            return
        last[channel] = now
        row["t"] = round(now - t0, 3)
        with lock:
            out[channel].append(row)

    def on_vr(s: datatypes.VrState):
        record("vr_cmd", {
            "eye": list(s.eye_pose), "l": list(s.left_pose), "r": list(s.right_pose),
            "l_grip": s.left_grip, "r_grip": s.right_grip,
            "l_grip_pressed": s.left_grip_pressed, "r_grip_pressed": s.right_grip_pressed,
            "l_trig": s.left_trigger, "r_trig": s.right_trigger,
            "l_js": list(s.left_joystick), "r_js": list(s.right_joystick),
            "btn": [s.button_x, s.button_y, s.button_a, s.button_b]})

    def on_sdk_vr(m: VRState):
        record("sdk_vr_cmd", {"frame_id": m.header.frame_id, "l": list(m.l), "r": list(m.r),
                              "leftGrip": m.leftGrip, "rightGrip": m.rightGrip,
                              "LG": m.LG, "RG": m.RG})

    def on_ee(p: datatypes.ArmEePose):
        if p.valid:
            record("ee_pose", {"l_pos": list(p.left_position), "l_quat_wxyz": list(p.left_quat),
                               "r_pos": list(p.right_position), "r_quat_wxyz": list(p.right_quat)})

    def on_teleop(s: datatypes.TeleopState):
        with lock:
            out["teleop"].append({"t": round(time.time() - t0, 3), "level": s.level, "code": s.code,
                                  "message": s.message, "healthy": s.healthy,
                                  "takeover_known": s.takeover_known,
                                  "takeover_active": s.takeover_active})

    robot.subscribeVrState(on_vr)
    robot.subscribeArmEePose(on_ee)
    robot.subscribeTeleopState(on_teleop)
    try:
        robot.subscribe(VRState, "/sdk_vr_cmd", on_sdk_vr)
    except Exception as e:
        out["sdk_vr_cmd_error"] = repr(e)

    print("Recording for {:.0f} s. Move the VR controllers now (Ctrl+C to stop early).".format(args.duration))
    try:
        end = time.time() + args.duration
        while time.time() < end:
            with lock:
                counts = {k: len(out[k]) for k in ("vr_cmd", "sdk_vr_cmd", "ee_pose", "teleop")}
            sys.stdout.write("\r{:4.0f} s left  {}   ".format(end - time.time(), counts))
            sys.stdout.flush()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass

    path = "tron2_probe_{}.json".format(time.strftime("%Y%m%d_%H%M%S"))
    with lock:
        with open(path, "w") as f:
            json.dump(out, f)
    print("\nSaved {}".format(path))


if __name__ == "__main__":
    main()

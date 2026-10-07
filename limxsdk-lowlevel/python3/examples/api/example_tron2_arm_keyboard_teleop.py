"""
@file example_tron2_arm_keyboard_teleop.py

@brief Keyboard tele-operation of the Tron2 dual-arm 2F-grippers, with live end-effector feedback.

Uses:
    - Robot.publishGripperCmd()     -> "/limx/2F-gripper/cmd"  ([left, right], % opening)
    - Robot.subscribeGripperState() <- "/limx/2F-gripper/state"
    - Robot.subscribeArmEePose()    <- "/arm/ee_pose_state"    (measured pose, base_Link frame)
It never publishes RobotCmd, so it does not take over the joint motors.

Keys (focus must be on this terminal):
    1 / 2 / b : select left / right / both grippers
    o / c     : fully open / close the selected gripper(s)
    ] / [     : open / close the selected gripper(s) by one step
    q, Ctrl+C : quit (grippers keep their last commanded opening)

Usage:
    python3 example_tron2_arm_keyboard_teleop.py [robot_ip]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import select
import sys
import termios
import time
import tty

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

OPENING_STEP = 10.0
OPENING_MAX = [100.0, 95.0]   # the SDK clamps the right finger to 95
GRIPPER_SPEED = 50.0
GRIPPER_FORCE = 50.0
DISPLAY_HZ = 10.0

SELECTIONS = {"1": ("left", [0]), "2": ("right", [1]), "b": ("both", [0, 1])}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main():
    robot_ip = sys.argv[1] if len(sys.argv) > 1 else "10.192.1.2"

    robot = Robot(RobotType.Tron2)
    if not robot.init(robot_ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    feedback = {"ee": None, "gripper_q": None}

    def on_ee_pose(pose: datatypes.ArmEePose):
        if pose.valid:
            feedback["ee"] = (list(pose.left_position), list(pose.right_position))

    def on_gripper_state(state: datatypes.GripperState):
        feedback["gripper_q"] = list(state.q)

    robot.subscribeArmEePose(on_ee_pose)
    robot.subscribeGripperState(on_gripper_state)

    # Start from the measured opening so the first key press does not jump the fingers.
    deadline = time.monotonic() + 2.0
    while feedback["gripper_q"] is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if feedback["gripper_q"] is not None and len(feedback["gripper_q"]) >= 2:
        opening = [clamp(feedback["gripper_q"][i], 0.0, OPENING_MAX[i]) for i in range(2)]
    else:
        print("WARNING: no gripper state yet; assuming both grippers are open.")
        opening = list(OPENING_MAX)

    cmd = datatypes.GripperCmd()
    cmd.speed = [GRIPPER_SPEED, GRIPPER_SPEED]
    cmd.force = [GRIPPER_FORCE, GRIPPER_FORCE]

    def send_gripper():
        cmd.opening = list(opening)
        cmd.stamp = time.time_ns()
        robot.publishGripperCmd(cmd)

    print(__doc__.split("Usage:")[0])

    sel_name, sel_idx = SELECTIONS["b"]
    period = 1.0 / DISPLAY_HZ

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            if select.select([sys.stdin], [], [], period)[0]:
                key = sys.stdin.read(1)
                if key == "q":
                    return
                elif key in SELECTIONS:
                    sel_name, sel_idx = SELECTIONS[key]
                elif key in ("o", "c", "]", "["):
                    for i in sel_idx:
                        if key == "o":
                            opening[i] = OPENING_MAX[i]
                        elif key == "c":
                            opening[i] = 0.0
                        else:
                            step = OPENING_STEP if key == "]" else -OPENING_STEP
                            opening[i] = clamp(opening[i] + step, 0.0, OPENING_MAX[i])
                    send_gripper()

            ee = feedback["ee"]
            ee_str = "n/a" if ee is None else "L({:+.3f},{:+.3f},{:+.3f}) R({:+.3f},{:+.3f},{:+.3f})".format(*ee[0], *ee[1])
            q = feedback["gripper_q"]
            q_str = "n/a" if not q else "/".join("{:.0f}".format(v) for v in q)
            sys.stdout.write("\r[{:5s}] grip cmd L={:3.0f} R={:3.0f} meas {} | ee {}   ".format(
                sel_name, opening[0], opening[1], q_str, ee_str))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        print("\nExit.")


if __name__ == "__main__":
    main()

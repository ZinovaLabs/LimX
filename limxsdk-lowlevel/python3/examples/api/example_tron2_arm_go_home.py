"""
@file example_tron2_arm_go_home.py

@brief Move the Tron2 arms (all 16 motors) slowly to the pose saved in home_pose.json.

home_pose.json is written by example_tron2_arm_drag_teach.py (--save-home or key 'p').
This firmware reports no motor names, so the pose is stored and sent by motor index
(motor_0 ... motor_15), in the same order as RobotState / RobotCmd.

What it sends: one RobotCmd for all 16 motors, in the same format as the robot's own
arm controller (no names, mode 0, dq 0, tau 0), with that controller's gains as
measured on this robot (DEFAULT_KP / DEFAULT_KD). Without a gravity feed-forward the
arms settle about a degree below the target, which is fine for a home pose.

Safety:
    - before anything is sent it listens for COMMAND_CHECK_S: if the robot's own
      controller is still commanding the motors ("/motor/cmd"), two masters would
      fight, so it refuses. Put the robot into its SDK / low-level control mode first
    - it moves at most MAX_SPEED rad/s per joint (at least MIN_TIME s), with a smooth
      start and stop
    - if any joint lags its target by more than MAX_TRACKING_ERROR (blocked, or
      something is wrong), it stops and holds the measured pose
    - on arrival it keeps holding the home pose until you quit
    Keep the e-stop in reach; what the motors do once the script stops publishing
    depends on the robot firmware.

Keys:
    s         start moving to home
    space     pause / resume (holds where the arms are)
    q, Ctrl+C quit

Usage:
    python3 example_tron2_arm_go_home.py [--ip IP] [--home home_pose.json] [--speed RAD_S]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import json
import math
import select
import sys
import termios
import threading
import time
import tty

import limxsdk.robot.Rate as Rate
import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes

# The robot's own controller gains, read from "/motor/cmd" on this robot (2026-10-07).
# Motors 0-6 and 7-13 are the two 7-joint arms, 14-15 the head / neck.
DEFAULT_KP = [420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0,
              420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0,
              350.0, 550.0]
DEFAULT_KD = [12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0,
              12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0,
              9.0, 9.0]

CMD_HZ = 500.0
MAX_SPEED = 0.2                # rad/s
MIN_TIME = 3.0                 # s
MAX_TRACKING_ERROR = 0.10      # rad (normal lag at full speed is about 1 deg)
AT_HOME_TOL = math.radians(2.0)
COMMAND_CHECK_S = 1.0


class Feedback(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.q = None
        self.cmd_frames = 0

    def on_state(self, s: datatypes.RobotState):
        with self.lock:
            self.q = list(s.q)

    def on_cmd(self, _c: datatypes.RobotCmd):
        with self.lock:
            self.cmd_frames += 1


def read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


def main():
    parser = argparse.ArgumentParser(description="Move the arms slowly to home_pose.json.")
    parser.add_argument("--ip", default="10.192.1.2")
    parser.add_argument("--home", default="home_pose.json")
    parser.add_argument("--speed", type=float, default=MAX_SPEED, help="max joint speed, rad/s (<= 0.5)")
    args = parser.parse_args()
    if not 0.0 < args.speed <= 0.5:
        parser.error("--speed must be in (0, 0.5]")

    with open(args.home) as f:
        home = json.load(f)
    goal = [float(v) for v in home["q"]]
    if len(goal) != len(DEFAULT_KP):
        print("ERROR: {} has {} joints, expected {}.".format(args.home, len(goal), len(DEFAULT_KP)))
        sys.exit(1)

    fb = Feedback()
    robot = Robot(RobotType.Tron2)
    if not robot.init(args.ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)
    robot.subscribeRobotState(fb.on_state)
    robot.subscribeRobotCmd(fb.on_cmd)

    deadline = time.monotonic() + 3.0
    while fb.q is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if fb.q is None:
        print("ERROR: no joint state from the robot.")
        sys.exit(1)
    if len(fb.q) != len(goal):
        print("ERROR: the robot has {} motors, {} has {}.".format(len(fb.q), args.home, len(goal)))
        sys.exit(1)

    # Is the robot's own controller still commanding the motors?
    with fb.lock:
        before = fb.cmd_frames
    time.sleep(COMMAND_CHECK_S)
    with fb.lock:
        rate_seen = (fb.cmd_frames - before) / COMMAND_CHECK_S
    if rate_seen > 0:
        print("ERROR: the robot's controller is commanding the motors ({:.0f} commands/s on /motor/cmd).\n"
              "Sending our commands too would make two controllers fight. Switch the robot to its\n"
              "SDK / low-level control mode (ask LimX how), then run this again.".format(rate_seen))
        sys.exit(1)

    with fb.lock:
        start = list(fb.q)
    diff = [abs(g - s) for g, s in zip(goal, start)]
    duration = max(MIN_TIME, max(diff) / args.speed)
    print("Home pose from {} (saved {}).".format(args.home, home.get("created", "?")))
    print("{:10s} {:>9s} {:>9s} {:>9s}".format("joint", "now deg", "home deg", "move deg"))
    for i, (s, g) in enumerate(zip(start, goal)):
        print("motor_{:<4d} {:+9.1f} {:+9.1f} {:+9.1f}".format(i, math.degrees(s), math.degrees(g), math.degrees(g - s)))
    print("Largest move {:.1f} deg, takes {:.1f} s. Press 's' to start, space to pause, 'q' to quit.".format(
        math.degrees(max(diff)), duration))

    cmd = datatypes.RobotCmd()
    n = len(goal)
    cmd.mode = [0] * n
    cmd.dq = [0.0] * n
    cmd.tau = [0.0] * n
    cmd.Kp = list(DEFAULT_KP)
    cmd.Kd = list(DEFAULT_KD)

    rate = Rate(CMD_HZ)
    phase = "wait"            # wait -> move -> hold
    paused = False
    t = 0.0
    target = None
    note = ""
    last = time.monotonic()
    next_display = 0.0

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            key = read_key()
            now = time.monotonic()
            dt, last = now - last, now
            with fb.lock:
                q = list(fb.q)
            if key == "q":
                break
            if key == "s" and phase == "wait":
                phase, t, start = "move", 0.0, list(q)
                diff = [abs(g - s) for g, s in zip(goal, start)]
                duration = max(MIN_TIME, max(diff) / args.speed)
                note = "moving home ({:.1f} s)".format(duration)
            elif key == " " and phase == "move":
                paused = not paused
                note = "paused" if paused else "moving home"

            if phase == "move" and not paused:
                t = min(duration, t + dt)
                s_ = t / duration
                s_ = s_ * s_ * (3.0 - 2.0 * s_)
                target = [a + (b - a) * s_ for a, b in zip(start, goal)]
                if t >= duration:
                    phase = "hold"
                    worst = max(range(len(q)), key=lambda i: abs(q[i] - goal[i]))
                    if abs(q[worst] - goal[worst]) <= AT_HOME_TOL:
                        note = "at home: holding (q to quit)"
                    else:
                        note = "holding, but motor_{} is {:.1f} deg from home".format(
                            worst, math.degrees(q[worst] - goal[worst]))
            if target is not None:
                err = max(abs(a - b) for a, b in zip(q, target))
                if err > MAX_TRACKING_ERROR:
                    worst = max(range(len(q)), key=lambda i: abs(q[i] - target[i]))
                    target, phase, paused = list(q), "hold", False
                    note = "STOPPED: motor_{} lags by {:.1f} deg (blocked?); holding the measured pose".format(
                        worst, math.degrees(err))
                cmd.stamp = time.time_ns()
                cmd.q = list(target)
                if not robot.publishRobotCmd(cmd):
                    note = "ERROR: publishRobotCmd failed"
                    break

            if now >= next_display:
                next_display = now + 0.1
                left = max(abs(a - b) for a, b in zip(q, goal))
                sys.stdout.write("\r\033[K{:5s}{} | furthest joint from home {:.1f} deg | {}".format(
                    phase, " PAUSED" if paused else "", math.degrees(left), note))
                sys.stdout.flush()
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        print("\nStopped publishing joint commands." if target is not None else "\nNothing was sent.")


if __name__ == "__main__":
    main()

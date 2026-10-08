"""
@file example_tron2_arm_record_replay.py

@brief Record the Tron2 arm trajectory (joint angles) and replay it on the robot.

record  - read only. Press 'r' to start, 'q' to stop and save. While you move the arms (VR tele-operation, keyboard, by hand),
          saves at REC_HZ:
            - measured joint state q / dq / tau of every motor   (subscribeRobotState)
            - the robot controller's own joint command, if one is
              published: q / dq / tau / Kp / Kd / mode            (subscribeRobotCmd)
            - measured end-effector poses                         ("/arm_pose")
            - gripper opening                                     (subscribeGripperState)
          Nothing is published, so recording is always safe.

replay  - drives the selected joints (default: arm joints) with publishRobotCmd:
            1. waits for you to press 's'
            2. moves slowly (MAX_RAMP_SPEED) from the current pose to the first
               recorded pose
            3. plays the recorded joint trajectory with its original timing
               (--speed scales it), interpolated to CMD_HZ
            4. holds the last pose until you press 'q'
          Gains (Kp / Kd) and the feed-forward torque come from the controller
          command recorded alongside the trajectory, so the arms are driven the way
          the robot's own controller drove them. If no controller command was
          recorded, --kp and --kd must be given and tau is 0 (no gravity
          compensation: the arms may sag).

Before replaying:
    - nothing else may command the arm motors (VR tele-operation off, no other SDK
      program); otherwise the commands fight
    - keep the e-stop in reach. What the motors do once this script stops
      publishing (hold, damp, go limp) depends on the robot firmware; support the
      arms or be ready to stop the robot when you quit.

Keys during record:
    r         start recording
    q, Ctrl+C stop and save (quitting before 'r' writes nothing)

Keys during replay:
    s         start (ramp to the first pose, then play)
    space     pause / resume (holds the current pose)
    q, Ctrl+C quit (see the warning above)

Usage:
    python3 example_tron2_arm_record_replay.py record FILE [--ip IP]
    python3 example_tron2_arm_record_replay.py replay FILE [--ip IP] [--speed S]
                                               [--joints REGEX] [--kp KP --kd KD]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import json
import re
import select
import sys
import termios
import threading
import time
import tty

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
from limxsdk.msg import Float32MultiArray

REC_HZ = 100.0                 # recording rate (state, command and EE are each capped to this)
CMD_HZ = 500.0                 # replay command rate
MAX_RAMP_SPEED = 0.2           # rad/s, move to the first recorded pose
MIN_RAMP_TIME = 3.0            # s
MAX_REPLAY_SPEED = 2.0         # rad/s, refuse to replay faster joint motion than this
DEFAULT_JOINTS = r"arm|shoulder|elbow|wrist|^motor_([0-9]|1[0-3])$"
# The robot arm controller's own gains (read from /motor/cmd, 2026-10-07): motors 0-6 left
# arm, 7-13 right arm, 14-15 head. Used for motors that are held but not replayed.
TRON2_DEFAULT_KP = [420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0,
                    420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0, 350.0, 550.0]
TRON2_DEFAULT_KD = [12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0,
                    12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0, 9.0, 9.0]


def label_names(names, n):
    """This firmware sends no motor names: label the joints motor_0 ... motor_<n-1>."""
    names = list(names)
    return names if any(names) else ["motor_{}".format(i) for i in range(n)]


class Pacer(object):
    """Fixed-rate loop timing in plain Python: sleeps at most one period, never blocks
    longer, and re-synchronises after an overrun (used instead of the SDK's Rate)."""

    def __init__(self, hz):
        self.period = 1.0 / hz
        self.next = time.monotonic() + self.period

    def sleep(self):
        now = time.monotonic()
        wait = self.next - now
        if wait > 0:
            time.sleep(min(wait, self.period))
            self.next += self.period
        else:
            self.next = now + self.period      # overrun: skip ahead instead of catching up


class Feedback(object):
    """Latest values from the robot, written by SDK callback threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = None      # (names, q, dq, tau)
        self.cmd = None        # dict of the controller's last joint command
        self.ee = None
        self.gripper = None
        self.state_frames = 0
        self.cmd_frames = 0

    def on_state(self, s: datatypes.RobotState):
        with self.lock:
            self.state = (label_names(s.motor_names, len(s.q)), list(s.q), list(s.dq), list(s.tau))
            self.state_frames += 1

    def on_cmd(self, c: datatypes.RobotCmd):
        with self.lock:
            self.cmd = {"names": label_names(c.motor_names, len(c.q)), "q": list(c.q), "dq": list(c.dq),
                        "tau": list(c.tau), "Kp": list(c.Kp), "Kd": list(c.Kd), "mode": list(c.mode)}
            self.cmd_frames += 1

    def on_arm_pose(self, m: Float32MultiArray):
        d = list(m.data)
        if len(d) >= 14:
            with self.lock:
                self.ee = d[:14]

    def on_gripper(self, g: datatypes.GripperState):
        with self.lock:
            self.gripper = list(g.q)


def connect(ip, fb):
    robot = Robot(RobotType.Tron2)
    if not robot.init(ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)
    robot.subscribeRobotState(fb.on_state)
    robot.subscribeGripperState(fb.on_gripper)
    sub = robot.subscribe(Float32MultiArray, "/arm_pose", fb.on_arm_pose)
    return robot, sub


def read_key():
    """Non-blocking single key, or '' if none is waiting."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


# ----------------------------------------------------------------------------- record

def record(args):
    fb = Feedback()
    robot, sub = connect(args.ip, fb)
    robot.subscribeRobotCmd(fb.on_cmd)

    deadline = time.monotonic() + 3.0
    while fb.state is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if fb.state is None:
        print("ERROR: no joint state from the robot.")
        sys.exit(1)
    names = fb.state[0]
    print("{} motors: {}".format(len(names), ", ".join(names)))
    print("Press 'r' to start recording to {}, 'q' (or Ctrl+C) to stop and save.".format(args.file))

    frames = []
    period = 1.0 / REC_HZ
    t0 = None               # set when 'r' is pressed
    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        next_t = time.monotonic()
        while True:
            key = read_key()
            if key == "q":
                break
            now = time.monotonic()
            if t0 is None:
                if key == "r":
                    t0 = now
                else:
                    sys.stdout.write("\r\033[Kwaiting: press 'r' to start recording")
                    sys.stdout.flush()
                    next_t += period
                    time.sleep(max(0.0, next_t - time.monotonic()))
                    continue
            with fb.lock:
                _, q, dq, tau = fb.state
                row = {"t": round(now - t0, 4), "q": q, "dq": dq, "tau": tau,
                       "cmd": fb.cmd, "ee": fb.ee, "gripper": fb.gripper}
                cmd_frames = fb.cmd_frames
            frames.append(row)
            sys.stdout.write("\r\033[KRECORDING {:6.1f} s  {} frames  controller cmd frames {}".format(
                now - t0, len(frames), cmd_frames))
            sys.stdout.flush()
            next_t += period
            time.sleep(max(0.0, next_t - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)

    if not frames:
        print("\nNothing recorded; no file written.")
        sub.close()
        return
    with open(args.file, "w") as f:
        json.dump({"robot_ip": args.ip, "rec_hz": REC_HZ, "motor_names": names,
                   "created": time.strftime("%Y-%m-%d %H:%M:%S"), "frames": frames}, f)
    print("\nSaved {} frames ({:.1f} s) to {}".format(len(frames), frames[-1]["t"] if frames else 0.0, args.file))
    if not any(r["cmd"] for r in frames):
        print("NOTE: no controller joint command was seen; replay will need --kp and --kd.")
    sub.close()


# ----------------------------------------------------------------------------- replay

def interp(a, b, s):
    return [x + (y - x) * s for x, y in zip(a, b)]


def replay(args):
    with open(args.file) as f:
        rec = json.load(f)
    frames = rec["frames"]
    names = rec["motor_names"]
    if len(frames) < 2:
        print("ERROR: {} has fewer than 2 frames.".format(args.file))
        sys.exit(1)

    sel = [i for i, n in enumerate(names) if re.search(args.joints, n, re.IGNORECASE)]
    if not sel:
        print("ERROR: --joints {!r} matches none of: {}".format(args.joints, ", ".join(names)))
        sys.exit(1)
    sel_names = [names[i] for i in sel]
    print("Replaying {} joints: {}".format(len(sel), ", ".join(sel_names)))

    # Per-frame targets for the selected joints. Prefer the controller's command
    # (same gains / feed-forward the robot used); fall back to the measured state.
    def pick(row):
        c = row.get("cmd")
        if c and c.get("names"):
            idx = {n: k for k, n in enumerate(c["names"])}
            if all(n in idx for n in sel_names):
                g = lambda key: [c[key][idx[n]] for n in sel_names]
                return g("q"), g("dq"), g("tau"), g("Kp"), g("Kd"), g("mode")
        return [row["q"][i] for i in sel], [row["dq"][i] for i in sel], None, None, None, None

    track = [pick(r) for r in frames]
    times = [r["t"] / args.speed for r in frames]
    from_cmd = all(t[3] is not None for t in track)
    if from_cmd:
        print("Using the recorded controller command (q, dq, tau, Kp, Kd, mode).")
    else:
        if args.kp is None or args.kd is None:
            print("ERROR: the recording has no complete controller command; pass --kp and --kd.")
            sys.exit(1)
        print("Using measured joint angles with Kp={} Kd={} and tau=0.".format(args.kp, args.kd))
        track = [(q, dq, [0.0] * len(sel), [args.kp] * len(sel), [args.kd] * len(sel), [0] * len(sel))
                 for q, dq, *_ in track]

    # Safety: joint speed of the replayed motion.
    peak = 0.0
    for k in range(1, len(track)):
        dt = times[k] - times[k - 1]
        if dt > 0:
            peak = max(peak, max(abs(b - a) / dt for a, b in zip(track[k - 1][0], track[k][0])))
    print("Peak joint speed in the replay: {:.2f} rad/s".format(peak))
    if peak > MAX_REPLAY_SPEED:
        print("ERROR: faster than {:.1f} rad/s; use a smaller --speed.".format(MAX_REPLAY_SPEED))
        sys.exit(1)

    fb = Feedback()
    robot, sub = connect(args.ip, fb)
    ctrl = {"n": 0}
    robot.subscribeRobotCmd(lambda _c: ctrl.__setitem__("n", ctrl["n"] + 1))
    time.sleep(1.0)
    if ctrl["n"] > 0:
        print("ERROR: the robot's arm controller is commanding the motors ({} commands in 1 s).\n"
              "Replaying would fight it. Switch the robot to low-level mode first.".format(ctrl["n"]))
        sys.exit(1)
    deadline = time.monotonic() + 3.0
    while fb.state is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if fb.state is None:
        print("ERROR: no joint state from the robot.")
        sys.exit(1)
    live_names = fb.state[0]
    if len(live_names) != len(names) or [live_names[i] for i in sel] != sel_names:
        print("ERROR: the robot's motor names differ from the recording.")
        sys.exit(1)

    start_q = [fb.state[1][i] for i in sel]
    first_q = track[0][0]
    max_diff = max(abs(a - b) for a, b in zip(start_q, first_q))
    ramp_time = max(MIN_RAMP_TIME, max_diff / MAX_RAMP_SPEED)
    print("Current pose is up to {:.2f} rad from the first recorded pose; ramp takes {:.1f} s.".format(
        max_diff, ramp_time))
    print("Press 's' to start, space to pause, 'q' to quit ('q' twice once the arms are held:\n"
          "in low-level mode they go limp when commands stop, so support them first).")

    cmd = datatypes.RobotCmd()
    n_all = len(live_names)
    # Unnamed firmware: command all motors in index order, named motor_<i> (the SDK's
    # publishRobotCmd segfaults on empty names); motors not replayed are held in place.
    full = n_all == len(TRON2_DEFAULT_KP) and all(n == "motor_{}".format(i) for i, n in enumerate(live_names))
    sel_pos = {i: k for k, i in enumerate(sel)}
    rest_q = list(fb.state[1])
    if full:
        cmd.motor_names = list(live_names)
        cmd.parallel_solve_required = [False] * n_all
    else:
        cmd.motor_names = sel_names
        cmd.parallel_solve_required = [False] * len(sel)

    def send(q, dq, tau, kp, kd, mode):
        cmd.stamp = time.time_ns()
        if full:
            pick_ = lambda vals, other: [vals[sel_pos[i]] if i in sel_pos else other(i) for i in range(n_all)]
            cmd.q = pick_(q, lambda i: rest_q[i])
            cmd.dq = pick_(dq, lambda i: 0.0)
            cmd.tau = pick_(tau, lambda i: 0.0)
            cmd.Kp = pick_(kp, lambda i: TRON2_DEFAULT_KP[i])
            cmd.Kd = pick_(kd, lambda i: TRON2_DEFAULT_KD[i])
            cmd.mode = [int(m) for m in pick_(mode, lambda i: 0)]
        else:
            cmd.q, cmd.dq, cmd.tau, cmd.Kp, cmd.Kd = list(q), list(dq), list(tau), list(kp), list(kd)
            cmd.mode = [int(m) for m in mode]
        return robot.publishRobotCmd(cmd)

    rate = Pacer(CMD_HZ)
    phase = "wait"          # wait -> ramp -> play -> hold
    paused = False
    clock = 0.0             # time inside the current phase (does not advance while paused)
    k = 0
    hold = None             # last command sent, repeated while paused / holding
    last = time.monotonic()
    next_display = 0.0
    release_armed = False
    note = ""

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            key = read_key()
            if key and key != "q":
                release_armed = False
            if key == "q":
                if hold is None or release_armed:
                    break
                release_armed, paused = True, True
                note = "holding: support the arms, then press q again (they go limp)"
            if key == "s" and phase == "wait":
                phase, clock = "ramp", 0.0
                start_q = [fb.state[1][i] for i in sel]     # re-read just before moving
                rest_q = list(fb.state[1])
            elif key == " " and phase in ("ramp", "play"):
                paused = not paused

            now = time.monotonic()
            dt, last = now - last, now
            if phase == "wait":
                rate.sleep()
                out = None
            elif paused:
                out = hold
            elif phase == "ramp":
                clock += dt
                s = min(1.0, clock / ramp_time)
                s = s * s * (3.0 - 2.0 * s)                  # smooth start / stop
                q0, _, tau0, kp0, kd0, mode0 = track[0]
                out = (interp(start_q, q0, s), [0.0] * len(sel), tau0, kp0, kd0, mode0)
                if clock >= ramp_time:
                    phase, clock, k = "play", 0.0, 0
            elif phase == "play":
                clock += dt
                while k + 1 < len(times) and times[k + 1] - times[0] <= clock:
                    k += 1
                if k + 1 >= len(times):
                    phase = "hold"
                    q, _, tau, kp, kd, mode = track[-1]
                    out = (q, [0.0] * len(sel), tau, kp, kd, mode)
                else:
                    a, b = track[k], track[k + 1]
                    span = times[k + 1] - times[k]
                    s = 0.0 if span <= 0 else (clock - (times[k] - times[0])) / span
                    out = (interp(a[0], b[0], s), [v * args.speed for v in interp(a[1], b[1], s)],
                           interp(a[2], b[2], s), a[3], a[4], a[5])
            else:   # hold
                out = hold

            if out is not None:
                if not send(*out):
                    print("\nERROR: publishRobotCmd failed; stopping.")
                    break
                hold = out
                rate.sleep()

            if now >= next_display:
                next_display = now + 0.1
                err = "n/a"
                if hold is not None and fb.state is not None:
                    err = "{:.3f}".format(max(abs(fb.state[1][i] - c) for i, c in zip(sel, hold[0])))
                label = "PAUSED" if paused else phase
                prog = "{:.1f}/{:.1f}s".format(clock, times[-1] - times[0]) if phase == "play" else ""
                sys.stdout.write("\r\033[K{:6s} {:14s} max tracking error {} rad  {}".format(label, prog, err, note))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        sub.close()
        print("\nStopped publishing joint commands." if hold is not None else "\nNothing was sent.")


def main():
    parser = argparse.ArgumentParser(description="Record / replay the Tron2 arm trajectory.")
    parser.add_argument("mode", choices=["record", "replay"])
    parser.add_argument("file")
    parser.add_argument("--ip", default="10.192.1.2")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    parser.add_argument("--joints", default=DEFAULT_JOINTS,
                        help="regex of motor names to replay (default: %(default)s)")
    parser.add_argument("--kp", type=float, help="Kp if the recording has no controller command")
    parser.add_argument("--kd", type=float, help="Kd if the recording has no controller command")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed must be > 0")
    if args.mode == "record":
        record(args)
    else:
        replay(args)


if __name__ == "__main__":
    main()

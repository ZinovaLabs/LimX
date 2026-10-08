"""
@file example_tron2_arm_drag_teach.py

@brief Drag-teach for the Tron2 arms: make the arms soft (gravity compensated), guide
them by hand, and record the motion for example_tron2_arm_record_replay.py.

How it works:
    - the arm weight model comes from the robot's URDF (getRobotDescription); the
      gravity torque of every selected joint is computed from it in pure Python
    - SOFT mode sends, per selected joint: Kp = --soft-kp (0 by default), a small Kd,
      feed-forward tau = gravity torque, and a target that follows the measured pose
      with a SOFT_FILTER_S lag (so a small --soft-kp gives gentle resistance).
      The arm then floats and follows your hand
    - HOLD mode holds the current pose stiffly (hold gains, plus gravity torque)

Robot modes:
    - normal mode (the robot's arm controller publishes /motor/cmd): only the model check
      runs (model vs the controller's gravity feed-forward); nothing is commanded, since
      two controllers would fight
    - low-level mode (nothing commands the motors): drag-teach runs as below

Flow in low-level mode:
    1. PASSIVE: nothing is sent. Lift the arms by hand into a free pose: off the body,
       off any joint limit
    2. space -> CHECK: the arms are held with position control only (no gravity help).
       After CHECK_SETTLE_S the torque the motors need to hold them is their real
       gravity load; it is compared with the URDF model for CHECK_SECONDS
    3. passed -> HOLD: gravity compensation is blended in over FF_RAMP_S, soft unlocks.
       failed -> HOLD without gravity help, soft stays locked; 'c' re-checks
    4. space -> SOFT: stiffness ramps down over RAMP_S; guide the arm by hand
    Commands cover all 16 motors in index order, named motor_0 ... motor_15 (the SDK
    crashes on unnamed commands); the head is held where it was. Gains are the robot arm controller's own (TRON2_DEFAULT_KP / _KD).

Safety:
    - soft mode is locked until the model check passes (tolerance max(CHECK_ABS_NM,
      CHECK_REL * |measured|) per joint; --force overrides, only if you know why)
    - HOLD -> SOFT ramps the stiffness down over RAMP_S; do not touch the arm while
      it ramps: if any joint drifts more than RAMP_ABORT from the hold pose, the
      model is not holding the arm and it goes straight back to HOLD
    - SOFT -> HOLD is immediate
    - if any joint moves faster than MAX_DRAG_SPEED in SOFT mode (arm falling,
      model wrong), it switches to HOLD at the current pose
    - near a URDF joint limit (LIMIT_MARGIN) a virtual spring pushes back
    - --gravity-scale only fine-tunes the model (e.g. 0.95 - 1.05). Well below 1 the
      arm does not sink slowly, it falls until the speed trip catches it
    - quitting while the arms are held takes 'q' twice: in low-level mode the arms go
      limp once commands stop, so support them first
    Hold the arm with your hand whenever you switch modes and keep the e-stop in reach.

Keys:
    space     PASSIVE -> hold + model check; then HOLD <-> SOFT
    c         re-run the model check (arms held, position control only)
    r         start / stop recording (each stop writes drag_<time>.json)
    p         save the current arm pose as home (written to --home, default
              home_pose.json)
              The home pose saved in that file is the default home pose on every
              start; only if there is none yet is the start-up pose saved as home.
    --save-home   save the arms' pose right now as the default home pose and exit
                  (sends nothing; no model check needed)
    h         go home: HOLD, then move slowly (MAX_HOME_SPEED) to the saved home
              pose; space stops the move and holds where the arm is
    q         PASSIVE: quit. Holding: press twice (support the arms first)

Usage:
    python3 example_tron2_arm_drag_teach.py [--ip IP] --check-only    # model check, sends nothing
    python3 example_tron2_arm_drag_teach.py [--ip IP] --save-home     # save the pose now as home, sends nothing
    python3 example_tron2_arm_drag_teach.py [--ip IP] [--joints REGEX] [--gravity-scale S]
           [--soft-kp KP] [--soft-kd KD] [--hold-kp KP --hold-kd KD]

Replay a recording:
    python3 example_tron2_arm_record_replay.py replay drag_<time>.json --speed 0.5

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import json
import math
import os
import re
import select
import sys
import termios
import threading
import time
import tty
import xml.etree.ElementTree as ET

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
from limxsdk.msg import Float32MultiArray

CMD_HZ = 500.0
REC_HZ = 100.0
GRAVITY = 9.81
DEFAULT_JOINTS = r"arm|shoulder|elbow|wrist|^motor_([0-9]|1[0-3])$"
# Shipped model of our robot (DACH_TRON2A), see limxsdk-lowlevel/urdf/README.md.
DEFAULT_URDF = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                             "..", "..", "..", "urdf", "DACH_TRON2A.urdf"))
# This firmware sends no motor names; motor_<i> -> URDF joint (checked against the
# robot's own gravity torques). The head motors 14 / 15 are not mapped.
_ARM = ["proximal_pitch", "proximal_roll", "proximal_yaw", "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll"]
UNNAMED_MOTOR_JOINTS = {"motor_{}".format(i): "{}_{}_Joint".format(j, side)
                        for side, base in (("L", 0), ("R", 7)) for i, j in ((base + k, a) for k, a in enumerate(_ARM))}


def label_names(names, n):
    """This firmware sends no motor names: label the joints motor_0 ... motor_<n-1>."""
    names = list(names)
    return names if any(names) else ["motor_{}".format(i) for i in range(n)]
CHECK_SECONDS = 1.0            # torque averaging window for the model check
CHECK_ABS_NM = 1.0             # model check tolerance, absolute part
CHECK_REL = 0.10               # model check tolerance, relative part
CTRL_CHECK_ABS_NM = 0.3        # tolerance against the robot controller's own gravity torque
CTRL_CHECK_REL = 0.05
RAMP_S = 2.0                   # HOLD -> SOFT stiffness ramp
RAMP_ABORT = math.radians(3.0) # drift allowed while the stiffness ramps down
SOFT_FILTER_S = 0.3            # SOFT target lags the measured pose by this time constant
MAX_DRAG_SPEED = 1.5           # rad/s, trips back to HOLD
LIMIT_MARGIN = math.radians(5.0)
HOLD_ON_EXIT_S = 1.0
CHECK_SETTLE_S = 1.5           # hold this long before measuring the holding torque
FF_RAMP_S = 1.0                # blend the gravity feed-forward in after the check
# The robot arm controller's own gains, read from /motor/cmd on this robot (2026-10-07):
# motors 0-6 left arm, 7-13 right arm, 14-15 head.
TRON2_DEFAULT_KP = [420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0,
                    420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0, 350.0, 550.0]
TRON2_DEFAULT_KD = [12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0,
                    12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0, 9.0, 9.0]
MAX_HOME_SPEED = 0.2           # rad/s, move to the home pose
MIN_HOME_TIME = 3.0            # s


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


# ----------------------------------------------------------------------------- math

def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def rpy_mat(r, p, y):
    """URDF rpy: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def axis_angle(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


# ----------------------------------------------------------------------------- URDF gravity model

def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def _unit(v):
    n = math.sqrt(dot(v, v))
    return [x / n for x in v] if n > 0 else [1.0, 0.0, 0.0]


class GravityModel(object):
    """Fixed-base, z-up gravity torques from a URDF."""

    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.links = {}           # name -> (mass, com xyz in link frame)
        for link in root.findall("link"):
            inertial = link.find("inertial")
            mass, com = 0.0, [0.0, 0.0, 0.0]
            if inertial is not None:
                m = inertial.find("mass")
                mass = float(m.get("value")) if m is not None else 0.0
                o = inertial.find("origin")
                if o is not None:
                    com = _floats(o.get("xyz"), [0, 0, 0])
            self.links[link.get("name")] = (mass, com)
        self.joints = {}          # name -> dict
        self.child_joint = {}     # child link -> joint name
        self.children = {}        # parent link -> [joint names]
        for j in root.findall("joint"):
            o = j.find("origin")
            a = j.find("axis")
            lim = j.find("limit")
            info = {
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "xyz": _floats(o.get("xyz") if o is not None else None, [0, 0, 0]),
                "rpy": _floats(o.get("rpy") if o is not None else None, [0, 0, 0]),
                "axis": _unit(_floats(a.get("xyz") if a is not None else None, [1, 0, 0])),
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
            }
            self.joints[j.get("name")] = info
            self.child_joint[info["child"]] = j.get("name")
            self.children.setdefault(info["parent"], []).append(j.get("name"))
        roots = [l for l in self.links if l not in self.child_joint]
        if len(roots) != 1:
            raise ValueError("URDF must have exactly one root link, found {}".format(roots))
        self.root = roots[0]

    def forward(self, q):
        """World poses of every link and joint for joint positions q (dict, missing = 0)."""
        link_pose = {self.root: ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])}
        joint_frame = {}          # joint -> (origin in world, axis in world)
        stack = [self.root]
        while stack:
            parent = stack.pop()
            p_pos, p_rot = link_pose[parent]
            for jn in self.children.get(parent, []):
                j = self.joints[jn]
                pos = add(p_pos, mat_vec(p_rot, j["xyz"]))
                rot = mat_mul(p_rot, rpy_mat(*j["rpy"]))
                axis_w = mat_vec(rot, j["axis"])
                joint_frame[jn] = (pos, axis_w)
                v = q.get(jn, 0.0)
                if j["type"] in ("revolute", "continuous"):
                    rot = mat_mul(rot, axis_angle(j["axis"], v))
                elif j["type"] == "prismatic":
                    pos = add(pos, [axis_w[0] * v, axis_w[1] * v, axis_w[2] * v])
                link_pose[j["child"]] = (pos, rot)
                stack.append(j["child"])
        return link_pose, joint_frame

    def subtree_links(self, joint):
        out, stack = [], [self.joints[joint]["child"]]
        while stack:
            link = stack.pop()
            out.append(link)
            stack.extend(self.joints[jn]["child"] for jn in self.children.get(link, []))
        return out

    def gravity(self, q, joint_names):
        """Torque each joint must apply to hold the arm still against gravity."""
        link_pose, joint_frame = self.forward(q)
        out = []
        for jn in joint_names:
            j = self.joints[jn]
            p_j, axis = joint_frame[jn]
            total = 0.0
            for link in self.subtree_links(jn):
                mass, com = self.links.get(link, (0.0, [0, 0, 0]))
                if mass <= 0.0:
                    continue
                pos, rot = link_pose[link]
                c = add(pos, mat_vec(rot, com))
                f = [0.0, 0.0, -mass * GRAVITY]
                if j["type"] == "prismatic":
                    total += dot(axis, f)
                else:
                    total += dot(axis, cross(sub(c, p_j), f))
            out.append(-total)
        return out


# ----------------------------------------------------------------------------- robot I/O

class Feedback(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None     # (names, q, dq, tau)
        self.cmd = None       # controller command seen before we publish anything
        self.cmd_frames = 0
        self.ee = None
        self.gripper = None

    def on_state(self, s: datatypes.RobotState):
        names = list(s.motor_names)
        if not any(names):
            # This firmware sends no motor names: label the joints by index.
            names = ["motor_{}".format(i) for i in range(len(s.q))]
        with self.lock:
            self.state = (names, list(s.q), list(s.dq), list(s.tau))

    def on_cmd(self, c: datatypes.RobotCmd):
        with self.lock:
            self.cmd = {"names": label_names(c.motor_names, len(c.Kp)), "Kp": list(c.Kp), "Kd": list(c.Kd),
                        "tau": list(c.tau)}
            self.cmd_frames += 1

    def on_arm_pose(self, m: Float32MultiArray):
        d = list(m.data)
        if len(d) >= 14:
            with self.lock:
                self.ee = d[:14]

    def on_gripper(self, g: datatypes.GripperState):
        with self.lock:
            self.gripper = list(g.q)


def read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


def main():
    parser = argparse.ArgumentParser(description="Drag-teach (gravity-compensated soft arms).")
    parser.add_argument("--ip", default="10.192.1.2")
    parser.add_argument("--joints", default=DEFAULT_JOINTS, help="regex of arm motor names (default: %(default)s)")
    parser.add_argument("--check-only", action="store_true", help="run the model check and exit; sends nothing")
    parser.add_argument("--force", action="store_true", help="go soft even if the model check fails")
    parser.add_argument("--gravity-scale", type=float, default=1.0)
    parser.add_argument("--soft-kp", type=float, default=0.0)
    parser.add_argument("--soft-kd", type=float, default=None, help="default: 10%% of the hold Kd")
    parser.add_argument("--hold-kp", type=float, help="hold Kp if the controller's gains cannot be read")
    parser.add_argument("--hold-kd", type=float, help="hold Kd if the controller's gains cannot be read")
    parser.add_argument("--home", default="home_pose.json", help="home pose file (default: %(default)s)")
    parser.add_argument("--urdf", default=DEFAULT_URDF,
                        help="URDF file (default: the shipped DACH_TRON2A model); '' fetches it from the robot")
    parser.add_argument("--save-home", action="store_true",
                        help="save the arms' current pose as the default home pose and exit; sends nothing")
    parser.add_argument("--no-parallel-solve", action="store_true",
                        help="send parallel_solve_required=False (default True, the SDK default)")
    args = parser.parse_args()
    if not 0.8 <= args.gravity_scale <= 1.2:
        parser.error("--gravity-scale must be in [0.8, 1.2]")

    fb = Feedback()
    robot = Robot(RobotType.Tron2)
    if not robot.init(args.ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)
    robot.subscribeRobotState(fb.on_state)
    robot.subscribeRobotCmd(fb.on_cmd)
    robot.subscribeGripperState(fb.on_gripper)
    sub_pose = robot.subscribe(Float32MultiArray, "/arm_pose", fb.on_arm_pose)

    deadline = time.monotonic() + 3.0
    while fb.state is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if fb.state is None:
        print("ERROR: no joint state from the robot.")
        sys.exit(1)
    names = fb.state[0]
    if args.save_home:
        with fb.lock:
            q_now = fb.state[1]
        qh = list(q_now)
        with open(args.home, "w") as f:
            json.dump({"motor_names": names, "q": qh, "created": time.strftime("%Y-%m-%d %H:%M:%S")}, f)
        print("Saved the current pose as the default home pose in {}:".format(args.home))
        for n, v in zip(names, qh):
            print("  {:28s} {:+.4f} rad ({:+.1f} deg)".format(n, v, math.degrees(v)))
        sub_pose.close()
        return

    sel = [i for i, n in enumerate(names) if re.search(args.joints, n, re.IGNORECASE)]
    if not sel:
        print("ERROR: --joints {!r} matches none of: {}".format(args.joints, ", ".join(names)))
        sys.exit(1)
    sel_names = [names[i] for i in sel]

    if args.urdf:
        if not os.path.isfile(args.urdf):
            print("ERROR: URDF file not found: {}\nLeave out --urdf to use the shipped model ({}).".format(
                args.urdf, DEFAULT_URDF))
            sys.exit(1)
        with open(args.urdf) as f:
            urdf = f.read()
    else:
        urdf = robot.getRobotDescription()
        if not urdf:
            print("ERROR: could not fetch the URDF from the robot (getRobotDescription); pass it with --urdf FILE.")
            sys.exit(1)
    model = GravityModel(urdf)
    jname = {n: n if n in model.joints else UNNAMED_MOTOR_JOINTS.get(n, n) for n in names}
    sel_joints = [jname[n] for n in sel_names]
    missing = [n for n in sel_names if jname[n] not in model.joints]
    if missing:
        print("ERROR: these motors are not URDF joints: {}\nURDF joints: {}".format(
            ", ".join(missing), ", ".join(sorted(model.joints))))
        sys.exit(1)

    def q_dict(q):
        return {jname[n]: q[i] for i, n in enumerate(names)}

    # ---- is the robot's own arm controller commanding the motors?
    with fb.lock:
        c0 = fb.cmd_frames
    print("Listening for the robot's arm controller for {:.0f} s...".format(CHECK_SECONDS))
    acc_c, count_c = [0.0] * len(sel), 0
    end = time.monotonic() + CHECK_SECONDS
    while time.monotonic() < end:
        with fb.lock:
            c = fb.cmd
        if c and c.get("tau") and all(n in c["names"] for n in sel_names):
            idx = {n: k for k, n in enumerate(c["names"])}
            for k, n in enumerate(sel_names):
                acc_c[k] += c["tau"][idx[n]]
            count_c += 1
        time.sleep(0.01)
    with fb.lock:
        controller_active = fb.cmd_frames > c0
        q_now = fb.state[1]

    if controller_active:
        # Normal mode: compare the model with the controller's gravity feed-forward. We must
        # not command the motors while that controller runs, so stop after the check.
        ctrl_ff = [a / max(1, count_c) for a in acc_c]
        g_model = model.gravity(q_dict(q_now), sel_joints)
        ok = True
        print("Reference: the robot controller's gravity feed-forward.")
        print("{:12s} {:26s} {:>9s} {:>11s} {:>4s}".format("motor", "URDF joint", "model Nm", "controller", "ok"))
        for k, (n, gm) in enumerate(zip(sel_names, g_model)):
            good = abs(gm - ctrl_ff[k]) <= max(CTRL_CHECK_ABS_NM, CTRL_CHECK_REL * abs(ctrl_ff[k]))
            ok = ok and good
            print("{:12s} {:26s} {:+9.2f} {:+11.2f} {:>4s}".format(n, sel_joints[k], gm, ctrl_ff[k], "yes" if good else "NO"))
        print("Model check {}.".format("PASSED" if ok else "FAILED"))
        if not args.check_only:
            print("The robot's arm controller is commanding the motors; drag-teach would fight it.\n"
                  "Switch the robot to low-level mode, then run this again.")
        sub_pose.close()
        return

    if args.check_only:
        print("Low-level mode: nothing commands the arms, so they carry no gravity torque to compare.\n"
              "Run without --check-only: the check runs after you put the arms in a free pose and\n"
              "press space (the script holds them first); soft mode stays locked until it passes.")
        sub_pose.close()
        return

    # ---- gains: command line, else the robot controller's own (measured 2026-10-07)
    n_all = len(names)
    full = n_all == len(TRON2_DEFAULT_KP) and all(n == "motor_{}".format(i) for i, n in enumerate(names))
    if args.hold_kp is not None and args.hold_kd is not None:
        hold_kp, hold_kd = [args.hold_kp] * len(sel), [args.hold_kd] * len(sel)
        print("Hold gains from the command line.")
    elif full:
        hold_kp = [TRON2_DEFAULT_KP[i] for i in sel]
        hold_kd = [TRON2_DEFAULT_KD[i] for i in sel]
        print("Hold gains: the robot arm controller's own (Kp {:.0f}-{:.0f}, Kd {:.0f}-{:.0f}).".format(
            min(hold_kp), max(hold_kp), min(hold_kd), max(hold_kd)))
    else:
        print("ERROR: hold gains unknown for these motors; pass --hold-kp and --hold-kd.")
        sys.exit(1)
    soft_kp = [args.soft_kp] * len(sel)
    soft_kd = [args.soft_kd] * len(sel) if args.soft_kd is not None else [0.1 * v for v in hold_kd]
    limits = [(model.joints[j]["lower"], model.joints[j]["upper"]) for j in sel_joints]

    # ---- command: this firmware takes unnamed commands for all motors, like its own
    # controller sends; motors we do not drive (head) are held where they were.
    cmd = datatypes.RobotCmd()
    sel_pos = {i: k for k, i in enumerate(sel)}
    rest_q = list(q_now)

    def send(target, kp, kd, tau):
        cmd.stamp = time.time_ns()
        if full:
            n = n_all
            # Names must be filled in: the SDK's publishRobotCmd segfaults on an empty
            # motor_names list (verified on the robot, 2026-10-07).
            cmd.motor_names, cmd.parallel_solve_required, cmd.mode = list(names), [False] * n, [0] * n
            cmd.q = [target[sel_pos[i]] if i in sel_pos else rest_q[i] for i in range(n)]
            cmd.Kp = [kp[sel_pos[i]] if i in sel_pos else TRON2_DEFAULT_KP[i] for i in range(n)]
            cmd.Kd = [kd[sel_pos[i]] if i in sel_pos else TRON2_DEFAULT_KD[i] for i in range(n)]
            cmd.tau = [tau[sel_pos[i]] if i in sel_pos else 0.0 for i in range(n)]
            cmd.dq = [0.0] * n
        else:
            cmd.motor_names = sel_names
            cmd.parallel_solve_required = [not args.no_parallel_solve] * len(sel)
            cmd.mode = [0] * len(sel)
            cmd.q, cmd.dq, cmd.tau, cmd.Kp, cmd.Kd = list(target), [0.0] * len(sel), list(tau), list(kp), list(kd)
        return robot.publishRobotCmd(cmd)

    rate = Pacer(CMD_HZ)
    mode = "PASSIVE"          # PASSIVE -> CHECK -> HOLD <-> SOFT
    ramp = 0.0                # 0 = hold gains, 1 = soft gains
    ff = 0.0                  # gravity feed-forward share, 0 .. 1 (blended in after the check)
    check_ok = False
    check = None              # dict(t, acc, n) while the check runs
    hold_q = [q_now[i] for i in sel]
    soft_target = list(hold_q)
    rec, rec_t0, rec_next = None, 0.0, 0.0
    note = "put the arms in a free pose (off the body and joint limits), then press space to hold them"
    homing = None             # dict(start, goal, t, duration) while moving home
    release_armed = False     # 'q' pressed once while commanding
    events = []

    def save_home(qh):
        with open(args.home, "w") as f:
            json.dump({"motor_names": sel_names, "q": qh, "created": time.strftime("%Y-%m-%d %H:%M:%S")}, f)

    home = None
    try:
        with open(args.home) as f:
            h = json.load(f)
        saved = dict(zip(h.get("motor_names", []), h["q"]))
        if all(n in saved for n in sel_names):
            home = [saved[n] for n in sel_names]
            print("Default home pose loaded from {} (saved {}).".format(args.home, h.get("created", "?")))
        else:
            print("NOTE: {} is for other joints; ignoring it.".format(args.home))
    except (OSError, ValueError, KeyError):
        pass
    if home is None:
        home = list(hold_q)
        save_home(home)
        print("No default home pose yet: saved the start-up pose to {}.".format(args.home))
    print("Low-level mode: nothing is sent until you press space.\n"
          "Keys: space hold/soft, c re-check, r record, p save home, h go home, q quit (twice while holding).")
    next_display = 0.0
    last = time.monotonic()

    def save(frames):
        path = "drag_{}.json".format(time.strftime("%Y%m%d_%H%M%S"))
        with open(path, "w") as f:
            json.dump({"robot_ip": args.ip, "rec_hz": REC_HZ, "motor_names": names, "source": "drag_teach",
                       "created": time.strftime("%Y-%m-%d %H:%M:%S"), "frames": frames}, f)
        return path

    def start_check(q_sel):
        return {"t": 0.0, "acc": [0.0] * len(sel), "n": 0, "q": list(q_sel)}

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    sent = False
    try:
        while True:
            key = read_key()
            now = time.monotonic()
            dt, last = now - last, now
            with fb.lock:
                _, q_all, dq_all, tau_all = fb.state
                ee, grip = fb.ee, fb.gripper
            q = [q_all[i] for i in sel]
            dq = [dq_all[i] for i in sel]

            if mode == "PASSIVE":
                if key == "q":
                    break
                if key == " ":
                    # Hold the arms where they are, position control only (no gravity help yet).
                    mode, hold_q, rest_q = "CHECK", list(q), list(q_all)
                    check, ff, check_ok = start_check(q), 0.0, False
                    note = "holding; model check in {:.1f} s - keep the arms still".format(CHECK_SETTLE_S + CHECK_SECONDS)
                    key = ""
                else:
                    if now >= next_display:
                        next_display = now + 0.1
                        sys.stdout.write("\r\033[KPASSIVE (sending nothing) | {}".format(note))
                        sys.stdout.flush()
                    rate.sleep()
                    continue

            if key and key != "q":
                release_armed = False
            if homing is not None and key in (" ", "q"):
                homing, hold_q = None, list(q)
                note = "homing stopped: holding"
                key = ""
            if key == "q":
                if mode == "SOFT":
                    mode, hold_q, ramp = "HOLD", list(q), 0.0
                    note = "holding. Support the arms, then press q again: they go limp when commands stop"
                elif not release_armed:
                    release_armed = True
                    note = "support the arms, then press q again: they go limp when commands stop"
                else:
                    break
            elif key == " ":
                if mode == "HOLD" and check_ok:
                    mode, soft_target, hold_q = "SOFT", list(q), list(q)
                    note = "going soft: do not touch until 0%"
                elif mode == "SOFT":
                    mode, hold_q, ramp = "HOLD", list(q), 0.0
                    note = "hold"
                elif not check_ok:
                    note = "soft is locked until the model check passes (c re-checks)"
            elif key == "c" and mode in ("HOLD", "CHECK"):
                mode, hold_q, check, ff, check_ok = "CHECK", list(q), start_check(q), 0.0, False
                note = "re-checking - keep the arms still"
            elif key == "p":
                home = list(q)
                save_home(home)
                note = "home pose saved to {}".format(args.home)
            elif key == "h" and mode != "CHECK" and max(abs(a - b) for a, b in zip(q, home)) > math.radians(30.0):
                note = "h refused: home is {:.0f} deg away (limit 30); save a new home with p".format(
                    math.degrees(max(abs(a - b) for a, b in zip(q, home))))
            elif key == "h" and mode != "CHECK":
                mode, ramp = "HOLD", 0.0
                diff = max(abs(a - b) for a, b in zip(q, home))
                homing = {"start": list(q), "goal": list(home), "t": 0.0,
                          "duration": max(MIN_HOME_TIME, diff / MAX_HOME_SPEED)}
                note = "going home ({:.1f} s)".format(homing["duration"])
            elif key == "r":
                if rec is None:
                    rec, rec_t0, rec_next = [], now, now
                    note = "recording"
                else:
                    note = "saved {}".format(save(rec)) if rec else "nothing recorded"
                    rec = None

            # model check: position hold only, then the holding torque is the real gravity load
            if mode == "CHECK":
                check["t"] += dt
                if check["t"] > CHECK_SETTLE_S:
                    for k, i in enumerate(sel):
                        check["acc"][k] += tau_all[i]
                    check["n"] += 1
                if check["t"] >= CHECK_SETTLE_S + CHECK_SECONDS and check["n"]:
                    measured = [a / check["n"] for a in check["acc"]]
                    g_chk = model.gravity(q_dict(q_all), sel_joints)
                    bad = []
                    events.append("model check (holding torque vs model, Nm):")
                    for k, (n, gm, tm) in enumerate(zip(sel_names, g_chk, measured)):
                        good = abs(gm - tm) <= max(CHECK_ABS_NM, CHECK_REL * abs(tm))
                        if not good:
                            bad.append(n)
                        events.append("  {:10s} {:24s} model {:+7.2f}  measured {:+7.2f}  {}".format(
                            n, sel_joints[k], gm, tm, "ok" if good else "NO"))
                    check_ok = not bad or args.force
                    mode, check = "HOLD", None
                    if check_ok:
                        note = "model check PASSED: gravity compensation on; space goes soft"
                    else:
                        note = "model check FAILED ({}): soft locked. Move the arms off any support, then c".format(
                            ", ".join(bad))

            if mode == "HOLD" and check_ok and ff < 1.0:
                ff = min(1.0, ff + dt / FF_RAMP_S)

            # speed trip
            if mode == "SOFT" and max(abs(v) for v in dq) > MAX_DRAG_SPEED:
                mode, hold_q, ramp = "HOLD", list(q), 0.0
                note = "TRIP: joint faster than {:.1f} rad/s -> HOLD".format(MAX_DRAG_SPEED)
            if mode == "SOFT" and ramp < 1.0:
                drift = max(abs(a - b) for a, b in zip(q, hold_q))
                if drift > RAMP_ABORT:
                    mode, hold_q, ramp = "HOLD", list(q), 0.0
                    note = "ABORT: arm drifted {:.1f} deg while softening; the model does not hold it".format(
                        math.degrees(drift))
            if mode == "SOFT":
                ramp = min(1.0, ramp + dt / RAMP_S)
            g = model.gravity(q_dict(q_all), sel_joints)
            tau_ff = [ff * args.gravity_scale * v for v in g]
            if mode == "SOFT":
                a = min(1.0, dt / SOFT_FILTER_S)
                soft_target = [t + (v - t) * a for t, v in zip(soft_target, q)]
                target = list(soft_target)
                for k, (lo, hi) in enumerate(limits):
                    kp_wall = hold_kp[k]
                    if lo is not None and q[k] < lo + LIMIT_MARGIN:
                        tau_ff[k] += kp_wall * (lo + LIMIT_MARGIN - q[k])
                    if hi is not None and q[k] > hi - LIMIT_MARGIN:
                        tau_ff[k] -= kp_wall * (q[k] - (hi - LIMIT_MARGIN))
            else:
                if homing is not None:
                    homing["t"] += dt
                    s_ = min(1.0, homing["t"] / homing["duration"])
                    s_ = s_ * s_ * (3.0 - 2.0 * s_)
                    hold_q = [a + (b - a) * s_ for a, b in zip(homing["start"], homing["goal"])]
                    if homing["t"] >= homing["duration"]:
                        homing = None
                        note = "at home"
                target = hold_q
            kp = [h + (s_k - h) * ramp for h, s_k in zip(hold_kp, soft_kp)]
            kd = [h + (s_k - h) * ramp for h, s_k in zip(hold_kd, soft_kd)]

            if not send(target, kp, kd, tau_ff):
                note = "ERROR: publishRobotCmd failed"
                break
            sent = True

            if rec is not None and now >= rec_next:
                rec_next += 1.0 / REC_HZ
                rec.append({"t": round(now - rec_t0, 4), "q": list(q_all), "dq": list(dq_all), "tau": list(tau_all),
                            "cmd": {"names": sel_names, "q": list(q), "dq": [0.0] * len(sel), "tau": g,
                                    "Kp": list(hold_kp), "Kd": list(hold_kd), "mode": [0] * len(sel)},
                            "ee": ee, "gripper": grip})

            while events:
                sys.stdout.write("\r\033[K" + events.pop(0) + "\n")
            if now >= next_display:
                next_display = now + 0.1
                rec_str = "REC {:.1f}s".format(now - rec_t0) if rec is not None else "rec off"
                label = "HOMING" if homing is not None else mode
                sys.stdout.write("\r\033[K{} stiffness {:3.0f}% gravity {:3.0f}% | max |dq| {:.2f} | {} | {}".format(
                    label, 100.0 * (1.0 - ramp), 100.0 * ff, max(abs(v) for v in dq), rec_str, note))
                sys.stdout.flush()
            rate.sleep()
    except KeyboardInterrupt:
        if sent:
            # Ctrl+C: hold the current pose for a moment before stopping.
            with fb.lock:
                q_all = fb.state[1]
            hold_q = [q_all[i] for i in sel]
            g = model.gravity(q_dict(q_all), sel_joints)
            end = time.monotonic() + HOLD_ON_EXIT_S
            while time.monotonic() < end:
                send(hold_q, hold_kp, hold_kd, [ff * args.gravity_scale * v for v in g])
                rate.sleep()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        if rec:
            print("\nSaved {}".format(save(rec)))
        sub_pose.close()
        print("\nStopped publishing joint commands." if sent else "\nNothing was sent.")


if __name__ == "__main__":
    main()

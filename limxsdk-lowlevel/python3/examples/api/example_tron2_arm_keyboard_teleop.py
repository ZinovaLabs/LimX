"""
@file example_tron2_arm_keyboard_teleop.py

@brief Keyboard end-effector tele-operation of the Tron2 arms, plus 2F-gripper control.

The arms are moved through the robot's own VR tele-operation node: this script plays
a virtual VR head-set on "/sdk_vr_cmd" (teleop_msgs/VRState) and moves the virtual
controllers with the keyboard. Protocol, as observed with tron2_probe on the robot:
    - the node needs a continuous stream; >1 s without a frame resets it
      ("Press X/A to re-enable"), so this script streams at STREAM_HZ the whole time
    - X/A re-enables the node                                  -> key 'e'
    - holding BOTH grips enters take-over and snapshots the VR
      baseline; releasing both grips exits take-over           -> key 't' (toggle)
    - while in take-over, controller motion relative to the
      baseline moves the end effectors                         -> move keys
Turn the real VR head-set off first, otherwise both inputs fight.

Uses:
    - Robot.publish(VRState, "/sdk_vr_cmd")   virtual head-set / controllers
    - Robot.publishGripperCmd()               -> "/limx/2F-gripper/cmd"
    - "/arm_pose" (Float32MultiArray, 14 floats) and Robot.subscribeArmEePose()
      for measured end-effector feedback (this firmware publishes "/arm_pose")
    - Robot.subscribeTeleopState()            tele-operation status

Keys (focus must be on this terminal):
    1 / 2 / b : select left / right / both arms (and grippers)
    e         : enable tele-operation (press X + A)
    t         : take-over on / off (hold / release both grips)
    w / s     : move selected controller(s) +x / -x  (VR frame)
    a / d     : move selected controller(s) +y / -y
    r / f     : move selected controller(s) +z / -z
    = / -     : bigger / smaller step
    h         : return selected controller(s) to the take-over start pose
    space     : stop (hold the current pose)
    o / c     : fully open / close the selected gripper(s)
    ] / [     : open / close the selected gripper(s) by one step
    q, Ctrl+C : release take-over and quit

Record / replay:
    --record FILE   save every published frame (virtual controller positions,
                    take-over, X/A, gripper opening, measured ee) to FILE (JSON)
                    when the script exits
    --replay FILE   play a recording back instead of reading move keys; it sends
                    the same stream with the same timing. Keys during replay:
                        s       start
                        space   pause / resume (holds the current pose)
                        q       abort: release take-over and quit
    --speed S       replay speed factor (default 1.0; use < 1 to slow down)

The VR frame is z-up (head-set height ~1.6 m in the probe); how its x/y map onto the
robot is decided by the tele-operation node. Start with small steps and watch the
"ee" read-out to learn the mapping.

Usage:
    python3 example_tron2_arm_keyboard_teleop.py [robot_ip] [--topic /sdk_vr_cmd | /vr_cmd]
                                                 [--record FILE | --replay FILE [--speed S]]

© [2025] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import argparse
import json
import select
from array import array
import sys
import termios
import time
import tty

import limxsdk.robot.Robot as Robot
import limxsdk.robot.RobotType as RobotType
import limxsdk.datatypes as datatypes
from limxsdk.msg import VRState, Float32MultiArray
from limxsdk.msg._runtime import Time

STREAM_HZ = 50.0               # VR frames per second; the node times out after 1 s
DISPLAY_HZ = 10.0
STEPS = [0.005, 0.01, 0.02, 0.05]   # m per key press
MAX_SPEED = 0.05               # m/s, published controller pose ramps toward the target
MAX_OFFSET = 0.30              # m, per-axis limit from the take-over start pose
BUTTON_PULSE_S = 0.3           # how long 'e' holds X + A

# Virtual controller start poses (VR frame, metres), close to where the real
# controllers sat in the probe. The node re-baselines on take-over, so only
# motion relative to these matters.
EYE_POS = [0.0, 0.0, 1.66]
CTRL_START = [[0.25, 0.25, 1.0], [0.25, -0.25, 1.0]]   # [left, right]

OPENING_STEP = 10.0
OPENING_MAX = [100.0, 95.0]    # the SDK clamps the right finger to 95
GRIPPER_SPEED = 50.0
GRIPPER_FORCE = 50.0

SELECTIONS = {"1": ("left", [0]), "2": ("right", [1]), "b": ("both", [0, 1])}
MOVE_KEYS = {"w": (0, +1), "s": (0, -1), "a": (1, +1), "d": (1, -1), "r": (2, +1), "f": (2, -1)}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pose16(p):
    """Row-major 4x4 transform with identity rotation and translation p."""
    return array("f", [1.0, 0.0, 0.0, p[0],
            0.0, 1.0, 0.0, p[1],
            0.0, 0.0, 1.0, p[2],
            0.0, 0.0, 0.0, 1.0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("robot_ip", nargs="?", default="10.192.1.2")
    parser.add_argument("--topic", default="/sdk_vr_cmd",
                        help="VR input topic to publish: /sdk_vr_cmd (default) or /vr_cmd")
    parser.add_argument("--record", metavar="FILE", help="save the session to FILE on exit")
    parser.add_argument("--replay", metavar="FILE", help="play back a recorded session")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    args = parser.parse_args()
    if args.record and args.replay:
        parser.error("use --record or --replay, not both")
    if args.speed <= 0.0:
        parser.error("--speed must be > 0")

    frames = None
    if args.replay:
        with open(args.replay) as f:
            frames = json.load(f)["frames"]
        if not frames:
            print("ERROR: {} has no frames.".format(args.replay))
            sys.exit(1)
        if frames[0]["takeover"]:
            print("ERROR: the recording starts in take-over; refusing to replay it.")
            sys.exit(1)
        print("Loaded {} frames ({:.1f} s) from {}".format(len(frames), frames[-1]["t"], args.replay))
    record = [] if args.record else None
    robot_ip = args.robot_ip
    own_vr = args.topic == "/vr_cmd"   # our own frames then show up on the /vr_cmd subscription

    robot = Robot(RobotType.Tron2)
    if not robot.init(robot_ip):
        print("ERROR: robot.init failed.")
        sys.exit(1)

    fb = {"ee": None, "gripper_q": None, "teleop": "n/a", "takeover": None, "headset_t": 0.0, "events": []}
    t0 = time.monotonic()

    def log(text):
        ee = fb["ee"]
        ee_str = "n/a" if ee is None else "L({:+.3f},{:+.3f},{:+.3f}) R({:+.3f},{:+.3f},{:+.3f})".format(*ee[0], *ee[1])
        fb["events"].append("{:7.2f}s  {:<60s} ee {}".format(time.monotonic() - t0, text, ee_str))

    def on_arm_pose(m: Float32MultiArray):
        d = list(m.data)
        if len(d) >= 14:
            fb["ee"] = (d[0:3], d[7:10])

    def on_ee_pose(pose: datatypes.ArmEePose):
        if pose.valid:
            fb["ee"] = (list(pose.left_position), list(pose.right_position))

    def on_gripper_state(state: datatypes.GripperState):
        fb["gripper_q"] = list(state.q)

    def on_teleop(s: datatypes.TeleopState):
        fb["teleop"] = s.message.replace("[TeleOperation] ", "")[:60]
        log("ROBOT: " + s.message.replace("[TeleOperation] ", "")[:52])
        if s.takeover_known:
            fb["takeover"] = s.takeover_active

    def on_headset(_s: datatypes.VrState):
        fb["headset_t"] = time.monotonic()

    subs = [robot.subscribe(Float32MultiArray, "/arm_pose", on_arm_pose)]
    robot.subscribeArmEePose(on_ee_pose)
    robot.subscribeGripperState(on_gripper_state)
    robot.subscribeTeleopState(on_teleop)
    robot.subscribeVrState(on_headset)
    # Check for a live head-set before publishing anything ourselves.
    time.sleep(1.5)
    if time.monotonic() - fb["headset_t"] < 1.0:
        print("ERROR: the real VR head-set is publishing /vr_cmd. Turn it off (or quit its"
              " tele-operation app) and run again.")
        sys.exit(1)
    vr_pub = robot.publish(VRState, args.topic)
    print("Publishing virtual VR state on {}".format(args.topic))

    # Gripper: start from the measured opening so the first key press does not jump.
    deadline = time.monotonic() + 2.0
    while fb["gripper_q"] is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if fb["gripper_q"] is not None and len(fb["gripper_q"]) >= 2:
        opening = [clamp(fb["gripper_q"][i], 0.0, OPENING_MAX[i]) for i in range(2)]
    else:
        print("WARNING: no gripper state yet; assuming both grippers are open.")
        opening = list(OPENING_MAX)

    gcmd = datatypes.GripperCmd()
    gcmd.speed = [GRIPPER_SPEED, GRIPPER_SPEED]
    gcmd.force = [GRIPPER_FORCE, GRIPPER_FORCE]

    def send_gripper():
        gcmd.opening = list(opening)
        gcmd.stamp = time.time_ns()
        robot.publishGripperCmd(gcmd)

    # Virtual VR state.
    origin = [list(p) for p in CTRL_START]      # pose at take-over start
    target = [list(p) for p in CTRL_START]      # where the keys want the controller
    current = [list(p) for p in CTRL_START]     # what is being published (rate limited)
    takeover = False
    buttons_until = 0.0
    step_i = 1

    msg = VRState()
    msg.eyePose[:] = pose16(EYE_POS)
    seq = [0]

    def send_vr(now):
        pressed = now < buttons_until
        msg.header.seq = seq[0]
        seq[0] += 1
        ns = time.time_ns()
        msg.header.stamp = Time(ns // 1000000000, ns % 1000000000)
        msg.l[:] = pose16(current[0])
        msg.r[:] = pose16(current[1])
        grip = 1.0 if takeover else 0.0
        msg.leftGrip = msg.rightGrip = grip
        msg.LG = msg.RG = int(takeover)
        msg.X = msg.A = int(pressed)
        vr_pub.publish(msg)

    print(__doc__.split("Usage:")[0])

    sel_name, sel_idx = SELECTIONS["b"]
    replay_started = False
    replay_paused = False
    replay_clock = 0.0
    replay_i = 0
    rec_t0 = None
    stream_period = 1.0 / STREAM_HZ
    next_display = 0.0
    last = time.monotonic()

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        while True:
            if select.select([sys.stdin], [], [], stream_period)[0]:
                key = sys.stdin.read(1)
                if frames is not None:
                    if key == "q":
                        return
                    elif key == "s" and not replay_started:
                        replay_started = True
                        log("replay started")
                    elif key == " " and replay_started:
                        replay_paused = not replay_paused
                        log("replay paused" if replay_paused else "replay resumed")
                    key = ""
                if key and (key in "et" or key in MOVE_KEYS):
                    log("key '{}'  takeover_cmd={}".format(key, not takeover if key == "t" else takeover))
                if key == "q":
                    return
                elif not key:
                    pass
                elif key in SELECTIONS:
                    sel_name, sel_idx = SELECTIONS[key]
                elif key == "e":
                    buttons_until = time.monotonic() + BUTTON_PULSE_S
                elif key == "t":
                    takeover = not takeover
                    if takeover:
                        # The node re-baselines on take-over: freeze the pose first.
                        target = [list(p) for p in current]
                        origin = [list(p) for p in current]
                elif key in MOVE_KEYS and takeover:
                    axis, sign = MOVE_KEYS[key]
                    for i in sel_idx:
                        target[i][axis] = clamp(target[i][axis] + sign * STEPS[step_i],
                                                origin[i][axis] - MAX_OFFSET,
                                                origin[i][axis] + MAX_OFFSET)
                elif key in MOVE_KEYS:
                    fb["teleop"] = "move ignored: press 'e' then 't' first"
                elif key == "=":
                    step_i = min(step_i + 1, len(STEPS) - 1)
                elif key == "-":
                    step_i = max(step_i - 1, 0)
                elif key == "h":
                    for i in sel_idx:
                        target[i] = list(origin[i])
                elif key == " ":
                    target = [list(p) for p in current]
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

            now = time.monotonic()
            dt, last = now - last, now
            if frames is not None and replay_started and not replay_paused:
                replay_clock += dt * args.speed
                while replay_i + 1 < len(frames) and frames[replay_i + 1]["t"] <= replay_clock:
                    replay_i += 1
                fr = frames[replay_i]
                if fr["takeover"] and not takeover:
                    origin = [list(fr["l"]), list(fr["r"])]
                takeover = fr["takeover"]
                buttons_until = now + 1.0 if fr["buttons"] else 0.0
                current = [list(fr["l"]), list(fr["r"])]
                target = [list(p) for p in current]
                if fr["opening"] != opening:
                    opening = list(fr["opening"])
                    send_gripper()
                if replay_i == len(frames) - 1:
                    log("replay finished")
                    return
            elif frames is not None:
                buttons_until = 0.0     # paused / not started: hold pose, no button presses
            else:
                max_move = MAX_SPEED * dt
                for i in range(2):
                    for k in range(3):
                        current[i][k] += clamp(target[i][k] - current[i][k], -max_move, max_move)
            send_vr(now)
            if record is not None:
                if rec_t0 is None:
                    rec_t0 = now
                record.append({"t": round(now - rec_t0, 4), "takeover": takeover,
                               "buttons": now < buttons_until,
                               "l": [round(v, 5) for v in current[0]],
                               "r": [round(v, 5) for v in current[1]],
                               "opening": list(opening),
                               "ee": fb["ee"]})

            while fb["events"]:
                sys.stdout.write("\r\033[K" + fb["events"].pop(0) + "\n")
            if now >= next_display:
                next_display = now + 1.0 / DISPLAY_HZ
                ee = fb["ee"]
                ee_str = "n/a" if ee is None else "L({:+.3f},{:+.3f},{:+.3f}) R({:+.3f},{:+.3f},{:+.3f})".format(
                    *ee[0], *ee[1])
                off = ["({:+.2f},{:+.2f},{:+.2f})".format(*[current[i][k] - origin[i][k] for k in range(3)])
                       for i in range(2)]
                node = {None: "?", True: "ON", False: "off"}[fb["takeover"]]
                headset = " HEADSET!" if not own_vr and now - fb["headset_t"] < 1.0 else ""
                if frames is not None:
                    state = ("press 's' to start" if not replay_started else
                             "PAUSED" if replay_paused else "playing")
                    sel_name = "{:.0f}%".format(100.0 * replay_i / max(1, len(frames) - 1))
                    fb["teleop"] = "replay {} {:.1f}/{:.1f}s | {}".format(
                        state, frames[replay_i]["t"], frames[-1]["t"], fb["teleop"].split(" | ")[-1])
                sys.stdout.write("\r\033[K[{:5s}] step {:.3f} | takeover cmd {} node {} | off L{} R{} | "
                                 "grip L={:3.0f} R={:3.0f} | ee {} | {}{}".format(
                                     sel_name, STEPS[step_i], "ON " if takeover else "off", node,
                                     off[0], off[1], opening[0], opening[1], ee_str, fb["teleop"], headset))
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        # Release take-over and keep streaming briefly so the node sees the release.
        takeover = False
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            send_vr(time.monotonic())
            time.sleep(stream_period)
        vr_pub.close()
        if record:
            with open(args.record, "w") as f:
                json.dump({"robot_ip": robot_ip, "topic": args.topic, "stream_hz": STREAM_HZ,
                           "created": time.strftime("%Y-%m-%d %H:%M:%S"), "frames": record}, f)
            print("\nSaved {} frames ({:.1f} s) to {}".format(len(record), record[-1]["t"], args.record))
        for s in subs:
            s.close()
        while fb["events"]:
            sys.stdout.write("\r\033[K" + fb["events"].pop(0) + "\n")
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        print("\nExit.")


if __name__ == "__main__":
    main()

"""
Move the Tron2 arms to a home pose in the background, for the collector's automatic return
home after each episode. The home is set with set_goal() (the collector uses the pose the
arms are in when the first episode starts) or read from a file with load_home()
(e.g. limxsdk-lowlevel/home_pose.json).

Same motion and safety as limxsdk-lowlevel/python3/examples/api/example_tron2_arm_go_home.py:
all 16 motors, the robot controller's gains, at most `speed` rad/s per joint (at least
MIN_TIME s) with a smooth start and stop, and a stop (holding the measured pose) when a
joint lags its target by more than MAX_TRACKING_ERROR.

It never commands the motors while the robot's own controller does (VR teleop sends
/motor/cmd all the time): two controllers would fight. So a request to go home waits until
/motor/cmd goes quiet (the robot is switched to SDK control mode), moves home, holds there,
and stops sending as soon as the robot's controller commands again (switched back to teleop).

Phases: idle -> waiting (for SDK mode) -> moving -> holding -> idle, or refused / stopped.
"""

import collections
import json
import math
import os
import threading
import time

import limxsdk.datatypes as datatypes

HOME_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                         "limxsdk-lowlevel", "home_pose.json")

# The robot's own controller gains, read from /motor/cmd on this robot (2026-10-07).
KP = [420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0,
      420.0, 420.0, 400.0, 400.0, 300.0, 200.0, 150.0,
      350.0, 550.0]
KD = [12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0,
      12.0, 12.0, 15.0, 15.0, 10.0, 10.0, 10.0,
      9.0, 9.0]

CMD_HZ = 500.0
MIN_TIME = 3.0                  # s
MAX_TRACKING_ERROR = 0.10       # rad
AT_HOME_TOL = math.radians(2.0)
QUIET_S = 0.5                   # /motor/cmd silent this long: the robot's controller let go


def load_home(path=HOME_PATH):
    """The 16 joint positions of a home pose file (as written by drag-teach --save-home)."""
    with open(path) as f:
        q = [float(v) for v in json.load(f)["q"]]
    if len(q) != len(KP):
        raise ValueError("{} has {} joints, expected {}".format(path, len(q), len(KP)))
    return q


class HomeMover(object):
    """Background go-home for a RobotSource. set_goal() sets the home pose, start() requests a
    move, status() describes it."""

    def __init__(self, source, goal=None, speed=0.2, max_move_deg=90.0):
        self.goal = None
        self.source, self.speed, self.max_move = source, speed, math.radians(max_move_deg)
        self.lock = threading.Lock()
        self.phase, self.note = "idle", ""
        self.own = collections.deque(maxlen=4000)       # stamps of the commands we sent
        self.own_set = set()
        self.last_foreign = time.monotonic()            # assume the controller is active at first
        self.seen = 0.0
        self.cmd = datatypes.RobotCmd()
        n = len(KP)
        self.cmd.mode = [0] * n
        self.cmd.motor_names = ["motor_{}".format(i) for i in range(n)]   # SDK crashes on empty names
        self.cmd.parallel_solve_required = [False] * n
        self.cmd.dq = [0.0] * n
        self.cmd.tau = [0.0] * n
        self.cmd.Kp = list(KP)
        self.cmd.Kd = list(KD)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

        if goal is not None:
            self.set_goal(goal)

    # -- control (any thread)

    def set_goal(self, q):
        if len(q) != len(KP):
            raise ValueError("home pose has {} joints, expected {}".format(len(q), len(KP)))
        with self.lock:
            self.goal = [float(v) for v in q]

    def start(self):
        with self.lock:
            if self.goal is None:
                self.phase, self.note = "refused", "no home pose set yet"
            elif self.phase in ("idle", "refused", "stopped"):
                self.phase, self.note = "waiting", ""

    def cancel(self):
        """Stop sending (the arms are left to the robot)."""
        with self.lock:
            self.phase, self.note = "idle", "go-home cancelled"

    @property
    def phase_name(self):
        with self.lock:
            return self.phase

    @property
    def busy(self):
        with self.lock:
            return self.phase in ("waiting", "moving", "holding")

    def status(self):
        with self.lock:
            phase, note = self.phase, self.note
        if phase == "holding" and note:
            return "HOME: " + note
        return {
            "idle": note,
            "waiting": "HOME: switch the robot to SDK control mode to go home (r skips it)",
            "moving": "HOME: moving home...",
            "holding": "HOME: at home, holding; switch back to teleop for the next episode",
            "refused": "HOME refused: " + note,
            "stopped": "HOME stopped: " + note,
        }[phase]

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)

    # -- loop

    def _foreign_active(self, now):
        """True while the robot's own controller sends /motor/cmd (commands that are not ours)."""
        for smp in self.source.streams["cmd"].since(self.seen):
            self.seen = smp[0]
            if smp[2][2] not in self.own_set:
                self.last_foreign = smp[3]
        return now - self.last_foreign < QUIET_S

    def _send(self, target):
        stamp = time.time_ns()
        if len(self.own) == self.own.maxlen:
            self.own_set.discard(self.own[0])
        self.own.append(stamp)
        self.own_set.add(stamp)
        self.cmd.stamp = stamp
        self.cmd.q = list(target)
        return self.source.robot.publishRobotCmd(self.cmd)

    def _set(self, phase, note=""):
        with self.lock:
            self.phase, self.note = phase, note

    def _loop(self):
        period = 1.0 / CMD_HZ
        start = target = None
        t = duration = 0.0
        last = time.monotonic()
        while self.running:
            time.sleep(period)
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                phase = self.phase
            foreign = self._foreign_active(now)
            state = self.source.streams["state"].latest()
            if phase not in ("waiting", "moving", "holding") or state is None:
                start = target = None
                continue
            q = state[2][0]

            if phase == "waiting":
                if foreign:
                    continue
                with self.lock:
                    goal = list(self.goal)
                diff = [abs(g - s) for g, s in zip(goal, q)]
                if max(diff) > self.max_move:
                    worst = max(range(len(diff)), key=lambda i: diff[i])
                    self._set("refused", "motor_{} would move {:.0f} deg (limit {:.0f}, --home-max-move)".format(
                        worst, math.degrees(diff[worst]), math.degrees(self.max_move)))
                    continue
                start, t = list(q), 0.0
                duration = max(MIN_TIME, max(diff) / self.speed)
                self._set("moving")
                continue

            if foreign:                                # teleop is back: let go at once
                self._set("idle", "robot controller took over; stopped sending")
                continue

            if phase == "moving":
                t = min(duration, t + dt)
                s = t / duration
                s = s * s * (3.0 - 2.0 * s)
                target = [a + (b - a) * s for a, b in zip(start, goal)]
                if t >= duration:
                    worst = max(range(len(q)), key=lambda i: abs(q[i] - goal[i]))
                    off = abs(q[worst] - goal[worst])
                    self._set("holding", "" if off <= AT_HOME_TOL else
                              "holding near home, motor_{} {:.1f} deg off; switch back to teleop".format(
                                  worst, math.degrees(off)))
            if target is None:
                target = list(q)
            err = max(abs(a - b) for a, b in zip(q, target))
            if err > MAX_TRACKING_ERROR:
                worst = max(range(len(q)), key=lambda i: abs(q[i] - target[i]))
                target = list(q)
                self._set("holding", "motor_{} lagged {:.1f} deg (blocked?): holding the measured pose".format(
                    worst, math.degrees(err)))
            if not self._send(target):
                self._set("stopped", "publishRobotCmd failed")

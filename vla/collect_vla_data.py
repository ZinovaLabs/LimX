"""
Record Tron2 demonstrations for VLA (pi0.5 / tron2_openpi) fine-tuning.

Operate the arms any way you like - VR tele-operation, drag-teach, keyboard - and this
script records episodes at a fixed rate (default 30 Hz, the tron2_openpi policy rate).
It only reads from the robot; it never sends commands.

Per frame it saves:
    state      16 values [left arm 7, left gripper, right arm 7, right gripper]
               (radians; gripper opening 0..1)              -> observation.state
    cmd_q      the robot controller's 16 joint targets from /motor/cmd, when the
               controller is driving the arms (VR teleop)  -> action (if --action cmd)
    q/dq/tau   all 16 motors, raw
    images     one JPEG per camera: cam_high (head), cam_left_wrist, cam_right_wrist
               (end-effector cameras); all three by default
    timing     receive age of the state and of every camera image

Raw episode layout (convert with vla/convert_to_lerobot.py):
    <out>/episode_000012/
        meta.json          task, fps, cameras, status (success / failure / incomplete), stats
        frames.jsonl       one JSON object per frame, written as it is recorded
        cam_high/000000.jpg ...

Keys:
    r        start an episode; r again: stop and save it as a success
    f        stop and save as a failure (kept, skipped by the converter by default)
    x        stop and discard the episode (deletes its folder)
    t        type a new task instruction for the next episodes
    d d      move the last saved episode to the trash (press d twice)
    u        undo the last delete
    l        list the episodes in this folder
    q        quit (an episode in progress is saved as "incomplete")

Deleted episodes go to <out>/.trash/ and can also be managed afterwards with
vla/manage_episodes.py (list, delete by number, restore, mark failure, edit task).

Usage:
    python3 vla/collect_vla_data.py --check          # are all three cameras streaming?
    python3 vla/collect_vla_data.py --task "pick up the cup and place it on the plate"
    python3 vla/collect_vla_data.py --task "..." --topic cam_left_wrist=/camera/left/...  # other topic
    python3 vla/collect_vla_data.py --task "..." --cameras bridge --bridge-host wss://<host>
    python3 vla/collect_vla_data.py --task "..." --cameras realsense \\
            --serial 123456789012=cam_high --serial 234567890123=cam_left_wrist
"""

import argparse
import json
import os
import queue
import re
import select
import shutil
import sys
import termios
import threading
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tron2_vla_io import (CAMERA_NAMES, LAYOUT_NAMES, SDK_CAMERA_TOPICS,  # noqa: E402
                          BridgeCameraSource, RealSenseCameraSource, RobotSource,
                          SdkCameraSource, build_vector)
from manage_episodes import (describe, list_episodes, list_trash,  # noqa: E402
                             restore_episode, summary_line, trash_episode)

FORMAT_VERSION = 1
MAX_STATE_AGE_S = 0.05        # joint state older than this marks the frame stale
MAX_IMAGE_AGE_S = 0.15        # camera image older than this marks the frame stale
CMD_FRESH_S = 0.05            # controller command counts as present if newer than this


def jpeg_size(data):
    """(width, height) from a JPEG's frame header, or None."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        if marker in (0xC0, 0xC1, 0xC2):
            return int.from_bytes(data[i + 7:i + 9], "big"), int.from_bytes(data[i + 5:i + 7], "big")
        i += 2 + int.from_bytes(data[i + 2:i + 4], "big")
    return None


def check_cameras(camera, cams, seconds):
    """Watch every camera for `seconds`; print topic, rate and resolution. True if all stream."""
    start = {c: (camera.latest(c) or (None, 0, 0))[2] for c in cams}
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(0.1)
    elapsed = time.monotonic() - t0
    ok = True
    published = None
    for c in cams:
        f = camera.latest(c)
        topic = camera.active_topic(c) if hasattr(camera, "active_topic") else None
        if f is None or f[2] == start[c]:
            ok = False
            tried = getattr(camera, "topics", {}).get(c)
            print("  {:16s} NO IMAGES{}".format(c, "  (tried {})".format(", ".join(tried)) if tried else ""))
            if tried and hasattr(camera, "published_topics"):
                if published is None:
                    published = camera.published_topics()
                if not published.intersection(tried):
                    print("  {:16s} no publisher on the robot for these topics: "
                          "the camera driver is not running".format(""))
            continue
        size = jpeg_size(f[0])
        print("  {:16s} {:5.1f} Hz  {}  {}".format(
            c, (f[2] - start[c]) / elapsed, "{}x{}".format(*size) if size else "?", topic or ""))
    return ok


def slug(text):
    return re.sub(r"[^a-z0-9]+", "_", text.lower()).strip("_")[:60] or "task"


def next_episode_index(out_dir):
    if not os.path.isdir(out_dir):
        return 0
    ids = [int(m.group(1)) for d in os.listdir(out_dir) for m in [re.match(r"episode_(\d+)$", d)] if m]
    return max(ids) + 1 if ids else 0


def read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return ""


class Preview(object):
    """Small live window with every camera side by side, drawn by its own thread so the
    recording loop never waits on it. Keys pressed in the window act like terminal keys.
    Each camera shows its image age and rate (red when stale); the bar is red while recording."""

    TITLE = "Tron2 cameras"
    HEIGHT = 200                  # pixel height of each camera tile

    def __init__(self, camera, cams, max_age):
        # opencv-python's Qt warns it ships no fonts; the overlay text does not use them
        os.environ.setdefault("QT_LOGGING_RULES", "default.warning=false")
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.camera, self.cams, self.max_age = camera, list(cams), max_age
        self.status, self.recording = "", False
        self.keys = queue.Queue()
        self.tiles = {}           # name -> (seq, resized image)
        self.rate = {c: (time.monotonic(), 0, 0.0) for c in self.cams}   # (t, seq, Hz)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def key(self):
        try:
            return self.keys.get_nowait()
        except queue.Empty:
            return None

    def _tile(self, name, frame, now):
        cv2, np = self.cv2, self.np
        h = self.HEIGHT
        cached = self.tiles.get(name)
        if frame is not None and (cached is None or cached[0] != frame[2]):
            img = cv2.imdecode(np.frombuffer(frame[0], np.uint8), cv2.IMREAD_COLOR)
            if img is not None:
                img = cv2.resize(img, (img.shape[1] * h // img.shape[0], h), interpolation=cv2.INTER_AREA)
                cached = self.tiles[name] = (frame[2], img)
        img = cached[1].copy() if cached else np.zeros((h, h * 4 // 3, 3), np.uint8)

        t, seq, hz = self.rate[name]
        if frame is not None and now - t >= 1.0:
            hz = (frame[2] - seq) / (now - t) if seq else 0.0
            self.rate[name] = (now, frame[2], hz)
        age = None if frame is None else now - frame[1]
        label = "{}  {}  {:.0f} Hz".format(name.replace("cam_", ""),
                                           "--" if age is None else "{:.0f} ms".format(age * 1000), hz)
        color = (0, 0, 255) if age is None or age > self.max_age else (255, 255, 255)
        cv2.putText(img, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return img

    def _loop(self):
        cv2, np = self.cv2, self.np
        cv2.namedWindow(self.TITLE, cv2.WINDOW_AUTOSIZE)
        while self.running:
            now = time.monotonic()
            img = np.hstack([self._tile(c, self.camera.latest(c), now) for c in self.cams])
            bar = np.full((26, img.shape[1], 3), (0, 0, 200) if self.recording else (60, 60, 60), np.uint8)
            cv2.putText(bar, self.status, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(self.TITLE, np.vstack([bar, img]))
            k = cv2.waitKey(60) & 0xFF
            if k != 255:
                self.keys.put(chr(k))
            if cv2.getWindowProperty(self.TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break             # window closed: recording goes on without it
        cv2.destroyAllWindows()

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)


class Episode(object):
    """One episode being written to disk, frame by frame."""

    def __init__(self, out_dir, index, task, fps, cameras, sources_desc):
        self.dir = os.path.join(out_dir, "episode_{:06d}".format(index))
        os.makedirs(self.dir)
        for cam in cameras:
            os.makedirs(os.path.join(self.dir, cam))
        self.index, self.task, self.fps, self.cameras = index, task, fps, list(cameras)
        self.frames = 0
        self.stale = 0
        self.cmd_frames = 0
        self.t0 = time.monotonic()
        self.jsonl = open(os.path.join(self.dir, "frames.jsonl"), "w")
        self.meta = {
            "format_version": FORMAT_VERSION, "task": task, "fps": fps, "cameras": self.cameras,
            "state_names": LAYOUT_NAMES, "sources": sources_desc,
            "started": time.strftime("%Y-%m-%d %H:%M:%S"), "status": "recording",
        }
        self._write_meta()

    def _write_meta(self):
        tmp = os.path.join(self.dir, "meta.json.tmp")
        with open(tmp, "w") as f:
            json.dump(self.meta, f, indent=2)
        os.replace(tmp, os.path.join(self.dir, "meta.json"))

    def add(self, row, images):
        i = self.frames
        for cam, data in images.items():
            with open(os.path.join(self.dir, cam, "{:06d}.jpg".format(i)), "wb") as f:
                f.write(data)
        row["i"] = i
        self.jsonl.write(json.dumps(row) + "\n")
        self.frames += 1
        self.stale += 0 if row["fresh"] else 1
        self.cmd_frames += 1 if row["cmd_q"] is not None else 0

    def finish(self, status):
        self.jsonl.close()
        self.meta.update({
            "status": status, "frames": self.frames, "duration_s": round(time.monotonic() - self.t0, 3),
            "stale_frames": self.stale, "cmd_frames": self.cmd_frames,
            "finished": time.strftime("%Y-%m-%d %H:%M:%S"),
        })
        self._write_meta()

    def discard(self):
        self.jsonl.close()
        shutil.rmtree(self.dir)


def main():
    parser = argparse.ArgumentParser(description="Record Tron2 demonstrations for VLA fine-tuning.")
    parser.add_argument("--task", help="natural-language instruction for these episodes")
    parser.add_argument("--check", action="store_true",
                        help="report which cameras stream (topic, rate, resolution) and exit")
    parser.add_argument("--out", default=None, help="output folder (default: vla_data/<task>)")
    parser.add_argument("--ip", default="10.192.1.2")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--cameras", choices=["sdk", "bridge", "realsense"], default="sdk")
    parser.add_argument("--cam", action="append", choices=CAMERA_NAMES,
                        help="sdk source: record only these camera(s) (default: all three)")
    parser.add_argument("--topic", action="append", default=[], metavar="NAME=TOPIC",
                        help="sdk source: topic for a camera, e.g. cam_left_wrist=/camera/left/color/image_raw/compressed")
    parser.add_argument("--bridge-host", help="bridge source: e.g. wss://<host>")
    parser.add_argument("--bridge-path", default="/bridge/ws")
    parser.add_argument("--serial", action="append", default=[], metavar="SERIAL=NAME",
                        help="realsense source: serial number to camera name, e.g. 1234=cam_high")
    parser.add_argument("--no-preview", action="store_true",
                        help="do not open the live camera window while collecting")
    parser.add_argument("--gripper-fill", type=float, default=0.0,
                        help="gripper value (0..1) recorded when no gripper state is published")
    args = parser.parse_args()
    if not args.check and not args.task:
        parser.error("--task is required (or use --check)")

    out_dir = args.out or os.path.join("vla_data", slug(args.task or "check"))

    robot = RobotSource(args.ip)
    if args.cameras == "sdk":
        cams = args.cam or list(CAMERA_NAMES)
        topics = {c: SDK_CAMERA_TOPICS[c] for c in cams}
        for item in args.topic:
            name, _, topic = item.partition("=")
            if name not in topics or not topic:
                parser.error("--topic expects NAME=TOPIC with NAME in {}".format(", ".join(cams)))
            topics[name] = [topic]
        camera = SdkCameraSource(robot.robot, topics)
        sources_desc = {"cameras": "sdk", "topics": topics}
    elif args.cameras == "bridge":
        if not args.bridge_host:
            parser.error("--cameras bridge needs --bridge-host")
        camera = BridgeCameraSource(args.bridge_host, args.bridge_path)
        cams = list(CAMERA_NAMES)
        sources_desc = {"cameras": "bridge", "host": args.bridge_host}
    else:
        mapping = dict(s.split("=", 1) for s in args.serial)
        if not mapping:
            parser.error("--cameras realsense needs --serial SERIAL=NAME")
        camera = RealSenseCameraSource(mapping, fps=int(args.fps))
        cams = list(mapping.values())
        sources_desc = {"cameras": "realsense", "serials": mapping}
    sources_desc["state"] = "sdk RobotState (motors 0-13), GripperState"

    print("Checking cameras ({:.0f} s) ...".format(3.0))
    cams_ok = check_cameras(camera, cams, 3.0)
    if args.check:
        camera.close()
        sys.exit(0 if cams_ok else 1)
    if not cams_ok:
        print("WARNING: not all cameras stream; episodes cannot start until they do.\n"
              "         Check the camera is plugged in / enabled, or give its topic with --topic NAME=TOPIC.")

    os.makedirs(out_dir, exist_ok=True)
    task = args.task
    index = next_episode_index(out_dir)
    episode = None
    period = 1.0 / args.fps
    note = "press r to start episode {}".format(index)
    tally = summary_line(list_episodes(out_dir)).split(",")[0]
    delete_armed = 0.0         # time of the first d press; a second d within 3 s deletes
    rates = {"t": time.monotonic(), "n": 0, "fps": 0.0}
    print("Recording to {} at {:.0f} Hz, cameras: {} ({}).".format(out_dir, args.fps, ", ".join(cams), args.cameras))
    print("Keys: r start/stop(save)  f stop(failure)  x discard  d d delete last  u undo  "
          "l list  t new task  q quit")

    preview = None
    if not args.no_preview:
        if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            print("No camera preview: no display.")
        else:
            try:
                preview = Preview(camera, cams, MAX_IMAGE_AGE_S)
            except ImportError:
                print("No camera preview: pip install opencv-python (or use --no-preview).")

    fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    next_t = time.monotonic()
    try:
        while True:
            key = read_key() or (preview.key() if preview else None)
            now = time.monotonic()
            state, cmd, grip = robot.snapshot()
            frames = {c: camera.latest(c) for c in cams}

            # health of the inputs
            state_ok = state is not None and now - state[0] <= MAX_STATE_AGE_S
            img_age = {c: (now - f[1]) if f else None for c, f in frames.items()}
            imgs_ok = all(a is not None and a <= MAX_IMAGE_AGE_S for a in img_age.values())
            cmd_ok = (cmd is not None and now - cmd[0] <= CMD_FRESH_S
                      and any(cmd[2][i] > 0 for i in range(14)))

            if key == "q":
                if episode:
                    episode.finish("incomplete")
                    note = "saved {} as incomplete".format(os.path.basename(episode.dir))
                break
            elif key == "r" and episode is None:
                if not (state_ok and imgs_ok):
                    note = "cannot start: {} not fresh".format("joint state" if not state_ok else
                                                               "camera " + ", ".join(c for c, a in img_age.items()
                                                                                     if a is None or a > MAX_IMAGE_AGE_S))
                else:
                    if hasattr(camera, "active_topic"):
                        sources_desc["active_topics"] = {c: camera.active_topic(c) for c in cams}
                    episode = Episode(out_dir, index, task, args.fps, cams, sources_desc)
                    note = "RECORDING episode {}".format(index)
                    next_t = time.monotonic()
            elif key in ("r", "f") and episode is not None:
                status = "success" if key == "r" else "failure"
                episode.finish(status)
                note = "saved {} ({}, {} frames, {} stale)".format(
                    os.path.basename(episode.dir), status, episode.frames, episode.stale)
                episode, index = None, index + 1
                tally = summary_line(list_episodes(out_dir)).split(",")[0]
            elif key == "d" and episode is None:
                saved = list_episodes(out_dir)
                if not saved:
                    note = "no episode to delete"
                elif now - delete_armed > 3.0:
                    delete_armed = now
                    m = saved[-1][2]
                    note = "press d again to delete episode {} ({}, {} frames)".format(
                        saved[-1][0], m.get("status", "?"), m.get("frames", "?"))
                else:
                    delete_armed = 0.0
                    trash_episode(saved[-1][1])
                    index = next_episode_index(out_dir)
                    note = "episode {} moved to the trash (u to undo)".format(saved[-1][0])
                    tally = summary_line(list_episodes(out_dir)).split(",")[0]
            elif key == "u" and episode is None:
                trash = list_trash(out_dir)
                if not trash:
                    note = "nothing to undo"
                else:
                    dest = restore_episode(out_dir, trash[-1][1], trash[-1][2])
                    index = next_episode_index(out_dir)
                    note = "restored {}".format(os.path.basename(dest))
                    tally = summary_line(list_episodes(out_dir)).split(",")[0]
            elif key == "l" and episode is None:
                saved = list_episodes(out_dir)
                sys.stdout.write("\r\033[K")
                for n, _, m in saved[-15:]:
                    sys.stdout.write(describe(n, m) + "\n")
                sys.stdout.write(summary_line(saved) + "\n")
                note = "listed"
            elif key == "x" and episode is not None:
                episode.discard()
                note = "discarded episode {}".format(index)
                episode = None
            elif key == "t" and episode is None:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
                sys.stdout.write("\nNew task instruction: ")
                sys.stdout.flush()
                text = sys.stdin.readline().strip()
                tty.setcbreak(fd)
                if text:
                    task = text
                note = "task: {}".format(task)

            # record one frame per tick
            if episode is not None and now >= next_t:
                next_t += period
                if now - next_t > period:          # fell behind: skip ahead, never burst
                    next_t = now + period
                if state is not None and all(frames.values()):
                    if grip is not None and len(grip[1]) >= 2:
                        gopen = [min(1.0, max(0.0, v / 100.0)) for v in grip[1][:2]]
                    else:
                        gopen = [args.gripper_fill, args.gripper_fill]
                    row = {
                        "t": round(now - episode.t0, 4),
                        "state": build_vector(state[1], gopen),
                        "cmd_q": [round(v, 6) for v in cmd[1]] if cmd_ok else None,
                        "q": state[1], "dq": state[2], "tau": state[3],
                        "gripper_raw": grip[1] if grip is not None else None,
                        "state_age_ms": round((now - state[0]) * 1000.0, 1),
                        "img_age_ms": {c: round(a * 1000.0, 1) for c, a in img_age.items()},
                        "img_seq": {c: f[2] for c, f in frames.items()},
                        "fresh": bool(state_ok and imgs_ok),
                    }
                    episode.add(row, {c: f[0] for c, f in frames.items()})
                    rates["n"] += 1

            # status line, 5 Hz
            if now - rates["t"] >= 0.2:
                rates["fps"] = rates["n"] / (now - rates["t"])
                rates["t"], rates["n"] = now, 0
                cam_str = " ".join("{}:{}".format(c.replace("cam_", ""), "--" if a is None else "{:.0f}ms".format(a * 1000))
                                   for c, a in img_age.items())
                rec = ("REC ep{} {:5.1f}s {:4d}f {:4.1f}Hz stale {}".format(
                    episode.index, now - episode.t0, episode.frames, rates["fps"], episode.stale)
                       if episode else "idle, " + tally)
                if preview:
                    preview.status = "{} | state {} | {}".format(rec, "ok" if state_ok else "STALE", note)
                    preview.recording = episode is not None
                sys.stdout.write("\r\033[K{} | state {} | cam {} | ctrl cmd {} | grip {} | {}".format(
                    rec, "ok" if state_ok else "STALE", cam_str, "yes" if cmd_ok else "no",
                    "ok" if grip is not None else "n/a", note))
                sys.stdout.flush()
            time.sleep(max(0.0, min(0.002, next_t - time.monotonic())))
    except KeyboardInterrupt:
        if episode:
            episode.finish("incomplete")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attrs)
        if preview:
            preview.close()
        camera.close()
        print("\nDone. Episodes are in {}".format(out_dir))


if __name__ == "__main__":
    # The SDK has no way to unsubscribe subscribeRobotState & co., and its native threads
    # keep calling those callbacks while the interpreter shuts down, which segfaults.
    # Episode files are closed by then, so leave without the interpreter teardown.
    code = 0
    try:
        main()
    except SystemExit as e:
        if isinstance(e.code, int) or e.code is None:
            code = e.code or 0
        else:
            print(e.code, file=sys.stderr)
            code = 1
    except BaseException:
        import traceback
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)

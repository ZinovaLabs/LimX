"""
Convert raw episodes from vla/collect_vla_data.py into a LeRobot dataset for
tron2_openpi (pi0.5) fine-tuning.

Run it inside the tron2_openpi environment (it pins the LeRobot version it trains with):

    cd tron2_openpi
    export HF_LEROBOT_HOME=/path/to/datasets
    uv run python /path/to/LimX/vla/convert_to_lerobot.py \\
        --raw /path/to/LimX/vla_data/<task> --repo-id tron2_<task>

Dataset written (the keys tron2_openpi's TRON2 task config reads by default):
    observation.state              float32 [16]  [left arm 7, left gripper, right arm 7, right gripper]
    action                         float32 [16]  same layout, absolute joint targets
    observation.images.cam_high    uint8 image   (and cam_left_wrist / cam_right_wrist if recorded)
    task                           the episode's instruction (use prompt_from_task: true)

Action per frame (--action):
    next_state   the state of the next frame (works for any way of operating the arms:
                 VR teleop, drag-teach, keyboard). The last frame repeats its own state.
    cmd          the robot controller's joint targets (/motor/cmd) recorded during VR
                 teleop; frames without a command fall back to next_state.
    auto         cmd when at least 95 % of an episode's frames carry a command, else next_state
    Gripper actions always use the next frame's gripper state.

Cameras: tron2_openpi's TRON2 task config reads cam_high (head), cam_left_wrist and
cam_right_wrist (end-effector cameras). Every episode must contain all three; for a quick
test without wrist cameras, --allow-missing-cameras writes black frames instead.

Episodes with status other than "success" are skipped unless --include-failed. Episodes with
more than --max-stale stale frames (old image or joint state) are skipped.
"""

import argparse
import io
import json
import os
import shutil
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
LAYOUT_NAMES = ([f"left_arm_{i}" for i in range(7)] + ["left_gripper"]
                + [f"right_arm_{i}" for i in range(7)] + ["right_gripper"])
ARM_SLOTS = list(range(0, 7)) + list(range(8, 15))        # layout index of every arm joint
ARM_MOTORS = list(range(0, 14))                           # matching motor index
GRIPPER_SLOTS = (7, 15)


def load_episode(ep_dir):
    with open(os.path.join(ep_dir, "meta.json")) as f:
        meta = json.load(f)
    rows = []
    with open(os.path.join(ep_dir, "frames.jsonl")) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    break                     # a crash can leave a partial last line
    return meta, rows


def build_actions(rows, mode):
    state = np.asarray([r["state"] for r in rows], dtype=np.float32)
    nxt = np.concatenate([state[1:], state[-1:]], axis=0)  # next_state; last frame holds
    if mode == "next_state":
        return nxt, "next_state"
    have_cmd = [r.get("cmd_q") is not None for r in rows]
    if mode == "auto" and sum(have_cmd) < 0.95 * len(rows):
        return nxt, "next_state"
    actions = nxt.copy()
    for k, r in enumerate(rows):
        if r.get("cmd_q") is not None:
            cmd = r["cmd_q"]
            for slot, motor in zip(ARM_SLOTS, ARM_MOTORS):
                actions[k, slot] = cmd[motor]
    return actions, "cmd"


def load_image(path, size):
    img = Image.open(path).convert("RGB")
    if size and img.size != (size[1], size[0]):
        img = img.resize((size[1], size[0]), Image.BILINEAR)
    return np.asarray(img, dtype=np.uint8)


def main():
    parser = argparse.ArgumentParser(description="Convert Tron2 raw episodes to a LeRobot dataset.")
    parser.add_argument("--raw", required=True, action="append",
                        help="folder with episode_* subfolders (repeat for several tasks)")
    parser.add_argument("--repo-id", required=True, help="LeRobot dataset id, e.g. tron2_pick_cup")
    parser.add_argument("--action", choices=["auto", "next_state", "cmd"], default="auto")
    parser.add_argument("--mode", choices=["video", "image"], default="video",
                        help="how LeRobot stores images (video needs ffmpeg)")
    parser.add_argument("--size", default="480x640", help="image size HxW stored in the dataset, '' keeps the source")
    parser.add_argument("--include-failed", action="store_true")
    parser.add_argument("--allow-missing-cameras", action="store_true",
                        help="write black frames for cameras not recorded (testing only)")
    parser.add_argument("--max-stale", type=float, default=0.05, help="max share of stale frames per episode")
    parser.add_argument("--overwrite", action="store_true", help="replace an existing dataset with this id")
    parser.add_argument("--dry-run", action="store_true", help="check the episodes, write nothing")
    args = parser.parse_args()
    size = tuple(int(v) for v in args.size.lower().split("x")) if args.size else None

    # ---- collect and check episodes
    episodes = []
    for raw in args.raw:
        for name in sorted(os.listdir(raw)):
            ep_dir = os.path.join(raw, name)
            if not (name.startswith("episode_") and os.path.isfile(os.path.join(ep_dir, "meta.json"))):
                continue
            meta, rows = load_episode(ep_dir)
            why = None
            if meta.get("status") != "success" and not args.include_failed:
                why = "status " + str(meta.get("status"))
            elif len(rows) < 2:
                why = "fewer than 2 frames"
            elif sum(1 for r in rows if not r.get("fresh", True)) > args.max_stale * len(rows):
                why = "too many stale frames"
            print("{:50s} {:5d} frames  {:9s} {}".format(ep_dir, len(rows), meta.get("status", "?"),
                                                         "SKIP: " + why if why else "ok"))
            if not why:
                episodes.append((ep_dir, meta, rows))
    if not episodes:
        print("No usable episodes.")
        sys.exit(1)
    fps_set = {round(m["fps"], 3) for _, m, _ in episodes}
    if len(fps_set) != 1:
        print("ERROR: episodes were recorded at different rates: {}".format(sorted(fps_set)))
        sys.exit(1)
    fps = fps_set.pop()
    cameras = sorted(set.intersection(*(set(m["cameras"]) for _, m, _ in episodes)))
    all_cameras = ["cam_high", "cam_left_wrist", "cam_right_wrist"]
    missing = [c for c in all_cameras if c not in cameras]
    if missing and (not args.allow_missing_cameras or "cam_high" in missing):
        print("ERROR: {} missing from some episodes (found in all: {}).\n"
              "       Record all three cameras (python3 vla/collect_vla_data.py --check);\n"
              "       --allow-missing-cameras writes black wrist frames for testing.".format(
                  ", ".join(missing), ", ".join(cameras) or "none"))
        sys.exit(1)
    if not size:
        first = episodes[0][0]
        w, h = Image.open(os.path.join(first, "cam_high", "000000.jpg")).size
        size = (h, w)
    print("\n{} episodes, {} frames, {} Hz, cameras {}, images {}x{}".format(
        len(episodes), sum(len(r) for _, _, r in episodes), fps, cameras, size[0], size[1]))
    if missing:
        print("WARNING: {} not recorded; written as black frames (testing only).".format(", ".join(missing)))
    if args.dry_run:
        return

    # ---- write the LeRobot dataset (LeRobot version pinned by tron2_openpi)
    from lerobot.common.datasets.lerobot_dataset import LEROBOT_HOME, LeRobotDataset
    target = LEROBOT_HOME / args.repo_id
    if target.exists():
        if not args.overwrite:
            print("ERROR: {} exists; pass --overwrite to replace it.".format(target))
            sys.exit(1)
        shutil.rmtree(target)
    features = {
        "observation.state": {"dtype": "float32", "shape": (16,), "names": [LAYOUT_NAMES]},
        "action": {"dtype": "float32", "shape": (16,), "names": [LAYOUT_NAMES]},
    }
    for cam in all_cameras:
        features["observation.images." + cam] = {
            "dtype": args.mode, "shape": (3, size[0], size[1]), "names": ["channels", "height", "width"]}
    dataset = LeRobotDataset.create(repo_id=args.repo_id, fps=int(round(fps)), robot_type="tron2",
                                    features=features, use_videos=args.mode == "video",
                                    image_writer_threads=4, image_writer_processes=2)
    black = np.zeros((size[0], size[1], 3), dtype=np.uint8)
    for ep_dir, meta, rows in episodes:
        actions, used = build_actions(rows, args.action)
        for k, r in enumerate(rows):
            frame = {"observation.state": np.asarray(r["state"], dtype=np.float32), "action": actions[k]}
            for cam in cameras:
                frame["observation.images." + cam] = load_image(
                    os.path.join(ep_dir, cam, "{:06d}.jpg".format(r["i"])), size)
            for cam in missing:
                frame["observation.images." + cam] = black
            dataset.add_frame(frame)
        dataset.save_episode(task=meta["task"])
        print("wrote {} ({} frames, action from {})".format(ep_dir, len(rows), used))
    if hasattr(dataset, "consolidate"):
        dataset.consolidate()

    print("\nDataset: {}\nIn your tron2_openpi task config (configs/train/tron2_tasks/<task>.yaml):\n"
          "  repo_id: {}\n  prompt_from_task: true\n  state_dim: 16\n  action_dim: 16\n"
          "  cam_high_key: observation.images.cam_high\n".format(target, args.repo_id), end="")
    for cam in ("cam_left_wrist", "cam_right_wrist"):
        print("  {}_key: observation.images.{}{}".format(
            cam, cam, "   # black frames: not recorded" if cam in missing else ""))


if __name__ == "__main__":
    main()

# VLA data collection for Tron2 (pi0.5 / tron2_openpi)

Record demonstrations on the Tron2 and turn them into the LeRobot dataset that LimX's
[`tron2_openpi`](https://github.com/limxdynamics/tron2_openpi) fine-tunes pi0.5 on. Camera
access follows LimX's [`tron2_env`](https://github.com/limxdynamics/tron2_env).

| File | Runs on | Purpose |
|---|---|---|
| `collect_vla_data.py` | the PC connected to the robot (`~/limx-venv`) | record episodes, read only |
| `manage_episodes.py` | anywhere | list, delete, restore, re-label episodes |
| `tron2_vla_io.py` | (module) | robot state and camera sources |
| `convert_to_lerobot.py` | the `tron2_openpi` environment | raw episodes → LeRobot dataset |
| `tron2_task_example.yaml` | `tron2_openpi` | training config template |

## Data format (matches tron2_openpi / tron2_env)

- **state / action**, 16 values: `[left arm 7, left gripper, right arm 7, right gripper]`.
  Arm joints in radians, robot motor order (motor 0-6 left, 7-13 right: shoulder pitch, roll,
  yaw, elbow, wrist yaw, pitch, roll). Gripper = opening 0..1.
- **cameras**, all three recorded in every frame: `cam_high` (head camera), `cam_left_wrist`
  and `cam_right_wrist` (the cameras on the left and right end-effectors).
- **rate**: 30 Hz, the tron2_openpi policy rate.
- **task**: the natural-language instruction, stored per episode.

## 1. Record

Operate the arms however you like (VR teleop, drag-teach, keyboard); the collector only reads.

```
source ~/limx-venv/bin/activate
cd /home/alfredo/Documents/Zinova/LimX
python3 vla/collect_vla_data.py --check      # all three cameras must show a rate and resolution
python3 vla/collect_vla_data.py --task "pick up the cup and place it on the plate"
```

`--check` prints, per camera, the rate, the resolution and the topic it found, then exits.
The collector runs the same check at start-up. A camera showing `NO IMAGES` lists the topics
it tried; give the right one with `--topic cam_left_wrist=/camera/...` (repeat per camera).

| Key | Action |
|---|---|
| `r` | start an episode; `r` again saves it as a success |
| `f` | stop, save as a failure (skipped by the converter by default) |
| `x` | stop and discard the episode |
| `d` `d` | move the last saved episode to the trash (press `d` twice within 3 s) |
| `u` | undo the last delete |
| `l` | list the episodes recorded so far |
| `t` | type a new task instruction for the next episodes |
| `q` | quit (an episode in progress is kept as "incomplete") |

The status line shows the state, every camera's image age, whether the robot controller's
joint targets are present (`ctrl cmd yes` during VR teleop), and the gripper. An episode only
starts when the joint state and all three cameras are fresh, so every saved frame has a
head, left-wrist and right-wrist image.

A small window shows all cameras side by side while collecting (needs `pip install
opencv-python`; `--no-preview` turns it off). Each camera is labelled with its image age and
rate, in red when stale, and the bar on top turns red while an episode records. The keys
above also work with the window focused; closing the window does not stop the collector.

Camera sources:

```
# robot camera topics through the LimX SDK (default, all three cameras):
#   head   /camera/top/color/image_raw/compressed
#   wrists /camera/left|right/color/image_rect_raw/compressed (image_resized, image_raw tried as fallbacks)
python3 vla/collect_vla_data.py --task "..."
# tron2_env Bridge (all three cameras): pip install -e "tron2_env[bridge]"
python3 vla/collect_vla_data.py --task "..." --cameras bridge --bridge-host wss://<bridge host>
# RealSense cameras on this PC: pip install -e "tron2_env[camera]"
python3 vla/collect_vla_data.py --task "..." --cameras realsense --serial <serial>=cam_high --serial <serial>=cam_left_wrist
```

Episodes land in `vla_data/<task>/episode_NNNNNN/` (`meta.json`, `frames.jsonl`, one JPEG per
camera per frame).

### Managing episodes

Deleted episodes go to `vla_data/<task>/.trash/`, which the converter ignores, so every
delete can be undone until the trash is emptied.

```
python3 vla/manage_episodes.py vla_data                      # summary of every task folder
python3 vla/manage_episodes.py vla_data/<task>               # list episodes
python3 vla/manage_episodes.py vla_data/<task> delete last   # or: delete 3 5 8-10 / delete failed
python3 vla/manage_episodes.py vla_data/<task> restore       # undo the last delete (restore all)
python3 vla/manage_episodes.py vla_data/<task> mark 4 failure
python3 vla/manage_episodes.py vla_data/<task> task 4 "pick up the red cup"
python3 vla/manage_episodes.py vla_data/<task> empty-trash   # delete the trash for good
```

## 2. Convert (in tron2_openpi)

```
cd tron2_openpi
export HF_LEROBOT_HOME=/path/to/datasets
uv run python /home/alfredo/Documents/Zinova/LimX/vla/convert_to_lerobot.py \
    --raw /home/alfredo/Documents/Zinova/LimX/vla_data/<task> --repo-id tron2_<task> --dry-run   # check first
uv run python /home/alfredo/Documents/Zinova/LimX/vla/convert_to_lerobot.py \
    --raw /home/alfredo/Documents/Zinova/LimX/vla_data/<task> --repo-id tron2_<task>
```

- `--action auto` (default): actions are the robot controller's joint targets when the episode
  was recorded during VR teleop, otherwise the next frame's state (drag-teach, keyboard).
- `--raw` can be repeated to merge several tasks into one dataset.
- Every episode must contain all three cameras; otherwise the converter stops with an error.
  `--allow-missing-cameras` writes black frames instead, for testing the pipeline only.

## 3. Train (in tron2_openpi)

Copy `tron2_task_example.yaml` to `tron2_openpi/configs/train/tron2_tasks/<task>.yaml`, set
`name` and `repo_id`, then:

```
uv run scripts/compute_norm_stats.py --task-config configs/train/tron2_tasks/<task>.yaml
uv run scripts/train_tron2_task.py --task-config configs/train/tron2_tasks/<task>.yaml
```

## Status

- Tested offline end to end with a simulated robot and a strict stand-in for LeRobot (shapes,
  dtypes, action derivation, task text); not yet against the real robot or real LeRobot.
- The wrist-camera topics (`/camera/left|right/...`) did not appear in the robot's topic list
  on 2026-10-07. Run `--check` with the wrist cameras connected; if they still show
  `NO IMAGES`, use the TRON2 Bridge (`--cameras bridge`, host from LimX).
- The robot published no gripper state in recent tests; the gripper then reads
  `--gripper-fill` (default 0).

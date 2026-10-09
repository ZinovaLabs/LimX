# VLA data collection for Tron2 (pi0.5 / tron2_openpi)

Record demonstrations on the Tron2 and turn them into the LeRobot dataset that LimX's
[`tron2_openpi`](https://github.com/limxdynamics/tron2_openpi) fine-tunes pi0.5 on. Camera
access follows LimX's [`tron2_env`](https://github.com/limxdynamics/tron2_env).

| File | Runs on | Purpose |
|---|---|---|
| `collect_vla_data.py` | the PC connected to the robot (`~/limx-venv`) | record episodes, read only |
| `dashboard.py` | the PC connected to the robot | live window: cameras, end-effector position, contact force |
| `manage_episodes.py` | anywhere | list, delete, restore, re-label episodes |
| `tron2_vla_io.py` | (module) | robot state, camera and force sources, arm kinematics |
| `convert_to_lerobot.py` | the `tron2_openpi` environment | raw episodes → LeRobot dataset |
| `tron2_task_example.yaml` | `tron2_openpi` | training config template |

## Data format (matches tron2_openpi / tron2_env)

- **state / action**, 16 values: `[left arm 7, left gripper, right arm 7, right gripper]`.
  Arm joints in radians, robot motor order (motor 0-6 left, 7-13 right: shoulder pitch, roll,
  yaw, elbow, wrist yaw, pitch, roll). Gripper = opening 0..1.
- **cameras**, all three recorded in every frame: `cam_high` (head camera), `cam_left_wrist`
  and `cam_right_wrist` (the cameras on the left and right end-effectors).
- **rate**: one rate for every stream, `--frequency` (default 30 Hz, the tron2_openpi policy
  rate), with all streams time-aligned in each frame (see *Synchronisation* below).
- **task**: the natural-language instruction, stored per episode.

## 1. Record

Operate the arms however you like (VR teleop, drag-teach, keyboard); the collector only reads.

```
source ~/limx-venv/bin/activate
cd /home/alfredo/Documents/Zinova/LimX
python3 vla/collect_vla_data.py --check      # every stream must show a rate of at least --frequency
python3 vla/collect_vla_data.py --task "pick up the cup and place it on the plate"
python3 vla/collect_vla_data.py --task "..." --frequency 10    # record every stream at 10 Hz
```

`--check` prints, for the joint state and each camera, the rate and latency (cameras also the
resolution and topic), and the highest `--frequency` all streams keep up with, then exits.
The collector runs the same check at start-up and refuses to start when a stream is slower
than `--frequency` (`SLOWER THAN --frequency`): either fix that stream or lower
`--frequency`. A camera showing `NO IMAGES` lists the topics it tried; give the right one with
`--topic cam_left_wrist=/camera/...` (repeat per camera). `--fps` still works as an alias.

### Synchronisation

Every frame is a synchronised observation of all streams at one instant:

- Each stream (joint state, controller command, gripper, each camera) is buffered with its
  capture time: the sender's own timestamp, mapped onto this PC's clock (each sender clock
  separately; the robot cameras share one).
- Frame k is the observation at time `t0 + k / frequency`; every stream contributes its sample
  nearest that time. Frames are assembled a short delay behind real time (measured at start-up
  from the slowest stream, e.g. ~80 ms at 30 Hz), so the samples on both sides have arrived.
- `frames.jsonl` stores, per frame, `t` (exactly `k / frequency`), `sync_ms` (each stream's
  sample time minus the frame time) and `img_seq` (which image each camera contributed). A frame
  is `fresh` when every `sync_ms` is within half a period.
- `meta.json` stores `frequency`, the alignment settings (`align`) and the counts of stale,
  skipped and repeated-image frames.

Gripper: the value is the measured opening on `/limx/2F-gripper/state` (percent, 0 closed ..
100 open, scaled to 0..1, ~100 Hz, aligned like the other streams). It is read with the SDK's
generic subscriber, since the robot publishes it as `controller_msgs/JointState`, which
`subscribeGripperState` does not receive. If that topic stops, the last command on
`/limx/2F-gripper/cmd` is used (it holds until the next), and with neither, `--gripper-fill`.
Each frame records `gripper_src` (`state`, `cmd` or `fill`), `gripper_raw` and the last
command `gripper_cmd`; the status line shows `grip L/R%`.

On the robot, the left wrist camera currently streams at 10 Hz (in a dim scene its
auto-exposure lowers the frame rate), so all three cameras record only with `--frequency 10`
until that is fixed; `--cam cam_high --cam cam_right_wrist` records the other two at 30 Hz.

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

**Going home after each episode.** The home pose is where the arms are when you start the
first episode (`--home FILE` uses a saved pose instead, e.g.
`limxsdk-lowlevel/home_pose.json`; `--no-home` turns this off). After every `r` (save) or `f`,
the arms return there slowly (`--home-speed`, default 0.2 rad/s, smooth start and stop; a
blocked joint stops the move). Going home sends joint commands, which must not fight the
robot's own controller, so:

1. press `r`: the episode is saved and the status shows `HOME: switch the robot to SDK control mode`;
2. switch the robot to SDK control mode: as soon as `/motor/cmd` goes quiet the arms move home and hold;
3. switch back to teleop: the collector stops sending at once, and the next `r` starts an episode.

While the collector is still waiting for SDK mode (nothing sent yet), `r` skips going home and
starts the next episode; once the arms move or hold, `r` waits until teleop is back.

The terminal only prints `EPS n is collecting ...`, `EPS n finished` and `EPS n deleted` (plus
errors). Numbers follow the saved episodes: after deleting the last one, the next episode reuses
its number. With `--verbose`, the start-up stream report and a live status line are printed too:
the status line shows the recording rate, the worst `sync_ms` of the last 0.2 s, the state,
every camera's image age, whether the robot controller's
joint targets are present (`ctrl cmd yes` during VR teleop), and the gripper. An episode only
starts when the joint state and all three cameras are fresh, so every saved frame has a
head, left-wrist and right-wrist image.

A small window shows all cameras side by side while collecting (needs `pip install
opencv-python`; `--no-preview` turns it off). Drag the window edges to resize it;
`--preview-height 300` sets the size it opens with (camera tile height in pixels). Each camera is labelled with its image age and
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

### Live dashboard

```
pip install pyqtgraph PyQt6            # once; PyQt6 also needs: sudo apt install libxcb-cursor0
python3 vla/dashboard.py               # --window 5|10|30, --cam ... to show fewer cameras
```

A dark, resizable window, read only, so it runs alone or next to the collector:

- **cameras**: all three, each with its rate and image age (yellow under 25 Hz, red when stale).
- **per arm**: the gripper-mount position X/Y/Z in mm as large readouts (robot base frame,
  computed from the joint angles with `limxsdk-lowlevel/urdf/DACH_TRON2A.urdf`; the
  controller's own `/arm_pose` is only published in some control modes), the change of
  X/Y/Z since the reference as curves (Δ mm), and the contact force Fx/Fy/Fz and |F| in N.
- **force**: the robot's estimate on `/dyn_identify/ee_force_kf` (no force sensor). It reads
  ~20 N with nothing touching, so press **Tare force** with the arms free. The topic carries
  no labels; `[left Fx Fy Fz Tx Ty Tz, right ...]` is assumed: push on one gripper to check.

Keys: `space` pause, `r` reset the Δ reference, `t` tare the force, `1`/`2`/`3` show 5/10/30 s,
`q` quit.

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

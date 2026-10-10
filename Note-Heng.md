Run these from limxsdk-lowlevel/, with the venv active (source ~/limx-venv/bin/activate).

Go home (move the arms to home_pose.json):
python3 python3/examples/api/example_tron2_arm_go_home.py
Press s to start, space to pause, q to quit.

Drag-teach:
python3 python3/examples/api/example_tron2_arm_drag_teach.py --check-only   # model check, sends nothing
python3 python3/examples/api/example_tron2_arm_drag_teach.py                # soft mode
In soft mode: space switches soft ↔ hold, r starts / stops recording, p saves home, h goes home, q quits.

Save the current pose as home (sends nothing):
python3 python3/examples/api/example_tron2_arm_drag_teach.py --save-home

Replay a drag recording:
python3 python3/examples/api/example_tron2_arm_record_replay.py replay drag_<time>.json --speed 0.5


# Drag replay
Record (in drag-teach, while soft):
1. Drag the arms to the start pose of your motion.
2. Press r: the status shows REC 0.0s.
3. Drag the arms through the motion, slowly.
4. Press r again to save drag_<date>_<time>.json in limxsdk-lowlevel/. Its name appears in the status line.
5. Press space (HOLD), support the arms, press q q.

Replay (robot still in low-level mode):
python3 -X faulthandler python3/examples/api/example_tron2_arm_record_replay.py replay drag_<date>_<time>.json --speed 0.5
1. It checks the recording (joint speeds below 2 rad/s) and that the robot's controller is silent, then prints how far the arms are from the recording's start pose. Nothing moves yet.
2. Press s: the arms move slowly (0.2 rad/s, at least 3 s) to the start pose, then replay the motion at the chosen speed (--speed 0.5 is half speed, a good first try).
3. Space pauses. At the end the arms hold the last pose.
4. Support the arms, press q q: the first q holds, the second stops (the arms go limp).

Notes:
- Replay uses the recorded gravity torque and the stiff hold gains, so the arms track the recording firmly. They won't feel soft during replay.
- Keep your hands near the arms the first time. If a joint stalls or something looks wrong, press space to pause, or hit the e-stop.
- Ls the folder (ls drag_*.json) to find your recordings.


# Data collection
The VLA (pi0.5) data-collection pipeline in vla/. It records demonstrations in the format LimX's tron2_openpi fine-tunes on. Camera reading reuses tron2_env's methods. It works end to end offline but hasn't run on the real robot yet.

Files
- vla/collect_vla_data.py: records episodes on the robot PC. It only reads from the robot and never sends commands, so you drive the arms with VR teleop, drag-teach or the keyboard while it runs.
- vla/tron2_vla_io.py: reads the joint state, the controller's joint targets and the gripper. Cameras come from one of three sources:
  - --cameras sdk (default): the robot's camera topics through the LimX SDK.
  - --cameras bridge: tron2_env's Bridge connection.
  - --cameras realsense: RealSense cameras plugged into your PC, through tron2_env's camera manager.
- vla/convert_to_lerobot.py: runs inside tron2_openpi and turns the raw episodes into a LeRobot dataset.
- vla/tron2_task_example.yaml: a training config template.
- vla/README.md has the full how-to, and the root README.md now points to it.

Data format
- Each frame has a 16-value state and a 16-value action: [left arm 7, left gripper, right arm 7, right gripper], joint positions in radians, gripper opening 0–1.
- Three camera images (cam_high, cam_left_wrist, cam_right_wrist), recorded at 30 Hz.
- Each episode stores its own task instruction.
- By default the action is the robot controller's joint target when you collected with VR teleop. Otherwise (drag-teach or keyboard) it is the next frame's state.

How to use it
New keys in the collector (they work between episodes)

┌─────┬───────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│ Key │                                                 What it does                                                  │
├─────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ d d │ Deletes the last saved episode. Press d twice within 3 s. The first press shows which episode it will delete. │
├─────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ u   │ Undoes the last delete.                                                                                       │
├─────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ l   │ Lists the episodes recorded so far, with status, frame count, length and task.                                │
├─────┼───────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ x   │ Discards the episode you're currently recording (unchanged).                                                  │
└─────┴───────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
## 1. record (on the robot PC)
source ~/limx-venv/bin/activate
cd ~/Documents/Zinova/LimX

## check the cameras (each one should show a rate and resolution)
python3 vla/collect_vla_data.py --check

## collect (the preview window opens automatically)
python3 vla/collect_vla_data.py --task "pick up the cup and place it on the plate"

## Change the --task text to your actual instruction. Keys while collecting:
##    r start an episode, r again saves it as a success · f save as a failure · x discard the episode · q quit

How to discard an episode:

┌──────────────┬─────────────────┬────────────────────────────────────────────────┐
│     When     │       Key       │                  What happens                  │
├──────────────┼─────────────────┼────────────────────────────────────────────────┤
│ while        │ x               │ stops and deletes the current episode          │
│ recording    │                 │                                                │
├──────────────┼─────────────────┼────────────────────────────────────────────────┤
│ after it's   │ d then d again  │ moves the last saved episode to the trash      │
│ saved        │ within 3 s      │ (vla_data/<task>/.trash/)                      │
├──────────────┼─────────────────┼────────────────────────────────────────────────┤
│ after d d    │ u               │ undoes the last delete                         │
└──────────────┴─────────────────┴────────────────────────────────────────────────┘

## Optional:
## record at 10 Hz to match the left camera until it's fixed
python3 vla/collect_vla_data.py --task "..." --fps 10

## no preview window
python3 vla/collect_vla_data.py --task "..." --no-preview

## 2. convert (in tron2_openpi)
export HF_LEROBOT_HOME=/path/to/datasets
cd ~/Documents/Zinova/tron2_openpi
unset VIRTUAL_ENV            # your limx-venv is active; uv ignores it but prints a warning
export HF_LEROBOT_HOME=~/Documents/Zinova/lerobot_datasets
D=/home/alfredo/Documents/Zinova/LimX
uv run python $D/vla/convert_to_lerobot.py \
  --raw $D/vla_data/drive_four_nails_evenly_into_the_blue_strip \
  --repo-id tron2_drive_four_nails

## 3. train: copy the yaml into configs/train/tron2_tasks/, then
uv run scripts/compute_norm_stats.py --task-config configs/train/tron2_tasks/<task>.yaml
uv run scripts/train_tron2_task.py --task-config configs/train/tron2_tasks/<task>.yaml

Training runs inside tron2_openpi on the GPU machine, in three steps: write a task config, compute normalization stats, then train.

0. One-time setup on the GPU machine

cd ~/tron2_openpi                        # wherever you cloned it there
GIT_LFS_SKIP_SMUDGE=1 uv sync            # as in its INSTALL.md
sudo apt install ffmpeg tmux
export HF_LEROBOT_HOME=~/lerobot_datasets    # where you rsynced the datasets; add to ~/.bashrc
ls $HF_LEROBOT_HOME                      # should list tron2_drive_four_nails etc.

1. Create the task config

Create configs/train/tron2_tasks/drive_four_nails.yaml:

name: pi05_tron2_drive_four_nails          # checkpoint folder name
repo_id: tron2_drive_four_nails            # must match the dataset folder name
prompt_from_task: true                     # use the instruction stored per episode
weight_loader: gs://openpi-assets/checkpoints/pi05_base/params

num_train_steps: 20000
save_interval: 5000
batch_size: 32
fsdp_devices: 1
action_horizon: 50
state_dim: 16
action_dim: 16
rtc_training_simulated_delay: 10

cam_high_key: observation.images.cam_high
cam_left_wrist_key: observation.images.cam_left_wrist
cam_right_wrist_key: observation.images.cam_right_wrist
state_key: observation.state
action_key: action

adapt_to_pi: false
use_delta_joint_actions: false
assets_base_dir: ./assets
checkpoint_base_dir: ./checkpoints

Don't add a prompt: line. The loader rejects it when prompt_from_task: true, and it also rejects any field it doesn't know.

2. Compute normalization stats (once per dataset)

uv run scripts/compute_norm_stats.py --task-config configs/train/tron2_tasks/drive_four_nails.yaml

This writes to assets/pi05_tron2_drive_four_nails/tron2_drive_four_nails/. Training and the deployed policy both need it.

3. Train (inside tmux, so it survives SSH disconnects)

tmux new -s train
export HF_LEROBOT_HOME=~/lerobot_datasets
export XLA_PYTHON_CLIENT_MEM_FRACTION=0.95     # let JAX use nearly all of the 48 GB
uv run scripts/train_tron2_task.py --task-config configs/train/tron2_tasks/drive_four_nails.yaml
# detach: Ctrl-b d     reattach: tmux attach -t train

- First run downloads the pi0.5 base weights from gs://openpi-assets (several GB), so the machine needs internet access.
- Weights & Biases: logging is on by default. Run uv run wandb login first, or add --no-wandb-enabled.
- Restart: --overwrite starts over, and --resume continues from the last checkpoint.
- Output: checkpoints are saved to checkpoints/pi05_tron2_drive_four_nails/pi05_tron2_drive_four_nails/<step>/ every 5000 steps.
- Monitoring: run watch -n 2 nvidia-smi in another pane.

The A6000 has 48 GB, which may not be enough

This config does a full fine-tune of pi0.5. The task YAML has no LoRA option, and upstream openpi says full fine-tuning needs more than 70 GB of GPU memory. Batch 32 will very likely run out of memory on the A6000.

1. Try a smaller batch first. If you get an out-of-memory error, set batch_size: 8 (or 4) and rerun. With a smaller batch you may want more steps, for example num_train_steps: 30000.
2. If batch 4 still runs out of memory, the model weights and optimizer state alone don't fit. You then need either LoRA fine-tuning, which openpi supports at about 22.5 GB, or a GPU with 80 GB.

tron2_openpi's task loader has no LoRA option, so I'd have to add one, for example a lora: true field. If option 1 fails, I can do that.

Training both tasks: to train the other dataset, copy the YAML, change name and repo_id to tron2_nail_the_white_dots_on_the_black_board, then repeat steps 2 and 3. One GPU runs one job at a time.

Testing: I ran it end to end against a simulated robot and a stand-in for LeRobot. Recording ran at 30 Hz with no stale frames. The actions came out right for both VR-style and drag-teach data, failure and discarded episodes were handled correctly, and the task text was kept. It has not run on the real robot or with the real LeRobot.

Things to check on the robot
- Wrist cameras: their topics didn't appear on the robot. tron2_openpi needs all three cameras, so any camera that wasn't recorded is filled with black frames, and the converter prints a warning. Without real wrist images the data isn't useful for training. You can get them through the Bridge (--cameras bridge --bridge-host wss://..., host from LimX) or with RealSense cameras on your PC.
- Gripper: the robot published no gripper state in our earlier tests. If that's still true, the gripper value is fixed at --gripper-fill (default 0).


###########
This repo is LimX's low-level SDK, and its Python wheel can talk to a real Tron2 over the network. I wrote a keyboard teleop script for you at python3/examples/api/example_tron2_keyboard_teleop.py. It compiles and the SDK calls it uses exist in the wheel, but I have no robot here, so it hasn't been run against real hardware.

1. Connect to the real robot

1. Connect your PC to the robot by Ethernet or the robot's network. Set your PC to a static IP on the same subnet, e.g. 10.192.1.200/24. The robot is at 10.192.1.2 by default (doc/ipconfig.png shows the setup). Check with ping 10.192.1.2.
2. Install the SDK wheel:
pip install python3/amd64/limxsdk-*.whl      # x86_64 PC
# pip install python3/aarch64/limxsdk-*.whl  # ARM, e.g. Jetson
3. Run a read-only example first to confirm the link works:
python3 python3/examples/api/example_tron2_chassis_lifter.py 10.192.1.2

2. Keyboard teleop

python3 python3/examples/api/example_tron2_keyboard_teleop.py 10.192.1.2

┌─────────────┬─────────────────────────────────────────────────┐
│     Key     │                     Action                      │
├─────────────┼─────────────────────────────────────────────────┤
│ w / s       │ Forward speed up / down, in small steps         │
├─────────────┼─────────────────────────────────────────────────┤
│ a / d       │ Steer left / right                              │
├─────────────┼─────────────────────────────────────────────────┤
│ space or x  │ Stop the base                                   │
├─────────────┼─────────────────────────────────────────────────┤
│ r / f       │ Raise / lower the lifting column (hold the key) │
├─────────────┼─────────────────────────────────────────────────┤
│ o / c       │ Open / close the gripper                        │
├─────────────┼─────────────────────────────────────────────────┤
│ q or Ctrl+C │ Stop everything and quit                        │
└─────────────┴─────────────────────────────────────────────────┘

How it works:
- Wheeled base: robot.publishChassisTwist(lin, ang, 0). Values are normalised to [-1, 1], and the script caps them at 0.3 for speed and 0.6 for steering. You can change this with MAX_LIN / MAX_ANG.
- Lifting column: robot.publishLifterVel(mm_s) at 20 mm/s.
- Gripper: robot.publishGripperCmd(...).
- Safety: commands are re-sent 20 times a second. The robot stops the base and holds the column if it gets no command for 300 ms, so if the script crashes or the network drops, the robot stops. On exit the script sends zero commands.
- The status line shows the lifter height and its fault flags. If the flags show "not calibrated" (0x040) or "state not ready" (0x080), the robot ignores lifter commands.

Things to know

- This only works on a Tron2 with a wheeled base and lifter (the mobile dual-arm model). These SDK calls drive the robot's own base and lifter controllers, so they're the safe way to teleop.
- If your Tron2 has legs, the base keys won't make it walk. This SDK's walking-level interface is publishRobotCmd, which sends raw joint position and torque targets. That takes over the motors from the robot's built-in controller, so walking would need your own balance controller. Don't send that to a real robot until it works in the Gazebo simulator (127.0.0.1) and the robot is on a hoist or stand.
- The script reads keys from the terminal, so the terminal must have focus. Over SSH it works as is.
- Keep the robot's physical e-stop and the remote control within reach the first time. Make sure nothing else, like the remote or the VR teleop, is controlling the robot at the same time. Otherwise the robot may ignore your commands or the two will fight over it.


Keys:

┌─────────────────┬──────────────────────────────────────────────────┐
│       Key       │                      Action                      │
├─────────────────┼──────────────────────────────────────────────────┤
│ 1 / 2 / b       │ Select left / right / both (arms and grippers)   │
├─────────────────┼──────────────────────────────────────────────────┤
│ e               │ Turn teleop on (presses X + A)                   │
├─────────────────┼──────────────────────────────────────────────────┤
│ t               │ Start / stop arm control                         │
├─────────────────┼──────────────────────────────────────────────────┤
│ w s / a d / r f │ Move ±x / ±y / ±z                                │
├─────────────────┼──────────────────────────────────────────────────┤
│ = / -           │ Bigger / smaller step (5 mm, 1 cm, 2 cm or 5 cm) │
├─────────────────┼──────────────────────────────────────────────────┤
│ h               │ Return to where arm control started              │
├─────────────────┼──────────────────────────────────────────────────┤
│ space           │ Stop and hold the current position               │
├─────────────────┼──────────────────────────────────────────────────┤
│ o c [ ]         │ Gripper open / close / step, as before           │
├─────────────────┼──────────────────────────────────────────────────┤
│ q               │ Quit (releases arm control first)                │
└─────────────────┴──────────────────────────────────────────────────┘






##########
I wrote a probe script, python3/examples/api/example_tron2_ee_probe.py, that collects everything into one file. It only reads from the robot and never sends commands, so it's safe to run. It passes a syntax check and the SDK fields it uses exist, but I couldn't run it against a robot: 10.192.1.2 doesn't answer from this machine.

Steps

1. Connect your PC to the robot. Use the same network setup as before (PC on 10.192.1.120), then check it responds:
ping 10.192.1.2

2. Get the robot ready for VR teleop. Power it on with the arms enabled, the way you normally would before using the headset.

3. Start the probe (it records for 60 seconds):
cd /home/heng/work/LimX/limxsdk-lowlevel
python3 python3/examples/api/example_tron2_ee_probe.py 10.192.1.2 --duration 60
It first lists all topics on the robot, then starts recording.

4. While it records, have someone use the VR controllers and do these moves slowly:
- Hold grip on the right controller, then move it about 10 cm forward and back, then left and right, then up and down.
- Release grip, move the controller somewhere else, grip again, and move it a little. This shows whether grip works as a clutch.
- Press the trigger once.
- Do the same with the left controller if there's time.

The status line shows a count for each recorded stream, e.g. {'vr_cmd': 412, 'ee_pose': 380, ...}. If vr_cmd stays at 0, the VR stream isn't reaching your PC.

5. Send me the result. The script saves tron2_probe_<date>_<time>.json in the folder you ran it from.
- If you ran it on this PC, just tell me the file name and I'll read it myself.
- If you ran it on another PC, copy the file here. Pasting it into chat won't work well because it's probably large.

No VR headset?

Still run step 3 without moving anything (--duration 10 is enough). The topic list alone tells me whether there's a separate topic for commanding the end-effector directly.

What I'll get from the file

- Which topic actually commands the end-effector.
- Whether controller positions are absolute or relative to where you pressed grip.
- Which coordinate frame and units they use, by comparing controller motion with how the arm moved.

With that, I can add arm movement keys to example_tron2_arm_keyboard_teleop.py without guessing.
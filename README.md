# LimX

Tools for the LimX Tron2 (dual 7-joint arms) built on the LimX low-level SDK in
`limxsdk-lowlevel/`. All scripts live in `limxsdk-lowlevel/python3/examples/api/`; run the
commands below from `limxsdk-lowlevel/`.

| Feature | Script | Commands the robot? | Status |
|---|---|---|---|
| [Teleop probe](#teleop-probe) | `example_tron2_ee_probe.py` | no, read only | works on the robot |
| [Keyboard arm teleop](#keyboard-arm-teleop) | `example_tron2_arm_keyboard_teleop.py` | yes, as a virtual VR headset | the robot ignores it so far |
| [Arm joint record / replay](#arm-joint-record--replay) | `example_tron2_arm_record_replay.py` | replay only | record: ready; replay: needs SDK control mode and 16-motor commands |
| [Drag-teach](#drag-teach) | `example_tron2_arm_drag_teach.py` | yes, soft / hold modes | model check passes on the robot; soft mode needs SDK control mode and 16-motor commands |
| [Home pose](#home-pose) | `example_tron2_arm_drag_teach.py --save-home`, `example_tron2_arm_go_home.py` | go-home only | saving: works; go-home: needs SDK control mode |

## Setup

1. Connect your PC to the robot network: set the PC to `10.192.1.120/24`, then check
   `ping 10.192.1.2`.
2. Install the SDK wheel (once). On Ubuntu 24.04 the system Python has no pip, so use a
   virtual environment:
   ```
   sudo apt install python3-venv python3-pip
   python3 -m venv ~/limx-venv
   source ~/limx-venv/bin/activate
   pip install python3/amd64/limxsdk-*.whl
   ```
   In every new terminal run `source ~/limx-venv/bin/activate` first.
3. Before anything that commands the robot: turn the VR headset fully off (hold the power
   button about 3 s and choose "Power off"), keep the robot's e-stop in reach, and make
   sure nothing else is controlling the arms.

### Known facts about this robot (firmware as of 2026-10-07)

- The robot is a **DACH_TRON2A** (dual arms + 2-DoF head). Its model is in
  [`limxsdk-lowlevel/urdf/`](limxsdk-lowlevel/urdf/README.md) (from LimX's
  `tron2-robot-description`, Apache-2.0); its gravity torques match the robot controller's
  within 0.01 Nm on all 14 arm joints.
- `RobotState` has 16 motors and **no motor names**. The scripts label them
  `motor_0` ... `motor_15`: motors 0-6 are the **left** arm, 7-13 the **right** arm (each
  shoulder pitch, shoulder roll, shoulder yaw, elbow, wrist yaw, wrist pitch, wrist roll),
  14-15 the head.
- The robot's own arm controller publishes `/motor/cmd` (~280 Hz) all the time, with
  Kp 150-550 and Kd 9-15. Our joint commands would fight it, so scripts that send joint
  commands refuse to run until the robot is in an **SDK / low-level control mode** (ask
  LimX how to switch to it).
- Its joint commands are unnamed and cover all 16 motors. Replay and drag-teach still send
  only the selected joints, by name; they need switching to full 16-motor commands (as
  `example_tron2_arm_go_home.py` already does) before they are used on this robot.
- `getRobotDescription()` fails ("malformed HTTP response ... /get_sn"), so drag-teach uses
  the shipped URDF by default.
- End-effector poses arrive on `/arm_pose` (14 floats: left xyz + wxyz, right xyz + wxyz),
  not on `/arm/ee_pose_state`.

## Teleop probe

Records the tele-operation channels to a JSON file. Read only; safe to run any time.

```
python3 python3/examples/api/example_tron2_ee_probe.py 10.192.1.2 --duration 40
```

1. Start the robot's normal VR teleop with the headset.
2. Run the command. It lists every topic, then records for `--duration` seconds.
3. While it records, use the VR controllers: X/A to enable, hold both grips and move a
   controller slowly, release.
4. It writes `tron2_probe_<time>.json` with `/vr_cmd`, `/vr_cmd_tron`, `/servop_L`,
   `/servop_R`, `/arm_pose`, `/arm_pose_des`, `/move_cmd`, `/robot_mode`, `/teleop_cmd`,
   `/robot_state` and the TeleOperation status messages.

## Keyboard arm teleop

Drives the arms through the robot's VR tele-operation by sending a virtual VR headset
whose controllers you move with the keyboard.

> Status: on the robot, neither `/sdk_vr_cmd` nor `/vr_cmd` got a response yet. The
> controls below work offline; the input channel still has to be found.

```
python3 python3/examples/api/example_tron2_arm_keyboard_teleop.py 10.192.1.2 --topic /vr_cmd
```

1. Turn the VR headset off (the script refuses to start while it publishes).
2. Press `b` (both arms), `-` (small steps), `e` (enable), `t` (take control).
3. Move with the keys below. Each line of the event log shows the key and the measured
   end-effector positions, so you can see which way each key moves the arm.
4. Press `t` to release, `q` to quit.

| Key | Action |
|---|---|
| `1` / `2` / `b` | select left / right / both arms (and grippers) |
| `e` | enable tele-operation (presses X + A) |
| `t` | take control on / off (holds / releases both grips) |
| `w` `s` / `a` `d` / `r` `f` | move ±x / ±y / ±z |
| `u` `m` / `i` `k` / `j` `l` | roll / pitch / yaw ± |
| `=` / `-` | bigger / smaller step: 5 mm/2°, 1 cm/5°, 2 cm/10°, 5 cm/15° |
| `h` | back to where take control started (position and orientation) |
| space | stop and hold |
| `o` `c` `]` `[` | gripper open / close / step open / step close |
| `q`, Ctrl+C | release and quit |

Limits: 5 cm/s and 30°/s; at most 30 cm and 60° per axis from where take control started.

**Record and replay a keyboard session** (the exact stream the script sent):
```
python3 python3/examples/api/example_tron2_arm_keyboard_teleop.py 10.192.1.2 --topic /vr_cmd --record session1.json
python3 python3/examples/api/example_tron2_arm_keyboard_teleop.py 10.192.1.2 --topic /vr_cmd --replay session1.json --speed 0.5
```
During replay: `s` start, space pause, `q` abort.

## Arm joint record / replay

Records the arms' measured joint angles (`RobotState`) while you move them any way you
like (VR teleop, keyboard, drag-teach), and replays them with joint commands.

**Record** (read only):
```
python3 python3/examples/api/example_tron2_arm_record_replay.py record traj1.json
```
1. Get ready to move the arms (for example start VR teleop).
2. Press `r` to start recording, move the arms, press `q` to stop and save.

At 100 Hz it saves every motor's q / dq / tau, the robot controller's joint command
(target, Kp, Kd, feed-forward torque), `/arm_pose` and the gripper opening.

**Replay** (commands the motors; needs SDK control mode):
```
python3 python3/examples/api/example_tron2_arm_record_replay.py replay traj1.json --speed 0.5
```
1. `s`: the arms move slowly (0.2 rad/s, at least 3 s) to the first recorded pose, then
   play the recording with its original timing (`--speed` scales it).
2. Space pauses, `q` quits; at the end it holds the last pose until `q`.

It replays only the joints matching `--joints` (default: the 14 arm motors, `motor_0` ...
`motor_13`; the head is left alone), with the gains
and feed-forward torque recorded from the robot controller, and refuses recordings that
would move a joint faster than 2 rad/s.

## Drag-teach

Makes the arms soft (gravity compensated) so you can guide them by hand, and records the motion for replay.

> Status: the model check passes on the robot. Soft / hold mode still needs SDK control
> mode and 16-motor commands; it is tested offline only.

1. Check the gravity model first (sends nothing):
   ```
   python3 python3/examples/api/example_tron2_arm_drag_teach.py --check-only
   ```
   With the arms held still, it compares the model's gravity torque on each arm joint with
   the gravity torque the robot's own controller sends (measured torque is shown too; it
   includes friction). Continue only if it says `PASSED`. It uses the shipped
   `urdf/DACH_TRON2A.urdf`; pass `--urdf FILE` for another model.
2. Start it, with one hand on the arm:
   ```
   python3 python3/examples/api/example_tron2_arm_drag_teach.py
   ```
3. Press space to go soft. Do not touch the arm until the stiffness shows `0%`, then guide
   it by hand.
4. `r` starts / stops recording (each stop writes `drag_<time>.json`).
5. Space goes back to stiff hold; `q` holds for 1 s and quits.
6. Replay a recording:
   ```
   python3 python3/examples/api/example_tron2_arm_record_replay.py replay drag_<time>.json --speed 0.5
   ```

| Key | Action |
|---|---|
| space | soft ↔ hold |
| `r` | start / stop recording |
| `p` | save the current pose as home |
| `h` | go home slowly (space stops) |
| `q`, Ctrl+C | hold, then quit |

Safety: refuses to go soft if the model check fails (tolerance max(0.3 Nm, 5 %) against
the controller's gravity torque, or max(1 Nm, 10 %) against measured torque when no
controller command is visible); ramps
stiffness down over 2 s and returns to hold if the arm drifts more than 3° meanwhile;
returns to hold if a joint moves faster than 1.5 rad/s; pushes back within 5° of a URDF
joint limit. Only the selected arm joints (`--joints`) are commanded.

## Home pose

**Save the arms' current pose as the default home** (read only):
```
python3 python3/examples/api/example_tron2_arm_drag_teach.py --save-home
```
It writes `home_pose.json` (all 16 motors) and prints the angles. You can also press `p`
in drag-teach. Drag-teach loads this file as its home on every start.

**Move to the home pose** (needs SDK control mode):
```
python3 python3/examples/api/example_tron2_arm_go_home.py
```
1. Power on the robot, enable the arms, switch to SDK control mode.
2. Run the command. It prints, per motor, the current angle, the home angle and the move,
   plus how long it takes. If the robot's controller is still commanding the motors it
   refuses and sends nothing.
3. Press `s`. All 16 motors move to home at most 0.2 rad/s (at least 3 s), with a smooth
   start and stop, using the robot controller's own gains. Space pauses.
4. It shows `at home: holding` once every joint is within 2°, or names the joint that is
   not. If a joint lags more than 5.7° (blocked), it stops and holds where the arms are.
5. Press `q` to quit.

## VLA data collection (pi0.5)

Record demonstrations (state, top and wrist cameras, task text) at 30 Hz and convert them
into the LeRobot dataset that LimX's `tron2_openpi` fine-tunes pi0.5 on. The collector only
reads from the robot, so drive the arms with VR teleop, drag-teach or the keyboard.

```
source ~/limx-venv/bin/activate
python3 vla/collect_vla_data.py --task "pick up the cup and place it on the plate"
# r start/save, f failure, x discard, t new task, q quit
```

Converting and training: see [`vla/README.md`](vla/README.md).

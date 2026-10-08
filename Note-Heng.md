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
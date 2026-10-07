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
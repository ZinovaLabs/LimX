# Tron2 URDF

`DACH_TRON2A.urdf` is `tron2a/DACH_TRON2A/urdf/robot.urdf` from
[limxdynamics/tron2-robot-description](https://github.com/limxdynamics/tron2-robot-description)
(branch `main`, copied 2026-10-07), unmodified.

Copyright (c) 2024-2026 LimX Dynamics Inc. Licensed under the Apache License 2.0
(<http://www.apache.org/licenses/LICENSE-2.0>).

It is the model of our robot: its gravity torques match the ones the robot's own arm
controller sends on `/motor/cmd` within 0.01 Nm on all 14 arm joints (checked on
2026-10-07). Only the joints, masses and inertias are used; the mesh files it references
are not included.

Motor order on this firmware (RobotState / RobotCmd carry no names):

| Motors | URDF joints |
|---|---|
| `motor_0` ... `motor_6` | `proximal_pitch_L_Joint`, `proximal_roll_L_Joint`, `proximal_yaw_L_Joint`, `elbow_L_Joint`, `wrist_yaw_L_Joint`, `wrist_pitch_L_Joint`, `wrist_roll_L_Joint` |
| `motor_7` ... `motor_13` | the same joints of the right arm (`..._R_Joint`) |
| `motor_14`, `motor_15` | head (yaw / pitch; order not confirmed) |

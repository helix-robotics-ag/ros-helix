## `/tendon_transmission_node` Namespace

### Source
`ros-helix/helix_transmission/helix_transmission/tendon_transmission.py`

### Topics

`~/commands` `Float64MultiArray[9]`
- Commands the arm tendon position setpoint in [m], relative to the calibrated tendon 0 position, within the configured limits.

`~/current_commands` `Float64MultiArray[9]`
- Commands the arm tendon current in [mA], within the configured limits.

`~/tendon_states` `JointState[9]`
- Position and velocity of arm tendons.

### Services

`~/switch_to_current_control` `std_srvs/Trigger`
- Switches all motor controllers to current mode so that position commands will be ignored.

`~/switch_to_position_control` `std_srvs/Trigger`
- Switches all motor controllers to position mode so that current commands will be ignored.


`~/tendon_transmission_node/set_current` `helix_transmission_interfaces/SetCurrent`

```
float64 current 0.0
---
bool success
string message
```

- Sets all arm tendon current commands to the [mA] value of `current`, within the configured limits.

`~/set_motor_offsets` `std_srvs/Trigger`
- Sets all arm tendon 0 calibration point to the current state, by writing the current motor joint positions to the calibration offsets file.

`~/check_calibration` `std_srvs/Trigger`
- Checks whether the current tendon positions are within +\\-Pi radians the calibrated 0 position. If they aren't, the joint positions in the calibrated offsets files will be offset by multiples of +\\-2*Pi radians until they are. This should be used when the robot is in the same state it was calibrated in (ie straight with nominal tension current applied), to account for the motors losing their absolute position refrence when switched off.

## `/helix_gripper_node` Namespace

### Source
`ros-helix/helix_gripper/helix_gripper/helix_gripper.py`

### Topics

`~/command_increment` `Float64`
- Moves the gripper tendon by the value in [m], clipped to the configured limit.

## `/helix_cartesian_control_node` Namespace

### Source
`ros-helix-proprietary/helix_cartesian_control/helix_cartesian_control/helix_cartesian_control_node.py`

### Topics

`~/dxdyl_state` `JointState[9]`
- The DxDyL state of the model `[Dx1, Dy1, L1, Dx2, Dy2, L2, Dx3, Dy3, L3]`. Generally updated whenever the model state changes, such as when the model is used to calucalte a Cartesian command, or if the DxDyL state is commanded directly.

`~/cartesian_state` `PoseArray[4]`
- Cartesian poses of the 3 arm section end frames `seg1_end_link`, `seg2_end_link` and `seg3_end_link`, and the gripper TCP frame `helix_tcp`. Calculated from the model FK and updated whenever `~/dxdyl_state` is published.

`~/dxdyl_command` `Float64MultiArray[9]`
- Direct `[Dx1, Dy1, L1, Dx2, Dy2, L2, Dx3, Dy3, L3]` command, which is converted to tendon lengths and commanded as an immediate setpoint. 

`~/delta_increment`  `TwistStamped`
- Incremental Cartesian command for the gripper TCP, applied in the `frame_id` supplied in the header (which defaults to `origin`). The components of `twist.angular` are applied as rotations around the extrinsic/static axes.

### Services

`~/reset_model` `std_srvs/Trigger`
- Reset the model to the initial/calibration state, and also command tendons to the zero/calibrated lengths.

`~/activate_joystick_control` `std_srvs/Trigger`
- Activate teleoperation control by connecting the `spacenav/twist` topic to `~/delta_increment`.

`~/deactivate_joystick_control` `std_srvs/Trigger`
- Deactivate teleoperation control by disconnecting the `spacenav/twist` topic from `~/delta_increment`.

`~/go_to_gripper_pose_vector` `helix_transmission_interfaces/GoToGripperPoseVector`

```
string frame_id
geometry_msgs/Point goal_point
geometry_msgs/Vector3 goal_direction
bool plan_linear
bool ik_uses_prev
---
bool success
string message
```
- Move the gripper to a goal defined by a gripper TCP point and vector direction.

`~/go_to_gripper_pose_quat` `helix_transmission_interfaces/GoToGripperPoseQuat`
```
string frame_id
geometry_msgs/Pose goal_pose
bool plan_linear
bool ik_uses_prev
---
bool success
string message
```
- Move the gripper to a goal defined by a gripper TCP point and quaternion direction.

`~/go_to_gripper_pose_euler` `helix_transmission_interfaces/GoToGripperPoseEuler`
```
string frame_id
geometry_msgs/Point goal_point
float64[3] goal_euler_angs
string axes "sxyz"
bool plan_linear
bool ik_uses_prev
---
bool success
string message
```
- Move the gripper to a goal defined by a gripper TCP point and Euler angle set direction.

## `ros2_control`

The below topics are used by `ros2_control` to interface directly to the motors. Prefer using the custom topics above, which include transmission, limits etc.

`motor_head_joint_position_controller/commands` `Float64MultiArray[9]`
- Arm tendon motor joint setpoint commands in [rad].

`motor_head_joint_effort_controller/commands` `Float64MultiArray[9]`
- Arm tendon motor current commands in [mA].

`motor_head_joint_state_broadcaster/joint_states` `JointState[9]`
- Arm tendon motor joint position, velocity and current.

`gripper_joint_position_controller/commands` `Float64MultiArray[1]`
- Gripper tendon motor joint setpoint commands in [rad].

`gripper_joint_state_broadcaster/joint_states` `JointState[1]`
- Gripper tendon motor joint position, velocity and current.

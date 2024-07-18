#### Model based control of the end effector in Cartesian space is implemented in the [`ros-helix-proprietary` repo](https://github.com/helix-robotics-ag/ros-helix-proprietary). 

# Model Details

### End effector goal specification
Within the inverse kinematics a 'gripper pose' consisting of two points is used: the 'wrist' point at the end of the arm (position of `seg3_end_link` frame), and the gripper TCP which is located some distance along the Z-direction of that frame. Together these specify the position of the `helix_tcp` frame and the direction it points in, although it does not constrain the orientation around its Z-axis, which remains uncontrolled.

![frames](https://github.com/helix-robotics-ag/ros-helix/assets/95340175/1c209feb-aff2-4ade-80e0-1b00c6aebda7)

### Model initialisation
When the system starts up, it initialises a model of the robot kinematics. Note that there is currently no feedback between the motor/tendon positions and the model, so it is important that the calibrated state of the robot matches the model. The default initial state of the model is with the arm sections straight, and with lengths of 0.12m, 0.24m, and 0.24m respectively, which should correspond with the physical state of the robot when `/tendon_transmission_node/tendon_states` are all 0. The best way to acheive this is to first do the current control based calibration, then in position control measure and adjust the tendons directly until they all match the correct section lengths, then call `/tendon_transmission_node/set_motor_offsets` again.

Calling the service `/helix_cartesian_control_node/reset_model` will reset the internal model to the initial state, and also command all the tendons lengths to 0.

### Model State
The internal model state is published on two topics: 
- `/helix_cartesian_control_node/dxdyl_state`: `JointState[9]` with DxDyL state in the `position` array (velocity/effort unused)
- `/helix_cartesian_control_node/cartesian_state`: `PoseArray[4]` containing the poses of frames `seg1_end_link`, `seg2_end_link`, `seg3_end_link` and `helix_tcp`.

Remember that these only represent the model state and have no connection to the physical robot or motor states, they are published along with the corresponding tendon length commands when the model is used. As such, external forces on the robot or commanding the tendon lengths directly will not be reflected in the model state.

## Cartesian control of the gripper

### Incremental control
When a `TwistStamped` message is received on the `/helix_cartesian_control_node/delta_increment` topic, the gripper pose will be adjusted by the linear (m) and angular (rad) components (relative to the provided `frame_id`, or the `origin` frame if this is not specfied). This is intended for small delta changes, although the scale of the input is not checked or limited, so be careful if publishing to it directly.

### Joystick control
`/spacenav/twist` topic input is forwarded to `/helix_cartesian_control_node/delta_increment` (at a certain rate and gain), allowing direct teleoperation of the gripper pose. This can be turned on by calling the `/helix_cartesian_control_node/activate_joystick_control` service, or off with the deactivate service. The joystick commands are in the `origin` frame.

### Pose to Pose control
Three services can be called to move the end effector to a specified goal, which differ only in how the desired gripper direction is specified. 

`/helix_cartesian_control_node/go_to_gripper_pose_vector` takes a `geometry_msgs/Point` for the TCP position and a `geometry_msgs/Vector3` for the direction of the `helix_tcp` frame Z-axis in the request.
```
{
  "goal_point" : {"x": <x>, "y": <y>, "z": <z>},
  "goal_direction" : {"x": <x>, "y": <y>, "z": <z>}
}
```
`/helix_cartesian_control_node/go_to_gripper_pose_quat` takes a `geometry_msgs/Pose` for the `helix_tcp` frame in the request. Note that, as discussed above, only the direction of the Z-axis specified by the orientation quaternion is actually constrained.
```
{
  "goal_pose":
  {
    "position" : {"x": <x>, "y": <y>, "z": <z>},
    "orientation" : {"x": <x>, "y": <y>, "z": <z>, "w": <w>}
  }
}
```
`/helix_cartesian_control_node/go_to_gripper_pose_euler` takes a `geometry_msgs/Point` for the TCP position, `float64[3]` set of Euler angles and `string` axes specification for the `helix_tcp` frame orientation in the request. Again only the direction of the Z-axis of the resulting orientation is actually constrained. The axes specification is a four letter string as described [here](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#specifying-angle-conventions).
```
{
  "goal_point" : {"x": <x>, "y": <y>, "z": <z>},
  "goal_euler_angs" : [<r>, <p>, <y>],
  "axes" : "<axes_spec>"
}
```
In addition to the goal, these services have several other optional parameters:
- `string frame_id`: indicating in which frame the goal is specified. Defaults to `origin` if not specified.
- `bool plan_linear`: when `true`, the services will interpolate a series of poses between the current state and goal, and calculate the IK for each to create a more linear end effector trajectory. Defaults to `false`, where the trajectory will be planned over one step only.
- `bool ik_uses_prev`: when `true`, the IK will use the previous end effector state as a starting point. This may be useful combined with the linear trajectory planning, since in theory from step to step the IK should converge more quickly, however it is also possible that the arm model ends up in an unnatural state where the IK gets stuck. Defaults to `false`, where the IK uses the initial calibrated arm state as the starting point each time, which generally converges more robustly.

### Trajectory Execution

The services first calculate a trajectory to the goal, start the trajectory execution and then return their response immediately (not after the trajectory is completed, as this continues separately from the service call). They will return a `success=true` response even if unable to plan to the goal fully, and will execute a partial trajectory up to the closest point to the goal that the IK converged to. A `success=false` response will only be returned if the frame in which the goal was specified was invalid.

### Cartesian Pose Service Call Examples

These examples will all move the gripper to the same state. 

Using `/helix_cartesian_control_node/go_to_gripper_pose_vector`:
```
{
  "frame_id":"origin",
  "goal_point":{"x": 0, "y": 0.25, "z": -0.4},
  "goal_direction":{"x": 0, "y": 1, "z": 0}
}
```
```
{
  "frame_id":"arm_base",
  "goal_point":{"x": 0, "y": -0.25, "z": 0.4},
  "goal_direction":{"x": 0, "y": -1, "z": 0}
}
```

Using `/helix_cartesian_control_node/go_to_gripper_pose_quat`:
```
{
  "frame_id":"origin",
  "goal_pose":
  {
    "position":{"x": 0, "y": 0.25, "z": -0.4},
    "orientation":{"x": -0.7071, "y": 0, "z": 0, "w": 0.7071}
  }
}
```
```
{
  "frame_id":"arm_base",
  "goal_pose":
  {
    "position":{"x": 0, "y": -0.25, "z": 0.4},
    "orientation":{"x": 0.7071, "y": 0, "z": 0, "w": 0.7071}
  }
}
```

Using `/helix_cartesian_control_node/go_to_gripper_pose_euler`:
```
{
  "frame_id":"origin",
  "goal_point":{"x": 0, "y": 0.25, "z": -0.4},
  "goal_euler_angs":[-1.57, 0, 0],
  "axes":"sxyz"
}
```
```
{
  "frame_id":"origin",
  "goal_point":{"x": 0, "y": 0.25, "z": -0.4},
  "goal_euler_angs":[0, 0, -1.57],
  "axes":"szyx"
}
```

## DxDyL Configuration Control
It is also possible to control the DxDyL configuration of the robot directly, by publishing a `Float64MultArray[9]` to `/helix_cartesian_control_node/dxdyl_command`. Currently the callback for this topic simply converts the DxDyL configuration to the corresponding tendon lengths using the kinematic model and commands them directly, so any smoother interpolation between DxDyL states needs to be done externally. Note that the calculations take ~15-50ms, so you should limit the rate at which you publish to this topic. Example message format: `Float64MultiArray({"data":[<Dx1>,<Dy1>,<L1>,<Dx2>,<Dy2>,<L2>,<Dx3>,<Dy3>,<L3>]})`.

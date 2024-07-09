## Cartesian Control
Model based control of the end effector in Cartesian space is implemented in the [`ros-helix-proprietary` repo](https://github.com/helix-robotics-ag/ros-helix-proprietary). 

### End effector specification
The "gripper pose" is used as an end effector goal, which is specified by two points: the 'wrist' point in the centre of the end of the 3rd section of the robot arm, and the gripper TCP which is located some distance along the Z-direction of that frame. Those two points are the positions of the `seg3_end_link` and `helix_tcp` frames respectively. Together this provides a specification for the position of the `helix_tcp` frame and the direction that the frame's Z-axis points in. The orientation of the `helix_tcp` frame around it's Z-axis remains unconstrained, so it is worth noting that the 'gripper_pose' is actually a 5DOF specification, not a full 6DOF pose.

### Model initialisation
When the system starts up, it initialises a model of the robot kinematics. Note that there is currently no feedback between the motor/tendon positions and the model, so it is important that the calibrated 'zero' state of the robot matches the model for it to work. The initial state of the model is with the arm sections straight, and with lengths of 0.105m, 0.255m, and 0.240m respectively (TBC - conigurable initial model state), which should correspond with the physical state of the robot when `/tendon_transmission_node/tendon_states` are all 0. The best way to acheive this is to first do the above current control based calibration, then in position control measure and adjust the tendons directly until they all match the correct section lengths, then call `/tendon_transmission_node/set_motor_offsets` again.

Calling the service `/helix_cartesian_control_node/reset_model` will reset the internal model to the initial state, and also command all the tendons lengths to 0.

### Incremental control
The `/helix_cartesian_control_node/delta_increment` topic provides a way to adjust the gripper pose incrementally. When a `TwistStamped` message is received, the gripper pose will be changed by its linear (m) and angular (rad) increment (relative to the provided `frame_id`, or the `origin` frame if this is not specfied). This is intended for small delta changes, although the scale is not checked or limited, so be careful.

### Joystick control
The '/spacenav/twist' input is forwarded to `/helix_cartesian_control_node/delta_increment` (at a certain rate and gain), allowing direct teleoperation of the gripper pose. This can be turned on and off by calling the `/helix_cartesian_control_node/activate_joystick_control` (or `deactivate_`...) services.

### Pose to Pose control
There are three services that can be called to move the end effector to a specified goal, which differ only in how the desired gripper direction is specified. They first calculate a trajectory to the goal, start the trajectory execution and then return their response immediately (not after the trajectory is completed, as this continues separately from the service call). It will return a `success=true` response even if it is unable to plan to the goal fully, and will execute a partial trajectory up to the closest point to the goal the IK converged to. A `success=false` response will only be returned if the frame in which the goal was specified was invalid.

`/helix_cartesian_control_node/go_to_gripper_pose_vector` takes a `geometry_msgs/Point` for the TCP position and `geometry_msgs/Vector3` for the `helix_tcp` frame Z-axis direction vector in the request.
```
{
  "goal_point":{"x": <x>, "y": <y>, "z": <z>},
  "goal_direction":{"x": <x>, "y": <y>, "z": <z>}
}
```
`/helix_cartesian_control_node/go_to_gripper_pose_quat` takes a `geometry_msgs/Pose` for the `helix_tcp` frame in the request. Note that, as discussed above, only the direction of the Z-axis specified by the orientation quaternion is actually constrained.
```
{
  "goal_pose":
  {
    "position":{"x": <x>, "y": <y>, "z": <z>},
    "orientation":{"x": <x>, "y": <y>, "z": <z>, "w": <w>}
  }
}
```
`/helix_cartesian_control_node/go_to_gripper_pose_euler` takes a `geometry_msgs/Point` for the TCP position and `float64[3]` set of Euler angles and `string` axes specification for the `helix_tcp` frame Z-axis direction vector in the request. Again only the direction of the Z-axis resulting from the Euler angle specification is actually constrained. The axes specification is a four letter string as described [here](https://matthew-brett.github.io/transforms3d/reference/transforms3d.euler.html#specifying-angle-conventions), and aso defaults to "sxyz" if not included.
```
{
  "goal_point":{"x": <x>, "y": <y>, "z": <z>},
  "goal_euler_angs":[<r>,<p>,<y>],
  "axes":"<axes_spec>"
}
```
In addition to the goal, these services have several other optional parameters:
- `string frame_id`: indicating in which frame the goal is specified. Defaults to `origin` if not included.
- `bool plan_linear`: when `true`, the service interpolate a series of points between the current state and goal, and calculates the IK for each point to create a more linear end effector trajectory. Defaults to `false`, where the trajectory will be planned over one step only.
- `bool ik_uses_prev`: when `true`, the IK will use the previous end effector state as a starting point. This may be useful combined with the linear trajectory planning, since in theory from step to step the IK should converge more quickly, however it is also possible that the arm model ends up in an unnatural state where the IK gets stuck. Defaults to `false`, where the IK uses the initial calibrated arm state as the starting point each time, which in general converges more robustly.

Complete examples of request formats which will each plan to the same gripper state:

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
  "goal_euler_angs":[-1.57, 0, 0]
}
```
```
{
  "frame_id":"arm_base",
  "goal_point":{"x": 0, "y": -0.25, "z": 0.4},
  "goal_euler_angs":[0, 0, 1.57],
  "axes":"szyx"
}
```

### DxDyL Configuration Control
It is also possible to control the DxDyL configuration of the robot directly, by publishing a `Float64MultArray[9]` to `/helix_cartesian_control_node/dxdyl_command`. Currently the callback for this topic simply converts the DxDyL configuration to the corresponding tendon lengths using the kinematic model and commands them directly, so any smoother interpolation between DxDyL states needs to be done externally. The calculations currently take ~50ms, so you should only publish to this topic at max ~10-20Hz. Example message format: `Float64MultiArray({"data":[<Dx1>,<Dy1>,<L1>,<Dx2>,<Dy2>,<L2>,<Dx3>,<Dy3>,<L3>]})`.

### Model State
The interal model state is published on two topics: 
- `/helix_cartesian_control_node/dxdyl_state`: `JointState[9]` with DxDyL state in the `position` array (velocity/effort unused)
- `/helix_cartesian_control_node/cartesian_state`: `PoseArray[4]` containing the poses of frames `seg1_end_link`, `seg2_end_link`, `seg3_end_link` and `helix_tcp`.

Remember that these only represent the model state and have no connection to the physical robot or motor states, they are published along with the corresponding tendon length commands when the model is used. As such, external forces on the robot or direct control of the tendon lengths will not be reflected in the model state.
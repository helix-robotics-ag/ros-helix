# Controlling the robot from an external device
Note: Most of the instructions below will be described for control from an external device running native ROS. As Foxglove and `roslibpy` provide ways for other devices that aren't running ROS to interact with the system from a browser or Python script respectively, the same instructions can be followed for those interfaces - the details for how this can be done are included later in the README. Please read the full set of instructions before using the system from any interface.

## Launching the embedded Helix ROS system
From the terminal of the Helix Raspberry Pi (assuming the Pi has previously been set up correctly):
- Navigate to the `main` repo directory
- `$ git status` Check that the `main` branch (of the `main` repo) is checked out
- `$ git pull` Check that the branch is up to date
- `$ git submodule update --init --recursive` Make sure all submodules are checked out correctly as well
- `$ docker compose pull` Make sure the most updated images are pulled
- `$ docker compose up` Start all the containers

Among the output, you should see the below messages, indicating that the system launched correctly:
```
Loaded motor_head_joint_position_controller (in blue)
Configured and activated motor_head_joint_position_controller (in green)
Loaded motor_head_joint_state_broadcaster (in blue)
Configured and activated motor_head_joint_state_broadcaster (in green)
Loaded motor_head_joint_effort_controller (in blue)
```
For further information, or troubleshooting in case launching fails, see the README in the [`main` repo](https://github.com/helix-robotics-ag/main/tree/main).

## Connecting to the Helix ROS system
Connect your device to the Helix RPi by Ethernet, configure network settings and ensure you are able to communicate with the Pi. If you can ping the Pi successfully, you should be able to see all the Helix topics with `$ ros2 topic list`. 

## Controlling the robot through the Tendon Transmission interface
The `helix_transmission` package provides the main way to interact with the robot by reading and commanding lengths of the nine tendons. This is done through topics and services under the `tendon_transmission_node` namespace.
- `/tendon_transmission_node/joint_states` publishes the current tendon position and velocities (effort is currently unused) in a `sensor_msgs/JointState.msg`
- `/tendon_transmission_node/commands` will read a 9-element `std_msgs/Float64MultiArray.msg` setting the desired tendon length setpoint, ie publishing `{'data': [0,0,0,0,0,0,0,0,0]})` will set the motor positions to the calibrated zero lengths; from here publishing `{'data': [-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01]})` will contract all tendons by 1cm.
- The interface uses a robot configuration saved in `~/.config/helix/helix_transmission_config.yml` on the Pi, as well as a calibration file `tendon_calib.yml` at the same location, defining the 'zero length' state of the tendons.

## Calibrating Tendon Zero State
To calibrate the nominal straightened zero configuration, follow this procedure: 

1. The system starts in position control mode, to switch to current control mode call the service `/tendon_transmission_node/switch_to_current_control`
2. The zero state should be set with the nominal tensioning current applied from a roughly straight state. If needed, release the tension on the tendons beforehand by calling the service `/tendon_transmission_node/set_current` with the request `{"current": 3.0}`, causing the motors to slowly start to unwind
3. Stop the motors by calling the same service with `{"current": 0.0}` (Note: setting zero current isn't equivalent to disabling motor torque - the motors will try to hold the current at 0, which will resist relaxing the tendons)
4. From the roughly straight state, apply the nominal tension current by calling the same service again with `{"current": -70.0}`
5. Now manipulate the robot into the desired zero state. Once there, save the calibration by calling the `/tendon_transmission_node/set_motor_offsets` service.
6. You can now switch back to position control mode by calling `/tendon_transmission_node/switch_to_position_control`

## Checking the zero state
The motors' absolute positions may be lost after shutting down. To check that they are still correct, follow the above steps up to and including (4), then call `/tendon_transmission_node/check_calibration`. This will check whether the motor positions are within half a turn of the calibration, and offset the calibration file with additional revolutions if not. **This should be done whenever the motor controller is turned off and on, to avoid accidentally overtensioning the arm.** Note that if in doubt, the state of the tendon lengths with the nominal tension current applied should all be within about 0.03 (3cm) of 0. If not, you should run the check calibration function, or recalibrate if needed. 

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

## Gripper Motor Controller
The gripper motor is assigned Dyanmixel ID #9. A separate controller is available for this motor, by publishing to `/helix_gripper_node/command_increment` the tendon length can be incrementally changed. The message to publish is of type `Float64` corresponding to a tendon length change in metres, and the value is currently limited to +/-0.005m to avoid overtightening (however if you publish the message in a 100Hz loop you will command 0.5m of tendon length change in 1 second). 

The system needs to be in position control mode (ie using `/tendon_transmission_node/switch_to_position_control`) for the gripper control to work (it is not currently possible to operate the motors in different control modes simultaneously). However it is possible to read the gripper motor current at `/gripper_joint_state_broadcaster/joint_states`, which could be useful to monitor gripping force.

## Commanding Motor Controllers Directly
The motor controllers are still available to command directly, but this is not recommended and there are some things to be aware of.
### Read the motor joint states
On the topic `/motor_head_joint_state_broadcaster/joint_states`. **Note: on this topic the joint states are not broadcast in order, you need to refer to the 'names' field of the message to match them. This is the only place where this is the case, all other joint and tendon broadcast or command topics are in order 0-8.**
### Command motor positions
On the topic `/motor_head_joint_position_controller/commands`. Units are radians and you need to take into account the orientation of the motors (increasing turns anticlockwise).
### Command motor currents
On the topic `/motor_head_joint_current_controller/commands`. Units are mA and you need to take into account the orientation of the motors (positive turns anticlockwise).

The right controller needs to be active to command it (best to use the switch controller services on `/tendon_transmission_node/` to avoid activating them both at the same time).

# Controlling the robot through Foxglove
After launching the system on the Pi, go to `<IP_of_Pi>:8080` in a browser on your device. Choose 'Open Connection' and use `ws://<IP_of_Pi>:8765`. You should find and interface as below:

![foxglove_layout](https://github.com/helix-robotics-ag/ros-helix/assets/95340175/b5bd3271-5c53-42a2-936d-4171c3a0dfbd)

The elements of the interface are:
- A: All available topics are shown in this sidebar, or if the Panel tab is selected, settings for the currently highlighted panel are shown
- B: A 3D visualisation of the robot
- C: A plot showing each of the tendon state positions
- D: A table with all information from the `/tendon_states` topic. (This seems to be the best way to display all this information, despite the `name` field taking up a lot of space. If the fields under `position` etc just say `Object`, click to expand them)
- E: This panel will allow you to call the service chosen in the settings. Fill the Request field if needed.
- F: This panel will allow you to publish messages to a topic, which can be chosen in its settings (you may also need to set the correct message schema here)
- G: This panel will display the messages received on the topic name entered in the bar at the top of the panel, though the actual data can be hard to read
- H: Teleop panel (currently unused)

_Note if the interface is different your browser may have cached an old or modified version, you will need to clear your cookies or use Incognito mode to load the default one._

# Controlling the robot through `roslibpy`
[`roslibpy`](https://roslibpy.readthedocs.io/en/latest/#) can be used to access ROS topics, publish messages and make service calls through Python scripts. See [this full step by step demo notebook](https://github.com/helix-robotics-ag/main/blob/main/demos/roslibpy_demo.ipynb), which covers how to use all of these functions, and provides examples of basic Helix control.

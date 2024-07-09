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
The "gripper pose" is used as an end effector goal, which is specified by two points in Cartesian space: the 'wrist' point in the centre of the end of the 3rd section of the robot arm, and the gripper TCP which is located some distance along the Z-direction of that frame. Those two points are the positions of the `seg3_end_link` and `helix_tcp` frames respectively. Together this provides a specification for the position of the `helix_tcp` frame and the direction that the frame's Z-axis points in. The orientation of the `helix_tcp` frame around it's Z-axis remains unconstrained, so it is worth noting that the 'gripper_pose' is actually a 5DOF specification, not a full 6DOF pose.

### Model initialisation
When the system starts up, it initialises a model of the robot kinematics. Note that there is currently no feedback between the motor/tendon positions and the model, so it is important that the calibrated 'zero' state of the robot matches the model for it to work. The initial state of the model is with the arm sections straight, and with lengths of 0.105m, 0.255m, and 0.240m respectively (TBC - conigurable initial model state), which should correspond with the physical state of the robot when `/tendon_transmission_node/tendon_states` are all 0. The best way to acheive this is to first do the above current control based calibration, then in position control measure and adjust the tendons directly until they all match the correct section lengths, then call `/tendon_transmission_node/set_motor_offsets` again.

### Incremental control
The `/helix_cartesian_control_node/delta_increment` topic provides a way to adjust the gripper pose incrementally. When a `TwistStamped` message is received, the gripper pose will be changed by its linear (m) and angular (rad) increment (relative to the provided `frame_id`, or the `origin` frame if this is not specfied). This is intended for small delta changes, although the scale is not checked or limited, so be careful.

### Joystick control
The '/spacenav/twist' input is forwarded to `/helix_cartesian_control_node/delta_increment` (at a certain rate and gain), allowing direct teleoperation of the gripper pose. This can be turned on and off by calling the `/helix_cartesian_control_node/activate_joystick_control` (or `deactivate_`...) services.

### Pose to Pose control


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

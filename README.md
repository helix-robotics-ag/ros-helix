# Controlling the robot from an external device
Note: Most of the instructions below will be described for an external device running native ROS. As Foxglove and `rosliby` provide ways for other devices that aren't running ROS to interact with the system from a browser or Python script respectively, the same instructions can be followed for those interfaces - the details for how this can be done are included later in the README. Please read the full set of instructions before using the system.

## Launching the embedded Helix ROS system
From the Helix RPi terminal (assuming the Pi has previously been set up correctly):
- Navigate to the `main` repo directory
- `$ git status` Check that the `main` branch (of the `main` repo) is checked out
- `$ git pull` Check that the branch is up to date
- `$ git submodule update --recursive` Make sure all submodules are checked out correctly as well
- `$ docker compose pull` Make sure the most updated images are pulled
- `$ docker compose run` Start all the containers

For further information, or troubleshooting in case launching fails, see the README in the [`main` repo](https://github.com/helix-robotics-ag/main/tree/main).

## Connecting to the Helix ROS system
Connect your device to the Helix RPi by Ethernet, configure network settings and ensure you are able to communicate with the Pi (further instructions TBC).

Check whether you can ping the Pi, and if the connection works, you should be able to see all the Helix topics with `$ ros2 topic list`.

## Controlling the robot through the Tendon Transmission interface
The `helix_transmission` package provides the main way to interact with the robot by reading and commanding lengths of the nine tendons. This is done through topics and services under the `tendon_transmission_node` namespace.
- `/tendon_transmission_node/joint_states` publishes the current tendon position and velocities (effort is currently unused) in a `sensor_msgs/JointState.msg`
- `/tendon_transmission_node/commands` will read a 9-element `std_msgs/Float64MultiArray.msg` setting the desired tendon length setpoint, ie publishing `{'data': [0,0,0,0,0,0,0,0,0]})` will set the motor positions to the calibrated zero lengths; publishing `{'data': [-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01]})` will contract all tendons by 1cm.
- The interface uses a configuration saved in `~/.config/helix/helix_transmission_config.yml`, as well as a calibration file `tendon_calib.yml` at the same location, defining the 'zero length' state of the tendons. Calling the service `/tendon_transmission_node/set_motor_offsets` will update the calibration, zeroing the tendon lengths at the current state.

## Calibrating Tendon Zero State
To calibrate the nominal straightened zero configuration, follow this procedure: 

1. The system starts in position control mode, to switch to current control mode call the service `/tendon_transmission_node/switch_to_current_control`
2. The zero state should be set with the nominal holding current applied from a roughly straight state. If needed, release the tension on the tendons by calling the service `/tendon_transmission_node/set_current` with the request `{"current": 3.0}`, causing the motors to slowly start to unwind
3. When it is roughly straight, stop the motors by calling the same service with `{"current": 0.0}` (Note: setting zero current isn't equivalent to disabling motor torque - the motors will try to hold the current at 0, which will resist relaxing the tendons)
4. From the roughly straight state, apply the nominal tension current by calling the same service again with `{"current": -70.0}`
5. Now manipulate the robot into the desired zero state. Once there, save the calibration by calling the `/tendon_transmission_node/set_motor_offsets` service.
6. You can now switch back to position control mode by calling `/tendon_transmission_node/switch_to_position_control`

## Checking the zero state
The motors' absolute positions may be lost after shutting down. To check that they are still correct, follow the above steps up to an including (4), then call `/tendon_transmission_node/check_calibration`. This will check whether the motor positions are within half a turn of the calibration, and offset the calibration file with additional revolutions if not. **This should be done whenever the motor controller is turned off and on.**

## Commanding Motor Controllers Directly
The motor controllers are still available to command directly, but there are some things to be aware of.
### Read the motor joint states
On the topic `/motor_head_joint_state_broadcaster/joint_states`. **Note: on this topic the joint states are not broadcast in order, you need to refer to the 'names' field of the message to match them. This is the only place where this is the case, all other joint and tendon broadcast or command topics are in order 0-8.**
### Command motor positions
On the topic `/motor_head_joint_position_controller/commands`. Units are radians and you need to take into account the orientation of the motors (increasing turns anticlockwise).
### Command motor currents
On the topic `/motor_head_joint_position_controller/commands`. Units are mA and you need to take into account the orientation of the motors (positive turns anticlockwise).

The right controller needs to be active to command it (best to use the switch controller services on `/tendon_transmission_node/` to avoid activating them both at the same time).

# Controlling the robot through Foxglove
After launching the system on the Pi, go to `<IP_of_Pi>:8080` in a browser on your device. Choose 'Open Connection' and use `ws://<IP_of_Pi>:8765`. A 3D visualisation should be visible in the foxglove interface, and all available topics listed on the left. You can access all of the functionality discussed above using the following:
- All available topics are visible in the panel on the left
- The unlabelled panel with a small +/- icon in its top left corner will display the messages received on the topic name entered in the bar at the top of the panel
- The panel labelled Publish will allow you to send messages to a topic, which can be chosen by clicking the cog icon in its top right corner (you may also need to set the correct message type here)
- Adding a 'Service Request' panel will allow you to call the service chosen in the settings panel. The request content can be filled in the blank input field (note most of the services mentioned above are of the type `std_srvs/Trigger.srv` which do not take any request content)
See the Foxglove documentation for other functionality.

# Controlling the robot through `roslibpy`
TBC

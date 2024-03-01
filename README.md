# Calibration and Tendon Commands
The `helix_transmission` package provides an interface for commanding the tendons of the robot, instead of the motors directly:
- Command tendon length instead of rotor angle, and also with limits applied
- Command tendon contraction/relaxation with consistent sign (convert the motor orientation)
- Allow for setting a tendon 'zero' position and using this as the command reference (calibrate motor positions)

Relevant parameters are stored in a config file on the host Pi, in `~/.config/helix_transmission.config.yml`. These can be modified locally for the specific robot, eg if the motor orientations or pulley size changes. Note: currently `~/.config/` needs to be created on the Pi before first launching the container, to avoid issues with permissions after mounting the volume. A default `helix_transmission.config.yml` will be created when launching the container in the case that it doesn't exist yet.

Using the interface is done through topics and services in the namespace `/tendon_transmission_node/`. See [this script](https://github.com/fstella97/HelixRobotics/blob/main/ROS/roslibpy_service_test.py) for an example of listing these, and calling a service through roslibpy.

## Calibrating Tendon Zero State
Make sure all the containers are running and all the controllers started successfully.

### Switch to Current Control
The system starts in position control mode, to switch to current control mode, call the service:
```
srv = roslibpy.Service(client, '/tendon_transmission_node/switch_to_current_control', 'std_srvs/Trigger')
```
### Manually Set the Zero State
The zero state should be set with the nominal holding current applied. However this should be applied from a roughly straight state, whereas the robot might start up in a curved state (solution to this TBC). Hence, first release the tension on the tendons by applying a small negative current - the motors will start to slowly unwind:
```
srv = roslibpy.Service(client, '/tendon_transmission_node/set_unwind_current', 'std_srvs/Trigger')
```
When it is roughly straight, stop the motors: 
```
srv = roslibpy.Service(client, '/tendon_transmission_node/set_zero_current', 'std_srvs/Trigger')
```
(Note: setting zero current isn't equivalent to disabling motor torque - the motors will try to hold the current at 0, which will resist relaxing the tendons)

From the roughly straight state, apply the holding current:
```
srv = roslibpy.Service(client, '/tendon_transmission_node/set_holding_current', 'std_srvs/Trigger')
```
Now, manipulate the robot into the desired zero state. Once there, save the state (write the current motor positions to the `tendon_calib.yml` file):
```
srv = roslibpy.Service(client, '/tendon_transmission_node/set_motor_offsets', 'std_srvs/Trigger')
```
Switch back to position control mode:
```
srv = roslibpy.Service(client, '/tendon_transmission_node/switch_to_position_control', 'std_srvs/Trigger')
```
## Use Tendon Commands
The `/tendon_transmission_node/tendon_states` topic publishes the tendon lengths. Publishing to the `/tendon_transmission_node/commands` topic will command the tendon lengths, ie publishing `{'data': [0,0,0,0,0,0,0,0,0]})` will set the motor positions to the positions saved in the calibration file; publishing `{'data': [0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]})` will contract all tendons by 1cm.

## Checking the zero state
The motors' absolute positions may be lost after shutting down. To check that they are still correct, follow the above steps up to applying the holding current, then call:
```
srv = roslibpy.Service(client, '/tendon_transmission_node/check_calibration', 'std_srvs/Trigger')
```
This will check whether the motor positions are within half a turn of the calibration, and offset the calibration file with additional revolutions if not. **This should be done after each launch (automated process TBC).**

# Commanding Motor Controllers Directly
The motor controllers are still available to command directly, but there are some things to be aware of.
### Read the motor joint states
On the topic `/motor_head_joint_state_broadcaster/joint_states`. **Note: on this topic the joint states are not broadcast in order, you need to refer to the 'names' field of the message to match them. This is the only place where this is the case, all other joint and tendon broadcast or command topics are in order 0-8.**
### Command motor positions
On the topic `/motor_head_joint_position_controller/commands`. Units are radians and you need to take into account the orientation of the motors (increasing turns anticlockwise).
### Command motor currents
On the topic `/motor_head_joint_position_controller/commands`. Units are mA and you need to take into account the orientation of the motors (positive turns anticlockwise).

The right controller needs to be active to command it (best to use the switch controller services on `/tendon_transmission_node/` to avoid activating them both at the same time).

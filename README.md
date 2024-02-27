# Calibration and Tendon Commands
The `helix_transmission` package provides an interface for commanding the tendons of the robot, instead of the motors directly:
- Command tendon length instead of rotor angle (abstract pulley ratio)
- Command tendon contraction/relaxation with consistent sign (abstract motor orientation), and also with limits applied
- Allow for setting a tendon 'zero' position and using this as the command reference (calibrate motor positions)
  
Relevant parameters are stored in the [config](https://github.com/helix-robotics-ag/ros-helix/blob/main/helix_transmission/config/helix_transmission.config.yml), except the motor position calibrations for the tendon zero, which are saved into an untracked file in the same directory. 

Using the interface is done through topics and services in the namespace `/tendon_transmission_node/`. See [this script](https://github.com/fstella97/HelixRobotics/blob/main/ROS/roslibpy_service_test.py) for an example of listing these, and calling a service through roslibpy.

## Calibrating Tendon Zero State
**In order to save the calibration file, it is currently necessary to start the container with the run script: `./run.sh` (inside the ros-helix repo directory), then `run` on the resulting container command line. Starting with `docker compose up ros-helix` will not work since the volume where the file is saved is not mounted in the compose file (solution to this is TBC).** (Note the rest of the containers can be started as normal: `docker compose up nginx studio ros-foxglove-bridge ros-rosbridge-suite`.)

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
### Use Tendon Commands
The `/tendon_transmission_node/tendon_states` topic publishes the tendon lengths. Publishing to the `/tendon_transmission_node/commands` topic will command the tendon lengths, ie publishing `{'data': [0,0,0,0,0,0,0,0,0]})` will set the motor positions to the positions saved in the calibration file; publishing `{'data': [0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01]})` will contract all tendons by 1cm.
### Checking the zero state
The motors' absolute positions may be lost after shutting down. To check that they are still correct, follow the above steps up to applying the holding current, then call:
```
srv = roslibpy.Service(client, '/tendon_transmission_node/check_calibration', 'std_srvs/Trigger')
```
This will check whether the motor positions are within half a turn of the calibration, and offset the calibration file with additional revolutions if not. This should be done after each launch (automated process TBC).


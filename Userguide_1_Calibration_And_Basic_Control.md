## Controlling the robot through the Tendon Transmission interface
The `helix_transmission` package provides an interface for reading and commanding lengths of the nine tendons through topics and services under the `tendon_transmission_node` namespace.
- `/tendon_transmission_node/joint_states` publishes a `JointState[9]` with the current tendon position and velocities (effort is currently unused).
- `/tendon_transmission_node/commands` will read a `Float64MultiArray[9]` message setting the desired tendon length setpoint, ie publishing `{'data': [0,0,0,0,0,0,0,0,0]})` will set the motor positions to the calibrated zero lengths; from here publishing `{'data': [-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01,-0.01]})` will contract all tendons by 1cm.
- The interface uses a robot configuration saved in `~/.config/helix/helix_transmission_config.yml` on the Pi, as well as a calibration file `tendon_calib.yml` at the same location, defining the 'zero length' state of the tendons.
- The `/tendon_transmission_node/current_commands` topic can be used to set individual currents (in mA) in a similar way (as long as current control mode is selected, see step 1 of the section below).

## Calibrating Tendon Zero State
To calibrate the nominal straightened zero configuration, follow this procedure: 

1. The system starts in position control mode, to switch to current control mode call the service `/tendon_transmission_node/switch_to_current_control`
2. The zero state should be set with the nominal tensioning current applied from a roughly straight state. If needed, release the tension on the tendons beforehand by calling the service `/tendon_transmission_node/set_current` with the request `{"current": 3.0}`, causing the motors to slowly start to unwind.
3. Stop the motors by calling the same service with `{"current": 0.0}` (Note: setting zero current isn't equivalent to disabling motor torque - the motors will try to hold the current at 0, which will resist relaxing the tendons)
4. From the roughly straight state, apply the nominal tension current by calling the same service again with `{"current": -70.0}`
5. Now manually put the robot into the desired zero state. Once there, save the calibration by calling the `/tendon_transmission_node/set_motor_offsets` service.
6. You can now switch back to position control mode by calling `/tendon_transmission_node/switch_to_position_control`

## Checking the zero state
The motors' absolute positions may be lost after shutting down. To check that they are still correct, follow the above steps up to and including (4), then call `/tendon_transmission_node/check_calibration`. This will check whether the motor positions are within half a turn of the calibration, and offset the calibration file with additional revolutions if not. **This should be done whenever the motor controller is turned off and on, to avoid accidentally overtensioning the arm.** Note that if in doubt, the state of the tendon lengths with the nominal tension current applied should all be within about 0.03 (3cm) of 0. If not, you should run the check calibration function, or recalibrate if needed. 

## Gripper Motor Controller
The gripper motor is assigned Dyanmixel ID #9. A separate controller is available for this motor, by publishing to `/helix_gripper_node/command_increment` the tendon length can be incrementally changed. The message to publish is of type `Float64` corresponding to a tendon length change in metres, and by default the value is limited to +/-0.005m to avoid overtightening (however if you publish the message in a 100Hz loop you will command 0.5m of tendon length change in 1 second). 

The system needs to be in position control mode (ie using `/tendon_transmission_node/switch_to_position_control`) for the gripper control to work (it is not currently possible to operate the motors in different control modes simultaneously). However it is possible to read the gripper motor current at `/gripper_joint_state_broadcaster/joint_states` while in position control mode, which could be useful to monitor gripping force.

## Commanding Motor Controllers Directly
The motor controllers are also available to command directly, but this is not recommended and there are some things to be aware of.

The motor joint states can be read on the topic `/motor_head_joint_state_broadcaster/joint_states`. **Note: on this topic the joint states are not broadcast in order, you need to refer to the 'names' field of the message to match them. This is the only place where this is the case, all other joint and tendon broadcast or command topics are in order 0-8.**

The motor positions can be commanded on the topic `/motor_head_joint_position_controller/commands`. Units are radians and you need to take into account the orientation of the motors (increasing turns anticlockwise).

The motor currents can be commanded on the topic `/motor_head_joint_current_controller/commands`. Units are mA and you need to take into account the orientation of the motors (positive turns anticlockwise).

The right controller needs to be active to command it (by using the switch controller services on `/tendon_transmission_node/` to avoid activating them both at the same time).

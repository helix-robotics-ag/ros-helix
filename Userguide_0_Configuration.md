# Configuration files

Several configuration files can be used to adjust parameters of the model (they are read on start up). The files are located at `~/.config/helix` on the Pi, which should have been created when [setting up the Pi](https://github.com/helix-robotics-ag/main?tab=readme-ov-file#i-want-to-set-up-a-new-robotrpi). The directory is mounted to `/tmp/config` in the running containers.

### `helix_transmission.config.yml`
This controls the main physical parameters of the tendon controllers. If no file is present on startup, one will be created with the default values below:
```
current_max_lim: 10.0    # Limits in [mA] applied when commanding the tendon currents
current_min_lim: -300.0  # Limit is per segment, so the limits for tendons 3-8 will be double
motor_orients:           # 1 for motor orientations where anticlockwise pulls the tendon
- 1.0
- 1.0
- 1.0
- 1.0
- 1.0
- 1.0
- -1.0
- -1.0
- -1.0
pulley_radius: 0.01      # [m]
tendon_max_lim: 0.1      # Absolute limit (relative to calibrated 0) in [m] applied when commanding tendon
tendon_min_lim: -0.1     # positions. Limit is per segment, so the limits for tendons 3-8 will be double
```

### `tendon_calib.yml`
Not really a configuration file, this stores the motor offsets used to set the tendon 0 position. It is written and/or read from when the `/tendon_transmission_node/set_motor_offsets` or `/tendon_transmission_node/check_calibration` services are called.

### `helix_gripper.config.yml`
Similar but for the seperate gripper motor. Defaults to:
```
increment_lim: 0.005 # Limit in [m] applied for each message received to increment the gripper tendon
motor_orient: 1.0
pulley_radius: 0.005
```

### `cartesian_control.config.yml`
This controls some parameters of the cartesian control. Defaults to:
```
cartesian_control_speed: 0.1          # Nominal TCP speed [m/s] for pose to pose. Angular [rad/s] is 5x faster
cartesian_spacing: 0.05               # Spacing in [m] used to interpolate points when "plan_linear" is used
ik_error_threshold: 0.001             # Threshold in [m] used for IK convergence for pose to pose control 
ik_joystick_error_threshold: 0.00025  # Joystick IK threshold needs to be smaller for fine adjustment
joystick_control_speed: 0.05          # Nominal TCP speed [m/s] for joystick. Angular [rad/s] is 5x faster
```

### `pcc_model.config.yml`
This defines the parameters used to initialise the underlying model for cartesian control. If no file it present, the defaults from the [`HelixKinematicPCCModel`](https://github.com/helix-robotics-ag/helix-models/blob/main/helix_models/helix_kinematic_pcc.py) class will be used.

If no file is present, one will **not** be created. If you want to change the parameters, create a file with the below fields. `gripper_length` is probably the parameter you are most likely to want to change, this is the [m] distance in the Z-direction from the end of the centre of the 3rd robot arm section to the gripper TCP. See the `helix_models` repo docs for more information on the others.
```
gripper_length: 0.05
section_lengths: [0.120, 0.240, 0.240]
k_curvature: 0.1
k_axial: 1.0
d_limits: [0.7854, 1.571, 1.571]
ws_limits: [[0.055, 0.7854, 0.065, 0.120, 0.7854, 0.085],[0.115, 1.571, 0.150, 0.255, 1.571, 0.150],[0.125, 1.571, 0.165, 0.240, 1.571, 0.185]]
```

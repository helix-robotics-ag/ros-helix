## Launching the embedded Helix ROS system
From the terminal of the Helix Raspberry Pi (assuming the Pi has previously been set up correctly):
- Navigate to the `main` repo directory
- `$ git status` Check that the `main` branch (of the `main` repo) is checked out
- `$ git pull` Check that the branch is up to date
- `$ git submodule update --init --recursive` Make sure all submodules are checked out correctly as well
- `$ docker compose pull` Make sure the most updated images are pulled
- `$ docker compose up` Start all the containers

Among the output, you should see some messages in green and blue saying 'Loaded...' and 'Configured...' the various controllers, indicating that the system launched correctly. For further information, or troubleshooting in case launching fails, see the README in the [`main` repo](https://github.com/helix-robotics-ag/main/tree/main).

## Connecting to the Helix ROS system
Connect your device to the Helix RPi, configure network settings and ensure you are able to communicate with the Pi. If you can ping the Pi successfully, you should be able to see the Helix ROS system as well. Probably the easiest way to quickly check if it is working is through Foxglove (see the external interfaces readme).

## Further information
See the other documentation for information about:
1. Calibrating and controlling the tendon lengths, and the gripper
2. Cartesian control of the end effector
3. General information about using the system from the browser-based Foxglove interface or Python scripting using `roslibpy`
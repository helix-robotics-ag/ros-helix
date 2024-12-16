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

## Reference for Topics and Services

A summary of the relevant topics and services is [here](Topics_and_Services.md). You should still read the userguide information for the relevant systems as well to understand how they work.

## ROS System Diagram Overview

A diagram overview of the system is [here](helix_ros2_diagram.pdf). This shows the connections and message flow between the ROS2 topics, services etc.

## Userguide
See the other documentation in this repo for information about:

0. [Configuration files available to adjust system parameters.](https://github.com/helix-robotics-ag/ros-helix/blob/main/Userguide_0_Configuration.md)
1. [Calibrating and controlling the arm and gripper tendons directly](https://github.com/helix-robotics-ag/ros-helix/blob/main/Userguide_1_Calibration_And_Basic_Control.md)
2. [Cartesian control of the end effector](https://github.com/helix-robotics-ag/ros-helix/blob/main/Userguide_2_Cartesian_Control.md)
3. [General information about using the system from the browser-based Foxglove interface or Python scripting using `roslibpy`](https://github.com/helix-robotics-ag/ros-helix/blob/main/Userguide_3_External_Interfaces.md)

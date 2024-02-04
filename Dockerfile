ARG ROS_DISTRO=iron

FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-tf2-msgs \
    ros-${ROS_DISTRO}-rosapi-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-rosbridge-msgs \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-dynamixel-workbench-toolbox \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh .

WORKDIR /colcon_ws
COPY helix_bringup src/helix_bringup
COPY helix_description src/helix_description
COPY dynamixel_hardware src/dynamixel_hardware

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --symlink-install --event-handlers console_direct+

RUN echo 'alias build="colcon build --symlink-install  --event-handlers console_direct+"' >> ~/.bashrc
RUN echo 'source /opt/ros/iron/setup.bash; source /colcon_ws/install/setup.bash; ros2 launch helix_bringup helix_bringup.launch.py' >> /run.sh && chmod +x /run.sh
RUN echo 'alias run="su - ros /run.sh"' >> /etc/bash.bashrc

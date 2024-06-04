import os
import socket

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit

def generate_launch_description():

    ld = LaunchDescription()

    robot_description = os.path.join(get_package_share_directory("helix_description"), "urdf", "helix.urdf.xacro")

    robot_description_config = xacro.process_file(robot_description, mappings={'mesh_url' : f'http://{socket.gethostname()}.local'})
    # robot_description_config = xacro.process_file(robot_description, mappings={'mesh_url' : 'package://helix_description'})

    # Publishes robot frames to tf using URDF and /joint_states
    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        )

    # Publishes full set of joint states to /joint_states, from multiple sources
    # TODO - add helix_arm_joint_state_broadcaster/joint_states to source list once implemented in ros-helix-proprietary.git/helix_nonlinear_model
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['motor_head_joint_state_broadcaster/joint_states','gripper_joint_state_broadcaster/joint_states'], 
        }]
    )

    controller_config = os.path.join(
        get_package_share_directory(
            "helix_description"), "config", "controllers.yaml"
    )

    # Main ros2_control startup node
    helix_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output="screen",
    )

    # ros2_control 'controller' (broadcaster) for motor joint states
    motor_head_joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_head_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ros2_control controller for motor joint positions
    motor_head_joint_position_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_head_joint_position_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    # ros2_control controller for motor joint efforts
    motor_head_joint_effort_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["motor_head_joint_effort_controller", "--inactive", "-c", "/controller_manager"],
            output="screen",
    )

    tendon_transmission_node = Node(
        package="helix_transmission",
        executable="tendon_transmission_node",
        name="tendon_transmission_node",
        output="screen",
    )

    # ros2_control 'controller' (broadcaster) for gripper joint state
    gripper_joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ros2_control controller for gripper joint position
    gripper_joint_position_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_joint_position_controller", "-c", "/controller_manager"],
        output="screen",
    )

    helix_gripper_node = Node(
        package="helix_gripper",
        executable="helix_gripper_node",
        name="helix_gripper_node",
        output="screen",
    )

    spacenav_node = Node(
        package="spacenav",
        executable="spacenav_node",
        name="spacenav_node",
        output="screen",
    )

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(helix_ros2_control_node)
    ld.add_action(motor_head_joint_state_broadcaster_node)
    ld.add_action(motor_head_joint_position_controller_node)
    ld.add_action(motor_head_joint_effort_controller_node)
    ld.add_action(tendon_transmission_node)
    ld.add_action(gripper_joint_state_broadcaster_node)
    ld.add_action(gripper_joint_position_controller_node)
    ld.add_action(helix_gripper_node)
    ld.add_action(spacenav_node)

    return ld

import os
import socket

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    robot_description = os.path.join(get_package_share_directory("helix_description"), "urdf", "helix.urdf.xacro")

    robot_description_config = xacro.process_file(robot_description, mappings={'mesh_url' : f'http://{socket.gethostname()}.local'})
    # robot_description_config = xacro.process_file(robot_description, mappings={'mesh_url' : 'package://helix_description'})

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        )

    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{
    #         'source_list': ['helix_joint_state_publisher/joint_states','joint_state_broadcaster/joint_states'],
    #     }]
    # )

    controller_config = os.path.join(
        get_package_share_directory(
            "helix_description"), "config", "controllers.yaml"
    )

    helix_ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output="screen",
    )

    helix_joint_position_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller", "--inactive", "-c", "/controller_manager"],
        output="screen",
    )
    
    helix_joint_effort_controller_node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_effort_controller", "--inactive", "-c", "/controller_manager"],
            output="screen",
    )

    helix_joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[controller_config],
        output="screen",
    )


    ld.add_action(robot_state_publisher)
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(helix_ros2_control_node)
    ld.add_action(helix_joint_position_controller_node)
    ld.add_action(helix_joint_effort_controller_node)
    ld.add_action(helix_joint_state_broadcaster_node)

    return ld
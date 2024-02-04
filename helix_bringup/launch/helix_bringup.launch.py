import os

import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()



    robot_description = os.path.join(get_package_share_directory("helix_description"), "urdf", "helix.urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen",
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['helix_description/joint_states','dynamixel_joint_state_publisher/joint_states'],
        }]
    )




    controller_config = os.path.join(
        get_package_share_directory(
            "helix_description"), "config", "controllers.yaml"
    )

    dynamixel_block_ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        )

    dynamixel_block_joint_state_broadcaster_node =   Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dynamixel_joint_state_publisher", "--controller-manager", "/controller_manager"],
            parameters=[controller_config],
            output="screen",
        )

    dynamixel_block_position_controller_node =    Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dynamixel_position_controller", "-c", "/controller_manager"],
            output="screen",
        )


    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(dynamixel_block_ros2_control_node)
    ld.add_action(dynamixel_block_joint_state_broadcaster_node)
    ld.add_action(dynamixel_block_position_controller_node)

    return ld
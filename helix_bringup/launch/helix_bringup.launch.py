import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    # Example nodes
    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
    )
    
    listener_node = Node(
        package="demo_nodes_py",
        executable="listener"
    )


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
            'source_list': ['franka_state_controller/joint_states','helix_description/joint_states','dynamixel_joint_state_publisher/joint_states'],
        }]
    )

    controller_config = os.path.join(
        get_package_share_directory(
            "dynamixel_block_description"), "controllers", "controllers.yaml"
    )

    dynamixel_block_ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output="screen",
        )

    joint_publisher_config = os.path.join(
        get_package_share_directory(
            "helix_description"), "config", "params.yaml"
        )

    dynamixel_block_joint_state_broadcaster_node =   Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dynamixel_joint_state_publisher", "--controller-manager", "/controller_manager", "-p", joint_publisher_config],
            output="screen",
        )

    dynamixel_block_position_controller_node =    Node(
            package="controller_manager",
            executable="spawner",
            arguments=["dynamixel_position_controller", "-c", "/controller_manager"],
            output="screen",
        )




    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(dynamixel_block_ros2_control_node)
    ld.add_action(dynamixel_block_joint_state_broadcaster_node)
    ld.add_action(dynamixel_block_position_controller_node)

    return ld
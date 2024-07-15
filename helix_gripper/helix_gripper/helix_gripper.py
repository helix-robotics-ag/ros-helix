import rclpy
from rclpy.node import Node
import numpy as np
import yaml

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


class HelixGripperNode(Node):

    def __init__(self):
        super().__init__('helix_gripper_node')

         # Load saved robot configuration from host, or create default config
        path_to_config = '/tmp/config/helix_gripper.config.yml'
        try:
            with open(path_to_config, 'r') as file:
                config = yaml.safe_load(file)
                self.PULLEY_RADIUS = config['pulley_radius']
                self.MOTOR_ORIENT = config['motor_orient']
                self.INCREMENT_LIM = config['increment_lim']
        except (FileNotFoundError):
            self.get_logger().info('No gripper configuration file found, setting defaults')
            with open(path_to_config, 'w') as file:
                self.PULLEY_RADIUS = 0.005  # [m]
                self.MOTOR_ORIENT = 1.0 # 1 for motor orientations where anticlockwise pulls the tendon
                self.INCREMENT_LIM = 0.005 # [m]
                yaml.dump({
                    'pulley_radius': self.PULLEY_RADIUS,
                    'motor_orient': self.MOTOR_ORIENT,
                    'increment_lim': self.INCREMENT_LIM,
                    }, file)
        except (yaml.YAMLError, TypeError):
            self.get_logger().error('Unable to read or write configuration file for startup')
            return

        self.last_gripper_joint_position = None

        self.gripper_command_increment_sub = self.create_subscription(
            Float64, 
            '~/command_increment', 
            self.gripper_command_increment_cb,
            10)
        self.gripper_command_increment_sub

        self.gripper_state_sub = self.create_subscription(
            JointState, 
            '/gripper_joint_state_broadcaster/joint_states', 
            self.gripper_state_cb, 
            10)
        self.gripper_state_sub

        self.gripper_command_increment_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_joint_position_controller/commands',
            10)

    def gripper_command_increment_cb(self, msg):
        command_limited = np.clip(msg.data, -self.INCREMENT_LIM, self.INCREMENT_LIM)
        if not np.array_equal(msg.data, command_limited):
            self.get_logger().info('Gripper command increment limit exceeded and was clipped')
        motor_command = Float64MultiArray()
        motor_command.data = np.array([self.last_gripper_joint_position + (command_limited * self.MOTOR_ORIENT) / self.PULLEY_RADIUS])
        self.gripper_command_increment_pub.publish(motor_command)

    def gripper_state_cb(self, msg):
        self.last_gripper_joint_position = msg.position[0]


def main(args=None):
    rclpy.init(args=args)
    helix_gripper_node = HelixGripperNode()
    rclpy.spin(helix_gripper_node)
    helix_gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

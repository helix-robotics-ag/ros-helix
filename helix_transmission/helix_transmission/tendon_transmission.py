import rclpy
from rclpy.node import Node
import numpy as np
import yaml

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class TendonTransmissionNode(Node):

    def __init__(self):
        super().__init__('tendon_transmission_node')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pulley_radius',rclpy.Parameter.Type.DOUBLE),
                ('motor_orients',rclpy.Parameter.Type.INTEGER_ARRAY),
                ('tendon_min_lim',rclpy.Parameter.Type.DOUBLE),
                ('tendon_max_lim',rclpy.Parameter.Type.DOUBLE),
                ('tendon_calib_file_path',rclpy.Parameter.Type.STRING)
            ])

        self.PULLEY_RADIUS = self.get_parameter('pulley_radius').value
        self.MOTOR_ORIENTS = np.array(self.get_parameter('motor_orients').value, dtype=np.float64)
        self.TENDON_LIMITS = np.array([self.get_parameter('tendon_min_lim').value,
                                       self.get_parameter('tendon_max_lim').value])
        self.MOTOR_OFFSETS = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        try:
            with open(self.get_parameter('tendon_calib_file_path').value, 'r') as file:
                data = yaml.safe_load(file)
                if 'motor_offsets' in data:
                    self.MOTOR_OFFSETS = data['motor_offsets']
                    self.get_logger().info('No offset data found in calibration file, offsets set to 0.')
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            self.get_logger().info('Failed to load calibration file, offsets set to 0.')

        self.tendon_command_sub = self.create_subscription(
            Float64MultiArray, 
            '/helix_arm_tendons/command', 
            self.tendon_command_cb,
            10)
        self.tendon_command_sub

        self.motor_state_sub = self.create_subscription(
            JointState, 
            '/motor_head_joint_state_broadcaster/joint_states', 
            self.motor_state_cb, 
            10)
        self.motor_state_sub

        self.tendon_command_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_head_joint_position_controller/commands',
            10)
        
        self.tendon_state_pub = self.create_publisher(
            JointState,
            '/helix_arm_tendons/tendon_states',
            10)

    def tendon_command_cb(self, msg):
        command = np.array(msg.data, dtype=np.float64)
        command_limited = np.clip(command, self.TENDON_LIMITS[0], self.TENDON_LIMITS[1])
        if not np.array_equal(command, command_limited):
            self.get_logger().info('Some tendon commands exceeded limits and were clipped.')
        motor_command = Float64MultiArray()
        motor_command.data = ((command_limited * self.MOTOR_ORIENTS) \
                               / self.PULLEY_RADIUS + self.MOTOR_OFFSETS).tolist()
        self.tendon_command_pub.publish(motor_command)

    def motor_state_cb(self, msg):
        tendon_state = JointState()
        motor_names = sorted(msg.name)
        motor_angs = [msg.position[msg.name.index(motor_name)] for motor_name in motor_names]
        motor_angvels = [msg.velocity[msg.name.index(motor_name)] for motor_name in motor_names]
        motor_currents = [msg.effort[msg.name.index(motor_name)] for motor_name in motor_names]
        tendon_state.name = motor_names
        tendon_state.position = ((np.array(motor_angs, dtype=np.float64) + self.MOTOR_OFFSETS) \
                                 * self.PULLEY_RADIUS * self.MOTOR_ORIENTS).tolist()
        tendon_state.velocity = (np.array(motor_angvels, dtype=np.float64) \
                                 * self.PULLEY_RADIUS * self.MOTOR_ORIENTS).tolist()
        tendon_state.effort = motor_currents
        self.tendon_state_pub.publish(tendon_state)


def main(args=None):
    rclpy.init(args=args)

    tendon_transmission_node = TendonTransmissionNode()
    rclpy.spin(tendon_transmission_node)

    tendon_transmission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class TendonTransmissionNode(Node):

    def __init__(self):
        super().__init__('tendon_transmission_node')
        
        self.PULLEY_RADIUS = 0.01  # [m]
        # For Motor 0,1,2,3,4,5 increasing rotor position/positive torque -> contraction
        # For Motor 6,7,8 decreasing rotor position/negative torque -> contraction 
        self.MOTOR_ORIENTS = np.array([1, 1, 1, 1, 1, 1, -1, -1, -1], dtype=np.float64)
        self.MOTOR_OFFSETS = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)

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
            '/motor_head_joint_position_controller/commands_test',
            10)
        
        self.tendon_state_pub = self.create_publisher(
            JointState,
            '/helix_arm_tendons/tendon_states',
            10)

    def tendon_command_cb(self, msg):
        motor_command = Float64MultiArray()
        motor_command.data = ((np.array(msg.data, dtype=np.float64) * self.MOTOR_ORIENTS) \
                               / self.PULLEY_RADIUS + self.MOTOR_OFFSETS).tolist()
        self.tendon_command_pub.publish(motor_command)

    def motor_state_cb(self, msg):
        tendon_state = JointState()
        motor_names = sorted(msg.name)
        motor_angs = [msg.position[msg.name.index(motor_name)] for motor_name in motor_names]
        motor_angvels = [msg.velocity[msg.name.index(motor_name)] for motor_name in motor_names]
        motor_currents = [msg.effort[msg.name.index(motor_name)] for motor_name in motor_names]
        tendon_state.name = motor_names
        tendon_state.position = ((np.array(motor_angs, dtype=np.float64) - self.MOTOR_OFFSETS) \
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

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import yaml

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController

class TendonTransmissionNode(Node):

    def __init__(self):
        super().__init__('tendon_transmission_node')
        
        # Config from helix_transmission.config.yml
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pulley_radius',rclpy.Parameter.Type.DOUBLE),
                ('motor_orients',rclpy.Parameter.Type.INTEGER_ARRAY),
                ('tendon_min_lim',rclpy.Parameter.Type.DOUBLE),
                ('tendon_max_lim',rclpy.Parameter.Type.DOUBLE),
                ('holding_current',rclpy.Parameter.Type.DOUBLE),
                ('tendon_calib_file_path',rclpy.Parameter.Type.STRING)
            ])
        self.PULLEY_RADIUS = self.get_parameter('pulley_radius').value
        self.MOTOR_ORIENTS = np.array(self.get_parameter('motor_orients').value, dtype=np.float64)
        self.TENDON_LIMITS = np.array([self.get_parameter('tendon_min_lim').value,
                                       self.get_parameter('tendon_max_lim').value])
        self.HOLDING_CURRENT = self.get_parameter('holding_current').value

        # Motor offsets from local tendon calibration file
        self.MOTOR_OFFSETS = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        try:
            with open(self.get_parameter('tendon_calib_file_path').value, 'r') as file:
                data = yaml.safe_load(file)
                if 'motor_offsets' in data:
                    self.MOTOR_OFFSETS = data['motor_offsets']
                else:
                    self.get_logger().info('No offset data found in calibration file, offsets set to 0.')
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            self.get_logger().info('Failed to load calibration file, offsets set to 0.')

        # Subscription/publication for motor<->tendon transmission
        self.tendon_command_sub = self.create_subscription(
            Float64MultiArray, 
            '/helix_arm_tendons/command', 
            self.tendon_to_motor_command_cb,
            10)
        self.tendon_command_sub

        self.motor_state_sub = self.create_subscription(
            JointState, 
            '/motor_head_joint_state_broadcaster/joint_states', 
            self.motor_state_cb, 
            10)
        self.motor_state_sub

        self.tendon_to_motor_command_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_head_joint_position_controller/commands',
            10)
        
        self.tendon_state_pub = self.create_publisher(
            JointState,
            '/helix_arm_tendons/tendon_states',
            10)
        
        # Publisher for motor current holding current
        self.motor_effort_command_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_head_joint_effort_controller/commands',
            10)
        
        # Client/service for setting holding current
        client_cb_group = MutuallyExclusiveCallbackGroup()
        self.controller_switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller', callback_group=client_cb_group)

        service_cb_group = MutuallyExclusiveCallbackGroup()
        self.set_holding_current_srv = self.create_service(
            Trigger, '~/set_holding_current_srv', self.set_holding_current_cb, callback_group=service_cb_group)

    # Callbacks for motor<->tendon transmission
    def tendon_to_motor_command_cb(self, msg):
        command = np.array(msg.data, dtype=np.float64)
        command_limited = np.clip(command, self.TENDON_LIMITS[0], self.TENDON_LIMITS[1])
        if not np.array_equal(command, command_limited):
            self.get_logger().info('Some tendon commands exceeded limits and were clipped.')
        motor_command = Float64MultiArray()
        motor_command.data = ((command_limited * self.MOTOR_ORIENTS) \
                               / self.PULLEY_RADIUS + self.MOTOR_OFFSETS).tolist()
        self.tendon_to_motor_command_pub.publish(motor_command)

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

    # Callback for setting holding current
    def set_holding_current_cb(self, request, response):
        while not self.controller_switch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller switch service')
        controller_switch_req = SwitchController.Request()
        controller_switch_req.activate_controllers = ['motor_head_joint_effort_controller']
        controller_switch_req.deactivate_controllers = ['motor_head_joint_position_controller']
        controller_switch_req.strictness = SwitchController.Request.STRICT
        controller_switch_future = self.controller_switch_cli.call_async(controller_switch_req)
        while self.executor.spin_until_future_complete(controller_switch_future):
            self.get_logger().info("Waiting for controller switch to complete")
        if controller_switch_future.result().ok == False:
            self.get_logger().error('Failed to switch to effort controller.')
            response.success = False
            response.message = 'Failed to switch to effort controller.'
            return response
        self.motor_effort_command_pub.publish(
            Float64MultiArray(data = self.HOLDING_CURRENT * self.MOTOR_ORIENTS))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    tendon_transmission_node = TendonTransmissionNode()

    # Unable to complete controller switch service call from within holding current service callback 
    # MultiThreadedExectuor solution based on https://answers.ros.org/question/412149/
    executor = MultiThreadedExecutor()
    rclpy.spin(tendon_transmission_node, executor=executor)

    tendon_transmission_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

from helix_transmission_interfaces.srv import SetCurrent

class TendonTransmissionNode(Node):

    def __init__(self):
        super().__init__('tendon_transmission_node')
        
        # Load saved robot configuration from host, or create default config
        self.path_to_config = '/tmp/config/helix_transmission.config.yml'
        self.path_to_calib = '/tmp/config/tendon_calib.yml'
        try:
            with open(self.path_to_config, 'r') as file:
                config = yaml.safe_load(file)
                self.PULLEY_RADIUS = config['pulley_radius']
                self.MOTOR_ORIENTS = np.array(config['motor_orients'], dtype=np.float64)
                self.TENDON_LIMITS = np.array([config['tendon_min_lim'],
                                               config['tendon_max_lim']])
                self.CURRENT_LIMITS = np.array([config['current_min_lim'],
                                                config['current_max_lim']])
        except (FileNotFoundError):
            self.get_logger().info('No configuration file found, setting defaults')
            with open(self.path_to_config, 'w') as file:
                self.PULLEY_RADIUS = 0.01  # [m]
                self.MOTOR_ORIENTS = np.array([1, 1, 1, 1, 1, 1, -1, -1, -1], dtype=np.float64)  # 1 for motor orientations where anticlockwise pulls the tendon
                self.TENDON_LIMITS = np.array([-0.1, 0.1], dtype=np.float64)  # [m] per  segment
                self.CURRENT_LIMITS = np.array([-300.0, 10.0], dtype=np.float64)  # [mA] per segment
                yaml.dump({
                    'pulley_radius': self.PULLEY_RADIUS,
                    'motor_orients': self.MOTOR_ORIENTS.tolist(),
                    'tendon_min_lim': float(self.TENDON_LIMITS[0]),
                    'tendon_max_lim': float(self.TENDON_LIMITS[1]),
                    'current_min_lim': float(self.CURRENT_LIMITS[0]),
                    'current_max_lim': float(self.CURRENT_LIMITS[1]),
                    }, file)
        except (yaml.YAMLError, TypeError):
            self.get_logger().error('Unable to read or write configuration file for startup')
            return

        # Motor offsets from tendon calibration file
        self.MOTOR_OFFSETS = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
        try:
            with open(self.path_to_calib, 'r') as file:
                data = yaml.safe_load(file)
                if 'motor_offsets' in data:
                    self.MOTOR_OFFSETS = data['motor_offsets']
                else:
                    self.get_logger().info('No offset data found in calibration file, offsets set to 0')
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            self.get_logger().info('Failed to load calibration file, offsets set to 0')

        self.last_motor_joint_positions = None

        # Subscription/publication for motor<->tendon transmission
        self.tendon_commands_sub = self.create_subscription(
            Float64MultiArray, 
            '~/commands', 
            self.tendon_to_motor_command_cb,
            10)
        self.tendon_commands_sub

        self.tendon_current_commands_sub = self.create_subscription(
            Float64MultiArray, 
            '~/current_commands', 
            self.tendon_to_motor_current_command_cb,
            10)
        self.tendon_current_commands_sub

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
            '~/tendon_states',
            10)
        
        # Publisher for motor current
        self.motor_effort_command_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_head_joint_effort_controller/commands',
            10)
        
        # Client for controller switching
        client_cb_group = MutuallyExclusiveCallbackGroup()

        self.controller_switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller', callback_group=client_cb_group)

        service_cb_group = MutuallyExclusiveCallbackGroup()

        # Services for current setting and calibration
        self.switch_to_current_control = self.create_service(
            Trigger, '~/switch_to_current_control', self.switch_to_current_control_cb, callback_group=service_cb_group)

        self.switch_to_position_control = self.create_service(
            Trigger, '~/switch_to_position_control', self.switch_to_position_control_cb, callback_group=service_cb_group)

        self.set_current_srv = self.create_service(
            SetCurrent, '~/set_current', self.set_current_cb, callback_group=service_cb_group)
        
        self.set_motor_offsets_srv = self.create_service(
            Trigger, '~/set_motor_offsets', self.set_motor_offsets_cb, callback_group=service_cb_group)
        
        self.check_calibration_srv = self.create_service(
            Trigger, '~/check_calibration', self.check_calibration_cb, callback_group=service_cb_group)

    # Callbacks for motor<->tendon transmission
    def tendon_to_motor_command_cb(self, msg):
        command = np.array(msg.data, dtype=np.float64)
        command_limited = np.hstack([np.clip(command[:3], self.TENDON_LIMITS[0], self.TENDON_LIMITS[1]),
                                     np.clip(command[3:], 2 * self.TENDON_LIMITS[0], self.TENDON_LIMITS[1])])
        if not np.array_equal(command, command_limited):
            self.get_logger().info('Some tendon commands exceeded limits and were clipped')
        motor_command = Float64MultiArray()
        motor_command.data = ((command_limited * self.MOTOR_ORIENTS) \
                               / self.PULLEY_RADIUS + self.MOTOR_OFFSETS).tolist()
        self.tendon_to_motor_command_pub.publish(motor_command)

    def tendon_to_motor_current_command_cb(self, msg):
        command = np.array(msg.data, dtype=np.float64)
        command_limited = np.hstack([np.clip(command[:3], self.CURRENT_LIMITS[0], self.CURRENT_LIMITS[1]),
                                     np.clip(command[3:], 2 * self.CURRENT_LIMITS[0], self.CURRENT_LIMITS[1])])
        if not np.array_equal(command, command_limited):
            self.get_logger().info('Some current commands exceeded limits and were clipped')
        motor_command = Float64MultiArray()
        motor_command.data = (command_limited * self.MOTOR_ORIENTS).tolist()
        self.motor_effort_command_pub.publish(motor_command)

    def motor_state_cb(self, msg):
        motor_names = sorted(msg.name)
        motor_angs = [msg.position[msg.name.index(motor_name)] for motor_name in motor_names]
        self.last_motor_joint_positions = np.array(motor_angs, dtype=np.float64)
        motor_angvels = [msg.velocity[msg.name.index(motor_name)] for motor_name in motor_names]
        motor_currents = [msg.effort[msg.name.index(motor_name)] for motor_name in motor_names]
        tendon_state = JointState()
        tendon_state.header.stamp = msg.header.stamp
        tendon_state.name = motor_names
        tendon_state.position = ((np.array(motor_angs, dtype=np.float64) - self.MOTOR_OFFSETS) \
                                 * self.PULLEY_RADIUS * self.MOTOR_ORIENTS).tolist()
        tendon_state.velocity = (np.array(motor_angvels, dtype=np.float64) \
                                 * self.PULLEY_RADIUS * self.MOTOR_ORIENTS).tolist()
        tendon_state.effort = motor_currents
        self.tendon_state_pub.publish(tendon_state)

    # Callbacks for controller switching
    def switch_to_current_control_cb(self, request, response):
        while not self.controller_switch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller switch service')
        controller_switch_req = SwitchController.Request()
        controller_switch_req.activate_controllers = ['motor_head_joint_effort_controller']
        controller_switch_req.deactivate_controllers = ['motor_head_joint_position_controller', 'gripper_joint_position_controller']
        controller_switch_req.strictness = SwitchController.Request.BEST_EFFORT
        controller_switch_future = self.controller_switch_cli.call_async(controller_switch_req)
        while self.executor.spin_until_future_complete(controller_switch_future):
            self.get_logger().info("Waiting for controller switch to complete")
        if controller_switch_future.result().ok == False:
            self.get_logger().error('Failed to switch to effort controller')
            response.success = False
            response.message = 'Failed to switch to effort controller'
            return response
        response.success = True
        return response

    def switch_to_position_control_cb(self, request, response):
        while not self.controller_switch_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller switch service')
        controller_switch_req = SwitchController.Request()
        controller_switch_req.activate_controllers = ['motor_head_joint_position_controller', 'gripper_joint_position_controller']
        controller_switch_req.deactivate_controllers = ['motor_head_joint_effort_controller']
        controller_switch_req.strictness = SwitchController.Request.BEST_EFFORT
        controller_switch_future = self.controller_switch_cli.call_async(controller_switch_req)
        while self.executor.spin_until_future_complete(controller_switch_future):
            self.get_logger().info("Waiting for controller switch to complete")
        if controller_switch_future.result().ok == False:
            self.get_logger().error('Failed to switch to position controller')
            response.success = False
            response.message = 'Failed to switch to position controller'
            return response
        response.success = True
        return response

    # Callbacks for calibration
    def set_current_cb(self, request, response):
        command_limited = np.clip(request.current, self.CURRENT_LIMITS[0], self.CURRENT_LIMITS[1])
        self.motor_effort_command_pub.publish(
            Float64MultiArray(data = command_limited * self.MOTOR_ORIENTS))
        response.success = True
        return response
    
    def set_motor_offsets_cb(self, request, response):
        write_succeeded = self.write_motor_offsets(self.last_motor_joint_positions.tolist())
        if write_succeeded:
            response.success = True
        else:
            response.success = False
            response.message = 'Failed to set motor offsets file'
        return response            
        
    def check_calibration_cb(self, request, response):
        recalibrated_offsets = self.MOTOR_OFFSETS \
            + 2*np.pi * np.round((self.last_motor_joint_positions-self.MOTOR_OFFSETS) / (2*np.pi))
        write_succeeded = self.write_motor_offsets(recalibrated_offsets.tolist())
        if write_succeeded:
            response.success = True
        else:
            response.success = False
            response.message = 'Failed to update motor offsets file'
        return response   
        
    def write_motor_offsets(self, new_offsets):
        try:
            with open(self.path_to_calib + '.bak', 'w') as backup:
                yaml.dump({'motor_offsets': self.MOTOR_OFFSETS}, backup)
            with open(self.path_to_calib, 'w') as file:
                self.MOTOR_OFFSETS = new_offsets
                yaml.dump({'motor_offsets': self.MOTOR_OFFSETS}, file)
            return True
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            self.get_logger().info('Failed to write motor offsets to file')
            return False


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

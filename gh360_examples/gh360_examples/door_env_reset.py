import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from gh360_interfaces.action import DoorEnvReset
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates, SetVelocity, PortStatus, MotorStatus
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep


class DoorEnvResetActionServer(Node):

    def __init__(self):
        super().__init__('door_env_reset_action_server')

        self.node.create_subscription(
            ArmEncoderStates,
            '/encoder_status',
            self.encoder_callback,
            10
        )

        self.node.create_subscription(
            PortStatus,
            '/shoulder/motor_status',
            self.motor_status_callback,
            10
        )
        self.node.create_subscription(
            PortStatus,
            '/upperarm/motor_status',
            self.motor_status_callback,
            10
        )
        self.node.create_subscription(
            PortStatus,
            '/lowerarm/motor_status',
            self.motor_status_callback,
            10
        )

        self.node.create_subscription(
            PortStatus,
            '/door/motor_status',
            self.hinge_callback,
            10
        )

        self.hinge_state = MotorStatus()
        self.motor_states = []
        for i in range(13):
            self.motor_states.append(MotorStatus())


        self.client_shoulder = self.node.create_client(MotorPositionStep, '/shoulder/motor_positions_step')
        while not self.client_shoulder.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_upperarm = self.node.create_client(MotorPositionStep, '/upperarm/motor_positions_step')
        while not self.client_upperarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_lowerarm = self.node.create_client(MotorPositionStep, '/lowerarm/motor_positions_step')
        while not self.client_lowerarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        self._action_server = ActionServer(
            self,
            DoorEnvReset,
            '/door_env_reset',
            self.execute_callback)
        
    def motor_status_callback(self, msg):
        for motor_state in msg.motors:
            self.motor_states[motor_state.motor_id] = motor_state

    def hinge_callback(self, msg):
        self.hinge_state = msg.motors[0]

    def encoder_callback(self, msg):
        self.encoder_states = msg.current_joint_states

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = DoorEnvReset.Feedback()
        # feedback_msg.partial_sequence = [0, 1]

        # for i in range(1, goal_handle.request.order):
        #     feedback_msg.partial_sequence.append(
        #         feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)

        goal_handle.succeed()

        result = DoorEnvReset.Result()
        # result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    door_env_reset_action_server = DoorEnvResetActionServer()

    rclpy.spin(door_env_reset_action_server)


if __name__ == '__main__':
    main()
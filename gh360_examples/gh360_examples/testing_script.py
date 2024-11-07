import time
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from gh360_interfaces.action import DoorEnvReset
from gh360_interfaces.msg import SetMotorVelocities, SetPosition, ArmEncoderStates, SetVelocity, PortStatus, MotorStatus
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep


class TestingNode(Node):

    def __init__(self):
        super().__init__('testing_node')

        # self.create_subscription(
        #     ArmEncoderStates,
        #     '/encoder_status',
        #     self.encoder_callback,
        #     10
        # )

        # self.node.create_subscription(
        #     PortStatus,
        #     '/shoulder/motor_status',
        #     self.motor_status_callback,
        #     10
        # )
        self.create_subscription(
            PortStatus,
            '/upperarm/motor_status',
            self.motor_status_callback,
            10
        )
        # self.node.create_subscription(
        #     PortStatus,
        #     '/lowerarm/motor_status',
        #     self.motor_status_callback,
        #     10
        # )

        self.right_target_position = 6.28
        self.left_target_position = 6.28

        self.right_motor_position = 0.0
        self.left_motor_position = 0.0
        self.right_motor_velocity = 0.0
        self.left_motor_velocity = 0.0

        self.max_velocity = 0.5
        self.kp = 1.0
        self.kd = 2 * np.sqrt(self.kp)

        self.right_motor_goal = SetVelocity()
        self.right_motor_goal.id = 9
        self.right_motor_goal.velocity = 0.0
        
        self.left_motor_goal = SetVelocity()
        self.left_motor_goal.id = 10
        self.left_motor_goal.velocity = 0.0

        self.pub_goal_velocity_upperarm = self.create_publisher(SetMotorVelocities, '/upperarm/motor_goal_velocity', 10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.right_motor_position == 0.0 or self.left_motor_position == 0.0:
            return
        
        right_pos_error = (self.right_target_position - self.right_motor_position)*self.kp #- self.right_motor_velocity*self.kd
        left_pos_error = (self.left_target_position - self.left_motor_position)*self.kp #- self.left_motor_velocity*self.kd
        # if self.right_target_position >= 0.0:
        #     if self.right_target_position > self.right_motor_position:
        #         right_pos_error = 10
        #     else:
        #         right_pos_error = 0
        # elif self.right_target_position < 0.0:
        #     if self.right_target_position < self.right_motor_position:
        #         right_pos_error = -10
        #     else:
        #         right_pos_error = 0
        
        # if self.left_target_position >= 0.0:
        #     if self.left_target_position > self.left_motor_position:
        #         left_pos_error = 10
        #     else:
        #         left_pos_error = 0
        # elif self.left_target_position < 0.0:
        #     if self.left_target_position < self.left_motor_position:
        #         left_pos_error = -10
        #     else:
        #         left_pos_error = 0
        

        motor_goal_velocities = SetMotorVelocities()
        self.right_motor_goal.velocity = np.clip(right_pos_error, -self.max_velocity, self.max_velocity)
        self.left_motor_goal.velocity = np.clip(left_pos_error, -self.max_velocity, self.max_velocity)
        motor_goal_velocities.motor_goal_velocities.append(self.right_motor_goal)
        motor_goal_velocities.motor_goal_velocities.append(self.left_motor_goal)

        self.pub_goal_velocity_upperarm.publish(motor_goal_velocities)
        
    def motor_status_callback(self, msg):
        for motor_state in msg.motors:
            if motor_state.motor_id == self.right_motor_goal.id:
                self.right_motor_position = motor_state.present_position
                self.right_motor_velocity = motor_state.present_velocity
            elif motor_state.motor_id == self.left_motor_goal.id:
                self.left_motor_position = motor_state.present_position
                self.left_motor_velocity = motor_state.present_velocity



def main(args=None):
    rclpy.init(args=args)

    testing_node = TestingNode()

    rclpy.spin(testing_node)


if __name__ == '__main__':
    main()
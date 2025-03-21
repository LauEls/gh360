import time
import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from gh360_interfaces.action import DoorEnvReset
from gh360_interfaces.msg import SetMotorVelocities, SetPosition, ArmEncoderStates, SetVelocity, PortStatus, MotorStatus
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from sensor_msgs.msg import JointState


class TestingNode(Node):

    def __init__(self):
        super().__init__('testing_node')

        self.joint_pos_msg = JointState()


        self.joint_pos_msg.name = ['shoulder_yaw', 'shoulder_roll', 'shoulder_pitch', 'upperarm_roll', 'elbow', 'forearm_roll', 'wrist_pitch']
        self.joint_pos_msg.position = [0.0204, -0.1854, 0.1467, 1.5825, 1.8675, 0.0, 0.0]

        self.joint_pos_publisher = self.create_publisher(JointState, '/gh360_control/cmd_joint_pos', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.joint_pos_publisher.publish(self.joint_pos_msg)



def main(args=None):
    rclpy.init(args=args)

    testing_node = TestingNode()

    rclpy.spin(testing_node)


if __name__ == '__main__':
    main()
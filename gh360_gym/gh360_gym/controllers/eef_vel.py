from gh360_gym.controllers.base import BaseController
from gh360_gym.utils.joints import MotorJoint, SoftJoint
from gh360_gym.utils.motor_interfaces import generate_velocities_msg, generate_positions_msg
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates, SetVelocity, PortStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

import numpy as np
import time

import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class EEFVelocityController(BaseController):
    def __init__(self, node, input_max=[], input_min=[], max_current=[], min_current=[], max_joint_pos=[], min_joint_pos=[]):
        super().__init__(node, max_joint_pos, min_joint_pos, max_current, min_current)

        print("Super class initialized")
        self.cmd_eef_vel_publisher = self.node.create_publisher(Twist, '/gh360_control/cmd_eef_vel', 10)
        
        self.control_dim = 6
        # print("control dimensions: ", self.control_dim)

        # self.input_max = np.ones(self.control_dim)
        # self.input_min = -np.ones(self.control_dim)
        if len(input_max) == self.control_dim:
            self.max_multiplier = input_max
        else:
            self.max_multiplier = np.ones(self.control_dim)
        if len(input_min) == self.control_dim:
            self.min_multiplier = input_min
        else:
            self.min_multiplier = -np.ones(self.control_dim)

        self.input_max = np.ones(self.control_dim)
        self.input_min = -np.ones(self.control_dim)

        self.last_time = 0

        self.reseted = False

        print("EEFVelocityController initialized")
    
    def set_step_goal(self, action):
        if self.last_time == 0:
            self.last_time = time.time()

        action = np.clip(action, self.input_min, self.input_max)
        for i in range(self.control_dim):
            if action[i] > 0.0:
                action[i] *= self.max_multiplier[i]
            else: 
                action[i] *= self.min_multiplier[i]
                
        eef_goal_vel = Twist()
        eef_goal_vel.linear.x = float(action[0])
        eef_goal_vel.linear.y = float(action[1])
        eef_goal_vel.linear.z = float(action[2])
        eef_goal_vel.angular.x = float(action[3])
        eef_goal_vel.angular.y = float(action[4])
        eef_goal_vel.angular.z = float(action[5])

        t_control_loop = self.publish_step_goal(self.cmd_eef_vel_publisher, eef_goal_vel)

        # while (time.time() - self.last_time) < (self.control_timestep-self.control_time_adj):
        #     self.cmd_eef_vel_publisher.publish(eef_goal_vel)
        #     rclpy.spin_once(self.node)

        #     if not self.robot_safety_check():
        #             # wait for user_input
        #             self.set_motor_torque(False)
        #             input("Press Enter to continue...")
        #             self.set_motor_torque(True)

        # t_control_loop = time.time() - self.last_time
        # self.last_time = time.time()
        return t_control_loop
from gh360_gym.controllers.base import BaseController
from gh360_gym.utils.joints import MotorJoint, SoftJoint
from gh360_gym.utils.motor_interfaces import generate_velocities_msg, generate_positions_msg
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates, SetVelocity, PortStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

import numpy as np
import time

import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MotorPositionController(BaseController):
    def __init__(self, node, input_max=[], input_min=[], max_current=[], max_joint_pos=[], min_joint_pos=[]):
        super().__init__(node, max_joint_pos, min_joint_pos, max_current)

        self.cmd_motor_pos_publisher = self.node.create_publisher(Float64MultiArray, '/gh360_control/cmd_motor_pos', 10)
        

        self.control_dim = self.motor_cnt
        print("control dimensions: ", self.control_dim)

        self.input_max = np.ones(self.control_dim)
        self.input_min = -np.ones(self.control_dim)
        if len(input_max) == self.control_dim:
            self.max_multiplier = input_max
        else:
            self.max_multiplier = np.ones(self.control_dim)
        if len(input_min) == self.control_dim:
            self.min_multiplier = input_min
        else:
            self.min_multiplier = -np.ones(self.control_dim)

        self.last_time = 0
        self.motor_goal_pos_msg = Float64MultiArray()

    
    def set_step_goal(self, action):
        while (time.time() - self.last_time) < (self.control_timestep-self.control_time_adj):
            self.motor_goal_pos_msg.data = action
            self.cmd_motor_pos_publisher.publish(self.motor_goal_pos_msg)

            rclpy.spin_once(self.node)

            if not self.robot_safety_check():
                    # wait for user_input
                    self.set_motor_torque(False)
                    input("Press Enter to continue...")
                    self.set_motor_torque(True)

        t_control_loop = time.time() - self.last_time
        self.last_time = time.time()
        return t_control_loop

        
    def set_goal_trajectory(self, goal_trajectory):
        for goal in goal_trajectory:
            while not self.motor_pos_goal_reached(goal, 0.2, False):
                self.motor_goal_pos_msg.data = goal
                self.cmd_motor_pos_publisher.publish(self.motor_goal_pos_msg)
                rclpy.spin_once(self.node)

        while not self.motor_pos_goal_reached(goal_trajectory[-1], 0.1):
            self.motor_goal_pos_msg.data = goal_trajectory[-1]
            self.cmd_motor_pos_publisher.publish(self.motor_goal_pos_msg)
            rclpy.spin_once(self.node)

    def motor_pos_goal_reached(self, motor_pos_goal, accuracy, velocity_check=True):
        pos_reached = True
        moving = False

        for i, motor in enumerate(self.motors):
            if abs(motor_pos_goal[i] - motor.present_position) > accuracy:
                pos_reached = False
            if motor.present_velocity != 0.0:
                moving = True

        if pos_reached:  
            if velocity_check and not moving:
                return True
            elif not velocity_check:
                return True
       
        return False


            
            
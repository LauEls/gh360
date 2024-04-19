import sys
import os
import gym
import numpy as np
import time
import csv
from gym import spaces

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt16, Float64
from ros2pkg.api import get_prefix_path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
# from dynamixel_sdk_custom_interfaces.msg import SetPosition
# from DynamixelSDK.dynamixel_sdk_custom_interfaces.msg import SetPosition
# sys.path.append('/home/laurenz/phd_project/ros2_gh360_ws/src/DynamixelSDK/dynamixel_sdk_custom_interfaces.msg')
# from dynamixel_sdk_custom_interfaces.msg import SetPosition
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates, SetVelocity, PortStatus
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep

from gh360_gym.utils.joints import SoftJoint, MotorJoint
from gh360_gym.utils.motor_interfaces import generate_velocities_msg, generate_positions_msg
from gh360_gym.controllers.eq_point import EqPointController


class TrajectoryFollowingEnv(gym.Env):
    def __init__(self,
                 input_max=1,
                 input_min=-1,
                 stiffness_mode = "variable",
                 motor_obs = False,
                 vel_obs = True,
                 ):
        """
        Have a variable the defines the action size

        """
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        self.motor_obs = motor_obs
        self.vel_obs = vel_obs
        print("motor obs: ", self.motor_obs)
        print("vel obs: ", self.vel_obs)
        self.stiffness_mode = stiffness_mode
        
        self.controller = EqPointController(self.node, stiffness_mode=self.stiffness_mode, input_min=input_min, input_max=input_max)
        
        # self.motor_msg = SetMotorPositions()
        self.internal_state = 0

        self.action_dim = self.controller.control_dim
        high = np.ones(self.action_dim)
        low = -high  
        self.action_space = spaces.Box(low, high)

        self.obs_dim = 27
        if self.motor_obs:
            self.obs_dim += 12
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        # file_base_dir = '/home/laurenz/phd_project/sac/scripts/test_data/v6'
        # self.motor_pos_file = os.path.join(file_base_dir, 'motor_pos.csv')
        # self.motor_vel_file = os.path.join(file_base_dir, 'motor_vel.csv')
        # self.joint_pos_file = os.path.join(file_base_dir, 'joint_pos.csv')

    def _get_obs(self):
        """
        Retrieve the following observations:
        Joint Angles
        Eef-Pose (requires having a kinematic model)

        OrderedDict:
            robot0_joint_pos
            robot0_joint_pos_cos
            robot0_joint_pos_sin
            robot0_joint_vel
            robot0_eef_pos
            robot0_eef_quat

            robot0_gripper_qpos (empty for hook)
            robot0_gripper_qvel
            
            door_pos
            handle_pos
            door_to_eef_pos
            handle_to_eef_pos
            hinge_qpos
            handle_qpos
            robot0_proprio-state (all the robot related data from above combined in one array) -> this + object state concatonated is what goes into the network
            object-state (all the object related data from above comined in one array)
        """
        obs = []
        controller_obs = self.controller.get_obs()
        # print("controller obs: ", controller_obs)
        
        self.robot_eef_pos = controller_obs["robot_eef_pos"]
        self.via_point_pos = [-0.31, 0.46, 1.22]
        # door_to_eef_pos = door_pos - robot_eef_pos
        via_point_to_eef_pos = self.via_point_pos - self.robot_eef_pos
        self.gripper_to_via_point = via_point_to_eef_pos
        # print("via_point: ", self.via_point_pos)
        # print("gripper_to_via_point: ", self.gripper_to_via_point)
        
        obs = np.concatenate((self.via_point_pos, self.gripper_to_via_point, controller_obs["robot_joint_pos"], controller_obs["robot_eef_pos"], controller_obs["robot_eef_quat"]), axis=-1)
        if self.motor_obs:
            obs = np.insert(obs, 13, controller_obs["motor_pos"])
        if self.vel_obs:
            obs = np.insert(obs, 13, controller_obs["robot_joint_vel"])    
        return obs

    def _get_info(self):
        """
        Maybe motor load signals
        """
        return {"internal_state":self.internal_state}

    def reset(self):
        self.controller.reset()

        obs = self._get_obs()
        info = self._get_info()
        # print("finished reset")
        self.reseted = True
        return obs
    
    def step(self, action):
        """
        Send action to the arm controller

        """
        self.controller.set_motor_goal(action)
        
        observation = self._get_obs()
        reward = self.reward()
        info = self._get_info()
        # done = reward >= 0.99
        done = False

        return observation, reward, done, info

    def check_success(self):
        return (self.robot_eef_pos == self.via_point_pos).all()

    def reward(self):
        """
        Compute the reward signal
        """
        reward = 0.0

        if self.check_success():
            reward = 1.0
        else:
            dist = np.linalg.norm(self.gripper_to_via_point)
            reaching_reward = 0.25 * (1 - np.tanh(10.0 * dist))
            reward += reaching_reward

        return reward

    def render(self):
        """
        Currently no rendering implemented since this environment is on the real robot.
        In the future a rendering of the camera view could be implemented
        """
        pass

    def close(self):
        pass

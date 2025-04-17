import sys
import os
import gym
import numpy as np
import time
import csv
from gym import spaces

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt16, Float64, Bool
from std_srvs.srv import SetBool
from ros2pkg.api import get_prefix_path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import tf2_geometry_msgs
from gh360_interfaces.msg import PortStatus, SetMotorVelocities, SetMotorCurrents, SetCurrent, SetVelocity, MotorStatus
from gh360_interfaces.msg import DoorEnv as DoorEnvMsg

from gh360_gym.utils.joints import SoftJoint, MotorJoint
from gh360_gym.controllers.eq_point import EqPointController
from gh360_gym.controllers.motor_vel import MotorVelocityController
from gh360_gym.controllers.eef_vel import EEFVelocityController
from gh360_gym.controllers.motor_pos import MotorPositionController
from gh360_gym.controllers.joint_pos import JointPositionController


class DoorEnv(gym.Env):

    def __init__(self,
                 input_max=[],
                 input_min=[],
                 max_joint_pos=[],
                 min_joint_pos=[],
                 max_motor_current=[],
                 min_motor_current=[],
                 stiffness_mode = "variable",
                 motor_obs = False,
                 node = None,
                 ):
        """
        Have a variable the defines the action size

        """
        # try:
        #     rclpy.init(args=None)
        # except:
        #     pass
        # self.node = rclpy.create_node(self.__class__.__name__)

        if node is None:
            rclpy.init(args=None)
            self.node = rclpy.create_node(self.__class__.__name__)
        else:
            self.node = node

        self.node.get_logger().info("Initializing Door Environment")    
        

        self.motor_obs = motor_obs
        # print("motor obs: ", self.motor_obs)
        self.stiffness_mode = stiffness_mode
        

        self.first_eef_msg = False
        self.first_door_msg = False
        self.reseted = False

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.eef_pos = np.array([0.0, 0.0, 0.0])
        self.handle_pos = np.array([0.0, 0.0, 0.0])
        self.handle_qpos = np.array([0.0])
        self.hinge_qpos = np.array([0.0])
        self.motor_pos = []

        self.step_cntr = 0

        # self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        # # self.via_point_pos = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        # self.via_point_pos_1 = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 7.5, 7.5, 0.0, 1.0, 1.0]
        # self.via_point_pos_2 = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        self.robot_reset_pos = [0.02, -0.19, 0.15, 1.58, 1.87, 0.0, 0.0]
        self.via_point_pos_1 = [0.3383, -0.1071, -0.0041, 1.6654, 1.1585, 0.2132, 0.6659]
        self.via_point_pos_2 = [0.0729, -0.1325, 0.1064, 1.9791, 1.9488, -0.1779, -0.0942]
        self.via_point_pos_3 = [0.02, -0.19, 0.15, 1.58, 1.87, 0.0, 1.4]
        self.robot_standby_pos = [-0.38, -0.05, -0.016, 1.12, 1.94, 0.08, 0.46]
        self.standby_pos = False
        self.reset_pos = False

        # self.controller = EqPointController(self.node, op_mode=self.stiffness_mode)
        # self.controller = EqPointController(self.node, stiffness_mode=self.stiffness_mode, input_min=input_min, input_max=input_max)
        # self.controller = MotorVelocityController(self.node, input_min=input_min, input_max=input_max, max_current=max_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)
        self.controller = EEFVelocityController(self.node, input_min=input_min, input_max=input_max, max_motor_current=max_motor_current, min_motor_current=min_motor_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)
        # self.reset_controller = MotorPositionController(self.node, input_min=input_min, input_max=input_max, max_current=max_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)
        print("EEFVelocityController initialized")
        self.reset_controller = JointPositionController(self.node)
        print("JointPositionController initialized")
        self.control_dim = self.controller.control_dim
        print("control_dim: ", self.control_dim)

        self.node.create_subscription(
            Pose,
            '/gh360/eef_pose',
            self.eef_pose_callback,
            10
        )

        self.node.create_subscription(
            DoorEnvMsg,
            '/door/environment_observations',
            self.door_env_callback,
            10
        )

        # self.node.create_subscription(
        #     PortStatus,
        #     '/door/motor_status',
        #     self.motor_status_callback,
        #     10
        # )

        # self.door_motor_current_publisher = self.node.create_publisher(SetMotorCurrents, '/door/motor_goal_current', 10)
        # self.door_motor_vel_publisher = self.node.create_publisher(SetMotorVelocities, '/door/motor_goal_velocity', 10)
        self.door_reset_publisher = self.node.create_publisher(Bool, '/door/reset', 10)

        # self.client_door_motor_torque = self.node.create_client(SetBool, '/door/motor_set_torque')
        # while not self.client_door_motor_torque.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('service not available, waiting again...')

        # self.goal_current_msg = SetMotorCurrents()
        # self.set_current_msg = SetCurrent()
        # self.set_current_msg.id = 31
        # self.set_current_msg.current = -500.0
        # self.goal_current_msg.motor_goal_currents.append(self.set_current_msg)

        # self.goal_velocity_msg = SetMotorVelocities()
        # self.set_velocity_msg = SetVelocity()
        # self.set_velocity_msg.id = 31
        # self.set_velocity_msg.velocity = 0.0
        # self.goal_velocity_msg.motor_goal_velocities.append(self.set_velocity_msg)

        # self.first_door_motor_status = False
        # self.door_motor_status = MotorStatus()
        # self.closing_start_pos = 2.2
        # self.closing_current = -600.0

        # self.motor_msg = SetMotorPositions()
        # self.internal_state = 0

        self.action_dim = self.control_dim
        high = np.ones(self.action_dim)
        # high = high * input_max
        print(f"high: {high}")
        low = -high  
        self.action_space = spaces.Box(low, high)

        self.obs_dim = 23
        if self.motor_obs:
            self.obs_dim += 26
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        while not self.first_eef_msg or not self.first_door_msg:
            rclpy.spin_once(self.node)

    def eef_pose_callback(self, msg):
        self.eef_pos = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
        self.eef_quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=np.float64)
        if not self.first_eef_msg:
            self.first_eef_msg = True

    def door_env_callback(self, msg):
        self.handle_pos = np.array([msg.handle_position.x, msg.handle_position.y, msg.handle_position.z], dtype=np.float64)
        self.handle_qpos = np.array([msg.handle_angle], dtype=np.float64)
        self.hinge_qpos = np.array([msg.hinge_angle], dtype=np.float64)
        # self.node.get_logger().info(f"handle pos: {self.handle_pos}")
        if not self.first_door_msg:
            self.first_door_msg = True

    def _get_obs(self):
        """
        Retrieve the following observations:
        Joint Angles
        Eef-Pose (requires having a kinematic model)

        OrderedDict:
            robot0_joint_pos
            robot0_joint_vel
            robot0_motor_pos
            robot0_motor_vel
            handle_to_eef_pos
            robot0_eef_quat
            
            door_pos
            handle_pos
            door_to_eef_pos
            
            hinge_qpos
            handle_qpos
            robot0_proprio-state (all the robot related data from above combined in one array) -> this + object state concatonated is what goes into the network
            object-state (all the object related data from above comined in one array)
        """
        obs = []
        robot_joint_pos = []
        robot_joint_vel = []
        robot_motor_pos = []
        robot_motor_vel = []

        self.handle_to_eef_pos = self.handle_pos - self.eef_pos
        
        for joint in self.controller.joints:
            robot_joint_pos.append(joint.joint_angle)
            robot_joint_vel.append(joint.joint_velocity)

        for motor in self.controller.motors:
            robot_motor_pos.append(motor.present_position)
            robot_motor_vel.append(motor.present_velocity)
            
        robot_motor_pos = np.array(robot_motor_pos)
        robot_motor_vel = np.array(robot_motor_vel)
        
        if self.motor_obs:
            obs = np.concatenate((self.handle_qpos, self.hinge_qpos, self.handle_to_eef_pos, self.eef_quat, robot_joint_pos, robot_joint_vel, robot_motor_pos, robot_motor_vel), axis=-1)
        else:
            obs = np.concatenate((self.handle_qpos, self.hinge_qpos, self.handle_to_eef_pos, self.eef_quat, robot_joint_pos, robot_joint_vel), axis=-1)
        # print(obs)
        return obs

    def _get_info(self):
        """
        Maybe motor load signals
        """
        return {}
    
    def door_reset(self):
        if not self.reset_controller.robot_safety_check():
            self.reset_controller.stop_robot(True)
            input("Press Enter to continue...")
            self.reset_controller.stop_robot(False)
        # if self.reset_pos:
        #     self.reset_controller.set_goal_trajectory([self.robot_standby_pos])
        # else:
        #     self.reset_controller.set_goal_trajectory([self.via_point_pos_3, self.robot_standby_pos])
        stuck_cntr = 0
        while not self.reset_controller.set_goal_trajectory([self.robot_standby_pos]) and stuck_cntr < 3:
            stuck_cntr += 1

        self.standby_pos = True
        self.reset_pos  = False
        
        while not self.first_door_msg:
            self.node.get_logger().info("Waiting to get motor status...")
            rclpy.spin_once(self.node)

        door_closed = False

        while not door_closed:
            self.door_reset_publisher.publish(Bool(data=True))
            rclpy.spin_once(self.node)
            if self.handle_qpos[0] < 0.0 and self.hinge_qpos[0] < 0.01:
                door_closed = True
        
        return
    
    def robot_reset(self):
        # if self.reseted:
        #     return
        
        reset_trajectory = []

        if not self.reset_controller.robot_safety_check():
            self.reset_controller.stop_robot(True)
            input("Press Enter to continue...")
            self.reset_controller.stop_robot(False)

        # reset_trajectory = [self.robot_reset_pos]
        stuck_cntr = 0
        if self.standby_pos:
            reset_trajectory = [self.robot_reset_pos]
            self.standby_pos = False
        elif self.handle_pos[2] < self.eef_pos[2]-0.0052 or self.handle_qpos > 0.1:
            reset_trajectory = [self.via_point_pos_3, self.robot_reset_pos]
        else:
            reset_trajectory = [self.via_point_pos_1, self.via_point_pos_2, self.robot_reset_pos]

        # if not self.reset_controller.set_goal_trajectory(reset_trajectory): return False
        while not self.reset_controller.set_goal_trajectory(reset_trajectory) and stuck_cntr < 3:
            if self.standby_pos:
                reset_trajectory = [self.robot_reset_pos]
                self.standby_pos = False
            elif self.handle_pos[2] < self.eef_pos[2]-0.0052 or self.handle_qpos > 0.1:
                reset_trajectory = [self.via_point_pos_3, self.robot_reset_pos]
            else:
                reset_trajectory = [self.via_point_pos_1, self.via_point_pos_2, self.robot_reset_pos]
            stuck_cntr += 1

        self.reset_pos = True
        self.controller.last_time = 0
        time_sum = 0
        for i in range(20):
            zero_action = np.zeros(self.controller.control_dim)
            time_sum += self.controller.set_step_goal(zero_action)

        self.controller.control_time_adj = (time_sum/20) - (self.controller.control_timestep)
        self.controller.last_time = 0
        self.reseted = True
        

        return True
        


    def reset(self):
        # print("resetting")
        # self.controller.reset(robot_eef_pos=self.eef_pos, handle_pos=self.handle_pos, handle_qpos=self.handle_qpos)
        self.node.get_logger().info("Resetting Door Environment")
        reset_success = False
        if not self.reseted:
            for _ in range(10):
                rclpy.spin_once(self.node)
            if self.hinge_qpos > 0.04:
                self.door_reset()

            reset_success = self.robot_reset()

            if self.hinge_qpos > 0.04:
                self.door_reset()
                self.robot_reset()
            
        # self.handle_pos = self.get_handle_pos()

        obs = self._get_obs()
        info = self._get_info()
        info["reset_success"] = reset_success
        # print("finished reset")
        self.reseted = True
        self.node.get_logger().info("Resetting Door Environment Finished")
        return obs, info

    def step(self, action):
        """
        Send action to the arm controller

        """
        if self.reseted:
            # print("step: ", self.step_cntr)
            self.step_cntr = 0
        
        self.reset_pos = False
        self.reseted = False
        self.step_cntr += 1
        
        t_loop = self.controller.set_step_goal(action)
        # print("t_loop: ", t_loop)

        observation = self._get_obs()
        reward = self.reward()
        info = self._get_info()
        # done = reward >= 0.99
        done = False

        return observation, reward, done, info

    def check_success(self):
        return self.hinge_qpos >= 0.4 and self.handle_qpos <= 0.1

    def reward(self):
        """
        Compute the reward signal
        """
        reward = 0.0

        if self.check_success():
            reward = 1.0
        else:
            dist = np.linalg.norm(self.handle_to_eef_pos)
            # reaching_reward = 1.0 * (1 - np.tanh(10.0 * dist))
            # reward += 1.0*reaching_reward

            reaching_reward = 0.25 * (1 - np.tanh(10.0 * dist))
            reward += reaching_reward

            if self.handle_qpos > 0.1:
                reward = 0.25
                reward += np.clip(0.25 * np.abs(self.handle_qpos / 0.6), 0, 0.25)
            
            if self.hinge_qpos > 0.1:
                reward = 0.5
                reward += np.clip(0.25 * np.abs(self.hinge_qpos / 0.4), 0, 0.25)

            # handle_qpos = self.sim.data.qpos[self.handle_qpos_addr]
            #reward += np.clip(0.25 * np.abs(self.handle_qpos / 0.54), 0, 0.25)

        return float(reward)

    def render(self):
        """
        Currently no rendering implemented since this environment is on the real robot.
        In the future a rendering of the camera view could be implemented
        """
        pass

    def close(self):
        pass


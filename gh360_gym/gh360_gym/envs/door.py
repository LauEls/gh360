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
from sensor_msgs.msg import JointState
import tf2_geometry_msgs
from gh360_interfaces.msg import PortStatus
from gh360_interfaces.msg import DoorEnv as DoorEnvMsg

from gh360_gym.utils.joints import SoftJoint, MotorJoint
from gh360_gym.controllers.eq_point import EqPointController
from gh360_gym.controllers.motor_vel import MotorVelocityController


class DoorEnv(gym.Env):

    def __init__(self,
                 input_max=[],
                 input_min=[],
                 max_joint_pos=[],
                 min_joint_pos=[],
                 max_current=[],
                 stiffness_mode = "variable",
                 motor_obs = False,
                 ):
        """
        Have a variable the defines the action size

        """
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        self.motor_obs = motor_obs
        print("motor obs: ", self.motor_obs)
        self.stiffness_mode = stiffness_mode
        self.control_dim = 7

        # if self.stiffness_mode == "variable":
        #     self.control_dim = 13#MAYBE READ THAT OUT OF A CONFIG FILE -> should be 13 at the end
        # elif self.stiffness_mode == "fixed" or stiffness_mode == "no_stiffness":
        #     self.control_dim = 7

        # print("control dimensions: ", self.control_dim)

        # self.control_timestep = 0.2
        # self.model_timestep = 0.1
        # self.reseted = False
        # print("control dimensions: ",self.control_dim)

        # input and output max and min (allow for either explicit lists or single numbers)
        # self.input_max = self.nums2array(input_max, self.control_dim)
        # self.input_max = np.ones(self.control_dim) * input_max
        # # self.input_min = self.nums2array(input_min, self.control_dim)
        # self.input_min = np.ones(self.control_dim) * input_min

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.handle_pos = np.array([0.0, 0.0, 0.0])
        self.handle_qpos = np.array([0.0])
        self.hinge_qpos = np.array([0.0])
        self.motor_pos = []

        self.step_cntr = 0

        # self.controller = EqPointController(self.node, op_mode=self.stiffness_mode)
        # self.controller = EqPointController(self.node, stiffness_mode=self.stiffness_mode, input_min=input_min, input_max=input_max)
        self.controller = MotorVelocityController(self.node, input_min=input_min, input_max=input_max, max_current=max_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)

        # self.node.create_subscription(
        #     Float64,
        #     '/door/filtered_handle_angle',
        #     self.handle_callback,
        #     10
        # )

        self.node.create_subscription(
            Pose,
            '/eef_pose',
            self.eef_pose_callback,
            10
        )

        self.node.create_subscription(
            DoorEnvMsg,
            '/door_env',
            self.door_env_callback,
            10
        )

        # self.node.create_subscription(
        #     PortStatus,
        #     '/door/motor_status',
        #     self.hinge_callback,
        #     10
        # )


        # self.motor_msg = SetMotorPositions()
        self.internal_state = 0

        self.action_dim = self.control_dim
        high = np.ones(self.action_dim)
        # high = high * input_max
        print(f"high: {high}")
        low = -high  
        self.action_space = spaces.Box(low, high)

        self.obs_dim = 21
        if self.motor_obs:
            self.obs_dim += 26
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

    # def handle_callback(self, msg):
    #     self.handle_qpos = np.clip(np.array([msg.data], dtype=np.float64),0.0,np.pi/2)

    def eef_pose_callback(self, msg):
        self.eef_pos = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
        self.eef_quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=np.float64)

    def door_env_callback(self, msg):
        self.handle_pos = np.array([msg.handle_position.x, msg.handle_position.y, msg.handle_position.z], dtype=np.float64)
        self.handle_qpos = np.array([msg.handle_angle], dtype=np.float64)
        self.hinge_qpos = np.array([msg.hinge_angle], dtype=np.float64)
        # self.node.get_logger().info(f"handle pos: {self.handle_pos}")

    # def hinge_callback(self, msg):
    #     motor_pos = msg.motors[0].present_position
    #     offset = 3.2505
    #     max_pos = 3.548 - offset
    #     hinge_angle_multi = (17.1887*np.pi/180) / max_pos

    #     self.hinge_qpos =   (motor_pos - offset) * hinge_angle_multi
    
    # def get_handle_pos(self):
    #     to_frame_rel = 'base_link'
    #     handle_frames = ['handle_1', 'handle_3']
    #     handle_pos = []

    #     for handle_frame in handle_frames:
    #         try:
    #             handle_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, handle_frame, rclpy.time.Time())
    #             handle_pos.append([handle_pose_trans.transform.translation.x, handle_pose_trans.transform.translation.y, handle_pose_trans.transform.translation.z])
    #         except TransformException as ex:
    #             self.node.get_logger().info(f'Could not transform {handle_frame} to {to_frame_rel}: {ex}')
                
        
    #     avg_handle_pos = np.mean(handle_pos, axis=0)
    #     # self.node.get_logger().info(f"handle pos: {avg_handle_pos}")
    #     return avg_handle_pos


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
        # robot_eef_pos = []
        # robot_eef_quat = []


        #from sim: door_pos = [-0.2300001, 0.46023886, 1.08]
        # door_pos = [-0.31, 0.46, 1.08]
        # #in sim: handle_pos = [-0.14994089, 0.40230259, 1.0534252]
        # handle_pos = [-0.15, 0.4, 1.05]
        

        #ROBOT SPECIFIC PARAMETERS
        for joint in self.controller.arm:
            # if type(joint) == MotorJoint:
            #     print("MotorJoint")
            robot_joint_pos.append(joint.joint_angle)
            robot_joint_vel.append(joint.joint_velocity)

        #CALCUALATE FORWARD KINEMATICS TO GET EEF POS AND QUAT
        from_frame_rel = 'eef'
        to_frame_rel = 'base_link'
        #Lookup the tranformation from from_frame_rel to to_frame_rel
        try:
            eef_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        #Tranform a Pose from from_frame_rel to to_frame_rel
        # eef_pose = tf2_geometry_msgs.do_transform_pose(Pose(), self.trans_eef_base)
        robot_eef_pos = np.array([eef_pose_trans.transform.translation.x, eef_pose_trans.transform.translation.y, eef_pose_trans.transform.translation.z], dtype=np.float64)
        robot_eef_quat = np.array([eef_pose_trans.transform.rotation.x, eef_pose_trans.transform.rotation.y, eef_pose_trans.transform.rotation.z, eef_pose_trans.transform.rotation.w], dtype=np.float64)

        self.handle_to_eef_pos = self.handle_pos - robot_eef_pos

        # hinge_qpos = np.array([0.0])
        # handle_qpos = np.array([0.0])
        # np.copyto(handle_qpos, self.handle_qpos)
        # np.copyto(hinge_qpos, self.hinge_qpos)

        
        for joint in self.controller.arm:
            if type(joint) == SoftJoint:
                robot_motor_pos.append(joint.right_motor_pos)
                robot_motor_pos.append(joint.left_motor_pos)
                robot_motor_vel.append(joint.right_motor_vel)
                robot_motor_vel.append(joint.left_motor_vel)
            elif type(joint) == MotorJoint:
                robot_motor_pos.append(joint.motor_pos)
                robot_motor_vel.append(joint.motor_vel)

        robot_motor_pos = np.array(robot_motor_pos)
        robot_motor_vel = np.array(robot_motor_vel)

        # controller_obs = self.controller.get_obs()
        # print("controller obs: ", controller_obs)
        
        if self.motor_obs:
            obs = np.concatenate((self.handle_to_eef_pos, robot_eef_quat, robot_joint_pos, robot_joint_vel, robot_motor_pos, robot_motor_vel), axis=-1)
        else:
            obs = np.concatenate((self.handle_to_eef_pos, robot_eef_quat, robot_joint_pos, robot_joint_vel), axis=-1)
        # print(obs)
        return obs

    def _get_info(self):
        """
        Maybe motor load signals
        """
        return {}
    

    def reset(self):
        print("resetting")
        self.controller.reset()

        # self.handle_pos = self.get_handle_pos()

        obs = self._get_obs()
        info = self._get_info()
        # print("finished reset")
        self.reseted = True
        return obs

    def step(self, action):
        """
        Send action to the arm controller

        """
        if self.reseted:
            print("step: ", self.step_cntr)
            self.step_cntr = 0
        
        self.reseted = False
        self.step_cntr += 1
        
        t_loop = self.controller.set_motor_goal(action)
        # print("t_loop: ", t_loop)

        observation = self._get_obs()
        reward = self.reward()
        info = self._get_info()
        # done = reward >= 0.99
        done = False

        return observation, reward, done, info

    def check_success(self):
        # hinge_qpos = self.sim.data.qpos[self.hinge_qpos_addr]
        # return self.hinge_qpos < -0.3
        #print("hinge qpos: "+str(self.hinge_qpos))
        # return np.abs(self.hinge_qpos) > 0.3
        return False

    def reward(self):
        """
        Compute the reward signal
        """
        # Handle Pos
        # Eef Pos
        # handle q pos
        # hinge q pos
        # if possible touch door handle
        # obs = self._get_obs()

        reward = 0.0

        if self.check_success():
            reward = 1.0
        else:
            dist = np.linalg.norm(self.handle_to_eef_pos)
            reaching_reward = 1.0 * (1 - np.tanh(10.0 * dist))
            reward += 1.0*reaching_reward


            # handle_qpos = self.sim.data.qpos[self.handle_qpos_addr]
            #reward += np.clip(0.25 * np.abs(self.handle_qpos / 0.54), 0, 0.25)

        # reward = np.tanh(obs)
        return reward

    def render(self):
        """
        Currently no rendering implemented since this environment is on the real robot.
        In the future a rendering of the camera view could be implemented
        """
        pass

    def close(self):
        pass


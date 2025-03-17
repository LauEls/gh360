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


class DoorEnv(gym.Env):

    def __init__(self,
                 input_max=[],
                 input_min=[],
                 max_joint_pos=[],
                 min_joint_pos=[],
                 max_current=[],
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
        

        self.motor_obs = motor_obs
        # print("motor obs: ", self.motor_obs)
        self.stiffness_mode = stiffness_mode
        

        self.first_eef_msg = False
        self.first_door_msg = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.eef_pos = np.array([0.0, 0.0, 0.0])
        self.handle_pos = np.array([0.0, 0.0, 0.0])
        self.handle_qpos = np.array([0.0])
        self.hinge_qpos = np.array([0.0])
        self.motor_pos = []

        self.step_cntr = 0

        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        # self.via_point_pos = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        self.via_point_pos_1 = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 7.5, 7.5, 0.0, 1.0, 1.0]
        self.via_point_pos_2 = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]

        # self.controller = EqPointController(self.node, op_mode=self.stiffness_mode)
        # self.controller = EqPointController(self.node, stiffness_mode=self.stiffness_mode, input_min=input_min, input_max=input_max)
        # self.controller = MotorVelocityController(self.node, input_min=input_min, input_max=input_max, max_current=max_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)
        self.controller = EEFVelocityController(self.node, input_min=input_min, input_max=input_max, max_current=max_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)
        self.reset_controller = MotorPositionController(self.node, input_min=input_min, input_max=input_max, max_current=max_current, max_joint_pos=max_joint_pos, min_joint_pos=min_joint_pos)
        self.control_dim = self.controller.control_dim

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

        self.node.create_subscription(
            PortStatus,
            '/door/motor_status',
            self.motor_status_callback,
            10
        )

        self.door_motor_current_publisher = self.node.create_publisher(SetMotorCurrents, '/door/motor_goal_current', 10)
        self.door_motor_vel_publisher = self.node.create_publisher(SetMotorVelocities, '/door/motor_goal_velocity', 10)

        self.client_door_motor_torque = self.node.create_client(SetBool, '/door/motor_set_torque')
        while not self.client_door_motor_torque.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        self.goal_current_msg = SetMotorCurrents()
        self.set_current_msg = SetCurrent()
        self.set_current_msg.id = 31
        self.set_current_msg.current = -500.0
        self.goal_current_msg.motor_goal_currents.append(self.set_current_msg)

        self.goal_velocity_msg = SetMotorVelocities()
        self.set_velocity_msg = SetVelocity()
        self.set_velocity_msg.id = 31
        self.set_velocity_msg.velocity = 0.0
        self.goal_velocity_msg.motor_goal_velocities.append(self.set_velocity_msg)

        self.first_door_motor_status = False
        self.door_motor_status = MotorStatus()
        self.closing_start_pos = 2.2
        self.closing_current = -600.0

        # self.motor_msg = SetMotorPositions()
        self.internal_state = 0

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



    # def handle_callback(self, msg):
    #     self.handle_qpos = np.clip(np.array([msg.data], dtype=np.float64),0.0,np.pi/2)

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

    def motor_status_callback(self, msg):
        if not self.first_door_motor_status:
            self.first_door_motor_status = True
        self.door_motor_status = msg.motors[0]

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
       
        

        #ROBOT SPECIFIC PARAMETERS
        # for joint in self.controller.arm:
            # if type(joint) == MotorJoint:
            #     print("MotorJoint")
            

        # #CALCUALATE FORWARD KINEMATICS TO GET EEF POS AND QUAT
        # from_frame_rel = 'eef'
        # to_frame_rel = 'base_link'
        # #Lookup the tranformation from from_frame_rel to to_frame_rel
        # try:
        #     eef_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        # except TransformException as ex:
        #     self.node.get_logger().info(
        #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return
        # #Tranform a Pose from from_frame_rel to to_frame_rel
        # # eef_pose = tf2_geometry_msgs.do_transform_pose(Pose(), self.trans_eef_base)
        # robot_eef_pos = np.array([eef_pose_trans.transform.translation.x, eef_pose_trans.transform.translation.y, eef_pose_trans.transform.translation.z], dtype=np.float64)
        # robot_eef_quat = np.array([eef_pose_trans.transform.rotation.x, eef_pose_trans.transform.rotation.y, eef_pose_trans.transform.rotation.z, eef_pose_trans.transform.rotation.w], dtype=np.float64)

        self.handle_to_eef_pos = self.handle_pos - self.eef_pos

        # hinge_qpos = np.array([0.0])
        # handle_qpos = np.array([0.0])
        # np.copyto(handle_qpos, self.handle_qpos)
        # np.copyto(hinge_qpos, self.hinge_qpos)

        
        for joint in self.controller.joints:
            robot_joint_pos.append(joint.joint_angle)
            robot_joint_vel.append(joint.joint_velocity)
            # if type(joint) == SoftJoint:
            #     robot_motor_pos.append(joint.right_motor_pos)
            #     robot_motor_pos.append(joint.left_motor_pos)
            #     robot_motor_vel.append(joint.right_motor_vel)
            #     robot_motor_vel.append(joint.left_motor_vel)
            # elif type(joint) == MotorJoint:
            #     robot_motor_pos.append(joint.motor_pos)
            #     robot_motor_vel.append(joint.motor_vel)

        for motor in self.controller.motors:
            robot_motor_pos.append(motor.motor_pos)
            robot_motor_vel.append(motor.motor_vel)
            
        robot_motor_pos = np.array(robot_motor_pos)
        robot_motor_vel = np.array(robot_motor_vel)

        # controller_obs = self.controller.get_obs()
        # print("controller obs: ", controller_obs)
        
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
        while not self.first_door_motor_status:
            self.node.get_logger().info("Waiting to get motor status...")
            rclpy.spin_once(self.node)

        status = 0
        
        # self.client_motor_torque.call_async(SetBool.Request(data=True))
        while status != 4:
            if status == 0: 
                if self.door_motor_status.present_position < 1.81:
                    return
                    # status = 3
                    # self.goal_current_msg.motor_goal_currents[0].current = 0.0
                    # self.node.get_logger().info("Changing to Satus 3")
                elif self.door_motor_status.present_position >= self.closing_start_pos:
                    status = 2
                    future = self.client_door_motor_torque.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(self.node, future)
                    self.goal_current_msg.motor_goal_currents[0].current = self.closing_current
                    self.node.get_logger().info("Changing to Satus 2")
                else:
                    status = 1    
                    future = self.client_door_motor_torque.call_async(SetBool.Request(data=True))
                    rclpy.spin_until_future_complete(self.node, future)
                    self.node.get_logger().info("Changing to Satus 1")
            elif status == 1 and self.door_motor_status.present_velocity <= 0.0 and self.door_motor_status.present_position >= self.closing_start_pos-0.1:
                status = 2
                self.goal_current_msg.motor_goal_currents[0].current = self.closing_current
                self.node.get_logger().info("Changing to Satus 2")
            elif status == 2 and self.door_motor_status.present_velocity < 0.0:
                status = 3
                self.node.get_logger().info("Changing to Satus 3")
            elif status == 3 and self.door_motor_status.present_velocity >= 0.0:
                status = 4
                self.goal_current_msg.motor_goal_currents[0].current = 0.0
                self.client_door_motor_torque.call_async(SetBool.Request(data=False))
                self.node.get_logger().info("Changing to Satus 4")

            # self.node.get_logger().info("Motor Current: " + str(self.goal_current_msg.motor_goal_currents[0].current))
            if status == 1:
                self.goal_velocity_msg.motor_goal_velocities[0].velocity = np.clip(self.closing_start_pos - self.door_motor_status.present_position, 0.0, 0.5)
                # if self.motor_status.present_position >= self.closing_start_pos-0.1:
                #     self.goal_velocity_msg.motor_goal_velocities[0].velocity = 0.0
                self.door_motor_vel_publisher.publish(self.goal_velocity_msg)
                # self.pos_pusblisher_.publish(self.door_opening_msg)
            else:
                self.door_motor_current_publisher.publish(self.goal_current_msg)

            rclpy.spin_once(self.node)
        
        return
    
    def robot_reset(self):
        if self.reseted:
            return
        
        # internal_state = 0
        reset_trajectory = []

        for _ in range(10):
            rclpy.spin_once(self.node)

        if not self.reset_controller.robot_safety_check():
            self.reset_controller.stop_robot(True)
            input("Press Enter to continue...")
            self.reset_controller.stop_robot(False)

        if self.handle_pos[2] > self.eef_pos[2]+0.01 and self.handle_qpos < 0.1:
            # internal_state = 2
            # pos_error = self.calc_motor_pos_error(self.robot_reset_pos)
            # pos_goal = self.robot_reset_pos
            reset_trajectory = [self.robot_reset_pos]
        else:
            # pos_error = self.calc_motor_pos_error(self.via_point_pos_1)
            # pos_goal = self.via_point_pos_1
            reset_trajectory = [self.via_point_pos_1, self.via_point_pos_2, self.robot_reset_pos]

        self.reset_controller.set_goal_trajectory(reset_trajectory)

        # while not self.goal_pos_reached() or internal_state != 2:
        #     if not self.robot_safety_check():
        #         self.set_motor_torque(False)
        #         input("Press Enter to continue...")
        #         self.set_motor_torque(True)
                
        #     if internal_state < 2 and self.goal_pos_reached():
        #         internal_state += 1
        #     if internal_state == 0:
        #         # pos_error = self.calc_motor_pos_error(self.via_point_pos_1)
        #         pos_goal = self.via_point_pos_1
        #     elif internal_state == 1:
        #         # pos_error = self.calc_motor_pos_error(self.via_point_pos_2)
        #         pos_goal = self.via_point_pos_2
        #     elif internal_state == 2:
        #         # pos_error = self.calc_motor_pos_error(self.robot_reset_pos)
        #         pos_goal = self.robot_reset_pos
        #     # pos_error = np.clip(pos_error, -1.0, 1.0)

        #     # motor_vel_msg = generate_velocities_msg(self.arm, pos_error)
        #     motor_pos_msg = generate_positions_msg(self.arm, pos_goal)
        #     # self.pub_goal_velocity_shoulder.publish(motor_vel_msg)
        #     # self.pub_goal_velocity_upperarm.publish(motor_vel_msg)
        #     # self.pub_goal_velocity_lowerarm.publish(motor_vel_msg)
        #     self.pub_motor_goal_position.publish(motor_pos_msg)
        #     rclpy.spin_once(self.node)

        # print("final internal state: ", internal_state)

        
        self.control_time_adj = 0.0
        self.last_time = 0
        time_sum = 0
        for i in range(20):
            zero_action = np.zeros(self.controller.control_dim)
            time_sum += self.controller.set_step_goal(zero_action)

        self.controller.control_time_adj = (time_sum/20) - (self.control_timestep)
        self.last_time = 0
        self.reseted = True
        

        return
        time.sleep(1)
        


    def reset(self):
        # print("resetting")
        # self.controller.reset(robot_eef_pos=self.eef_pos, handle_pos=self.handle_pos, handle_qpos=self.handle_qpos)

        if not self.reseted:
            self.robot_reset()
            self.door_reset()
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
            # print("step: ", self.step_cntr)
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
        return self.hinge_qpos >= 0.4 and self.handle_qpos <= 0.1
        # return False

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

        # reward = np.tanh(obs)
        # print("hinge qpos: ", self.hinge_qpos)
        # print("reward: ", reward)
        
        # return reward
        return float(reward)

    def render(self):
        """
        Currently no rendering implemented since this environment is on the real robot.
        In the future a rendering of the camera view could be implemented
        """
        pass

    def close(self):
        pass


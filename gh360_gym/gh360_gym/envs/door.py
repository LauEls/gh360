import sys
import os
import gym
import numpy as np
import time
import csv
from gym import spaces

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt16
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

from gh360_gym.envs.utils import SoftJoint, MotorJoint


class DoorEnv(gym.Env):

    def __init__(self,
                 input_max=1,
                 input_min=-1,
                 stiffness_mode = "variable",
                 ):
        """
        Have a variable the defines the action size

        """
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # self.motor_publisher = self.node.create_publisher(SetPosition, '/set_position', 10)
        # self.motor_publisher = self.node.create_publisher(SetMotorPositions, "/lowerarm/set_motor_positions", 10)

        self.stiffness_mode = stiffness_mode

        if self.stiffness_mode == "variable":
            self.control_dim = 13#MAYBE READ THAT OUT OF A CONFIG FILE -> should be 13 at the end
        elif self.stiffness_mode == "fixed" or stiffness_mode == "no_stiffness":
            self.control_dim = 7

        print("control dimensions: ", self.control_dim)

        self.control_timestep = 0.2
        self.model_timestep = 0.1
        self.reseted = False
        # print("control dimensions: ",self.control_dim)

        # input and output max and min (allow for either explicit lists or single numbers)
        # self.input_max = self.nums2array(input_max, self.control_dim)
        self.input_max = np.ones(self.control_dim) * input_max
        # self.input_min = self.nums2array(input_min, self.control_dim)
        self.input_min = np.ones(self.control_dim) * input_min

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.handle_qpos = np.array([0.0])
        self.hinge_qpos = np.array([0.0])

        # self.node.create_subscriber()

        self.node.create_subscription(
            ArmEncoderStates,
            '/encoder_status',
            self.encoder_callback,
            10
        )
        self.node.create_subscription(
            UInt16,
            '/handle_angle',
            self.handle_callback,
            10
        )
        self.node.create_subscription(
            PortStatus,
            '/door/motor_status',
            self.hinge_callback,
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

        self.client_delta_shoulder = self.node.create_client(MotorPositionStep, '/shoulder/motor_delta_positions_step')
        while not self.client_delta_shoulder.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_delta_upperarm = self.node.create_client(MotorPositionStep, '/upperarm/motor_delta_positions_step')
        while not self.client_delta_upperarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_delta_lowerarm = self.node.create_client(MotorPositionStep, '/lowerarm/motor_delta_positions_step')
        while not self.client_delta_lowerarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        self.client_shoulder = self.node.create_client(MotorPositionStep, '/shoulder/motor_positions_step')
        while not self.client_shoulder.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_upperarm = self.node.create_client(MotorPositionStep, '/upperarm/motor_positions_step')
        while not self.client_upperarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_lowerarm = self.node.create_client(MotorPositionStep, '/lowerarm/motor_positions_step')
        while not self.client_lowerarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')

        self.client_velocity_shoulder = self.node.create_client(MotorVelocityStep, '/shoulder/motor_velocities_step')
        while not self.client_velocity_shoulder.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_velocity_upperarm = self.node.create_client(MotorVelocityStep, '/upperarm/motor_velocities_step')
        while not self.client_velocity_upperarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_velocity_lowerarm = self.node.create_client(MotorVelocityStep, '/lowerarm/motor_velocities_step')
        while not self.client_velocity_lowerarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        

        self.motor_msg = SetMotorPositions()
        self.internal_state = 0

        self.action_dim = self.control_dim
        high = np.ones(self.action_dim)
        low = -high  
        self.action_space = spaces.Box(low, high)

        self.obs_dim = 32
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

        self.arm = []
        new_joint = SoftJoint(joint_name="shoulder_yaw", port_name="shoulder", id_right_motor=1, id_left_motor=2, max_pos=1.0, min_pos=-1.0)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="shoulder_roll", port_name="shoulder", id_right_motor=3, id_left_motor=4, max_pos=0.5, min_pos=-0.5)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="shoulder_pitch", port_name="shoulder", id_right_motor=5, id_left_motor=6, max_pos=0.5, min_pos=0.0)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="upperarm_roll", port_name="upperarm", id_right_motor=7, id_left_motor=8, max_pos=2.0, min_pos=0.0)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="elbow", port_name="upperarm", id_right_motor=10, id_left_motor=9, max_pos=1.7, min_pos=0.0)
        self.arm.append(new_joint)
        new_joint = MotorJoint(joint_name="lowerarm_roll", port_name="lowerarm", id_motor=11, max_pos=np.pi/2, min_pos=-np.pi/2)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="wrist_pitch", port_name="lowerarm", id_right_motor=13, id_left_motor=12, max_pos=1.4, min_pos=-1.4)
        self.arm.append(new_joint)

        file_base_dir = '/home/laurenz/phd_project/sac/scripts/test_data/v6'
        self.motor_pos_file = os.path.join(file_base_dir, 'motor_pos.csv')
        self.motor_vel_file = os.path.join(file_base_dir, 'motor_vel.csv')
        self.joint_pos_file = os.path.join(file_base_dir, 'joint_pos.csv')
        
    def encoder_callback(self, msg):
        # print("recieved encoder message")
        for joint_msg in msg.current_joint_states:
            for joint in self.arm:
                if joint.joint_name == joint_msg.joint_name:
                    joint.joint_velocity = joint_msg.current_vel
                    joint.joint_angle = joint_msg.current_pos
                    # gui_joint.joint_angle.config(text="Joint Angle: "+self.get_label_str(joint.current_pos))

    def handle_callback(self, msg):
        offset = 0.0
        self.handle_qpos = np.array([(msg.data-offset)*(5.236/1023)], dtype=np.float64)

    def hinge_callback(self, msg):
        motor_pos = msg.motors[0].present_position
        offset = 3.2505
        max_pos = 3.548 - offset
        hinge_angle_multi = 17.1887 / max_pos

        self.hinge_qpos =   (motor_pos - offset) * hinge_angle_multi * np.pi/180

        # print("Hinge qpos: "+str(self.hinge_qpos))
    
    def motor_status_callback(self, msg):
        # print("recieved message!")
        for motor in msg.motors:
            for joint in self.arm:
                if type(joint) == SoftJoint:
                    if joint.id_right_motor == motor.motor_id:
                        joint.right_motor_safety_check = motor.safety_check
                        joint.right_motor_moving = motor.moving
                    elif joint.id_left_motor == motor.motor_id:
                        joint.left_motor_safety_check = motor.safety_check
                        joint.left_motor_moving = motor.moving
                elif type(joint) == MotorJoint:
                    if joint.id_motor == motor.motor_id:
                        joint.motor_safety_check = motor.safety_check
                        joint.motor_moving = motor.moving

    def safe_to_file(self):
        arm_status = np.concatenate((self.shoulder_motor_states_msg.motor_status, self.upperarm_motor_states_msg.motor_status, self.lowerarm_motor_states_msg.motor_status), axis=None)
        for motor_state in arm_status:
            for joint in self.arm:
                if type(joint) == SoftJoint:
                    if motor_state.motor_id == joint.id_right_motor:
                        joint.right_motor_pos = motor_state.present_position
                    elif motor_state.motor_id == joint.id_left_motor:
                        joint.left_motor_pos = motor_state.present_position
                else:
                    if motor_state.motor_id == joint.id_motor:
                        joint.joint_angle = motor_state.present_position


        timestamp = time.time()

        joint_pos = []
        motor_pos = []

        for joint in self.arm:
            joint_pos.append(joint.joint_angle)
            if type(joint) == SoftJoint:
                motor_pos.append(joint.right_motor_pos)
                motor_pos.append(joint.left_motor_pos)
            else:
                motor_pos.append(joint.joint_angle)



        joint_pos_write = np.concatenate((timestamp, joint_pos), axis=None)
        motor_pos_write = np.concatenate((timestamp, motor_pos), axis=None)
        # robot_state = [timestamp, self.joint_pos, self.ee_pos]
        # robot_state = np.insert(robot_state, 0, timestamp)

        f = open(self.joint_pos_file, 'a')
        data_writer = csv.writer(f)
        data_writer.writerow(joint_pos_write)
        f.close()

        f = open(self.motor_pos_file, 'a')
        data_writer = csv.writer(f)
        data_writer.writerow(motor_pos_write)
        f.close()

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
        robot_joint_pos = []
        robot_joint_pos_cos = []
        robot_joint_pos_sin = []
        robot_joint_vel = []
        # robot_eef_pos = []
        # robot_eef_quat = []

        robot_gripper_qpos = []
        robot_gripper_qvel = []

        door_pos = [-0.271, 0.411, 0.908]
        handle_pos = [-0.2161, 0.4061, 0.88251]
        

        #ROBOT SPECIFIC PARAMETERS
        for joint in self.arm:
            # if type(joint) == MotorJoint:
            #     print("MotorJoint")
            robot_joint_pos.append(joint.joint_angle)
            robot_joint_pos_cos.append(np.cos(joint.joint_angle))
            robot_joint_pos_sin.append(np.sin(joint.joint_angle))
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


        #ENVIRONMENT SPECIFIC PARAMETERS
        door_pos = [-0.271, 0.411, 0.908]
        handle_pos = [-0.2161, 0.4061, 0.88251]
        # door_to_eef_pos = door_pos - robot_eef_pos
        handle_to_eef_pos = handle_pos - robot_eef_pos
        self.gripper_to_handle = handle_to_eef_pos

        hinge_qpos = np.array([0.0])
        handle_qpos = np.array([0.0])
        # np.copyto(handle_qpos, self.handle_qpos)
        # np.copyto(hinge_qpos, self.hinge_qpos)


        # obs = np.array(np.float32(self.internal_state/100))
        obs = np.concatenate((robot_joint_pos, robot_joint_vel, robot_eef_pos, robot_eef_quat, door_pos, handle_pos, handle_to_eef_pos, hinge_qpos, handle_qpos), axis=-1)
        # print(obs)
        return obs

    def _get_info(self):
        """
        Maybe motor load signals
        """
        return {"internal_state":self.internal_state}
    
    def robot_moving_check(self):
        moving = False
        for joint in self.arm:
            if type(joint) == SoftJoint:
                if joint.right_motor_moving or joint.left_motor_moving:
                    moving = True
            elif type(joint) == MotorJoint:
                if joint.motor_moving:
                    moving = True

        return moving
    
    def robot_safety_check(self):
        safety_check = True
        for joint in self.arm:
            if type(joint) == SoftJoint:
                if not joint.right_motor_safety_check or not joint.left_motor_safety_check:
                    safety_check = True
            elif type(joint) == MotorJoint:
                if not joint.motor_safety_check:
                    safety_check = True

        return safety_check

    def reset(self):
        if self.reseted:
            obs = self._get_obs()
            return obs
        
        if not self.robot_safety_check():
            # wait for user_input
            input("Press Enter to continue...")
            pass

        self.internal_state = 0

        # action = [0.0, 0.0, 4.0, 2.5, 6.28, 0.0, 0.0]
        action = [0.0, 0.7854, 4.0, 2.5, 6.28, 0.0, 0.0]

        motor_pos_req = MotorPositionStep.Request()

        for i in range(len(self.arm)):
            motor_pos = action[i]

            if type(self.arm[i]) == SoftJoint:
                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[i].id_right_motor
                set_motor_msg.position = motor_pos

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[i].id_left_motor
                set_motor_msg.position = motor_pos

                motor_pos_req.motor_goal_positions.append(set_motor_msg)
            else:
                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[i].id_motor
                set_motor_msg.position = motor_pos

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

        shoulder_future = self.client_shoulder.call_async(motor_pos_req)
        upperarm_future = self.client_upperarm.call_async(motor_pos_req)
        lowerarm_future = self.client_lowerarm.call_async(motor_pos_req)

        rclpy.spin_until_future_complete(self.node, shoulder_future)
        rclpy.spin_until_future_complete(self.node, upperarm_future)
        rclpy.spin_until_future_complete(self.node, lowerarm_future)
        shoulder_motor_states_msg = shoulder_future.result()
        upperarm_motor_states_msg = upperarm_future.result()
        lowerarm_motor_states_msg = lowerarm_future.result()

        # time.sleep(6)
        # start_time = time.time()
        # while time.time() - start_time < 6:
        #     rclpy.spin_once(self.node)
        #     print(self.robot_moving_check())
            # rclpy.spin_once(self.node)
        
        start_time = time.time()
        while time.time() - start_time < 1:
            rclpy.spin_once(self.node)
            # print(self.robot_moving_check())

        # rclpy.spin_once(self.node)
        while self.robot_moving_check():
            rclpy.spin_once(self.node)

        action = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        motor_vel_req = MotorVelocityStep.Request()

        for i in range(len(self.arm)):
            motor_vel = action[i]

            if type(self.arm[i]) == SoftJoint:
                set_motor_msg = SetVelocity()
                set_motor_msg.id = self.arm[i].id_right_motor
                set_motor_msg.velocity = motor_vel

                motor_vel_req.motor_goal_velocities.append(set_motor_msg)

                set_motor_msg = SetVelocity()
                set_motor_msg.id = self.arm[i].id_left_motor
                set_motor_msg.velocity = motor_vel

                motor_vel_req.motor_goal_velocities.append(set_motor_msg)
            else:
                set_motor_msg = SetVelocity()
                set_motor_msg.id = self.arm[i].id_motor
                set_motor_msg.velocity = motor_vel

                motor_vel_req.motor_goal_velocities.append(set_motor_msg)

        shoulder_velocity_future = self.client_velocity_shoulder.call_async(motor_vel_req)
        upperarm_velocity_future = self.client_velocity_upperarm.call_async(motor_vel_req)
        lowerarm_velocity_future = self.client_velocity_lowerarm.call_async(motor_vel_req)

        rclpy.spin_until_future_complete(self.node, shoulder_velocity_future)
        rclpy.spin_until_future_complete(self.node, upperarm_velocity_future)
        rclpy.spin_until_future_complete(self.node, lowerarm_velocity_future)
        shoulder_motor_states_msg = shoulder_velocity_future.result()
        upperarm_motor_states_msg = upperarm_velocity_future.result()
        lowerarm_motor_states_msg = lowerarm_velocity_future.result()

        time.sleep(1)

        obs = self._get_obs()
        info = self._get_info()
        print("finished reset")
        self.reseted = True
        return obs
    
    def set_eq_motor_goal_position(self, delta_action):
        """
        Translate action to motor movements for equilibrium point control
        """
        assert len(delta_action) == self.control_dim, "Delta torque must be equal to the robot's joint dimension space!"

        delta_action = np.clip(delta_action, self.input_min, self.input_max)
        delta_action = delta_action/10

        joint_iter = 0
        motor_iter = 0

        motor_pos_req = MotorPositionStep.Request()

        while motor_iter < len(delta_action):
            if type(self.arm[joint_iter]) == SoftJoint:
                delta_eq_pos = delta_action[motor_iter]
                delta_stiffness = delta_action[motor_iter+1]

                # delta_motor_right = delta_eq_pos + delta_stiffness
                # delta_motor_left = delta_eq_pos - delta_stiffness

                delta_motor_right = delta_eq_pos
                delta_motor_left = delta_eq_pos

                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[joint_iter].id_right_motor
                set_motor_msg.position = delta_motor_right

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[joint_iter].id_left_motor
                set_motor_msg.position = delta_motor_left

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                motor_iter += 2
            else:
                delta_motor_pos = delta_action[motor_iter]



                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[joint_iter].id_motor
                set_motor_msg.position = delta_motor_pos

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                motor_iter += 1

            joint_iter += 1

        
        return motor_pos_req

    def set_eq_motor_goal_velocity(self, delta_action):
        """
        Translate action to motor movements for equilibrium point control
        """
        assert len(delta_action) == self.control_dim, "Delta torque must be equal to the robot's joint dimension space!"

        delta_action = np.clip(delta_action, self.input_min, self.input_max)
        delta_action = delta_action/10

        joint_iter = 0
        motor_iter = 0

        motor_vel_req = MotorVelocityStep.Request()

        while motor_iter < len(delta_action):
            if type(self.arm[joint_iter]) == SoftJoint:
                delta_eq_pos = delta_action[motor_iter]

                if delta_eq_pos > 0 and self.arm[joint_iter].joint_angle >= self.arm[joint_iter].max_pos:
                    # print("max joint pos reached")
                    delta_eq_pos = 0.0
                elif delta_eq_pos < 0 and self.arm[joint_iter].joint_angle <= self.arm[joint_iter].min_pos:
                    # print("min joint pos reached")
                    delta_eq_pos = 0.0

                if self.control_dim == 13:
                    delta_stiffness = delta_action[motor_iter+1]
                    motor_iter += 2
                else:
                    delta_stiffness = 0
                    motor_iter += 1

                delta_motor_right = delta_eq_pos + delta_stiffness
                delta_motor_left = delta_eq_pos - delta_stiffness



                motor_right_velocity = delta_motor_right / self.control_timestep
                motor_left_velocity = delta_motor_left / self.control_timestep

                set_motor_msg = SetVelocity()
                set_motor_msg.id = self.arm[joint_iter].id_right_motor
                set_motor_msg.velocity = motor_right_velocity

                motor_vel_req.motor_goal_velocities.append(set_motor_msg)

                set_motor_msg = SetVelocity()
                set_motor_msg.id = self.arm[joint_iter].id_left_motor
                set_motor_msg.velocity = motor_left_velocity

                motor_vel_req.motor_goal_velocities.append(set_motor_msg)

                
            else:
                delta_motor_pos = delta_action[motor_iter]

                if delta_motor_pos > 0:
                    if self.arm[joint_iter].joint_angle + delta_motor_pos > self.arm[joint_iter].max_pos:
                        delta_motor_pos = self.arm[joint_iter].max_pos - self.arm[joint_iter].joint_angle
                        # print("max joint pos reached")
                else:
                    if self.arm[joint_iter].joint_angle + delta_motor_pos < self.arm[joint_iter].min_pos:
                        delta_motor_pos = self.arm[joint_iter].min_pos - self.arm[joint_iter].joint_angle
                        # print("min joint pos reached")


                motor_velocity = delta_motor_pos / self.control_timestep

                set_motor_msg = SetVelocity()
                set_motor_msg.id = self.arm[joint_iter].id_motor
                set_motor_msg.velocity = motor_velocity

                motor_vel_req.motor_goal_velocities.append(set_motor_msg)

                motor_iter += 1

            joint_iter += 1

        
        return motor_vel_req

    def step(self, action):
        """
        Send action to the arm controller

        """
        self.reseted = False
        # print("step execution")
        # self.motor_goal_req = self.set_eq_motor_goal_position(action)

        self.motor_goal_req = self.set_eq_motor_goal_velocity(action)
        # zero_step = np.zeros(13)
        # zero_action_req = self.set_eq_motor_goal_position(zero_step)
        # print(self.motor_goal_req)
        # print(self.motor_pos_req)

        policy_step = True

        # start = time.time()
        #THIS MIGHT NOT WORK WITH DELTA POSITION SERVICE BECAUSE IT SENDS THE COMMAND MULTIPLE TIMES??????????????????????????
        for i in range(int(self.control_timestep / self.model_timestep)):
        # for i in range(4):
            # start_2 = time.time()
            start = time.time()

            self.shoulder_future = self.client_velocity_shoulder.call_async(self.motor_goal_req)
            self.upperarm_future = self.client_velocity_upperarm.call_async(self.motor_goal_req)
            self.lowerarm_future = self.client_velocity_lowerarm.call_async(self.motor_goal_req)
               
            rclpy.spin_until_future_complete(self.node, self.shoulder_future)
            rclpy.spin_until_future_complete(self.node, self.upperarm_future)
            rclpy.spin_until_future_complete(self.node, self.lowerarm_future)
            self.shoulder_motor_states_msg = self.shoulder_future.result()
            self.upperarm_motor_states_msg = self.upperarm_future.result()
            self.lowerarm_motor_states_msg = self.lowerarm_future.result()

            # self.safe_to_file()

            end = time.time()
            sleep_time = self.model_timestep - (end-start)
            if sleep_time > 0.0:
            # print(sleep_time)
                time.sleep(sleep_time)
            else:
                print("Took too long!")
            # end_2 = time.time()
            # print(end_2 - start)

        # end = time.time()
        # print(end-start)
                
        for motor_status in self.lowerarm_motor_states_msg.motor_status:
            if motor_status.motor_id == self.arm[5].id_motor:
                self.arm[5].joint_angle = motor_status.present_position
                self.arm[5].joint_velocity = motor_status.present_velocity

        observation = self._get_obs()
        reward = self.reward()
        info = self._get_info()
        done = reward >= 0.99
        # done = False

        return observation, reward, done, info

    def check_success(self):
        # hinge_qpos = self.sim.data.qpos[self.hinge_qpos_addr]
        # return self.hinge_qpos < -0.3
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
            dist = np.linalg.norm(self.gripper_to_handle)
            reaching_reward = 0.25 * (1 - np.tanh(10.0 * dist))
            reward += reaching_reward

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


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = DoorEnv()

#     # rclpy.spin(minimal_subscriber)

#     # # Destroy the node explicitly
#     # # (optional - otherwise it will be done automatically
#     # # when the garbage collector destroys the node object)
#     # minimal_subscriber.destroy_node()
#     # rclpy.shutdown()

# if __name__ == '__main__':
#     main()
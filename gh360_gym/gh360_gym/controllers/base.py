import numpy as np

from gh360_gym.utils.joints import MotorJoint, SoftJoint, Joint
from gh360_gym.utils.motor import Motor
from gh360_gym.utils.filters import freq_filter, median_filter
from gh360_gym.utils.motor_interfaces import generate_velocities_msg

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt16, Float64
from ros2pkg.api import get_prefix_path
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose
import tf2_geometry_msgs
from gh360_interfaces.msg import ArmEncoderStates, PortStatus, SetMotorVelocities, SetMotorPositions
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from std_srvs.srv import SetBool
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState


class BaseController:
    def __init__(self, node: Node, max_joint_pos=[], min_joint_pos=[], max_current=[]):
        self. node = node 

        self.motor_controller = "velocity"
        self.control_timestep = 0.2
        self.control_time_adj = 0.0
        self.model_timestep = 0.1
        self.reseted = False
        self.motor_cnt = 0
        self.joint_cnt = 0
        self.motors = []
        self.joints = []

        

        if len(max_joint_pos) < 7:
            max_joint_pos = [1.5, 1.5, 0.7, 3.0, 2.2, np.pi/2, np.pi/2]
        if len(min_joint_pos) < 7:
            min_joint_pos = [-1.5, -1.5, 0.0, -3.0, -0.4, -np.pi/2, -np.pi/2]
        if len(max_current) < 7:
            max_current = [3500, 3500, 3500, 3500, 3500, 2000, 2000]

        # self.node.create_subscription(ArmEncoderStates,'/encoder_status',self.encoder_callback,10)
        self.node.create_subscription(JointState,'/gh360/joint_states',self.joint_states_callback,10)
        self.node.create_subscription(PortStatus,'/gh360/motor_states_sorted',self.motor_status_callback,10)
        # self.node.create_subscription(PortStatus,'/upperarm/motor_status',self.motor_status_callback,10)
        # self.node.create_subscription(PortStatus,'/lowerarm/motor_status',self.motor_status_callback,10)

        self.pub_step = self.node.create_publisher(Bool, '/gym_stepping', 10)

        self.pub_motor_goal_velocity = self.node.create_publisher(SetMotorVelocities, '/gh360/motor_goal_velocity', 10)
        self.pub_motor_goal_position = self.node.create_publisher(SetMotorPositions, '/gh360/cmd_motor_pos', 10)

        self.client_robot_stop = self.node.create_client(SetBool, '/gh360_control/robot_stop')
        while not self.client_robot_stop.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        # self.client_set_motor_torque_shoulder = self.node.create_client(SetBool, '/gh360/shoulder/motor_set_torque')
        # while not self.client_set_motor_torque_shoulder.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('service not available, waiting again...')
        # self.client_set_motor_torque_upperarm = self.node.create_client(SetBool, '/gh360/upperarm/motor_set_torque')
        # while not self.client_set_motor_torque_upperarm.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('service not available, waiting again...')
        # self.client_set_motor_torque_lowerarm = self.node.create_client(SetBool, '/gh360/lowerarm/motor_set_torque')
        # while not self.client_set_motor_torque_lowerarm.wait_for_service(timeout_sec=1.0):
        #     self.node.get_logger().info('service not available, waiting again...')

        # self.arm = []
        # new_joint = SoftJoint(joint_name="shoulder_yaw", port_name="shoulder", id_right_motor=1, id_left_motor=2, max_pos=max_joint_pos[0], min_pos=min_joint_pos[0], max_current=max_current[0])
        # self.arm.append(new_joint)
        # new_joint = SoftJoint(joint_name="shoulder_roll", port_name="shoulder", id_right_motor=3, id_left_motor=4, max_pos=max_joint_pos[1], min_pos=min_joint_pos[1], max_current=max_current[1])
        # self.arm.append(new_joint)
        # new_joint = SoftJoint(joint_name="shoulder_pitch", port_name="shoulder", id_right_motor=5, id_left_motor=6, max_pos=max_joint_pos[2], min_pos=min_joint_pos[2], max_current=max_current[2])
        # self.arm.append(new_joint)
        # new_joint = SoftJoint(joint_name="upperarm_roll", port_name="upperarm", id_right_motor=7, id_left_motor=8, max_pos=max_joint_pos[3], min_pos=min_joint_pos[3], max_current=max_current[3])
        # self.arm.append(new_joint)
        # new_joint = SoftJoint(joint_name="elbow", port_name="upperarm", id_right_motor=10, id_left_motor=9, max_pos=max_joint_pos[4], min_pos=min_joint_pos[4], max_current=max_current[4])
        # self.arm.append(new_joint)
        # new_joint = MotorJoint(joint_name="lowerarm_roll", port_name="lowerarm", id_motor=11, max_pos=max_joint_pos[5], min_pos=min_joint_pos[5], max_current=max_current[5])
        # self.arm.append(new_joint)
        # new_joint = SoftJoint(joint_name="wrist_pitch", port_name="lowerarm", id_right_motor=13, id_left_motor=12, max_pos=max_joint_pos[6], min_pos=min_joint_pos[6], max_current=max_current[6])
        # self.arm.append(new_joint)
        while self.motor_cnt == 0 or self.joint_cnt == 0:
            rclpy.spin_once(self.node)

    def joint_states_callback(self, msg):
        self.joint_cnt = len(msg.name)

        if len(self.joints) != self.joint_cnt:
            self.joints = []
            for i in range(self.joint_cnt):
                self.joints.append(Joint())

        for i in range(len(msg.name)):
            self.joints[i].joint_name = msg.name[i]
            self.joints[i].joint_angle = msg.position[i]
            self.joints[i].joint_velocity = msg.velocity[i]
            # for joint in self.joints:
                
            #     if joint.joint_name == msg.name[i]:
            #         joint.joint_angle = msg.position[i]
            #         joint.joint_velocity = msg.velocity[i]

    def motor_status_callback(self, msg):
        # print("recieved message!")
        self.motor_cnt = len(msg.motors)
        if len(self.motors) != self.motor_cnt:
            self.motors = []
            for i in range(self.motor_cnt):
                self.motors.append(Motor())
        
        for i, motor in enumerate(msg.motors):
            self.motors[i].motor_id = motor.motor_id
            self.motors[i].safety_check = motor.safety_check
            self.motors[i].moving = motor.moving
            self.motors[i].present_current = motor.present_current
            self.motors[i].present_position = motor.present_position
            self.motors[i].present_velocity = motor.present_velocity


            # for joint in self.arm:
            #     if type(joint) == SoftJoint:
            #         if joint.id_right_motor == motor.motor_id:
            #             joint.right_motor_safety_check = motor.safety_check
            #             joint.right_motor_moving = motor.moving
            #             joint.right_motor_current = motor.present_current
            #             joint.right_motor_pos = motor.present_position
            #             joint.right_motor_vel = motor.present_velocity
            #         elif joint.id_left_motor == motor.motor_id:
            #             joint.left_motor_safety_check = motor.safety_check
            #             joint.left_motor_moving = motor.moving
            #             joint.left_motor_current = motor.present_current
            #             joint.left_motor_pos = motor.present_position
            #             joint.left_motor_vel = motor.present_velocity
            #     elif type(joint) == MotorJoint:
            #         if joint.id_motor == motor.motor_id:
            #             joint.motor_safety_check = motor.safety_check
            #             joint.motor_moving = motor.moving
            #             joint.motor_current = motor.present_current
            #             joint.motor_pos = motor.present_position
            #             joint.motor_vel = motor.present_velocity
            #             joint.joint_angle = motor.present_position
            #             joint.joint_velocity = motor.present_velocity

    def stop_robot(self, stop=True):
        msg = SetBool.Request()
        msg.data = stop

        future = self.client_robot_stop.call_async(msg)
        rclpy.spin_until_future_complete(self.node, future)
        # shoulder_future = self.client_set_motor_torque_shoulder.call_async(msg)
        # upperarm_future = self.client_set_motor_torque_upperarm.call_async(msg)
        # lowerarm_future = self.client_set_motor_torque_lowerarm.call_async(msg)

        # rclpy.spin_until_future_complete(self.node, shoulder_future)
        # rclpy.spin_until_future_complete(self.node, upperarm_future)
        # rclpy.spin_until_future_complete(self.node, lowerarm_future)

    # def stop_motors(self):
    #     action = np.zeros(13)
    #     motor_vel_req = generate_velocities_msg(self.arm, action, srv=True)
        
    #     shoulder_velocity_future = self.client_velocity_shoulder.call_async(motor_vel_req)
    #     upperarm_velocity_future = self.client_velocity_upperarm.call_async(motor_vel_req)
    #     lowerarm_velocity_future = self.client_velocity_lowerarm.call_async(motor_vel_req)

    #     rclpy.spin_until_future_complete(self.node, shoulder_velocity_future)
    #     rclpy.spin_until_future_complete(self.node, upperarm_velocity_future)
    #     rclpy.spin_until_future_complete(self.node, lowerarm_velocity_future)
        
    def robot_safety_check(self):
        safety_check = True
        for motor in self.motors:
            if not motor.safety_check:
                safety_check = False
                break
        
        # for joint in self.arm:
        #     if type(joint) == SoftJoint:
        #         if not joint.right_motor_safety_check or not joint.left_motor_safety_check:
        #             safety_check = False
        #             print(f"{joint.joint_name} safety check: {safety_check}")
        #             break
        #     elif type(joint) == MotorJoint:
        #         if not joint.motor_safety_check:
        #             safety_check = False
        #             print("Forearm Roll safety check: "+str(safety_check))
        #             break

        return safety_check

    # def robot_position_check(self, target_pos):
    #     position_check = True
    #     for joint in self.arm:
    #         if type(joint) == SoftJoint:
    #             if abs(target_pos[joint.id_right_motor-1] - joint.right_motor_pos) > 0.15 or abs(target_pos[joint.id_left_motor-1] - joint.left_motor_pos) > 0.15:
    #                 position_check = False
    #                 # print(f"{joint.joint_name} position check: {position_check}")
    #                 break
    #         elif type(joint) == MotorJoint:
    #             if abs(target_pos[joint.id_motor-1] - joint.motor_pos) > 0.15:
    #                 position_check = False
    #                 # print("Forearm Roll position check: "+str(position_check))
    #                 break

    #     return position_check
    

    # def robot_moving_check(self):
    #     moving = False
    #     for joint in self.arm:
    #         if type(joint) == SoftJoint:
    #             if joint.right_motor_moving or joint.left_motor_moving:
    #                 moving = True
    #         elif type(joint) == MotorJoint:
    #             if joint.motor_moving:
    #                 moving = True

    #     return moving
import numpy as np
import time
from gh360_gym.utils.joints import Joint
from gh360_gym.utils.motor import Motor

import rclpy
from rclpy.node import Node
from gh360_interfaces.msg import PortStatus, SetMotorVelocities, SetMotorPositions
from gh360_interfaces.srv import SetRobotLimits
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState


class BaseController:
    def __init__(self, node: Node, max_joint_pos=[], min_joint_pos=[], max_motor_current=[], min_motor_current=[]):
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

        

        # if len(max_joint_pos) < 7:
        #     max_joint_pos = [1.5, 1.5, 0.7, 3.0, 2.2, np.pi/2, np.pi/2]
        # if len(min_joint_pos) < 7:
        #     min_joint_pos = [-1.5, -1.5, 0.0, -3.0, -0.4, -np.pi/2, -np.pi/2]
        # if len(max_current) < 7:
        #     max_current = [3500, 3500, 3500, 3500, 3500, 2000, 2000]
        # if len(min_current) < 7:
        #     min_current = [-3500, -3500, -3500, -3500, -3500, -2000, -2000]
        self.node.get_logger().info("Initializing Base Controller...")
        self.client_set_robot_limits = self.node.create_client(SetRobotLimits, '/gh360_control/set_robot_limits')
        while not self.client_set_robot_limits.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('robot limits service not available, waiting again...')
        self.node.get_logger().info("Set Robot Limits Service available!")

        self.node.create_subscription(JointState,'/gh360/joint_states',self.joint_states_callback,10)
        self.node.create_subscription(PortStatus,'/gh360/motor_states_sorted',self.motor_status_callback,10)

        self.pub_step = self.node.create_publisher(Bool, '/gym_stepping', 10)

        self.pub_motor_goal_velocity = self.node.create_publisher(SetMotorVelocities, '/gh360/motor_goal_velocity', 10)
        self.pub_motor_goal_position = self.node.create_publisher(SetMotorPositions, '/gh360/cmd_motor_pos', 10)

        self.client_robot_stop = self.node.create_client(SetBool, '/gh360_control/robot_stop')
        while not self.client_robot_stop.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('robot stop service not available, waiting again...')
        self.node.get_logger().info("Robot Stop Service available!")
       
        while self.motor_cnt == 0 or self.joint_cnt == 0:
            self.node.get_logger().info("Waiting for joint_states and motor_states_sorted topics...")
            rclpy.spin_once(self.node)
            # time.sleep(1.0)

        self.node.get_logger().info("Joint and Motor states received!")
        msg = SetRobotLimits.Request()
        if len(max_joint_pos) == self.joint_cnt and len(min_joint_pos) == self.joint_cnt: 
            msg.max_joint_angles = max_joint_pos
            msg.min_joint_angles = min_joint_pos
            msg.max_motor_currents = max_motor_current
            msg.min_motor_currents = min_motor_current

            self.node.get_logger().info(f"Max Motor Currents: {max_motor_current}, Min Motor Currents: {min_motor_current}")

            future_set_robot_limits = self.client_set_robot_limits.call_async(msg)
            rclpy.spin_until_future_complete(self.node, future_set_robot_limits)




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

    def stop_robot(self, stop=True):
        msg = SetBool.Request()
        msg.data = stop

        future = self.client_robot_stop.call_async(msg)
        rclpy.spin_until_future_complete(self.node, future)
        
    def robot_safety_check(self):
        safety_check = True
        for motor in self.motors:
            if not motor.safety_check:
                safety_check = False
                break

        return safety_check
    
    def publish_step_goal(self, publisher, action_msg):
        while (time.time() - self.last_time) < (self.control_timestep-self.control_time_adj):
            # self.joint_goal_pos_msg.data = action
            publisher.publish(action_msg)

            rclpy.spin_once(self.node)

            if not self.robot_safety_check():
                    # wait for user_input
                    self.stop_robot(True)
                    input("Press Enter to continue...")
                    self.stop_robot(False)

        t_control_loop = time.time() - self.last_time
        self.last_time = time.time()
        return t_control_loop

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
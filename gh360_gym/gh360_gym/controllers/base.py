import numpy as np

from gh360_gym.utils.joints import MotorJoint, SoftJoint
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
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState


class BaseController:
    def __init__(self, node: Node):
        self. node = node 

        self.motor_controller = "velocity"
        self.control_timestep = 0.2
        self.control_time_adj = 0.0
        self.model_timestep = 0.1
        self.reseted = False

        # self.node.create_subscription(ArmEncoderStates,'/encoder_status',self.encoder_callback,10)
        self.node.create_subscription(JointState,'/gh360_joint_states',self.joint_states_callback,10)
        self.node.create_subscription(PortStatus,'/shoulder/motor_status',self.motor_status_callback,10)
        self.node.create_subscription(PortStatus,'/upperarm/motor_status',self.motor_status_callback,10)
        self.node.create_subscription(PortStatus,'/lowerarm/motor_status',self.motor_status_callback,10)

        self.pub_step = self.node.create_publisher(Bool, '/gym_stepping', 10)

        self.pub_goal_velocity_shoulder = self.node.create_publisher(SetMotorVelocities, '/shoulder/motor_goal_velocity', 10)
        self.pub_goal_velocity_upperarm = self.node.create_publisher(SetMotorVelocities, '/upperarm/motor_goal_velocity', 10)
        self.pub_goal_velocity_lowerarm = self.node.create_publisher(SetMotorVelocities, '/lowerarm/motor_goal_velocity', 10)

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

        self.arm = []
        new_joint = SoftJoint(joint_name="shoulder_yaw", port_name="shoulder", id_right_motor=1, id_left_motor=2, max_pos=1.4, min_pos=-1.4, max_current=3000)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="shoulder_roll", port_name="shoulder", id_right_motor=3, id_left_motor=4, max_pos=1.4, min_pos=-1.4, max_current=3000)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="shoulder_pitch", port_name="shoulder", id_right_motor=5, id_left_motor=6, max_pos=1.5, min_pos=0.0, max_current=3000)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="upperarm_roll", port_name="upperarm", id_right_motor=7, id_left_motor=8, max_pos=3.0, min_pos=-3.0, max_current=3000)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="elbow", port_name="upperarm", id_right_motor=10, id_left_motor=9, max_pos=2.2, min_pos=0.0, max_current=3000)
        self.arm.append(new_joint)
        new_joint = MotorJoint(joint_name="lowerarm_roll", port_name="lowerarm", id_motor=11, max_pos=np.pi/2, min_pos=-np.pi/2, max_current=1000)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="wrist_pitch", port_name="lowerarm", id_right_motor=13, id_left_motor=12, max_pos=1.4, min_pos=-1.4, max_current=1000)
        self.arm.append(new_joint)

    def joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            for joint in self.arm:
                if joint.joint_name == msg.name[i]:
                    joint.joint_angle = msg.position[i]
                    joint.joint_velocity = msg.velocity[i]

    def motor_status_callback(self, msg):
        # print("recieved message!")
        for motor in msg.motors:
            for joint in self.arm:
                if type(joint) == SoftJoint:
                    if joint.id_right_motor == motor.motor_id:
                        joint.right_motor_safety_check = motor.safety_check
                        joint.right_motor_moving = motor.moving
                        joint.right_motor_current = motor.present_current
                        joint.right_motor_pos = motor.present_position
                    elif joint.id_left_motor == motor.motor_id:
                        joint.left_motor_safety_check = motor.safety_check
                        joint.left_motor_moving = motor.moving
                        joint.left_motor_current = motor.present_current
                        joint.left_motor_pos = motor.present_position
                elif type(joint) == MotorJoint:
                    if joint.id_motor == motor.motor_id:
                        joint.motor_safety_check = motor.safety_check
                        joint.motor_moving = motor.moving
                        joint.motor_current = motor.present_current
                        joint.joint_angle = motor.present_position
                        joint.joint_velocity = motor.present_velocity

    def stop_motors(self):
        action = np.zeros(13)
        motor_vel_req = generate_velocities_msg(self.arm, action, srv=True)
        
        shoulder_velocity_future = self.client_velocity_shoulder.call_async(motor_vel_req)
        upperarm_velocity_future = self.client_velocity_upperarm.call_async(motor_vel_req)
        lowerarm_velocity_future = self.client_velocity_lowerarm.call_async(motor_vel_req)

        rclpy.spin_until_future_complete(self.node, shoulder_velocity_future)
        rclpy.spin_until_future_complete(self.node, upperarm_velocity_future)
        rclpy.spin_until_future_complete(self.node, lowerarm_velocity_future)
        

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
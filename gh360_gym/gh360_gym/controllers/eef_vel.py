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
    def __init__(self, node, input_max=[], input_min=[], max_current=[], max_joint_pos=[], min_joint_pos=[]):
        super().__init__(node, max_joint_pos, min_joint_pos, max_current)

        # self.goal_joint_velocity = JointState()
        # self.goal_joint_velocity.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.goal_joint_velocity = np.zeros(6)

        self.cmd_vel_publisher = self.node.create_publisher(Twist, '/cmd_eef_vel', 10)
        self.inverse_jacobian_subscriber = self.node.create_subscription(JointState, '/inverse_jacobian', self.inverse_jacobian_callback, 10)

        # self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        # self.control_dim = 13
        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        # self.via_point_pos = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        self.via_point_pos_1 = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 7.5, 7.5, 0.0, 1.0, 1.0]
        self.via_point_pos_2 = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        self.control_dim = 6
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
        
        active_pulleys = [18, 18, 18, 15.6, 15.6, 1, 15.6]
        passive_pulleys = [74.5, 50.5, 50, 43, 40, 1, 30]
        self.joint_to_motor_scaler = np.array(passive_pulleys)/np.array(active_pulleys)
        print("joint to motor scaler: ", self.joint_to_motor_scaler)
        # self.max_motor_vel = 1.5
        self.max_joint_vel = 0.5

        self.last_time = 0

        self.reseted = False

    def inverse_jacobian_callback(self, msg):
        self.goal_joint_velocity = np.array(msg.velocity)
        

    def get_obs(self):
        obs = []
        

        return obs
    
    def calc_motor_pos_error(self, target_pos):
        pos_error = []
        for joint in self.arm:
            if type(joint) == SoftJoint:
                pos_error.append(target_pos[joint.id_right_motor-1] - joint.right_motor_pos)
                pos_error.append(target_pos[joint.id_left_motor-1] - joint.left_motor_pos)
            elif type(joint) == MotorJoint:
                pos_error.append(target_pos[joint.id_motor-1] - joint.motor_pos)

        return pos_error

    def reset(self, robot_eef_pos, handle_pos, handle_qpos=0.0):
        if self.reseted:
            return

        internal_state = 0

        for _ in range(10):
            rclpy.spin_once(self.node)

        if not self.robot_safety_check():
            self.set_motor_torque(False)
            input("Press Enter to continue...")
            self.set_motor_torque(True)

        if handle_pos[2] > robot_eef_pos[2]+0.01 and handle_qpos < 0.1:
            internal_state = 2
            pos_error = self.calc_motor_pos_error(self.robot_reset_pos)
        else:
            pos_error = self.calc_motor_pos_error(self.via_point_pos_1)

        while np.max(np.absolute(pos_error)) > 0.1 or internal_state != 2:
            if not self.robot_safety_check():
                self.set_motor_torque(False)
                input("Press Enter to continue...")
                self.set_motor_torque(True)
                
            if internal_state < 2 and np.max(np.absolute(pos_error)) < 0.2:
                internal_state += 1
            if internal_state == 0:
                pos_error = self.calc_motor_pos_error(self.via_point_pos_1)
            elif internal_state == 1:
                pos_error = self.calc_motor_pos_error(self.via_point_pos_2)
            elif internal_state == 2:
                pos_error = self.calc_motor_pos_error(self.robot_reset_pos)
            pos_error = np.clip(pos_error, -1.0, 1.0)

            motor_vel_msg = generate_velocities_msg(self.arm, pos_error)
            self.pub_goal_velocity_shoulder.publish(motor_vel_msg)
            self.pub_goal_velocity_upperarm.publish(motor_vel_msg)
            self.pub_goal_velocity_lowerarm.publish(motor_vel_msg)
            rclpy.spin_once(self.node)

        # print("final internal state: ", internal_state)

        
        self.control_time_adj = 0.0
        self.last_time = 0
        time_sum = 0
        for i in range(20):
            zero_action = np.zeros(self.control_dim)
            time_sum += self.set_motor_goal(zero_action)

        self.control_time_adj = (time_sum/20) - (self.control_timestep)
        self.last_time = 0
        self.reseted = True
        

        return
        # time.sleep(1)

    def set_motor_goal(self, action):
        """
        Translate action to motor movements for equilibrium point control
        """
        if self.last_time == 0:
            self.last_time = time.time()
        assert len(action) == self.control_dim, "Delta torque must be equal to the robot's joint dimension space!"
        # print("action pre clip: ", action)

        action = np.clip(action, self.input_min, self.input_max)
        
        # print("action post clip: ", action)

        # for i in range(len(action)):
        #     if action[i] > 0.0:
        #         action[i] *= self.max_multiplier[i]
        #     elif action[i] < 0.0:
        #         action[i] *= abs(self.min_multiplier[i])
        # 
        # print("action post multiplier: ", action)
        
        # print("action: ", action)
        
        self.reseted = False
        motor_action = np.zeros(7)
    
        while (time.time() - self.last_time) < (self.control_timestep-self.control_time_adj):
            eef_goal_vel = Twist()
            eef_goal_vel.linear.x = action[0]
            eef_goal_vel.linear.y = action[1]
            eef_goal_vel.linear.z = action[2]
            eef_goal_vel.angular.x = action[3]
            eef_goal_vel.angular.y = action[4]
            eef_goal_vel.angular.z = action[5]

            self.cmd_vel_publisher.publish(eef_goal_vel)
            rclpy.spin_once(self.node)

            # m_cntr = 0
            # max_vel = 0.0
            # self.goal_joint_velocity = np.array(self.goal_joint_velocity)
            max_joint_goal_vel = max(np.absolute(self.goal_joint_velocity))
            if max_joint_goal_vel > self.max_joint_vel:
                # print("max joint goal velocity: ", max_joint_goal_vel)
                # print("goal_joint_velocity: ", self.goal_joint_velocity)
                self.goal_joint_velocity = self.goal_joint_velocity * (self.max_joint_vel/max_joint_goal_vel)
            # print("goal joint velocity: ", self.goal_joint_velocity)

            motor_action = self.goal_joint_velocity * self.joint_to_motor_scaler

            # for i, joint_vel in enumerate(self.goal_joint_velocity):
            #     motor_vel = joint_vel * self.joint_to_motor_scaler[i]
            #     motor_action[i] = motor_vel
                # if abs(motor_vel) > max_vel and abs(motor_vel) > self.max_motor_vel:
                #     max_vel = abs(motor_vel)
                
                # m_cntr += 1
                # if (i != 5):
                #     motor_vel = self.goal_joint_velocity[i] * self.joint_to_motor_scaler[m_cntr]
                #     motor_action[m_cntr] = motor_vel
                #     if abs(motor_vel) > max_vel and abs(motor_vel) > self.max_motor_vel:
                #         max_vel = abs(motor_vel)
                #     m_cntr += 1

            # if max_vel > self.max_motor_vel:
            #     motor_action = motor_action * (self.max_motor_vel/max_vel)
                # if joint_vel > 0.0:
                #     self.goal_joint_velocity[i] *= self.max_multiplier[i]
                # elif joint_vel < 0.0:
                #     self.goal_joint_velocity[i] *= abs(self.min_multiplier[i])


            action_adj = []

            for i, joint in enumerate(self.arm):
                if type(joint) == SoftJoint:
                    action_adj.append(motor_action[i])
                    action_adj.append(motor_action[i])
                    if action_adj[joint.id_right_motor-1] > 0.0:
                        if joint.right_motor_current >= joint.max_current or joint.joint_angle >= joint.max_pos-0.1:
                            action_adj[joint.id_right_motor-1] = 0.0
                    elif action_adj[joint.id_right_motor-1] < 0.0:
                        if joint.right_motor_current <= joint.min_current or joint.joint_angle <= joint.min_pos+0.1:
                            action_adj[joint.id_right_motor-1] = 0.0
                    if action_adj[joint.id_left_motor-1] > 0.0:
                        if joint.left_motor_current >= joint.max_current or joint.joint_angle >= joint.max_pos-0.1:
                            action_adj[joint.id_left_motor-1] = 0.0
                    elif action_adj[joint.id_left_motor-1] < 0.0:
                        if joint.left_motor_current <= joint.min_current or joint.joint_angle <= joint.min_pos+0.1:
                            action_adj[joint.id_left_motor-1] = 0.0
                elif type(joint) == MotorJoint:
                    action_adj.append(motor_action[i])
                    if action_adj[joint.id_motor-1] > 0.0:
                        if joint.motor_current >= joint.max_current or joint.joint_angle >= joint.max_pos-0.1:
                            action_adj[joint.id_motor-1] = 0.0
                    elif action_adj[joint.id_motor-1] < 0.0:
                        if joint.motor_current <= joint.min_current or joint.joint_angle <= joint.min_pos+0.1:
                            action_adj[joint.id_motor-1] = 0.0

            # print("action adj: ", action_adj)
            motor_msg = generate_velocities_msg(self.arm, action_adj)

            self.pub_goal_velocity_shoulder.publish(motor_msg)
            self.pub_goal_velocity_upperarm.publish(motor_msg)
            self.pub_goal_velocity_lowerarm.publish(motor_msg)
            # waited = True
            rclpy.spin_once(self.node)

            if not self.robot_safety_check():
                # wait for user_input
                self.set_motor_torque(False)
                input("Press Enter to continue...")
                self.set_motor_torque(True)

        t_control_loop = time.time() - self.last_time

        
        self.last_time = time.time()
    
        return t_control_loop
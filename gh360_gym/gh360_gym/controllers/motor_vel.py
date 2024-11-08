from gh360_gym.controllers.base import BaseController
from gh360_gym.utils.joints import MotorJoint, SoftJoint
from gh360_gym.utils.motor_interfaces import generate_velocities_msg, generate_positions_msg
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates, SetVelocity, PortStatus
from std_msgs.msg import Bool

import numpy as np
import time

import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class MotorVelocityController(BaseController):
    def __init__(self, node, input_max=[], input_min=[], max_current=[], max_joint_pos=[], min_joint_pos=[]):
        super().__init__(node, max_joint_pos, min_joint_pos, max_current)


        # self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        # self.control_dim = 13
        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        self.via_point_pos = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        self.control_dim = 7
        print("control dimensions: ", self.control_dim)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # self.input_max = np.ones(self.control_dim) * input_max
        # self.input_min = np.ones(self.control_dim) * input_min
        # if len(input_max) != self.control_dim:
        #     self.input_max = np.ones(self.control_dim)
        # else:
        #     self.input_max = input_max
        # if len(input_min) != self.control_dim:
        #     self.input_min = -np.ones(self.control_dim)
        # else:
        #     self.input_min = input_min

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
        # self.input_max = ([0.75, 0.75, 0.5, 0.5, 1.0, 1.0, 0.8, 0.8, 0.8, 0.8, 0.4, 1.3, 1.3])
        # self.input_min = -self.input_max
        # print(f"max_multiplier: {self.max_multiplier}")
        # print(f"min_multiplier: {self.min_multiplier}")
        # print(f"input_max: {self.input_max}")
        # print(f"input_min: {self.input_min}")
        # for joint in self.arm:
        #     print(f"joint: {joint.joint_name}")
        #     print(f"max_pos: {joint.max_pos}")
        #     print(f"min_pos: {joint.min_pos}")
        #     print(f"max_current: {joint.max_current}")
        #     print(f"min_current: {joint.min_current}")
        # print(f"max_current: {max_current}")
        # print(f"max_joint_pos: {max_joint_pos}")
        # print(f"min_joint_pos: {min_joint_pos}")

        self.last_time = 0

        self.reseted = False

        

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

    def reset(self):
        if self.reseted:
            return

        internal_state = 0

        for _ in range(10):
            rclpy.spin_once(self.node)

        if not self.robot_safety_check():
            self.set_motor_torque(False)
            input("Press Enter to continue...")
            self.set_motor_torque(True)

        pos_error = self.calc_motor_pos_error(self.via_point_pos)

        stuck = False
        while np.max(np.absolute(pos_error)) > 0.1 or internal_state != 1:
            if not self.robot_safety_check():
                self.set_motor_torque(False)
                input("Press Enter to continue...")
                self.set_motor_torque(True)
                
            if internal_state == 0 and np.max(np.absolute(pos_error)) < 0.2 and not stuck:
                internal_state = 1
            if internal_state == 0:
                stuck = False
                pos_error = self.calc_motor_pos_error(self.via_point_pos)
                if self.arm[4].joint_angle >= 1.0:
                    pos_error[8] = -1.0
                    pos_error[9] = -1.0
                    stuck = True
            elif internal_state == 1:
                pos_error = self.calc_motor_pos_error(self.robot_reset_pos)
            pos_error = np.clip(pos_error, -0.5, 0.5)

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

        for i in range(len(action)):
            if action[i] > 0.0:
                action[i] *= self.max_multiplier[i]
            elif action[i] < 0.0:
                action[i] *= abs(self.min_multiplier[i])
        # 
        # print("action post multiplier: ", action)
        

        
        self.reseted = False

    
        while (time.time() - self.last_time) < (self.control_timestep-self.control_time_adj):
            action_adj = []

            for i, joint in enumerate(self.arm):
                if type(joint) == SoftJoint:
                    action_adj.append(action[i])
                    action_adj.append(action[i])
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
                    action_adj.append(action[i])
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
            self.stop_motors()
            input("Press Enter to continue...")
            pass    

        t_control_loop = time.time() - self.last_time

        
        self.last_time = time.time()
    
        return t_control_loop
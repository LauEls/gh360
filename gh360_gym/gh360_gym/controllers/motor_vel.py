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
    def __init__(self, node, input_max=1, input_min=-1):
        super().__init__(node)

        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        self.control_dim = 13
        print("control dimensions: ", self.control_dim)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.input_max = np.ones(self.control_dim) * input_max
        self.input_min = np.ones(self.control_dim) * input_min

        self.last_time = 0

        self.reseted = False

        

    def get_obs(self):
        obs = []
        

        return obs

    def reset(self):
        if self.reseted:
            return

        self.internal_state = 0

        motor_pos_req = generate_positions_msg(self.arm, self.robot_reset_pos)

        shoulder_future = self.client_shoulder.call_async(motor_pos_req)
        upperarm_future = self.client_upperarm.call_async(motor_pos_req)
        lowerarm_future = self.client_lowerarm.call_async(motor_pos_req)

        rclpy.spin_until_future_complete(self.node, shoulder_future)
        rclpy.spin_until_future_complete(self.node, upperarm_future)
        rclpy.spin_until_future_complete(self.node, lowerarm_future)
        shoulder_motor_states_msg = shoulder_future.result()
        upperarm_motor_states_msg = upperarm_future.result()
        lowerarm_motor_states_msg = lowerarm_future.result()
        
        start_time = time.time()
        while time.time() - start_time < 1:
            rclpy.spin_once(self.node)
            # print(self.robot_moving_check())

        print("reseted2")

        # rclpy.spin_once(self.node)
        while self.robot_moving_check():
            rclpy.spin_once(self.node)
            # if not self.robot_safety_check():
            #     # wait for user_input
            #     input("Press Enter to continue...")
            #     pass

        self.stop_motors()

        time_sum = 0
        for i in range(20):
            zero_action = np.zeros(13)
            time_sum += self.set_motor_goal(zero_action)

        self.control_time_adj = (time_sum/20) - (self.control_timestep)


        # time.sleep(1)

    def set_motor_goal(self, action):
        """
        Translate action to motor movements for equilibrium point control
        """
        if self.last_time == 0:
            self.last_time = time.time()
        assert len(action) == self.control_dim, "Delta torque must be equal to the robot's joint dimension space!"

        # delta_action = np.clip(delta_action, self.input_min, self.input_max)
        # delta_action = delta_action/10

        if self.motor_controller == "velocity":
            motor_msg = generate_velocities_msg(self.arm, action)
        elif self.motor_controller == "position":
            motor_msg = generate_positions_msg(self.arm, action)

        # if not self.robot_safety_check():
        #     # wait for user_input
        #     input("Press Enter to continue...")
        #     pass    
        self.reseted = False

        

        # if self.motor_controller == "velocity":
        #     self.shoulder_future = self.client_velocity_shoulder.call_async(motor_req)
        #     self.upperarm_future = self.client_velocity_upperarm.call_async(motor_req)
        #     self.lowerarm_future = self.client_velocity_lowerarm.call_async(motor_req)
        # elif self.motor_controller == "position":
        #     self.shoulder_future = self.client_delta_shoulder.call_async(motor_req)
        #     self.upperarm_future = self.client_delta_upperarm.call_async(motor_req)
        #     self.lowerarm_future = self.client_delta_lowerarm.call_async(motor_req)
        #     print("sending position request")
        
        # self.pub_step.publish(msg)

        # msg = Bool(data=True)
        # waited = False
        while (time.time() - self.last_time) < (self.control_timestep-self.control_time_adj):
            self.pub_goal_velocity_shoulder.publish(motor_msg)
            self.pub_goal_velocity_upperarm.publish(motor_msg)
            self.pub_goal_velocity_lowerarm.publish(motor_msg)
            # waited = True
            rclpy.spin_once(self.node)

        t_control_loop = time.time() - self.last_time
        print(f"seconds: {t_control_loop}")
        
        # if not waited:
        #     print("not waited")
        # else:
        #     print("waited")
        
        self.last_time = time.time()
    
        return t_control_loop
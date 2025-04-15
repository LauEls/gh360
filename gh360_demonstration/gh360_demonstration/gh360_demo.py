#from mujoco_py import MjSim
import gh360_gym
import gym
from robosuite.utils.input_utils import *
import rclpy
import time
import json
import os
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from gh360_interfaces.msg import SpaceMouse, PortStatus

import sys
from robosuite.wrappers import GymWrapper
from .collect_demos import collect_demonstrations
# sys.path.insert(0, '/home/laurenz/phd_project/sac/sac_2')
# from wrappers import NormalizedBoxEnv


class GH360Teleop(Node):

    def __init__(self):
        super().__init__('gh360_teleop')
        self.get_logger().info("Initializing GH360 Teleop Node")
        # self.joint_goal_publisher = self.create_publisher(JointState, '/gh360_joint_states', 10)
        # self.joint_state_msg = JointState()
        # self.joint_state_msg.name = ['shoulder_yaw', 'shoulder_roll', 'shoulder_pitch', 'upperarm_roll', 'elbow', 'forearm_roll', 'wrist_pitch']
        # self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      
        
        self.create_subscription(
            SpaceMouse,
            'spacemouse',
            self.spacemouse_callback,
            10)
        
        
        
        # self.create_subscription(JointState, '/inverse_jacobian', self.inverse_jacobian_callback, 10)
        
        # self.rotation_scaling = 1.0
        # self.translation_scaling = 1.0
        self.reset = False
        self.record_data = False
        self.btn1_pressed = False
        self.btn2_pressed = False

        self.save_file = '/home/gh360/ros2_gh360_ws/src/gh360/gh360_demonstration/data/spacemouse_demonstrations/door/gh360_door_demonstration_v9'
        self.save_file_path = self.save_file+'.npy'
        # config_file = '/home/laurenz/phd_project/TD7/runs/door/real_gh360/eef_vel/online/v7_refactor_test/variant.json'
        self.config_path = '/home/gh360/TD7/runs/door/real_gh360/eef_vel/online/v3_train_with_demo_buffer/'
        config_file = self.config_path+'variant.json'
        self.expert_paths = ''

        # kwargs_fpath = os.path.join(load_dir, "variant.json")
        try:
            with open(config_file) as f:
                variant = json.load(f)
        except FileNotFoundError:
            print("Error opening default controller filepath at: {}. "
                "Please check filepath and try again.".format(config_file))
            
        
        env_name = variant["environment_kwargs"].pop("env_name")
        #variant["environment_kwargs"].pop("input_max")
        #variant["environment_kwargs"].pop("input_min")
        # variant["environment_kwargs"].pop("max_joint_pos")
        # variant["environment_kwargs"].pop("min_joint_pos")
        env_config = variant["environment_kwargs"]
        # variant["environment_kwargs"].pop("max_current")

        raw_env = gym.make('gh360_gym/'+env_name, **env_config, node=self)
        # self.env = NormalizedBoxEnv(raw_env)
        self.env = raw_env

        self.max_joint_pos = np.ones(self.env.controller.joint_cnt)*-1000
        self.min_joint_pos = np.ones(self.env.controller.joint_cnt)*1000
        self.max_current = np.ones(self.env.controller.motor_cnt)*-1000
        self.min_current = np.ones(self.env.controller.motor_cnt)*1000

        self.create_subscription(JointState,'/gh360/joint_states',self.joint_states_callback,10)
        self.create_subscription(PortStatus,'/gh360/motor_states_sorted',self.motor_status_callback,10)


        self.ep_length = variant["episode_length"]

        self.observation = self.env.reset()

        self.observations = []
        self.next_observations = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.infos = []

        self.paths = []

        self.eef_vel = None


        while self.eef_vel is None:
            rclpy.spin_once(self)

        self.get_logger().info("GH360 Teleop Node initialized")
        self.start_time = time.time()

    def spacemouse_callback(self, msg):
        self.eef_vel = np.zeros(6)
        self.eef_vel[0] = msg.velocity.linear.x
        self.eef_vel[1] = msg.velocity.linear.y
        self.eef_vel[2] = msg.velocity.linear.z
        self.eef_vel[3] = msg.velocity.angular.x
        self.eef_vel[4] = msg.velocity.angular.y
        self.eef_vel[5] = msg.velocity.angular.z

        if msg.button1:
            self.btn1_pressed = True
        elif not msg.button1 and self.btn1_pressed:
            self.reset = True
            self.btn1_pressed = False
            if self.record_data:
                self.get_logger().info("Stopped and deleted recorded data")

        if msg.button2:
            self.btn2_pressed = True
        elif not msg.button2 and self.btn2_pressed:
            self.record_data = not self.record_data
            self.btn2_pressed = False
            if self.record_data:
                self.get_logger().info("Recording data")
            else:
                self.get_logger().info("Stopped recording data")

    def joint_states_callback(self, msg):
        # self.joint_cnt = len(msg.name)

        # if len(self.joints) != self.joint_cnt:
        #     self.joints = []
        #     for i in range(self.joint_cnt):
        #         self.joints.append(Joint())

        for i in range(len(msg.name)):
            if msg.position[i] > self.max_joint_pos[i]:
                self.max_joint_pos[i] = msg.position[i]
            if msg.position[i] < self.min_joint_pos[i]:
                self.min_joint_pos[i] = msg.position[i]

            # self.joints[i].joint_name = msg.name[i]
            # self.joints[i].joint_angle = msg.position[i]
            # self.joints[i].joint_velocity = msg.velocity[i]

    def motor_status_callback(self, msg):
        # print("recieved message!")
        # self.motor_cnt = len(msg.motors)
        # if len(self.motors) != self.motor_cnt:
        #     self.motors = []
        #     for i in range(self.motor_cnt):
        #         self.motors.append(Motor())
        
        for i, motor in enumerate(msg.motors):
            if motor.present_current > self.max_current[i]:
                self.max_current[i] = motor.present_current
            if motor.present_current < self.min_current[i]:
                self.min_current[i] = motor.present_current

            # self.motors[i].motor_id = motor.motor_id
            # self.motors[i].safety_check = motor.safety_check
            # self.motors[i].moving = motor.moving
            # self.motors[i].present_current = motor.present_current
            # self.motors[i].present_position = motor.present_position
            # self.motors[i].present_velocity = motor.present_velocity

    def write_limits_file(self):
        self.get_logger().info("Writing limits to file")
        limits = {
            "max_joint_pos": self.max_joint_pos.tolist(),
            "min_joint_pos": self.min_joint_pos.tolist(),
            "max_motor_current": self.max_current.tolist(),
            "min_motor_current": self.min_current.tolist(),
            "duration": self.end_time - self.start_time,
        }

        with open(self.save_file+'_limits.json', 'w') as f:
            json.dump(limits, f)
        self.get_logger().info("Limits written to file")

    def expert_action(self, observation):
        if self.reset:
            self.reset_env()

        rclpy.spin_once(self)

        action = self.eef_vel

        return action, self.record_data
    
    def reset_env(self):
        observation = self.env.reset()
        # self.env.render()

        self.reset = False
        self.record_data = False

        return observation

    def collect_demonstrations(self, mode, num_eps: int):
        paths = []
        self.start_time = time.time()

        self.get_logger().info("Collecting expert demonstrations")
        paths = collect_demonstrations(self, mode, num_eps, self.expert_paths)
        self.get_logger().info("Expert demonstrations collected")
        self.env.reset()
        file_array = np.array(paths)
        np.save(self.save_file_path, file_array)

        self.end_time = time.time()


def main(args=None):
    rclpy.init(args=args)

    gh360_teleop = GH360Teleop()

    # rclpy.spin(robosuite_teleop)
    # while True:
    #     rclpy.spin_once(robosuite_teleop)
    gh360_teleop.collect_demonstrations("expert", 20)
    gh360_teleop.write_limits_file()
    # robosuite_teleop.collect_gradual_demonstrations(50)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # robosuite_teleop.get_logger().info("Shutting down")
    # robosuite_teleop.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

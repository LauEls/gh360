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
from gh360_interfaces.msg import SpaceMouse

import sys
from robosuite.wrappers import GymWrapper
from .collect_demos import collect_demonstrations
sys.path.insert(0, '/home/laurenz/phd_project/sac/sac_2')
from wrappers import NormalizedBoxEnv


class GH360Teleop(Node):

    def __init__(self):
        super().__init__('gh360_teleop')
        # self.joint_goal_publisher = self.create_publisher(JointState, '/gh360_joint_states', 10)
        # self.joint_state_msg = JointState()
        # self.joint_state_msg.name = ['shoulder_yaw', 'shoulder_roll', 'shoulder_pitch', 'upperarm_roll', 'elbow', 'forearm_roll', 'wrist_pitch']
        # self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      
        
        self.create_subscription(
            SpaceMouse,
            '/spacemouse',
            self.spacemouse_callback,
            10)
        
        # self.create_subscription(JointState, '/inverse_jacobian', self.inverse_jacobian_callback, 10)
        
        # self.rotation_scaling = 1.0
        # self.translation_scaling = 1.0
        self.reset = False
        self.record_data = False
        self.btn1_pressed = False
        self.btn2_pressed = False


        self.save_file_path = '/home/laurenz/phd_project/ros2_gh360_ws/src/gh360/gh360_examples/data/spacemouse_demonstrations/door/gh360_door_demonstration_v1.npy'
        config_file = '/home/laurenz/phd_project/TD7/runs/door/real_gh360/motor_vel/online/v6_eef_vel_test/variant.json'
        self.expert_paths = ''

        # kwargs_fpath = os.path.join(load_dir, "variant.json")
        try:
            with open(config_file) as f:
                variant = json.load(f)
        except FileNotFoundError:
            print("Error opening default controller filepath at: {}. "
                "Please check filepath and try again.".format(config_file))
            
        env_config = variant["environment_kwargs"]
        env_name = variant["environment_kwargs"].pop("env_name")
        variant["environment_kwargs"].pop("max_joint_pos")
        variant["environment_kwargs"].pop("min_joint_pos")
        variant["environment_kwargs"].pop("max_current")

        raw_env = gym.make('gh360_gym/'+env_name, **env_config, node=self)
        self.env = NormalizedBoxEnv(raw_env)

        self.ep_length = variant["episode_length"]

        # self.env = NormalizedBoxEnv(env)
        self.observation = self.env.reset()
        # self.env.render()
        # joint_positions = self.observation[5:12].tolist()
        # self.joint_state_msg.position = joint_positions

        self.observations = []
        self.next_observations = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.infos = []

        self.paths = []

        self.eef_vel = None
        # self.goal_joint_velocity = None


        while self.eef_vel is None:
            rclpy.spin_once(self)
        
        

        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        # self.last_time = time.time()

    def inverse_jacobian_callback(self, msg):
        self.goal_joint_velocity = np.array(msg.velocity)

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

    def expert_action(self, observation):
        if self.reset:
            self.reset_env()
        # joint_positions = observation[5:12].tolist()
        # self.joint_state_msg.position = joint_positions
        # self.joint_goal_publisher.publish(self.joint_state_msg)

        # self.env.render()
        rclpy.spin_once(self)

        action = self.eef_vel

        # rclpy.spin_once(self)

        return action, self.record_data
    
    def reset_env(self):
        observation = self.env.reset()
        # self.env.render()

        self.reset = False
        self.record_data = False

        return observation

    def collect_demonstrations(self, mode, num_eps: int):
        paths = []

        self.get_logger().info("Collecting expert demonstrations")
        paths = collect_demonstrations(self, mode, num_eps, self.expert_paths)
        self.get_logger().info("Expert demonstrations collected")
        
        file_array = np.array(paths)
        np.save(self.save_file_path, file_array)


def main(args=None):
    rclpy.init(args=args)

    robosuite_teleop = GH360Teleop()

    # rclpy.spin(robosuite_teleop)
    # while True:
    #     rclpy.spin_once(robosuite_teleop)
    robosuite_teleop.collect_demonstrations("expert", 5)
    # robosuite_teleop.collect_gradual_demonstrations(50)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # robosuite_teleop.get_logger().info("Shutting down")
    # robosuite_teleop.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

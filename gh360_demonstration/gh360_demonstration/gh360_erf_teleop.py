#from mujoco_py import MjSim
import gh360_gym
import gym
from robosuite.utils.input_utils import *
import rclpy
import time
import json
import os
from rclpy.node import Node

from gh360_interfaces.srv import LogTime
from std_msgs.msg import String
from gh360_interfaces.msg import SpaceMouse

import sys
from robosuite.wrappers import GymWrapper
from .collect_demos import collect_demonstrations
# sys.path.insert(0, '/home/laurenz/phd_project/sac/sac_2')
# from wrappers import NormalizedBoxEnv


class GH360Teleop(Node):

    def __init__(self):
        super().__init__('gh360_teleop')
        self.get_logger().info("Initializing GH360 Teleop Node")
      
        
        self.create_subscription(
            SpaceMouse,
            'spacemouse',
            self.spacemouse_callback,
            10)
        
        
        self.reset = False
        self.btn1_pressed = False
        self.btn2_pressed = False
        self.future = None

        # config_file = '/home/laurenz/phd_project/TD7/runs/door/real_gh360/eef_vel/online/v7_refactor_test/variant.json'
        self.config_path = '/home/gh360/TD7/runs/door/real_gh360/eef_vel/online/v16_erf/'
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
        env_config = variant["environment_kwargs"]

        raw_env = gym.make('gh360_gym/'+env_name, **env_config, node=self)
        self.env = raw_env


        self.ep_length = variant["episode_length"]

        self.observation = self.env.reset()

        self.eef_vel = None

        self.cli = self.create_client(LogTime, '/erf_log_time')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LogTime.Request()

        while self.eef_vel is None:
            rclpy.spin_once(self)

        
        self.get_logger().info("GH360 Teleop Node initialized")

    def send_request(self, time):
        username = String()
        self.req.username = username
        self.req.time = time
        self.future = self.cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # return self.future.result()

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
            

        if msg.button2:
            self.btn2_pressed = True
        elif not msg.button2 and self.btn2_pressed:
            self.btn2_pressed = False


    def expert_action(self):
        if self.reset:
            self.reset_env()

        rclpy.spin_once(self)

        action = self.eef_vel

        return action
    
    def reset_env(self):
        observation = self.env.reset()

        self.reset = False
        self.start_time = -1
        self.get_logger().info("Start Teleop Demonstration")

        return observation

    def collect_demonstrations(self):
    
        while True:
            success = self.rollout()
            self.end_time = time.time()
            self.get_logger().info("Demonstration Finished in {} seconds".format(self.end_time - self.start_time))
            if success:
                response = self.send_request(self.end_time - self.start_time)
                # self.get_logger().info('Result of erf_log_time: %s' % response.topten)

        

    def rollout(self):
        action = np.zeros(7)
        self.reset_env()
        
        self.start_time = -1
        success = False
        
        if self.future is not None:
            while not self.future.done():
                time.sleep(1)

        while not success:
            
            action = self.expert_action()
            
            if self.start_time == -1 and action.any() != 0.0:
                self.start_time = time.time()
                self.get_logger().info("Time started")

            if not self.reset:
                next_observation, reward, done, info = self.env.step(action)
            # print(f"next_observation: {len(next_observation)}")

            if reward == 1:
                success = True
                self.get_logger().info(f"Epsiode successful")
                return True

        return False


def main(args=None):
    rclpy.init(args=args)

    gh360_teleop = GH360Teleop()

    try:
        gh360_teleop.collect_demonstrations()
    except KeyboardInterrupt:
        gh360_teleop.reset_env()


if __name__ == '__main__':
    main()

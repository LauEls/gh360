#from mujoco_py import MjSim
import robosuite as suite
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
sys.path.insert(0, '/home/laurenz/phd_project/sac/sac_2')
from wrappers import NormalizedBoxEnv


class RobosuiteTeleop(Node):

    def __init__(self):
        super().__init__('robosuite_teleop')
        self.joint_goal_publisher = self.create_publisher(JointState, '/gh360_joint_states', 10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['shoulder_yaw', 'shoulder_roll', 'shoulder_pitch', 'upperarm_roll', 'elbow', 'forearm_roll', 'wrist_pitch']
        self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      
        
        self.create_subscription(
            SpaceMouse,
            '/spacemouse',
            self.spacemouse_callback,
            10)
        
        self.create_subscription(JointState, '/inverse_jacobian', self.inverse_jacobian_callback, 10)
        
        self.rotation_scaling = 1.0
        self.translation_scaling = 1.0
        self.reset = False
        self.record_data = False
        self.btn1_pressed = False
        self.btn2_pressed = False
        self.save_file_path = '/home/laurenz/phd_project/ros2_gh360_ws/src/gh360/gh360_examples/data/spacemouse_demonstrations/robosuite/door_mirror/robosuite_door_mirror_demonstration_v3.npy'

        config_file = '/home/laurenz/phd_project/TD7/runs/door_mirror/gh360/joint_velocity/offline/v5_new_gather_method/variant.json'
        
        # kwargs_fpath = os.path.join(load_dir, "variant.json")
        try:
            with open(config_file) as f:
                variant = json.load(f)
        except FileNotFoundError:
            print("Error opening default controller filepath at: {}. "
                "Please check filepath and try again.".format(config_file))
            
        env_config = variant["environment_kwargs"]
        controller = env_config.pop("controller")
        self.ep_length = variant["episode_length"]

        if controller in set(suite.ALL_CONTROLLERS):
            print("Controller: "+controller)
            # This is a default controller
            controller_config = suite.load_controller_config(default_controller=controller)
            
            if "controller_config" in env_config.keys():
                controller_settings = env_config.pop("controller_config")
                for config in controller_settings:
                    controller_config[config] = controller_settings[config]
        else:
            # This is a string to the custom controller
            controller_config = suite.load_controller_config(custom_fpath=controller)
            
        env = suite.make(**env_config,
            #  has_renderer=variant["render"],
            has_renderer=True,
            has_offscreen_renderer=False,
            use_object_obs=True,
            use_camera_obs=False,
            controller_configs=controller_config,
            render_camera="agentview",
            )

        # options = {}
        # options["env_name"] = 'DoorMirror'
        # options["robots"] = 'GH360'
        # options["gripper_types"] = 'HookGripper' #'PandaGripper'
        # controller_name = 'OSC_POSE'
        # controller_name = 'JOINT_VELOCITY'
        # options["controller_configs"] = suite.load_controller_config(default_controller=controller_name)
        # options["table_offset"] = (-0.43, 0.412, 0.81)
        # env = suite.make(
        #     **options,
        #     has_renderer=True,
        #     has_offscreen_renderer=False,
        #     use_object_obs=True,
        #     # ignore_done=True,
        #     use_camera_obs=False,
        #     hard_reset=False,
        #     ignore_done=True,
        #     # control_freq=20,
        #     reward_shaping=True,
        #     render_camera="agentview",
        # )
        self.env = NormalizedBoxEnv(GymWrapper(env))
        self.observation = self.env.reset()
        self.env.render()
        joint_positions = self.observation[5:12].tolist()
        self.joint_state_msg.position = joint_positions

        self.observations = []
        self.next_observations = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.infos = []

        self.paths = []

        self.eef_vel = None
        self.goal_joint_velocity = None
        while self.eef_vel is None or self.goal_joint_velocity is None:
            rclpy.spin_once(self)
        
        

        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        # self.last_time = time.time()

    def inverse_jacobian_callback(self, msg):
        self.goal_joint_velocity = np.array(msg.velocity)
        self.goal_joint_velocity[0] *= self.translation_scaling
        self.goal_joint_velocity[1] *= self.translation_scaling
        self.goal_joint_velocity[2] *= self.translation_scaling
        self.goal_joint_velocity[3] *= self.rotation_scaling
        self.goal_joint_velocity[4] *= self.rotation_scaling
        self.goal_joint_velocity[5] *= self.rotation_scaling

    def spacemouse_callback(self, msg):
        self.eef_vel = np.zeros(6)
        self.eef_vel[0] = msg.velocity.linear.x*self.translation_scaling
        self.eef_vel[1] = msg.velocity.linear.y*self.translation_scaling
        self.eef_vel[2] = msg.velocity.linear.z*self.translation_scaling
        self.eef_vel[3] = -msg.velocity.angular.y*self.rotation_scaling
        self.eef_vel[4] = msg.velocity.angular.x*self.rotation_scaling
        self.eef_vel[5] = -msg.velocity.angular.z*self.rotation_scaling

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

    def collect_demonstrations(self, num_eps: int):
        paths = []
        eps = 0
        steps = 0
        self.get_logger().info(f"Start collectin {num_eps} demonstrations")
        while eps < num_eps:
            print(f"Episode {eps}")
            path = self.demo_episode_rollout()
            steps += len(path['actions'])
            paths.append(path)
            eps += 1

        self.get_logger().info(f"Collected {eps} episodes with {steps} steps. Start generating random data.")
        rnd_steps = 0
        while rnd_steps < steps:
            path = self.exploration_rollout()
            rnd_steps += len(path['actions'])
            paths.append(path)

        self.get_logger().info(f"Finished collecting {rnd_steps} random steps. Saving data to {self.save_file_path}")
        file_array = np.array(paths)
        np.save(self.save_file_path, file_array)
        return paths
    
    def collect_gradual_demonstrations(self, num_eps: int):
        paths = []
        expert_paths = []
        eps = 0
        steps = 0
        self.get_logger().info(f"Start collectin {num_eps} demonstrations")
        while eps < num_eps:
            print(f"Episode {eps}")
            path = self.demo_episode_rollout(horizon=self.ep_length)
            steps += len(path['actions'])
            paths.append(path)
            expert_paths.append(path)
            eps += 1

        self.get_logger().info(f"Collected {eps} episodes with {steps} steps. Start generating hybrid data.")

        expert_ratios = [0.0, 0.25, 0.5, 0.75]
        for expert_ratio in expert_ratios:
            print(f"Expert ratio: {expert_ratio}")
            expert_steps = int(expert_ratio * self.ep_length)
            eps = 0
            while eps < num_eps:
                print(f"Episode {eps}")
                path = self.hybrid_rollout(horizon=self.ep_length, expert_steps=expert_steps, expert_episodes=expert_paths)
                steps += len(path['actions'])
                paths.append(path)
                eps += 1

        self.get_logger().info(f"Finished collecting {steps} steps. Saving data to {self.save_file_path}")

        file_array = np.array(paths)
        np.save(self.save_file_path, file_array)
        return paths

    def hybrid_rollout(self, horizon, expert_steps, expert_episodes):
        action = np.zeros(7)
        observation = self.env.reset()

        observations = []
        next_observations = []
        actions = []
        rewards = []
        dones = []
        infos = []

        #choose random expert episode
        expert_episode = np.random.choice(expert_episodes)

        for i in range(horizon):
            if i < expert_steps:
                #expert action
                action = expert_episode['actions'][i]
            else:
                #random action
                action = self.env.action_space.sample()

            next_observation, reward, done, info = self.env.step(action)
            # self.env.render()

            observations.append(observation)
            next_observations.append(next_observation)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            infos.append(info)

            observation = next_observation

        actions = np.array(actions)
        observations = np.array(observations)
        next_observations = np.array(next_observations)
        rewards = np.array(rewards)
        dones=np.array(dones)
        infos = np.array(infos)

        path = dict(
            observations=observations,
            actions=actions,
            rewards=rewards,
            next_observations=next_observations,
            dones=dones,
            infos=infos,
        )

        return path
        
    def exploration_rollout(self):
        action = np.zeros(7)
        observation = self.env.reset()

        observations = []
        next_observations = []
        actions = []
        rewards = []
        dones = []
        infos = []
        for _ in range(self.ep_length):
            action = self.env.action_space.sample()
            next_observation, reward, done, info = self.env.step(action)

            observations.append(observation)
            next_observations.append(next_observation)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            infos.append(info)

            observation = next_observation

        actions = np.array(actions)
        observations = np.array(observations)
        next_observations = np.array(next_observations)
        rewards = np.array(rewards)
        dones=np.array(dones)
        infos = np.array(infos)

        path = dict(
            observations=observations,
            actions=actions,
            rewards=rewards,
            next_observations=next_observations,
            dones=dones,
            infos=infos,
        )

        return path


    def demo_episode_rollout(self, horizon: int = 0):
        observation = self.env.reset()
        cntr = 0
        self.reset = True
        self.record_data = False  
        succ_cntr = 0
        success = False
        # print("New epsiode")
        while cntr < horizon or not success:
            # print("Step")
            if self.reset or cntr >= horizon:
                self.env.reset()
                self.env.render()
                observations = []
                next_observations = []
                actions = []
                rewards = []
                dones = []
                infos = []
                self.reset = False
                self.record_data = False  
                cntr = 0

            next_observation, reward, done, info = self.env.step(self.goal_joint_velocity)

            # print(f"Reward: {reward}")
            
            if reward == 1.0:
                if succ_cntr == 0:
                    self.get_logger().info("Epsiode success")
                    if horizon != 0:
                        success = True
                succ_cntr += 1
                if succ_cntr > 30 and self.record_data:
                    success = True
            else:
                success = False
                succ_cntr = 0

            if self.record_data:
                actions.append(self.goal_joint_velocity)
                observations.append(observation)
                next_observations.append(next_observation)
                rewards.append(reward)
                dones.append(done)
                infos.append(info)
                cntr += 1

            observation = next_observation
            joint_positions = observation[5:12].tolist()
            self.joint_state_msg.position = joint_positions
            self.joint_goal_publisher.publish(self.joint_state_msg)

            self.env.render()
            rclpy.spin_once(self)

        self.get_logger().info(f"Episode finished after {cntr} steps")

        actions = np.array(actions)
        observations = np.array(observations)
        next_observations = np.array(next_observations)
        rewards = np.array(rewards)
        dones=np.array(dones)
        infos = np.array(infos)

        path = dict(
            observations=observations,
            actions=actions,
            rewards=rewards,
            next_observations=next_observations,
            dones=dones,
            infos=infos,
        )

        return path


def main(args=None):
    rclpy.init(args=args)

    robosuite_teleop = RobosuiteTeleop()

    # rclpy.spin(minimal_publisher)
    # while True:
    #     rclpy.spin_once(robosuite_teleop)
    # robosuite_teleop.collect_demonstrations(100)
    robosuite_teleop.collect_gradual_demonstrations(50)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # robosuite_teleop.get_logger().info("Shutting down")
    # robosuite_teleop.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()

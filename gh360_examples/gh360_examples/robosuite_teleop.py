#from mujoco_py import MjSim
import robosuite as suite
from robosuite.utils.input_utils import *
import rclpy
import time
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
        # self.create_subscription(
        #     Twist,
        #     '/cmd_eef_vel',
        #     self.listener_callback,
        #     10)
        
        self.create_subscription(
            SpaceMouse,
            '/spacemouse',
            self.spacemouse_callback,
            10)
        
        self.create_subscription(JointState, '/inverse_jacobian', self.inverse_jacobian_callback, 10)
        
        self.rotation_scaling = 0.25
        self.translation_scaling = 0.75
        self.reset = False
        self.record_data = False
        self.btn1_pressed = False
        self.btn2_pressed = False
        self.save_file_path = '/home/laurenz/phd_project/TD7/demonstrations/data/robosuite_data.npy'
        
        options = {}
        options["env_name"] = 'DoorMirror'
        options["robots"] = 'GH360'
        options["gripper_types"] = 'HookGripper' #'PandaGripper'
        # controller_name = 'OSC_POSE'
        controller_name = 'JOINT_VELOCITY'
        options["controller_configs"] = suite.load_controller_config(default_controller=controller_name)
        options["table_offset"] = (-0.43, 0.412, 0.81)
        options["obs_optimization"] = True
        env = suite.make(
            **options,
            has_renderer=True,
            has_offscreen_renderer=False,
            use_object_obs=True,
            # ignore_done=True,
            use_camera_obs=False,
            hard_reset=False,
            ignore_done=True,
            # control_freq=20,
            reward_shaping=True,
            
            render_camera="agentview",
        )
        self.env = NormalizedBoxEnv(GymWrapper(env))
        self.observation = self.env.reset()
        # joint_positions = self.observation[11:18].tolist()
        joint_positions = self.observation[5:12].tolist()
        self.joint_state_msg.position = joint_positions

        self.observations = []
        self.next_observations = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.infos = []

        self.paths = []
        

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.last_time = time.time()

    # def listener_callback(self, msg):
    #     self.eef_vel = np.zeros(6)
    #     self.eef_vel[0] = msg.linear.x
    #     self.eef_vel[1] = msg.linear.y
    #     self.eef_vel[2] = msg.linear.z
    #     self.eef_vel[3] = -msg.angular.y*self.rotation_scaling
    #     self.eef_vel[4] = msg.angular.x*self.rotation_scaling
    #     self.eef_vel[5] = -msg.angular.z*self.rotation_scaling

    def inverse_jacobian_callback(self, msg):
        self.goal_joint_velocity = np.array(msg.velocity)

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

    def timer_callback(self):
        if not self.record_data and len(self.observations) > 0:
            actions = np.array(self.actions)
            observations = np.array(self.observations)
            next_observations = np.array(self.next_observations)
            rewards = np.array(self.rewards)
            dones=np.array(self.dones)
            infos = np.array(self.infos)

            self.paths.append(dict(
                observations=observations,
                actions=actions,
                rewards=rewards,
                next_observations=next_observations,
                dones=dones,
                infos=infos,
            ))

            self.reset = True
            self.get_logger().info(f"Saved paths: {len(self.paths)}")
            if len(self.paths) > 100:
                file_array = np.array(self.paths)
                np.save(self.save_file_path, file_array)
                self.paths = []

        if self.reset:
            self.env.reset()
            self.env.render()
            self.observations = []
            self.next_observations = []
            self.actions = []
            self.rewards = []
            self.dones = []
            self.infos = []
            self.reset = False
            self.record_data = False    

        next_observation, reward, done, info = self.env.step(self.goal_joint_velocity)
        # print(f"next_observation: {next_observation}")

        if self.record_data:
            self.actions.append(self.goal_joint_velocity)
            self.observations.append(self.observation)
            self.next_observations.append(next_observation)
            self.rewards.append(reward)
            self.dones.append(done)
            self.infos.append(info)

        self.observation = next_observation

        joint_positions = self.observation[5:12].tolist()
        self.joint_state_msg.position = joint_positions
        self.joint_goal_publisher.publish(self.joint_state_msg)

        self.env.render()
        # print(f"time: {time.time()-self.last_time}")
        # self.last_time = time.time()
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RobosuiteTeleop()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.get_logger().info("Shutting down")
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

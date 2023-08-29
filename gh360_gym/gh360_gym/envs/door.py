import sys
import gym
import numpy as np
import time
from gym import spaces

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros2pkg.api import get_prefix_path
# from dynamixel_sdk_custom_interfaces.msg import SetPosition
# from DynamixelSDK.dynamixel_sdk_custom_interfaces.msg import SetPosition
# sys.path.append('/home/laurenz/phd_project/ros2_gh360_ws/src/DynamixelSDK/dynamixel_sdk_custom_interfaces.msg')
# from dynamixel_sdk_custom_interfaces.msg import SetPosition
from gh360_interfaces.msg import SetMotorPositions, SetPosition
from gh360_interfaces.srv import MotorPositionStep


class DoorEnv(gym.Env):

    def __init__(self):
        """
        Have a variable the defines the action size

        """
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        print("Test")
        # self.motor_publisher = self.node.create_publisher(SetPosition, '/set_position', 10)
        # self.motor_publisher = self.node.create_publisher(SetMotorPositions, "/lowerarm/set_motor_positions", 10)

        self.cli = self.node.create_client(MotorPositionStep, '/lowerarm/motor_positions_step')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        

        self.motor_msg = SetMotorPositions()
        self.internal_state = 0

        low = -np.pi * np.ones(1)
        high = np.pi * np.ones(1)
        self.action_space = spaces.Box(low, high)

        self.obs_dim = 1
        high = np.inf*np.ones(self.obs_dim)
        low = -high
        self.observation_space = spaces.Box(low, high)

    def _get_obs(self):
        """
        Retrieve the following observations:
        Joint Angles
        Eef-Pose (requires having a kinematic model)

        """
        obs = np.array(np.float32(self.internal_state/100))
        #print(obs)
        return obs

    def _get_info(self):
        """
        Maybe motor load signals
        """
        return {"internal_state":self.internal_state}

    def reset(self):
        self.internal_state = 0
        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def step(self, action):
        """
        Send action to the arm controller

        """
        self.motor_msg = SetMotorPositions()
        # self.motor_pos_req = MotorPositionStep()
        self.motor_pos_req = MotorPositionStep.Request()

        set_motor_msg = SetPosition()
        set_motor_msg.id = 60
        set_motor_msg.position = action[0]

        self.motor_msg.motor_goal_positions.append(set_motor_msg)
        self.motor_pos_req.motor_goal_positions.append(set_motor_msg)

        set_motor_msg = SetPosition()
        set_motor_msg.id = 61
        set_motor_msg.position = action[0]

        self.motor_msg.motor_goal_positions.append(set_motor_msg)
        self.motor_pos_req.motor_goal_positions.append(set_motor_msg)


        set_motor_msg = SetPosition()
        set_motor_msg.id = 62
        set_motor_msg.position = action[0]

        self.motor_msg.motor_goal_positions.append(set_motor_msg)
        self.motor_pos_req.motor_goal_positions.append(set_motor_msg)

        # self.motor_publisher.publish(self.motor_msg)
        self.future = self.cli.call_async(self.motor_pos_req)
        rclpy.spin_until_future_complete(self.node, self.future)
        
        print(self.future.result().motor_status[0].present_position)

        # self.internal_state += action
        # self.motor_msg.id = 30
        # self.motor_msg.position = self.internal_state

        # self.control_timestep = 0.2
        # self.model_timestep = 0.05

        # start = time.time()
        # for i in range(int(self.control_timestep / self.model_timestep)):
        # # for i in range(4):
        #     self.motor_publisher.publish(self.motor_msg)
        #     time.sleep(self.model_timestep)

        # end = time.time()
        # print(end-start)
        
        reward = self.reward()
        observation = self._get_obs()
        info = self._get_info()
        done = reward >= 0.99
        # done = False

        return observation, reward, done, info


    def reward(self):
        """
        Compute the reward signal
        """
        obs = self._get_obs()
        reward = np.tanh(obs)
        return reward

    def render(self):
        """
        Currently no rendering implemented since this environment is on the real robot.
        In the future a rendering of the camera view could be implemented
        """
        pass

    def close(self):
        pass


# def main(args=None):
#     rclpy.init(args=args)

#     minimal_subscriber = DoorEnv()

#     # rclpy.spin(minimal_subscriber)

#     # # Destroy the node explicitly
#     # # (optional - otherwise it will be done automatically
#     # # when the garbage collector destroys the node object)
#     # minimal_subscriber.destroy_node()
#     # rclpy.shutdown()

# if __name__ == '__main__':
#     main()
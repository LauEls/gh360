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
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates
from gh360_interfaces.srv import MotorPositionStep

from gh360_gym.envs.utils import SoftJoint, MotorJoint


class DoorEnv(gym.Env):

    def __init__(self,
                 input_max=1,
                 input_min=-1,
                 ):
        """
        Have a variable the defines the action size

        """
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.__class__.__name__)

        # self.motor_publisher = self.node.create_publisher(SetPosition, '/set_position', 10)
        # self.motor_publisher = self.node.create_publisher(SetMotorPositions, "/lowerarm/set_motor_positions", 10)

        self.control_dim = 13#MAYBE READ THAT OUT OF A CONFIG FILE -> should be 13 at the end
        # print("control dimensions: ",self.control_dim)

        # input and output max and min (allow for either explicit lists or single numbers)
        # self.input_max = self.nums2array(input_max, self.control_dim)
        self.input_max = np.ones(self.control_dim) * input_max
        # self.input_min = self.nums2array(input_min, self.control_dim)
        self.input_min = np.ones(self.control_dim) * input_min

        # self.node.create_subscriber()

        self.node.create_subscription(
            ArmEncoderStates,
            '/encoder_status',
            self.encoder_callback,
            10
        )

        self.client_shoulder = self.node.create_client(MotorPositionStep, '/shoulder/motor_delta_positions_step')
        while not self.client_shoulder.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_upperarm = self.node.create_client(MotorPositionStep, '/upperarm/motor_delta_positions_step')
        while not self.client_upperarm.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.client_lowerarm = self.node.create_client(MotorPositionStep, '/lowerarm/motor_delta_positions_step')
        while not self.client_lowerarm.wait_for_service(timeout_sec=1.0):
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

        self.arm = []
        new_joint = SoftJoint(joint_name="shoulder_yaw", port_name="shoulder", id_right_motor=1, id_left_motor=2)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="shoulder_roll", port_name="shoulder", id_right_motor=3, id_left_motor=4)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="shoulder_pitch", port_name="shoulder", id_right_motor=5, id_left_motor=6)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="upperarm_roll", port_name="upperarm", id_right_motor=7, id_left_motor=8)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="elbow", port_name="upperarm", id_right_motor=10, id_left_motor=9)
        self.arm.append(new_joint)
        new_joint = MotorJoint(joint_name="lowerarm_roll", port_name="lowerarm", id_motor=11)
        self.arm.append(new_joint)
        new_joint = SoftJoint(joint_name="wrist_pitch", port_name="lowerarm", id_right_motor=13, id_left_motor=12)
        self.arm.append(new_joint)

        
        

        
    def encoder_callback(self, msg):
        for joint_msg in msg.current_joint_states:
            for joint in self.arm:
                if joint.joint_name == joint_msg.joint_name:
                    joint.joint_velocity = joint_msg.current_vel
                    joint.joint_angle = joint_msg.current_pos
                    # gui_joint.joint_angle.config(text="Joint Angle: "+self.get_label_str(joint.current_pos))

    def _get_obs(self):
        """
        Retrieve the following observations:
        Joint Angles
        Eef-Pose (requires having a kinematic model)

        OrderedDict:
            robot0_joint_pos
            robot0_joint_pos_cos
            robot0_joint_pos_sin
            robot0_joint_vel
            robot0_eef_pos
            robot0_eef_quat

            robot0_gripper_qpos (empty for hook)
            robot0_gripper_qvel
            
            door_pos
            handle_pos
            door_to_eef_pos
            handle_to_eef_pos
            hinge_qpos
            handle_qpos
            robot0_proprio-state (all the robot related data from above combined in one array) -> this + object state concatonated is what goes into the network
            object-state (all the object related data from above comined in one array)
        """
        obs = []
        obs_joint_pos = []
        obs_joint_pos_cos = []
        obs_joint_pos_sin = []

        #ROBOT SPECIFIC PARAMETERS
        for joint in self.arm:
            # if type(joint) == MotorJoint:
            #     print("MotorJoint")
            obs_joint_pos.append(joint.joint_angle)
            obs_joint_pos_cos.append(np.cos(joint.joint_angle))
            obs_joint_pos_sin.append(np.sin(joint.joint_angle))

        #CALCUALATE FORWARD KINEMATICS TO GET EEF POS AND QUAT


        #ENVIRONMENT SPECIFIC PARAMETERS

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
    
    def set_eq_motor_goal(self, delta_action):
        """
        Translate action to motor movements for equilibrium point control
        """
        assert len(delta_action) == self.control_dim, "Delta torque must be equal to the robot's joint dimension space!"

        delta_action = np.clip(delta_action, self.input_min, self.input_max)
        delta_action = delta_action/10

        joint_iter = 0
        motor_iter = 0

        motor_pos_req = MotorPositionStep.Request()

        while motor_iter < len(delta_action):
            if type(self.arm[joint_iter]) == SoftJoint:
                delta_eq_pos = delta_action[motor_iter]
                delta_stiffness = delta_action[motor_iter+1]

                delta_motor_right = delta_eq_pos + delta_stiffness
                delta_motor_left = delta_eq_pos - delta_stiffness

                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[joint_iter].id_right_motor
                set_motor_msg.position = delta_motor_right

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[joint_iter].id_left_motor
                set_motor_msg.position = delta_motor_left

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                motor_iter += 2
            else:
                delta_motor_pos = delta_action[motor_iter]

                set_motor_msg = SetPosition()
                set_motor_msg.id = self.arm[joint_iter].id_motor
                set_motor_msg.position = delta_motor_pos

                motor_pos_req.motor_goal_positions.append(set_motor_msg)

                motor_iter += 1

            joint_iter += 1

        
        return motor_pos_req



        


    def step(self, action):
        """
        Send action to the arm controller

        """
        self.motor_pos_req = self.set_eq_motor_goal(action)
        zero_step = np.zeros(13)
        zero_action_req = self.set_eq_motor_goal(zero_step)

        # print(self.motor_pos_req)

        self.control_timestep = 0.2
        self.model_timestep = 0.1

        policy_step = True

        # start = time.time()
        #THIS MIGHT NOT WORK WITH DELTA POSITION SERVICE BECAUSE IT SENDS THE COMMAND MULTIPLE TIMES??????????????????????????
        for i in range(int(self.control_timestep / self.model_timestep)):
        # for i in range(4):
            # start_2 = time.time()
            start = time.time()
            if policy_step:
                # self.motor_publisher.publish(self.motor_msg)
                self.shoulder_future = self.client_shoulder.call_async(self.motor_pos_req)
                self.upperarm_future = self.client_upperarm.call_async(self.motor_pos_req)
                self.lowerarm_future = self.client_lowerarm.call_async(self.motor_pos_req)
                policy_step = False
            else:
                self.shoulder_future = self.client_shoulder.call_async(zero_action_req)
                self.upperarm_future = self.client_upperarm.call_async(zero_action_req)
                self.lowerarm_future = self.client_lowerarm.call_async(zero_action_req)
               
            rclpy.spin_until_future_complete(self.node, self.shoulder_future)
            rclpy.spin_until_future_complete(self.node, self.upperarm_future)
            rclpy.spin_until_future_complete(self.node, self.lowerarm_future)
            self.shoulder_motor_states_msg = self.shoulder_future.result()
            self.upperarm_motor_states_msg = self.upperarm_future.result()
            self.lowerarm_motor_states_msg = self.lowerarm_future.result()

            end = time.time()
            sleep_time = self.model_timestep - (end-start)
            if sleep_time > 0.0:
            # print(sleep_time)
                time.sleep(sleep_time)
            # end_2 = time.time()
            # print(end_2 - start)

        # end = time.time()
        # print(end-start)

        observation = self._get_obs()
        reward = self.reward()
        info = self._get_info()
        done = reward >= 0.99
        # done = False

        return observation, reward, done, info


    def reward(self):
        """
        Compute the reward signal
        """
        # Handle Pos
        # Eef Pos
        # handle q pos
        # hinge q pos
        # if possible touch door handle
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
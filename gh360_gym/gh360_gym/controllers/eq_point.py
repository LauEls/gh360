from gh360_gym.controllers.base import BaseController
from gh360_gym.utils.joints import MotorJoint, SoftJoint
from gh360_gym.utils.motor_interfaces import generate_velocities_msg, generate_positions_msg
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from gh360_interfaces.msg import SetMotorPositions, SetPosition, ArmEncoderStates, SetVelocity, PortStatus

import numpy as np
import time

import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class EqPointController(BaseController):
    def __init__(self, node, stiffness_mode = 'variable', input_max=1, input_min=-1):
        super().__init__(node)

        self.stiffness_mode = stiffness_mode
        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        if self.stiffness_mode == "variable":
            self.control_dim = 13#MAYBE READ THAT OUT OF A CONFIG FILE -> should be 13 at the end
        elif self.stiffness_mode == "fixed" or self.stiffness_mode == "no_stiffness":
            self.control_dim = 7
        print("control dimensions: ", self.control_dim)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.input_max = np.ones(self.control_dim) * input_max
        self.input_min = np.ones(self.control_dim) * input_min

        self.reseted = False

        

    def get_obs(self):
        obs = []
        robot_joint_pos = []
        robot_joint_pos_cos = []
        robot_joint_pos_sin = []
        robot_joint_vel = []
        robot_eef_pos = []
        robot_eef_quat = []

        robot_gripper_qpos = []
        robot_gripper_qvel = []

        for joint in self.arm:
            # if type(joint) == MotorJoint:
            #     print("MotorJoint")
            robot_joint_pos.append(joint.joint_angle)
            robot_joint_pos_cos.append(np.cos(joint.joint_angle))
            robot_joint_pos_sin.append(np.sin(joint.joint_angle))
            robot_joint_vel.append(joint.joint_velocity)

        from_frame_rel = 'eef'
        to_frame_rel = 'base_link'
        #Lookup the tranformation from from_frame_rel to to_frame_rel
        try:
            eef_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.node.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        #Tranform a Pose from from_frame_rel to to_frame_rel
        # eef_pose = tf2_geometry_msgs.do_transform_pose(Pose(), self.trans_eef_base)
        robot_eef_pos = np.array([eef_pose_trans.transform.translation.x, eef_pose_trans.transform.translation.y, eef_pose_trans.transform.translation.z], dtype=np.float64)
        robot_eef_quat = np.array([eef_pose_trans.transform.rotation.x, eef_pose_trans.transform.rotation.y, eef_pose_trans.transform.rotation.z, eef_pose_trans.transform.rotation.w], dtype=np.float64)

        motor_pos = []
        for joint in self.arm:
            if type(joint) == SoftJoint:
                motor_pos.append(joint.right_motor_pos)
                motor_pos.append(joint.left_motor_pos)

        names = ["robot_joint_pos", "robot_joint_pos_cos", "robot_joint_pos_sin", "robot_joint_vel", "robot_eef_pos", "robot_eef_quat", "motor_pos"]
        obs_dict = dict(zip(names, [robot_joint_pos, robot_joint_pos_cos, robot_joint_pos_sin, robot_joint_vel, robot_eef_pos, robot_eef_quat, motor_pos]))

        return obs_dict

    def robot_safety_check(self):
        safety_check = True
        for joint in self.arm:
            if type(joint) == SoftJoint:
                if not joint.right_motor_safety_check or not joint.left_motor_safety_check:
                    safety_check = False
                    break
            elif type(joint) == MotorJoint:
                if not joint.motor_safety_check:
                    safety_check = False
                    print("Forearm Roll safety check: "+str(safety_check))
                    break

        if not safety_check:
            self.stop_motors()

        return safety_check

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

        # rclpy.spin_once(self.node)
        while self.robot_moving_check():
            rclpy.spin_once(self.node)
            if not self.robot_safety_check():
                # wait for user_input
                input("Press Enter to continue...")
                pass

        self.stop_motors()

        time.sleep(1)

    def set_motor_goal(self, delta_action):
        """
        Translate action to motor movements for equilibrium point control
        """
        assert len(delta_action) == self.control_dim, "Delta torque must be equal to the robot's joint dimension space!"

        delta_action = np.clip(delta_action, self.input_min, self.input_max)
        delta_action = delta_action/10

        joint_iter = 0
        motor_iter = 0

        motor_goal = []

        while motor_iter < len(delta_action):
            if type(self.arm[joint_iter]) == SoftJoint:
                delta_eq_pos = delta_action[motor_iter]

                if delta_eq_pos > 0 and self.arm[joint_iter].joint_angle >= self.arm[joint_iter].max_pos:
                    # print("max joint pos reached")
                    delta_eq_pos = 0.0
                elif delta_eq_pos < 0 and self.arm[joint_iter].joint_angle <= self.arm[joint_iter].min_pos:
                    # print("min joint pos reached")
                    delta_eq_pos = 0.0
                elif delta_eq_pos > 0 and (self.arm[joint_iter].right_motor_current >= self.arm[joint_iter].max_current or self.arm[joint_iter].left_motor_current >= self.arm[joint_iter].max_current):
                    delta_eq_pos = 0.0
                elif delta_eq_pos < 0 and (self.arm[joint_iter].right_motor_current <= self.arm[joint_iter].min_current or self.arm[joint_iter].left_motor_current <= self.arm[joint_iter].min_current):
                    delta_eq_pos = 0.0

                if self.control_dim == 13:
                    delta_stiffness = delta_action[motor_iter+1]
                    motor_iter += 2
                else:
                    delta_stiffness = 0
                    motor_iter += 1

                delta_motor_right = delta_eq_pos + delta_stiffness
                delta_motor_left = delta_eq_pos - delta_stiffness

                if self.motor_controller == "velocity":
                    motor_right_velocity = delta_motor_right / self.control_timestep
                    motor_left_velocity = delta_motor_left / self.control_timestep
                    motor_goal.append(motor_right_velocity)
                    motor_goal.append(motor_left_velocity)
                elif self.motor_controller == "position":
                    motor_goal.append(delta_motor_right)
                    motor_goal.append(delta_motor_left)                
            elif type(self.arm[joint_iter]) == MotorJoint:
                delta_motor_pos = delta_action[motor_iter]

                if delta_motor_pos > 0:
                    if self.arm[joint_iter].joint_angle + delta_motor_pos > self.arm[joint_iter].max_pos:
                        delta_motor_pos = self.arm[joint_iter].max_pos - self.arm[joint_iter].joint_angle
                        # print("max joint pos reached")
                    if self.arm[joint_iter].motor_current >= self.arm[joint_iter].max_current:
                        delta_motor_pos = 0.0
                else:
                    if self.arm[joint_iter].joint_angle + delta_motor_pos < self.arm[joint_iter].min_pos:
                        delta_motor_pos = self.arm[joint_iter].min_pos - self.arm[joint_iter].joint_angle
                        # print("min joint pos reached")
                    if self.arm[joint_iter].motor_current <= self.arm[joint_iter].min_current:
                        delta_motor_pos = 0.0


                if self.motor_controller == "velocity":
                    motor_velocity = delta_motor_pos / self.control_timestep
                    motor_goal.append(motor_velocity)
                elif self.motor_controller == "position":
                    motor_goal.append(delta_motor_pos)


                motor_iter += 1

            joint_iter += 1

        if self.motor_controller == "velocity":
            motor_req = generate_velocities_msg(self.arm, motor_goal)
        elif self.motor_controller == "position":
            motor_req = generate_positions_msg(self.arm, motor_goal)

        if not self.robot_safety_check():
            # wait for user_input
            input("Press Enter to continue...")
            pass
        self.reseted = False

        start = time.time()

        if self.motor_controller == "velocity":
            self.shoulder_future = self.client_velocity_shoulder.call_async(motor_req)
            self.upperarm_future = self.client_velocity_upperarm.call_async(motor_req)
            self.lowerarm_future = self.client_velocity_lowerarm.call_async(motor_req)
        elif self.motor_controller == "position":
            self.shoulder_future = self.client_delta_shoulder.call_async(motor_req)
            self.upperarm_future = self.client_delta_upperarm.call_async(motor_req)
            self.lowerarm_future = self.client_delta_lowerarm.call_async(motor_req)
            print("sending position request")

        while (time.time() - start) < self.control_timestep:
            rclpy.spin_once(self.node)
    
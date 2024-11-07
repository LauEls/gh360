import rosbag2_py
import numpy as np

from dataclasses import dataclass
from rclpy.serialization import deserialize_message
from enum import Enum

from gh360_interfaces.msg import SpaceMouse, PortStatus, SetMotorVelocities, SetVelocity, DoorEnv
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

class JointNames(Enum):
    SHOULDERYAW = 1
    SHOULDERROLL = 2
    SHOULDERPITCH = 3
    UPPERARMROLL = 4
    ELBOW = 5
    FOREARMROLL = 6
    WRISTPITCH = 7

@dataclass
class ROSBagMsg:
    time: int
    data: any

@dataclass
class ROSBagUtil:
    def __init__(self, file_path):
        
        self.num_motors = 13
        self.num_joints = 7
        self.shoulder_motor_ids = [1, 2, 3, 4, 5, 6]
        self.upperarm_motor_ids = [7, 8, 9, 10]
        self.forearm_motor_ids = [11, 12, 13]

        self.motor_positions = {}
        self.motor_velocities = {}
        self.motor_currents = {}
        self.motor_goal_velocities = {}
        self.joint_positions = {}
        self.joint_velocities = {}
        self.door_handle_positions = []
        self.eef_poses = []

        self.observations = []
        self.actions = []
        self.next_observations = []
        self.rewards = []
        self.dones = []
        self.infos = []

        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            # print(f"motor_{i}")
            self.motor_positions[f'motor_{i}'] = []
            self.motor_velocities[f'motor_{i}'] = []
            self.motor_currents[f'motor_{i}'] = []
            self.motor_goal_velocities[f'motor_{i}'] = []
            if i <= self.num_joints:
                self.joint_positions[f'joint_{i}'] = []
                self.joint_velocities[f'joint_{i}'] = []

        self.read_bag_data(file_path)
        self.filter_bag_data()
        # self.generate_demonstration_set()

    def read_bag_data(self, file_path):
        rosbag_reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=file_path,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        rosbag_reader.open(storage_options, converter_options)

        while rosbag_reader.has_next():
            topic, msg, t = rosbag_reader.read_next()

            if topic.endswith("motor_status"):
                msg_dec = deserialize_message(msg, PortStatus)
                for motor in msg_dec.motors:
                    if motor.motor_id in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
                        self.motor_positions[f"motor_{motor.motor_id}"].append(ROSBagMsg(data=motor.present_position,time=t))
                        self.motor_velocities[f"motor_{motor.motor_id}"].append(ROSBagMsg(data=motor.present_velocity,time=t))
                        self.motor_currents[f"motor_{motor.motor_id}"].append(ROSBagMsg(data=motor.present_current,time=t))
            elif topic.endswith("motor_goal_velocity"):
                msg_dec = deserialize_message(msg, SetMotorVelocities)
                port = topic.split("/")[1]
                for motor_vel in msg_dec.motor_goal_velocities:
                    if motor_vel.id in self.shoulder_motor_ids and port == "shoulder":
                        self.motor_goal_velocities[f"motor_{motor_vel.id}"].append(ROSBagMsg(data=motor_vel.velocity,time=t))
                    elif motor_vel.id in self.upperarm_motor_ids and port == "upperarm":
                        self.motor_goal_velocities[f"motor_{motor_vel.id}"].append(ROSBagMsg(data=motor_vel.velocity,time=t))
                    elif motor_vel.id in self.forearm_motor_ids and port == "lowerarm":
                        self.motor_goal_velocities[f"motor_{motor_vel.id}"].append(ROSBagMsg(data=motor_vel.velocity,time=t))
            elif topic == "/gh360_joint_states":
                msg_dec = deserialize_message(msg, JointState)
                for i in range(len(msg_dec.name)):
                    joint_id = JointNames[msg_dec.name[i].upper().replace(" ", "").replace("_","")].value
                    self.joint_positions[f"joint_{joint_id}"].append(ROSBagMsg(data=msg_dec.position[i],time=t))
                    self.joint_velocities[f"joint_{joint_id}"].append(ROSBagMsg(data=msg_dec.velocity[i],time=t))
            elif topic == "/door_env":
                msg_dec = deserialize_message(msg, DoorEnv)
                self.door_handle_positions.append(ROSBagMsg(data=msg_dec.handle_position, time=t))
            elif topic == "/eef_pose":
                msg_dec = deserialize_message(msg, Pose)
                self.eef_poses.append(ROSBagMsg(data=msg_dec, time=t))




    def filter_bag_data(self):
        start_move_time = self.motor_goal_velocities["motor_1"][-1].time
        end_move_time = 0

        # Find the start and end time of the movement
        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            for motor_goal_vel in self.motor_goal_velocities[f"motor_{i}"]:
                if motor_goal_vel.data != 0.0:
                    start_move_time = motor_goal_vel.time - 1.0e9
                    break
            for motor_goal_vel in reversed(self.motor_goal_velocities[f"motor_{i}"]):
                if motor_goal_vel.data != 0.0:
                    end_move_time = motor_goal_vel.time + 1.0e9
                    break

        # Filter the data to only include the movement
        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            self.motor_positions[f"motor_{i}"] = [msg for msg in self.motor_positions[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            self.motor_velocities[f"motor_{i}"] = [msg for msg in self.motor_velocities[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            self.motor_currents[f"motor_{i}"] = [msg for msg in self.motor_currents[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            self.motor_goal_velocities[f"motor_{i}"] = [msg for msg in self.motor_goal_velocities[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            if i <= self.num_joints:
                self.joint_positions[f"joint_{i}"] = [msg for msg in self.joint_positions[f"joint_{i}"] if start_move_time <= msg.time <= end_move_time]
                self.joint_velocities[f"joint_{i}"] = [msg for msg in self.joint_velocities[f"joint_{i}"] if start_move_time <= msg.time <= end_move_time]

        self.door_handle_positions = [msg for msg in self.door_handle_positions if start_move_time <= msg.time <= end_move_time]
        self.eef_poses = [msg for msg in self.eef_poses if start_move_time <= msg.time <= end_move_time]

    def get_velocity_goal_steps(self):
        velocity_goal_steps = []

        max_length = max([len(self.motor_goal_velocities[f"motor_{i}"]) for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids])

        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            step_cntr = 0
            vel_sum = 0
            vel_cntr = 0
            motor_vel_goal_steps = []
            t_init = self.motor_goal_velocities[f"motor_{i}"][0].time
            # t_list = self.motor_goal_velocities[f"motor_{i}"][:].time
            # t_list = [x - t_list[0] for x in t_list]
            for msg in self.motor_goal_velocities[f"motor_{i}"]:
                t = msg.time - t_init
                vel_sum += msg.data
                vel_cntr += 1
                if t >= step_cntr*200e6:
                    motor_vel_goal_steps.append(vel_sum/vel_cntr)
                    vel_sum = 0
                    vel_cntr = 0
                    step_cntr += 1

            # for _ in range(len(motor_vel_goal_steps), max_length+1):
            #     motor_vel_goal_steps.append(0.0)
            
            velocity_goal_steps.append(motor_vel_goal_steps)

        min_length = min(len(vector) for vector in velocity_goal_steps)
        velocity_goal_steps = [vector[:min_length] for vector in velocity_goal_steps]

        np_vel_goal_steps = np.array(velocity_goal_steps)
        np_vel_goal_steps = np_vel_goal_steps.T


        return np_vel_goal_steps
    
    def generate_rewards(self, gripper_to_handle_list):
        rewards = []
        for gripper_to_handle in gripper_to_handle_list:
            dist = np.linalg.norm(gripper_to_handle)
            reward = 1.0 * (1 - np.tanh(10.0*dist))
            rewards.append(reward)

        rewards = np.array(rewards)
        return rewards
    
    def generate_demonstration_set(self):
        self.py_obs = []

        # Generate Motor Position and Velocity Observation Steps
        motor_positions = []
        motor_velocities = []
        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            new_motor_positions = []
            new_motor_velocities = []
            t_init = self.motor_positions[f"motor_{i}"][0].time
            step_cntr = 0
            for msg in self.motor_positions[f"motor_{i}"]:
                t = msg.time - t_init
                if t>= step_cntr*200e6:
                    new_motor_positions.append(msg.data)
                    step_cntr += 1
                # motor_positions.append(msg.data)
            step_cntr = 0
            t_init = self.motor_velocities[f"motor_{i}"][0].time
            for msg in self.motor_velocities[f"motor_{i}"]:
                t = msg.time - t_init
                if t>= step_cntr*200e6:
                    new_motor_velocities.append(msg.data)
                    step_cntr += 1
                # new_motor_velocities.append(msg.data)

            motor_positions.append(new_motor_positions)
            motor_velocities.append(new_motor_velocities)
            # self.observations.append(np.array([motor_positions, motor_velocities]).T)

        min_length = min(len(vector) for vector in motor_positions)
        motor_positions = [vector[:min_length] for vector in motor_positions]
        motor_positions = np.array(motor_positions).T
        print(f"motor position shape: {motor_positions.shape}")

        min_length = min(len(vector) for vector in motor_velocities)
        motor_velocities = [vector[:min_length] for vector in motor_velocities]
        motor_velocities = np.array(motor_velocities).T
        print(f"motor velocity shape: {motor_velocities.shape}")
        
        #Generate Joint Position and Velocity Observation Steps
        joint_positions = []
        joint_velocities = []
        for i in range(1, self.num_joints+1):
            new_joint_positions = []
            new_joint_velocities = []
            t_init = self.joint_positions[f"joint_{i}"][0].time
            step_cntr = 0
            for msg in self.joint_positions[f"joint_{i}"]:
                t = msg.time - t_init
                if t>= step_cntr*200e6:
                    new_joint_positions.append(msg.data)
                    step_cntr += 1
            t_init = self.joint_velocities[f"joint_{i}"][0].time
            step_cntr = 0
            for msg in self.joint_velocities[f"joint_{i}"]:
                t = msg.time - t_init
                if t>= step_cntr*200e6:
                    new_joint_velocities.append(msg.data)
                    step_cntr += 1

            joint_positions.append(new_joint_positions)
            joint_velocities.append(new_joint_velocities)
            # self.observations.append(np.array([joint_positions, joint_velocities]).T)

        min_length = min(len(vector) for vector in joint_positions)
        joint_positions = [vector[:min_length] for vector in joint_positions]
        joint_positions = np.array(joint_positions).T
        print(f"joint position shape: {joint_positions.shape}")

        min_length = min(len(vector) for vector in joint_velocities)
        joint_velocities = [vector[:min_length] for vector in joint_velocities]
        joint_velocities = np.array(joint_velocities).T
        print(f"joint velocity shape: {joint_velocities.shape}")


        # Generate Door Handle Position Observation Steps
        door_handle_positions = []
        t_init = self.door_handle_positions[0].time
        step_cntr = 0
        for msg in self.door_handle_positions:
            t = msg.time - t_init
            if t>= step_cntr*200e6:
                door_handle_positions.append(np.array([msg.data.x, msg.data.y, msg.data.z]))
                step_cntr += 1

        door_handle_positions = np.array(door_handle_positions)
        # print(f"door handle position shape: {door_handle_positions.shape}")

        # Generate EEF Pose Observation Steps
        eef_pos = []
        eef_quat = []
        t_init = self.eef_poses[0].time
        step_cntr = 0
        for msg in self.eef_poses:
            t = msg.time - t_init
            if t>= step_cntr*200e6:
                eef_pos.append(np.array([msg.data.position.x, msg.data.position.y, msg.data.position.z]))
                eef_quat.append(np.array([msg.data.orientation.x, msg.data.orientation.y, msg.data.orientation.z, msg.data.orientation.w]))
                step_cntr += 1
        eef_pos = np.array(eef_pos)
        eef_quat = np.array(eef_quat)

        min_length = min(door_handle_positions.shape[0], eef_pos.shape[0])
        door_handle_positions = door_handle_positions[:min_length]
        eef_pos = eef_pos[:min_length]

        gripper_to_handle = door_handle_positions - eef_pos
        print(f"gripper to handle shape: {gripper_to_handle.shape}")
        print(f"eef_quat shape: {eef_quat.shape}")

        min_length = min(motor_positions.shape[0], motor_velocities.shape[0], joint_positions.shape[0], joint_velocities.shape[0], eef_quat.shape[0], gripper_to_handle.shape[0])
        motor_positions = motor_positions[:min_length]
        motor_velocities = motor_velocities[:min_length]
        joint_positions = joint_positions[:min_length]
        joint_velocities = joint_velocities[:min_length]
        eef_quat = eef_quat[:min_length]
        gripper_to_handle = gripper_to_handle[:min_length]

        self.observations = np.hstack((gripper_to_handle, eef_quat, joint_positions, joint_velocities, motor_positions, motor_velocities))
        print(f"Observations shape: {self.observations.shape}")

        self.rewards = self.generate_rewards(gripper_to_handle)
        print(f"rewards shape: {self.rewards.shape}")

        self.actions = self.get_velocity_goal_steps()
        print(f"actions: {self.actions[0]}")
        print(f"actions shape: {self.actions.shape}")
        indices_to_remove = [1, 3, 5, 7, 9, 12]
        self.actions = np.delete(self.actions, indices_to_remove, axis=1)
        print(f"actions reduced: {self.actions[0]}")
        print(f"actions reduced shape: {self.actions.shape}")

        min_length = min(self.observations.shape[0], self.rewards.shape[0], self.actions.shape[0])
        self.observations = self.observations[:min_length]
        self.rewards = self.rewards[:min_length]
        self.actions = self.actions[:min_length]
        self.dones = np.array([False for _ in range(min_length)])
        self.infos = np.array([{} for _ in range(min_length)])

        print("Shapes after filtering: ")
        print(f"Observations shape: {self.observations.shape}")
        print(f"Rewards shape: {self.rewards.shape}")
        print(f"Actions shape: {self.actions.shape}")

        
        for i in range(self.num_joints):
            print(f"max joint {i} position: {np.max(joint_positions[:,i])}")
            print(f"min joint {i} position: {np.min(joint_positions[:,i])}")
        # print(f"max joint position: {np.max(joint_positions)}")
        # print(f"min joint position: {np.min(joint_positions)}")

        # paths = []

        # for i in range(min_length-1):
        #     paths.append(dict(
        #         observations=self.observations[i],
        #         actions=self.actions[i],
        #         next_observations=self.observations[i+1],
        #         rewards=self.rewards[i+1],
        #         dones=self.dones[i+1],
        #         infos=self.infos[i+1]
        #     ))

        path = dict(
            observations=self.observations,
            actions=self.actions,
            next_observations=self.observations,
            rewards=self.rewards,
            dones=self.dones,
            infos=self.infos
        )

        return path
        
            

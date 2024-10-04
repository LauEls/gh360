import rosbag2_py
import numpy as np

from dataclasses import dataclass
from rclpy.serialization import deserialize_message

from gh360_interfaces.msg import SpaceMouse, PortStatus, SetMotorVelocities, SetVelocity
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

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
        self.motor_goal_velocities = {}
        self.joint_positions = {}

        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            print(f"motor_{i}")
            self.motor_positions[f'motor_{i}'] = []
            self.motor_velocities[f'motor_{i}'] = []
            self.motor_goal_velocities[f'motor_{i}'] = []
            if i <= self.num_joints:
                self.joint_positions[f'joint_{i}'] = []

        self.read_bag_data(file_path)
        self.filter_bag_data()

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

    def filter_bag_data(self):
        start_move_time = self.motor_goal_velocities["motor_1"][-1].time
        end_move_time = 0

        # Find the start and end time of the movement
        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            for motor_goal_vel in self.motor_goal_velocities[f"motor_{i}"]:
                if motor_goal_vel.data != 0.0:
                    start_move_time = motor_goal_vel.time
                    break
            for motor_goal_vel in reversed(self.motor_goal_velocities[f"motor_{i}"]):
                if motor_goal_vel.data != 0.0:
                    end_move_time = motor_goal_vel.time
                    break

        # Filter the data to only include the movement
        for i in self.shoulder_motor_ids + self.upperarm_motor_ids + self.forearm_motor_ids:
            self.motor_positions[f"motor_{i}"] = [msg for msg in self.motor_positions[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            self.motor_velocities[f"motor_{i}"] = [msg for msg in self.motor_velocities[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            self.motor_goal_velocities[f"motor_{i}"] = [msg for msg in self.motor_goal_velocities[f"motor_{i}"] if start_move_time <= msg.time <= end_move_time]
            if i <= self.num_joints:
                self.joint_positions[f"joint_{i}"] = [msg for msg in self.joint_positions[f"joint_{i}"] if start_move_time <= msg.time <= end_move_time]

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

            for _ in range(len(motor_vel_goal_steps), max_length+1):
                motor_vel_goal_steps.append(0.0)
            
            velocity_goal_steps.append(motor_vel_goal_steps)

        np_vel_goal_steps = np.array(velocity_goal_steps)
        np_vel_goal_steps = np_vel_goal_steps.T


        return np_vel_goal_steps
        
            

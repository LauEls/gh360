# import rclpy
import gym
import gh360_gym
import argparse
import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from gh360_interfaces.msg import PortStatus, SetMotorVelocities
from sensor_msgs.msg import JointState

def filter_bag_data(bag_data):
    goal_velocities = bag_data["motor_goal_velocities"]
    start_move_time = 10e20
    end_move_time = 0

    for i in range(1, 14):
        if i < 7:
            time_list = bag_data["time"]["shoulder_motor_goal_velocities"]
        elif i < 11:
            time_list = bag_data["time"]["upperarm_motor_goal_velocities"]
        else:
            time_list = bag_data["time"]["lowerarm_motor_goal_velocities"]

        for z, t in enumerate(time_list):
            if goal_velocities[f"motor_{i}"][z] != 0.0 and t < start_move_time:
                start_move_time = t
                break

        for t in reversed(time_list):
            z = time_list.index(t)
            if goal_velocities[f"motor_{i}"][z] != 0.0 and t > end_move_time:
                end_move_time = t
                break

    for i in range(1, 14):
        if i < 7:
            time_list = bag_data["time"]["shoulder_motor_goal_velocities"]
            time_list_pos = bag_data["time"]["shoulder_motors"]
        elif i < 11:
            time_list = bag_data["time"]["upperarm_motor_goal_velocities"]
            time_list_pos = bag_data["time"]["upperarm_motors"]
        else:
            time_list = bag_data["time"]["lowerarm_motor_goal_velocities"]
            time_list_pos = bag_data["time"]["lowerarm_motors"]

        for z in range(len(time_list) - 1, -1, -1):
            if time_list[z] < start_move_time or time_list[z] > end_move_time:
                # bag["time"][time_list].pop(i)
                bag_data["motor_goal_velocities"][f"motor_{i}"].pop(z)
                if i == 6:
                    bag_data["time"]["shoulder_motor_goal_velocities"].pop(z)
                elif i == 10:
                    bag_data["time"]["upperarm_motor_goal_velocities"].pop(z)
                elif i == 13:
                    bag_data["time"]["lowerarm_motor_goal_velocities"].pop(z)

        for z in range(len(time_list_pos) - 1, -1, -1):
            if time_list_pos[z] < start_move_time or time_list_pos[z] > end_move_time:
                bag_data["motor_positions"][f"motor_{i}"].pop(z)
                if i == 6:
                    bag_data["time"]["shoulder_motors"].pop(z)
                elif i == 10:
                    bag_data["time"]["upperarm_motors"].pop(z)
                elif i == 13:
                    bag_data["time"]["lowerarm_motors"].pop(z)

        
    return bag_data

def read_bag(rosbag_uri):
    rosbag_reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=rosbag_uri,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    rosbag_reader.open(storage_options, converter_options)

    bag_data = {}

    bag_data["motor_positions"] = {}
    bag_data["motor_goal_velocities"] = {}
    for i in range(1, 14):
        bag_data["motor_positions"][f"motor_{i}"] = []
        bag_data["motor_goal_velocities"][f"motor_{i}"] = []
    bag_data["joint_positions"] = {}
    bag_data["joint_positions"]["shoulder_yaw"] = []
    bag_data["joint_positions"]["shoulder_roll"] = []
    bag_data["joint_positions"]["shoulder_pitch"] = []
    bag_data["joint_positions"]["upperarm_roll"] = []
    bag_data["joint_positions"]["elbow"] = []
    bag_data["joint_positions"]["lowerarm_roll"] = []
    bag_data["joint_positions"]["wrist_pitch"] = []
    bag_data["time"] = {}
    bag_data["time"]["joint_positions"] = []
    bag_data["time"]["shoulder_motors"] = []
    bag_data["time"]["upperarm_motors"] = []
    bag_data["time"]["lowerarm_motors"] = []
    bag_data["time"]["shoulder_motor_goal_velocities"] = []
    bag_data["time"]["upperarm_motor_goal_velocities"] = []
    bag_data["time"]["lowerarm_motor_goal_velocities"] = []
    while rosbag_reader.has_next():
        topic, msg, t = rosbag_reader.read_next()

        if topic.endswith("motor_status"):
            msg_dec = deserialize_message(msg, PortStatus)
            for motor in msg_dec.motors:
                if motor.motor_id < 14:
                    bag_data["motor_positions"][f"motor_{motor.motor_id}"].append(motor.present_position)
            port = topic.split("/")[1]
            bag_data["time"][f"{port}_motors"].append(t)
        elif topic.endswith("motor_goal_velocity"):
            msg_dec = deserialize_message(msg, SetMotorVelocities)
            port = topic.split("/")[1]
            # print(f"Message: {msg_dec}")
            for motor_vel in msg_dec.motor_goal_velocities:
                # print(f"Motor {motor_vel.id} Goal Velocity: {motor_vel.velocity}")
                # if motor_vel.id < 14:
                if (port == "shoulder" and motor_vel.id < 7) or (port == "upperarm" and 6 < motor_vel.id < 11) or (port == "lowerarm" and 10 < motor_vel.id < 14):
                    bag_data["motor_goal_velocities"][f"motor_{motor_vel.id}"].append(motor_vel.velocity)
            # print(f"Port: {port}")
            bag_data["time"][f"{port}_motor_goal_velocities"].append(t)

        elif topic == "/gh360_joint_states":
            msg_dec = deserialize_message(msg, JointState)
            bag_data["joint_positions"]["shoulder_yaw"].append(msg_dec.position[0])
            bag_data["joint_positions"]["shoulder_roll"].append(msg_dec.position[1])
            bag_data["joint_positions"]["shoulder_pitch"].append(msg_dec.position[2])
            bag_data["joint_positions"]["upperarm_roll"].append(msg_dec.position[3])
            bag_data["joint_positions"]["elbow"].append(msg_dec.position[4])
            bag_data["joint_positions"]["lowerarm_roll"].append(msg_dec.position[5])
            bag_data["joint_positions"]["wrist_pitch"].append(msg_dec.position[6])
            bag_data["time"]["joint_positions"].append(t)

    return bag_data

def parse_bag_file(bag_file_path):
    # rosbag_reader = rosbag2_py.SequentialReader()
    # storage_options = rosbag2_py._storage.StorageOptions(
    #         uri=bag_file_path,
    #         storage_id='sqlite3')
    # converter_options = rosbag2_py._storage.ConverterOptions('', '')
    # rosbag_reader.open(storage_options, converter_options)

    og_bag_data = filter_bag_data(read_bag(bag_file_path))
    bag_data = {}
    pos_steps = []
    for i in range(1, 14):
        bag_data[f"motor_{i}_position"] = []
        bag_data[f"motor_{i}_time"] = []
        # pos_steps[f"motor_{i}_pos_step"] = []

        if i < 7:
            time_list = og_bag_data["time"]["shoulder_motors"]
        elif i < 11:
            time_list = og_bag_data["time"]["upperarm_motors"]
        else:
            time_list = og_bag_data["time"]["lowerarm_motors"]

        for z, t in enumerate(time_list):
            bag_data[f"motor_{i}_position"].append(og_bag_data["motor_positions"][f"motor_{i}"][z])
            bag_data[f"motor_{i}_time"].append(t)
    
    # while rosbag_reader.has_next():
    #     topic, msg, t = rosbag_reader.read_next()
    #     if topic.endswith("motor_status"):
    #         msg_dec = deserialize_message(msg, PortStatus)
    #         for motor in msg_dec.motors:
    #             if motor.motor_id < 14:
    #                 bag_data[f"motor_{motor.motor_id}_position"].append(motor.present_position)
    #                 bag_data[f"motor_{motor.motor_id}_time"].append(t)


    print(f"Len parsed data: {len(bag_data[f'motor_1_position'])}")
    for i in range(1, 14):
        step_cntr = 0
        motor_steps = []
        t_init = bag_data[f"motor_{i}_time"][0]
        for z, t in enumerate(bag_data[f"motor_{i}_time"]):
            t = t - t_init

            if t >= step_cntr*200e6:
                # pos_steps[f"motor_{i}_pos_step"].append(bag_data[f"motor_{i}_position"][z])
                if t == 0:
                    prev_pos = bag_data[f"motor_{i}_position"][0]
                else:
                    # vel_steps[f"motor_{i}_vel_step"].append((bag_data[f"motor_{i}_position"][z] - prev_pos)/0.05)
                    pos_diff = (bag_data[f"motor_{i}_position"][z] - prev_pos)*10
                    motor_steps.append(pos_diff)
                    # motor_steps.append((pos_diff/2))
                    # motor_steps.append((pos_diff/2))
                    prev_pos = bag_data[f"motor_{i}_position"][z]

                step_cntr += 1
        


        pos_steps.append(motor_steps)

    print(f"len original data: {len(bag_data[f'motor_1_position'])}")

    

    # Determine the minimum length of the sublists
    min_length = min(len(sublist) for sublist in pos_steps)

    # Truncate each sublist to the minimum length
    pos_steps = [sublist[:min_length] for sublist in pos_steps]

    for i in range(1, 14):
        print(f"Motor {i} len: {len(pos_steps[i-1])}")
    # print(f"pos_steps: {pos_steps}")
    np_pos_steps = np.array(pos_steps)
    pos_steps_2 = []
    for i in range(7):
        if i == 5:
            pos_steps_2.append(pos_steps[i*2])
        elif i == 6:
            pos_steps_2.append(np.divide(np.add(pos_steps[i*2-1], pos_steps[i*2]),2))
            pos_steps_2.append(np.multiply(pos_steps[i*2],0.0))
        #     pos_steps_2.append(np.divide(np.add(pos_steps[i], pos_steps[i+6]),2))
        #     pos_steps_2.append(np.multiply(pos_steps[i],0.0))
        else:
            # pos_steps_2.append((pos_steps[i]+pos_steps[i+7])/2)
            pos_steps_2.append(np.divide(np.add(pos_steps[i*2], pos_steps[i*2+1]),2))
            pos_steps_2.append(np.multiply(pos_steps[i*2],0.0))
    np_pos_steps_2 = np.array(pos_steps_2)
    print(f"len step data: {len(pos_steps[0])}")
    print(f"np shape: {np_pos_steps.shape}")
    np_pos_steps = np.transpose(np_pos_steps)
    np_pos_steps_2 = np.transpose(np_pos_steps_2)
    print(f"np shape: {np_pos_steps.shape}")
    print(f"len action: {len(np_pos_steps[0])}")
    print(f"pos steps iter 100: {np_pos_steps[50]}")
    print(f"pos steps 2 iter 100: {np_pos_steps_2[50]}")
    print(f"pos steps iter 101: {np_pos_steps[51]}")
    print(f"pos steps 2 iter 101: {np_pos_steps_2[51]}")
    return np_pos_steps_2
           


def run_step_data(step_data, env):
    base_action = np.zeros(13)

    for action in step_data:
        # print(f"Action: {action}")
        obs, reward, done, _ = env.step(action)

    obs, reward, done, _ = env.step(base_action)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag_file", type=str, default="")

    args = parser.parse_args()
    # env = gym.make('gh360_gym/Door-v0')
    env = gym.make('gh360_gym/TrajectoryFollowing-v0', stiffness_mode="variable", input_max=10, input_min=-10)
    
    env.reset()

    step_bag_data = parse_bag_file(args.bag_file)

    run_step_data(step_bag_data, env)



    
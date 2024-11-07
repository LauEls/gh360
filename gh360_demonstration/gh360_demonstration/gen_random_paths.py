import gym
import gh360_gym
import argparse
import os
import numpy as np

from gh360_demonstration.rosbag_util import ROSBagUtil

def generate_eq_control_steps(velocity_steps):
    # control_freq = np.zeros(13)
    control_freq = 2
    eq_pos_steps = []
    # for i in range(len(control_freq)): control_freq[i] = 2

    for i in range(7):
        if i == 5:
            eq_pos_steps.append(velocity_steps[:,i*2]*control_freq)
        elif i == 6:
            eq_pos_steps.append(np.divide(np.add(velocity_steps[:,i*2-1], velocity_steps[:,i*2]), 2)*control_freq)
            eq_pos_steps.append(np.abs(np.subtract(velocity_steps[:,i*2-1], velocity_steps[:,i*2]))*control_freq)
        else:
            eq_pos_steps.append(np.divide(np.add(velocity_steps[:,i*2], velocity_steps[:,i*2+1]), 2)*control_freq)
            eq_pos_steps.append(np.abs(np.subtract(velocity_steps[:,i*2], velocity_steps[:,i*2+1]))*control_freq)

    # velocity_steps *= control_freq
    np_eq_pos_steps = np.array(eq_pos_steps)
    np_eq_pos_steps = np.transpose(np_eq_pos_steps)
    print(np_eq_pos_steps.shape)

    return np_eq_pos_steps

def generate_random_paths(env, args):
    base_action = np.zeros(7)
    steps = 0

    observations = []
    next_observations = []
    actions = []
    rewards = []
    dones = []
    infos = []

    paths = []
    
    while steps < args.num_steps:
        obs = env.reset()
        for i in range(args.episode_length):
            # print(f"Action: {action}")
            action = env.action_space.sample()
            next_obs, reward, done, info = env.step(action)

            observations.append(obs)
            rewards.append(reward)
            next_observations.append(next_obs)
            actions.append(action)
            dones.append(done)
            infos.append(info)

            obs = next_obs
            steps += 1
        env.step(base_action)


        
        path = dict(
            observations=np.array(observations),
            next_observations=np.array(next_observations),
            actions=np.array(actions),
            rewards=np.array(rewards),
            dones=np.array(dones),
            infos=np.array(infos)
        )
        paths.append(path)

    env.reset()

    paths = np.array(paths)    

    return paths

    # obs, reward, done, _ = env.step(base_action)

def list_of_floats(string):
    string = string.replace('[', '').replace(']', '')
    return [float(x) for x in string.split(',')]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_steps", type=int, default=0)
    parser.add_argument("--episode_length", type=int, default=0)
    # parser.add_argument("--env", type=str, default="Door-v0")
    parser.add_argument("--save_file_name", type=str, default="random_set")
    parser.add_argument("--input_max", type=list_of_floats, default=[])
    parser.add_argument("--input_min", type=list_of_floats, default=[])
    parser.add_argument("--joint_max_pos", type=list_of_floats, default=[])
    parser.add_argument("--joint_min_pos", type=list_of_floats, default=[])
    parser.add_argument("--max_current", type=list_of_floats, default=[])
    args = parser.parse_args()
    env = gym.make('gh360_gym/Door-v0', 
                   input_max=args.input_max, 
                   input_min=args.input_min, 
                   max_joint_pos=args.joint_max_pos, 
                   min_joint_pos=args.joint_min_pos, 
                   max_current=args.max_current,
                   motor_obs=True)
    
    # env = gym.make('gh360_gym/FreeMove-v0', stiffness_mode="variable", input_max=10, input_min=-10)
    
    env.reset()

    # rosbag_util = ROSBagUtil(args.bag_file)
    # vel_steps = rosbag_util.get_velocity_goal_steps()
    # eq_pos_steps = generate_eq_control_steps(vel_steps)
    # step_bag_data = parse_bag_file(args.bag_file)

    random_paths = generate_random_paths(env, args)

    path_ros2_ws = '/home/laurenz/phd_project/ros2_gh360_ws'
    path_learning_data_dir = f'{path_ros2_ws}/src/gh360/gh360_demonstration/data/learning_datasets/'
    save_file_name = args.save_file_name + '_random_paths'
    env_name = "door"
    save_file_path = os.path.join(path_learning_data_dir, env_name, save_file_name)
    np.save(save_file_path, random_paths)


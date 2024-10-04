import gym
import gh360_gym
import argparse
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

    rosbag_util = ROSBagUtil(args.bag_file)
    vel_steps = rosbag_util.get_velocity_goal_steps()
    eq_pos_steps = generate_eq_control_steps(vel_steps)
    # step_bag_data = parse_bag_file(args.bag_file)

    run_step_data(eq_pos_steps, env)
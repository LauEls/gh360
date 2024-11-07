import gym
import gh360_gym

import numpy as np

def run_recorded_actions():
    recorded_actions = np.loadtxt("/home/laurenz/phd_project/sac/scripts/test_data/v6/delta_action.csv", delimiter=",", dtype=float)

    base_action = np.zeros(13)

    for action in recorded_actions[:, 1:14]:
        print(action)
        action *= 10

        # mod_action = np.concatenate((base_action, action), axis=None)
        mod_action = action

        obs, reward, done, _ = env.step(mod_action)

    obs, reward, done, _ = env.step(base_action)
    

# env.reset()

def run_zero_actions():
    # motor_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    motor_action = np.zeros(13)
    inc = 0.01
    cnt = 0
    done = False

    while not done:

        # print(motor_action)
        # if (motor_action[0] > np.pi*6):
        #     break
        obs, reward, done, info = env.step(motor_action)
        print("obs: ", obs)
        cnt += 1

        if cnt > 50: break


    # motor_action += inc
    # print([obs, reward])



if __name__ == "__main__":

    env = gym.make('gh360_gym/Door-v0')
    # env = gym.make('gh360_gym/TrajectoryFollowing-v0', stiffness_mode="no_stiffness",)
    
    env.reset()


    run_zero_actions()
    # run_recorded_actions()

    # env.reset()


import gym
import gh360_gym

import numpy as np

recorded_actions = np.loadtxt("/home/laurenz/phd_project/sac/scripts/test_data/v6/delta_action.csv", delimiter=",", dtype=float)


env = gym.make('gh360_gym/Door-v0')
done = False
env.reset()


motor_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
inc = 0.01
cnt = 0

env.reset()

base_action = np.zeros(11)

for action in recorded_actions[:, 1:14]:
    print(action)
    action *= 10

    # mod_action = np.concatenate((base_action, action), axis=None)
    mod_action = action

    obs, reward, done, _ = env.step(mod_action)

env.reset()
    
# while not done:

#     # print(motor_action)
#     # if (motor_action[0] > np.pi*6):
#     #     break
#     obs, reward, done, info = env.step(motor_action)

#     cnt += 1

#     if cnt > 10: break
#     # motor_action += inc
#     # print([obs, reward])
import gym
import gh360_gym

import numpy as np

env = gym.make('gh360_gym/Door-v0')
done = False
env.reset()

motor_action = np.array([0.0, 0.0, 0.0])
inc = 0.01
while not done:

    print(motor_action)
    if (motor_action[0] > np.pi*6):
        break
    obs, reward, done, info = env.step(motor_action)

    motor_action += inc
    # print([obs, reward])
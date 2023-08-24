import gym
import gh360_gym

env = gym.make('gh360_gym/Door-v0')
done = False
env.reset()
while not done:
    obs, reward, done, info = env.step(1)
    print([obs, reward])
from gym.envs.registration import register

register(
    id="gh360_gym/Door-v0",
    entry_point="gh360_gym.envs:DoorEnv",
)


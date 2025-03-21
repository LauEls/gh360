import numpy as np

def collect_demonstrations(node, mode, num_eps: int, expert_episodes=None):
    paths = []
    eps = 0
    steps = 0
    node.get_logger().info(f"Start collectin {num_eps} demonstrations")
    while eps < num_eps:
        # print(f"Episode {eps}")
        node.get_logger().info(f"Episode {eps}")
        if mode == 'expert':
            path = rollout(node, mode)
        elif mode == 'random':
            path = rollout(node, mode)
        elif mode == 'gradual_random':
            expert_steps = np.random.randint(0, node.ep_length)
            path = rollout(node, mode, expert_steps=expert_steps, expert_episodes=expert_episodes)

        # path = rollout(node, mode)
        steps += len(path['actions'])
        paths.append(path)
        eps += 1

    node.get_logger().info(f"Collected {eps} episodes with {steps} steps. Start generating random data.")
    
    # rnd_steps = 0
    # while rnd_steps < steps:
    #     path = self.exploration_rollout()
    #     rnd_steps += len(path['actions'])
    #     paths.append(path)

    # node.get_logger().info(f"Finished collecting {rnd_steps} random steps.")

    # file_array = np.array(paths)
    # np.save(self.save_file_path, file_array)
    return paths

def rollout(node, mode, expert_steps=0, expert_episodes=None):
    action = np.zeros(7)
    observation = node.reset_env()

    observations = []
    next_observations = []
    actions = []
    rewards = []
    dones = []
    infos = []

    i = 0
    record_data = True
    success = False

    if mode == 'gradual_random':
        expert_episode = np.random.choice(expert_episodes)

    while i < node.ep_length:
        if mode == 'expert':
            action, record_data = node.expert_action(observation)
        elif mode == 'random':
            action = node.env.action_space.sample()
        elif mode == 'gradual_random':
            if i < expert_steps:
                action = expert_episode['actions'][i]
            else:
                action = node.env.action_space.sample()

        next_observation, reward, done, info = node.env.step(action)

        if reward == 1 and not success:
            success = True
            node.get_logger().info(f"Epsiode successful")

        if record_data:
            observations.append(observation)
            next_observations.append(next_observation)
            actions.append(action)
            rewards.append(reward)
            dones.append(done)
            infos.append(info)
            i += 1
        elif not record_data and i > 0:
            node.get_logger().info(f"Reseting environemnt")
            i = 0
            observations = []
            next_observations = []
            actions = []
            rewards = []
            dones = []
            infos = []

            next_observation = node.reset_env()

        observation = next_observation

    actions = np.array(actions)
    observations = np.array(observations)
    next_observations = np.array(next_observations)
    rewards = np.array(rewards)
    dones=np.array(dones)
    infos = np.array(infos)

    path = dict(
        observations=observations,
        actions=actions,
        rewards=rewards,
        next_observations=next_observations,
        dones=dones,
        infos=infos,
    )

    return path
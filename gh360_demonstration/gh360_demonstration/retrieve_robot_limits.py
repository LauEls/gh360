import numpy as np

file_path = '/home/laurenz/phd_project/ros2_gh360_ws/src/gh360/gh360_demonstration/gh360_demonstration/data/spacemouse_demonstrations/door/gh360_door_demonstration_v2.npy'
max_joint_pos = np.ones(7)*-1000
min_joint_pos = np.ones(7)*1000


npy_data = np.load(file_path, allow_pickle=True)

for path in npy_data:
    new_max = np.amax(path['observations'][:, 9:16], axis=0)
    new_min = np.amin(path['observations'][:, 9:16], axis=0)
    for i in range(7):
        if new_max[i] > max_joint_pos[i]:
            max_joint_pos[i] = new_max[i]
        if new_min[i] < min_joint_pos[i]:
            min_joint_pos[i] = new_min[i]

print('Max joint positions: ', max_joint_pos)
print('Min joint positions: ', min_joint_pos)

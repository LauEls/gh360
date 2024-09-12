import subprocess
import signal
import time
import os

# p = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 launch gh360 gh360_startup.launch.py', shell=True, executable="/bin/bash")


# # for i in range(10):
# #     print(".")
# #     time.sleep(1)

# # launch_process.send_signal(SIGINT)
# # launch_process.wait(timeout=5)


# print('sleeping')
# # time.sleep(2)
# for i in range(10):
#     print(p.stderr)
#     time.sleep(1)

# os.killpg(os.getpgid(p.pid), signal.SIGINT)
# print('interrupt')


# p.wait()
# print('process finished')
# ; source ~/phd_project/ros2_gh360_ws/install/setup.bash; ros2 run gh360_examples stream_deck_test
# launch_process = subprocess.Popen(["ros2", "launch", "gh360_examples", "stream_deck_test"], text=True)


p = subprocess.Popen('source ~/phd_project/robosuite_venv/bin/activate; source ~/phd_project/ros2_gh360_ws/install/setup.bash; cd ~/phd_project/ros2_gh360_ws/; colcon build --symlink-install', shell=True, executable="/bin/bash", preexec_fn=os.setsid) # 

time.sleep(5)
# p.kill()
os.killpg(os.getpgid(p.pid), signal.SIGINT)

p.wait()
print('process finished')

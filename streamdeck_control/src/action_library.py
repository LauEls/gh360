from src.process_handler import ProcessHandler
from src.utils import load_icon_byte_arr
from src.environments import Env
import os

venv = '~/phd_project/robosuite_venv'
ros_ws = '~/phd_project/gh360_ws'
ros_bridge_ws = '~/phd_project/bridge_ws'
ros2_ws = '~/phd_project/ros2_gh360_ws'


class KeyActionHandler:
    def __init__(self, process_handler, on_icon, off_icon):
        self.process_handler = process_handler
        self.on_icon = on_icon
        self.off_icon = off_icon

    def keypress(self, env: Env = Env.default):
        print("key press env: ", env)
        if self.process_handler.run_process(env):
            return self.on_icon
        else:
            return self.off_icon
    def close(self):
        self.process_handler.close_process()
        return self.off_icon

def encoder_startup():
    process_handler = ProcessHandler()
    process_handler.add_process(
        f'source {ros_bridge_ws}/install/setup.bash; ros2 run ros1_bridge dynamic_bridge --bridge-all-topics')
    process_handler.add_process(
        f'source {ros_ws}/devel/setup.bash; roslaunch gh360_control encoder_manager.launch', 
        env=Env.no_env)
    process_handler.add_process(
        f'source {ros_ws}/devel/setup.bash; roslaunch gh360_control door_env.launch', 
        env=Env.door)

    action_handler = KeyActionHandler(
        process_handler=process_handler, 
        on_icon=load_icon_byte_arr('code_on_icon.png'), 
        off_icon=load_icon_byte_arr('code_off_icon.png',overlay=True))
    
    return action_handler

def motor_startup():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 launch gh360 gh360_startup.launch.py', 
        env=Env.no_env)
    process_handler.add_process(
        f'{pre_command} ros2 launch gh360 gh360_door_env.launch.py', 
        env=Env.door)
    
    action_handler = KeyActionHandler(
        process_handler=process_handler, 
        on_icon=load_icon_byte_arr('motor_on_icon.png'), 
        off_icon=load_icon_byte_arr('motor_off_icon_v2.png', overlay=True))
    
    return action_handler

def state_monitor():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 run gh360_examples monitor')

    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('gui_on_icon_v2.png'),
        off_icon=load_icon_byte_arr('gui_off_icon_v2.png', overlay=True))
    
    return action_handler

def motor_torque_off():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 service call /shoulder/motor_set_torque std_srvs/srv/SetBool "{{data: False}}"')
    process_handler.add_process(
        f'{pre_command} ros2 service call /upperarm/motor_set_torque std_srvs/srv/SetBool "{{data: False}}"')
    process_handler.add_process(
        f'{pre_command} ros2 service call /lowerarm/motor_set_torque std_srvs/srv/SetBool "{{data: False}}"')
    process_handler.add_process(
        f'{pre_command} ros2 service call /door/motor_set_torque std_srvs/srv/SetBool "{{data: False}}"',
        env=Env.door)

    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('stop_sign_v2.png', white_background=True),
        off_icon=load_icon_byte_arr('stop_sign_v2.png', white_background=True))
    
    return action_handler

def colcon_build():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} cd {ros2_ws}/; colcon build --symlink-install')
    
    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('build_icon.png'),
        off_icon=load_icon_byte_arr('build_icon.png'))
    
    return action_handler

def move_home():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 service call /shoulder/motor_move_home std_srvs/srv/SetBool "{{data: True}}"')
    process_handler.add_process(
        f'{pre_command} ros2 service call /upperarm/motor_move_home std_srvs/srv/SetBool "{{data: True}}"')
    process_handler.add_process(
        f'{pre_command} ros2 service call /lowerarm/motor_move_home std_srvs/srv/SetBool "{{data: True}}"')

    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('home_icon.png'),
        off_icon=load_icon_byte_arr('home_icon.png'))
    
    return action_handler

def spacemouse_teleop():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 launch gh360 spacemouse_teleop.launch.py')
    process_handler.add_process(
        f'{pre_command} ros2 run gh360_examples door_motor_control',
        env=Env.door)
    
    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('spacemouse_icon_3.png'),
        off_icon=load_icon_byte_arr('spacemouse_icon_3.png', overlay=True))
    
    return action_handler

def rosbag_record():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 bag record -o {ros2_ws}/src/gh360/gh360_examples/data/spacemouse_demonstrations/no_env/test /shoulder/motor_goal_velocity /upperarm/motor_goal_velocity /lowerarm/motor_goal_velocity')
    
    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('record_icon.png'),
        off_icon=load_icon_byte_arr('record_icon.png'))
    
    return action_handler

def rosbag_play():
    pre_command = f'source {venv}/bin/activate; source {ros2_ws}/install/setup.bash;'

    process_handler = ProcessHandler()
    process_handler.add_process(
        f'{pre_command} ros2 bag play subset')
    
    action_handler = KeyActionHandler(
        process_handler=process_handler,
        on_icon=load_icon_byte_arr('play_icon.png'),
        off_icon=load_icon_byte_arr('play_icon.png'))
    
    return action_handler
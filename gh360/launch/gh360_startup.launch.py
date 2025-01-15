#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'gh360'
    robot_name = 'gh360'
    model_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', robot_name+'.urdf')
    robot_description_raw = xacro.process_file(model_file_path).toxml()

    motor_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/motor_handler.launch.py'])
    )

    encoder_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/encoder_handler.launch.py'])
    )

    inverse_jacobian_cmd = Node(
        package='gh360',
        executable='inverse_jacobian',
        name='inverse_jacobian',
        # emulate_tty=True,
        parameters=[{
        'robot_description': robot_description_raw,
        'tcp_link_name': 'eef',
        'joint_states_topic': '/gh360_joint_states'}]
    )
    # door_motor = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/door_motor.launch.py'])
    # )
    
    return LaunchDescription([
        motor_handler,
        encoder_handler,
        inverse_jacobian_cmd,
        # door_motor,
    ])
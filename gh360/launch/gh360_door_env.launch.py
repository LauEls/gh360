#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gh360_startup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/gh360_startup.launch.py'])
    )

    door_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/door_motor.launch.py'])
    )

    camera_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/camera_handler.launch.py'])
    )

    handle_sensor_filter_node = Node(
        package='gh360_examples',
        executable='handle_sensor_filter',
        name='handle_sensor_filter',
        namespace='door',
        output='screen')
    
    eef_pos_in_world_cmd = Node(
        package='gh360_examples',
        executable='eef_pos_in_world',
        name='eef_pos_in_world'
    )

    door_env_obs_cmd = Node(
        package='gh360_demonstration',
        executable='door_env_obs',
        name='door_env_obs'
    )
    
    return LaunchDescription([
        gh360_startup,
        door_motor,
        camera_handler,
        handle_sensor_filter_node,
        eef_pos_in_world_cmd,
        door_env_obs_cmd,
    ])
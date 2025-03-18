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

    door_startup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360_examples'), 'launch'), '/door_startup.launch.py'])
    )

    camera_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/camera_handler.launch.py'])
    )

    # door_reset_node = Node(
    #     package='gh360_examples',
    #     executable='door_reset',
    #     name='door_reset',
    #     namespace='door',
    #     output='screen')

    # door_handle_angle_filter_node = Node(
    #     package='gh360_examples',
    #     executable='door_handle_angle_filter',
    #     name='door_handle_angle_filter',
    #     namespace='door',
    #     output='screen')

    # door_env_obs_cmd = Node(
    #     package='gh360_demonstration',
    #     executable='door_env_obs',
    #     name='door_env_obs',
    #     namespace='door',
    #     output='screen')
    
    return LaunchDescription([
        gh360_startup,
        camera_handler,
        door_startup,
        # door_reset_node,
        # door_handle_angle_filter_node,
        # door_env_obs_cmd,
    ])
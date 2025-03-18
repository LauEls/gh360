#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    spacemouse_config_file = PathJoinSubstitution([FindPackageShare('gh360_demonstration'), 'config', 'spacemouse_config.yaml'])

    demo_node = Node(
        package='gh360_demonstration',
        executable='gh360_demo',
        name='gh360_demo',
        namespace='gh360_control',
        output='screen'
    )

    spacemouse_node = Node(
        package='gh360_demonstration',
        executable='spacemouse',
        name='spacemouse',
        namespace='gh360_control',
        parameters=[spacemouse_config_file],
        output='screen')
    
    return LaunchDescription([
        # controller_nodes,
        demo_node,
        spacemouse_node,
    ])
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
    robot_config_file = PathJoinSubstitution([FindPackageShare('gh360'), 'config', 'gh360_config.yaml'])
    controller_config_file = PathJoinSubstitution([FindPackageShare('gh360_control'), 'config', 'controller_config.yaml'])

    controller_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360_control'), 'launch'), '/controllers.launch.py'])
    )

    teleop_node = Node(
        package='gh360_control',
        executable='teleop_eef_velocity',
        name='teleop_eef_velocity',
        namespace='gh360_control',
        parameters=[robot_config_file, controller_config_file],
        output='screen'
    )

    spacemouse_node = Node(
        package='gh360_examples',
        executable='spacemouse',
        name='spacemouse',
        namespace='spacemouse',
        output='screen')
    
    return LaunchDescription([
        controller_nodes,
        teleop_node,
        spacemouse_node,
    ])
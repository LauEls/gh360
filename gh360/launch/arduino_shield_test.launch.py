#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    motor_handler_node = Node(
        package='gh360',
        executable='motor_handler',
        name='motor_handler',
        respawn=True,
        respawn_delay=2,
        namespace='arduino',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'arduino_shield_test.yaml')],
        output='screen')
    
    return LaunchDescription([
        motor_handler_node,
    ])
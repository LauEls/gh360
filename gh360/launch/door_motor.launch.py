#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    door_node = Node(
        package='gh360',
        executable='motor_handler',
        name='motor_handler',
        respawn=True,
        respawn_delay=2,
        namespace='door',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'door_motor_config.yaml')],
        output='screen')
    
    return LaunchDescription([
        door_node,
    ])
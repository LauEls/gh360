#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    return LaunchDescription([

        Node(
            package='gh360',
            executable='encoder_handler',
            name='encoder_handler',
            parameters=[os.path.join(
                get_package_share_directory('gh360'),
                'config', 'encoder_handler_config.yaml')],
            output='screen'),

    ])
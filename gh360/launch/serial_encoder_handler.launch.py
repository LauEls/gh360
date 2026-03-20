#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    shoulder_serial_encoder_handler = Node(
        package='gh360_examples',
        executable='serial_encoder_handler',
        name='serial_encoder_handler',
        namespace='shoulder',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'gh360_config.yaml')],
        # parameters=[
        #     {'serial_port': '/dev/serial/by-id/usb-FieldworkRobotics_GH2_Lower-MICRO_D361ZNY8-if00-port0'},
        #     {'baud_rate': 115200}
        # ],
        output='screen'
    )

    upperarm_serial_encoder_handler = Node(
        package='gh360_examples',
        executable='serial_encoder_handler',
        name='serial_encoder_handler',
        namespace='upperarm',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'gh360_config.yaml')],
        # parameters=[
        #     {'serial_port': '/dev/serial/by-id/usb-FieldworkRobotics_GH2_Lower-MICRO_D361ZNY8-if00-port0'},
        #     {'baud_rate': 115200}
        # ],
        output='screen'
    )

    lowerarm_serial_encoder_handler = Node(
        package='gh360_examples',
        executable='serial_encoder_handler',
        name='serial_encoder_handler',
        namespace='lowerarm',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'gh360_config.yaml')],
        # parameters=[
        #     {'serial_port': '/dev/serial/by-id/usb-FieldworkRobotics_GH2_Lower-MICRO_D361ZNY8-if00-port0'},
        #     {'baud_rate': 115200}
        # ],
        output='screen'
    )

    return LaunchDescription([
        shoulder_serial_encoder_handler,
        upperarm_serial_encoder_handler,
        lowerarm_serial_encoder_handler
    ])
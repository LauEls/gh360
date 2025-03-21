#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'gh360'
    robot_name = 'gh360'
    model_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', robot_name+'.urdf')
    robot_description_raw = xacro.process_file(model_file_path).toxml()

    motor_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/motor_handler.launch.py'])
    )

    motor_handler_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace='gh360'),
            motor_handler
        ]
    )

    encoder_handler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360'), 'launch'), '/encoder_handler.launch.py'])
    )

    eef_pose_node = Node(
        package='gh360',
        executable='eef_pose',
        name='eef_pose',
        namespace='gh360',
        output='screen'
    )

    controller_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gh360_control'), 'launch'), '/controllers.launch.py'])
    )

    
    return LaunchDescription([
        motor_handler_with_namespace,
        encoder_handler,
        eef_pose_node,
        controller_nodes
    ])
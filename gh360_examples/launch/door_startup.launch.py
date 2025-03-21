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
        namespace='door',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'door_motor_config.yaml')],
        output='screen')

    door_reset_node = Node(
        package='gh360_examples',
        executable='door_reset',
        name='door_reset',
        namespace='door',
        output='screen')

    door_handle_angle_filter_node = Node(
        package='gh360_examples',
        executable='door_handle_angle_filter',
        name='door_handle_angle_filter',
        namespace='door',
        output='screen')

    door_env_obs_node = Node(
        package='gh360_examples',
        executable='door_env_obs',
        name='door_env_obs',
        namespace='door',
        remappings=[('/door/tf', '/tf'), ('/door/tf_static', '/tf_static')],
        output='screen')
    
    door_handle_frame_node = Node(
        package='gh360_examples',
        executable='door_handle_pose',
        name='door_handle_pose',
        namespace='door',
        output='screen')
    
    aruco_recognition_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros2_aruco'), 'launch'), '/aruco_recognition.launch.py'])
    )

    aruco_recognition_node_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace='door'),
            aruco_recognition_node
        ]
    )
    
    return LaunchDescription([
        motor_handler_node,
        # camera_handler,
        door_reset_node,
        door_handle_angle_filter_node,
        door_handle_frame_node,
        door_env_obs_node,
        aruco_recognition_node_with_namespace
    ])
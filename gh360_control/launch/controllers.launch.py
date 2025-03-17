#!/usr/bin/env python3

import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    model_file_path = os.path.join(get_package_share_directory('gh360'), 'urdf', 'gh360.urdf')
    robot_description_raw = xacro.process_file(model_file_path).toxml()
    robot_description = {'robot_description': robot_description_raw}
    robot_config_file = PathJoinSubstitution([FindPackageShare('gh360'), 'config', 'gh360_config.yaml'])
    controller_config_file = PathJoinSubstitution([FindPackageShare('gh360_control'), 'config', 'controller_config.yaml'])

    eef_velocity_node = Node(
        package='gh360_control',
        executable='eef_velocity',
        name='eef_velocity',
        namespace='gh360_control',
        parameters=[robot_description, robot_config_file, controller_config_file],
        output='screen')
    
    joint_velocity_node = Node(
        package='gh360_control',
        executable='joint_velocity',
        name='joint_velocity',
        namespace='gh360_control',
        parameters=[robot_config_file, controller_config_file],
        output='screen')
    
    motor_position_node = Node(
        package='gh360_control',
        executable='motor_position',
        name='motor_position',
        namespace='gh360_control',
        parameters=[robot_config_file, controller_config_file],
        output='screen')
    
    move_home_node = Node(
        package='gh360_control',
        executable='move_home',
        name='move_home',
        namespace='gh360_control',
        output='screen')
    
    robot_stop_node = Node(
        package='gh360_control',
        executable='robot_stop',
        name='robot_stop',
        namespace='gh360_control',
        output='screen')
    
    return LaunchDescription([
        joint_velocity_node,
        eef_velocity_node,
        motor_position_node,
        move_home_node,
        robot_stop_node
    ])
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import xacro
# from launch.events.process.process_exited import ProcessExited
# from launch.event_handlers.on_process_exit import OnProcessExit
# from launch.actions import RegisterEventHandler
# from launch.launch_context import LaunchContext



def generate_launch_description():
    package_name = 'gh360'
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'urdf_2.rviz')
    model_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'gh360.urdf')
    robot_description_raw = xacro.process_file(model_file_path).toxml()

    shoulder_node = Node(
        package='gh360',
        executable='motor_handler',
        name='motor_handler',
        respawn=True,
        respawn_delay=2,
        namespace='shoulder',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'motor_handler_config.yaml')],
        output='screen')
    
    upperarm_node = Node(
        package='gh360',
        executable='motor_handler',
        name='motor_handler',
        respawn=True,
        respawn_delay=2,
        namespace='upperarm',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'motor_handler_config.yaml')],
        output='screen')
    
    lowerarm_node = Node(
        package='gh360',
        executable='motor_handler',
        name='motor_handler',
        respawn=True,
        respawn_delay=2,
        namespace='lowerarm',
        parameters=[os.path.join(
            get_package_share_directory('gh360'),
            'config', 'motor_handler_config.yaml')],
        output='screen')
    
    start_joint_state_publisher_cmd = Node(
    # condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'source_list': ['gh360_joint_states']}])
 
#   # A GUI to manipulate the joint state values
#   start_joint_state_publisher_gui_node = Node(
#     condition=IfCondition(gui),
#     package='joint_state_publisher_gui',
#     executable='joint_state_publisher_gui',
#     name='joint_state_publisher_gui')
 
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            # 'use_sim_time': use_sim_time, 
        #     'robot_description': Command(['xacro ', urdf_model])}],
            'robot_description': robot_description_raw}])
        # arguments=[default_urdf_model_path])
    
    gh360_joint_states_cmd = Node(
        package='gh360',
        executable='joint_states',
        name='joint_states',
        # parameters=[os.path.join(
        #     get_package_share_directory('gh360'),
        #     'config', 'motor_handler_config.yaml')],
        output='screen')
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])

    
    return LaunchDescription([
        shoulder_node,
        upperarm_node,
        lowerarm_node,
        start_rviz_cmd,
        start_joint_state_publisher_cmd,
        start_robot_state_publisher_cmd,
        gh360_joint_states_cmd

        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart)),
        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart(upperarm_node))),
        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart(lowerarm_node)))
    ])
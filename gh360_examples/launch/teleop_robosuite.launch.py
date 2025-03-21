#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    package_name = 'gh360'
    robot_name = 'gh360'
    # model_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'gh360.urdf')
    model_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', robot_name+'.urdf')
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'urdf_2.rviz')
    robot_description_raw = xacro.process_file(model_file_path).toxml()

    inverse_jacobian_cmd = Node(
        package='gh360',
        executable='inverse_jacobian',
        name='inverse_jacobian',
        # emulate_tty=True,
        parameters=[{
        'robot_description': robot_description_raw,
        'tcp_link_name': 'eef',
        'joint_states_topic': '/gh360_joint_states'}]
    )

    # teleop_robosuite_cmd = Node(
    #     package='gh360_examples',
    #     executable='robosuite_teleop',
    #     name='robosuite_teleop',    
    # )
    
    teleop_robosuite_cmd = Node(
        package='gh360_demonstration',
        executable='robosuite_demo',
        name='robosuite_demo',    
    )

    space_mouse_cmd = Node(
            package='gh360_examples',
            executable='spacemouse',
            name='spacemouse'
    )

    start_joint_state_publisher_cmd = Node(
        # condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['gh360_joint_states']}]
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # camera_frame_cmd = Node(
    #     package='gh360_examples',
    #     executable='camera_frame',
    #     name='camera_frame'
    # )

    # door_handle_pose_cmd = Node(
    #     package='gh360_examples',
    #     executable='door_handle_pose',
    #     name='door_handle_pose'
    # )
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
        # start_rviz_cmd,
        inverse_jacobian_cmd,
        teleop_robosuite_cmd,
        start_joint_state_publisher_cmd,
        start_robot_state_publisher_cmd,
        # camera_frame_cmd,
        # door_handle_pose_cmd,
        space_mouse_cmd

    ])
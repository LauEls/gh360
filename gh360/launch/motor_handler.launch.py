#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# from launch.events.process.process_exited import ProcessExited
# from launch.event_handlers.on_process_exit import OnProcessExit
# from launch.actions import RegisterEventHandler
# from launch.launch_context import LaunchContext



def generate_launch_description():
    # def shoulder_description():
    #     return Node(
    #     package='gh360',
    #     executable='motor_handler',
    #     name='motor_handler',
    #     namespace='shoulder',
    #     parameters=[os.path.join(
    #         get_package_share_directory('gh360'),
    #         'config', 'motor_handler_config.yaml')],
    #     output='screen')
    
    # def on_exit_restart(event:ProcessExited, context:LaunchContext):
    #     print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(
    #         event.action.name, event.pid, event.returncode))
    #     if event.returncode != 0 and 'controller' in event.action.name:
    #         return shoulder_description() # respawn node action
        

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
    
    return LaunchDescription([
        shoulder_node,
        upperarm_node,
        lowerarm_node,
        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart)),
        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart(upperarm_node))),
        # RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart(lowerarm_node)))
    ])
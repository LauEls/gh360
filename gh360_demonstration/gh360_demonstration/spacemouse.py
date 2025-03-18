import pyspacemouse
import sys
import time
import rclpy
from rclpy.node import Node
import numpy as np
import os

from geometry_msgs.msg import Twist
from gh360_interfaces.msg import SpaceMouse, BoolMultiArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64

import tf2_geometry_msgs

class SpaceMouseNode(Node):
    def __init__(self):
        super().__init__('spacemouse')

        self.declare_parameter("translation_scaler", 1.0)
        self.declare_parameter("rotation_scaler", 1.0)
        self.translation_scaler = self.get_parameter("translation_scaler").get_parameter_value().double_value
        self.rotation_scaler = self.get_parameter("rotation_scaler").get_parameter_value().double_value

        self.get_logger().info("Translation Scaler: " + str(self.translation_scaler))
        self.get_logger().info("Rotation Scaler: " + str(self.rotation_scaler))

        self.spacemouse_publisher_ = self.create_publisher(SpaceMouse, 'spacemouse', 10)
        self.cmd_eef_vel_publisher_ = self.create_publisher(Twist, 'teleop_eef_velocity', 10)
        self.button_publisher_ = self.create_publisher(BoolMultiArray, 'teleop_buttons', 10)

        success = pyspacemouse.open()
        if not success:
            print("Failed to open SpaceMouse")

        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        state = pyspacemouse.read()
        
        msg = SpaceMouse()
        buttons = BoolMultiArray()
        twist = Twist()
        twist.linear.x = float(state.x)*self.translation_scaler
        twist.linear.y = float(state.y)*self.translation_scaler
        twist.linear.z = float(state.z)*self.translation_scaler
        twist.angular.x = -float(state.pitch)*self.rotation_scaler
        twist.angular.y = float(state.roll)*self.rotation_scaler
        twist.angular.z = -float(state.yaw)*self.rotation_scaler

        msg.velocity = twist
        msg.button1 = bool(state.buttons[0])
        buttons.data.append(bool(state.buttons[0]))
        msg.button2 = bool(state.buttons[14])
        buttons.data.append(bool(state.buttons[14]))
        self.spacemouse_publisher_.publish(msg)
        self.cmd_eef_vel_publisher_.publish(twist)
        self.button_publisher_.publish(buttons)
        pass

def main(args=None):
    rclpy.init(args=args)

    spacemouse_node = SpaceMouseNode()
    rclpy.spin(spacemouse_node)
    spacemouse_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
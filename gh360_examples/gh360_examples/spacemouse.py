import pyspacemouse
import sys
import time
import rclpy
from rclpy.node import Node
import numpy as np
import os

from geometry_msgs.msg import Twist
from gh360_interfaces.msg import SpaceMouse

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64

import tf2_geometry_msgs

class SpaceMouseNode(Node):
    def __init__(self):
        super().__init__('spacemouse_node')

        self.publisher_ = self.create_publisher(SpaceMouse, '/spacemouse', 10)

        success = pyspacemouse.open()
        if not success:
            print("Failed to open SpaceMouse")


        # state = pyspacemouse.read()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        state = pyspacemouse.read()
        
        msg = SpaceMouse()
        twist = Twist()
        twist.linear.x = float(state.x)
        twist.linear.y = float(state.y)
        twist.linear.z = float(state.z)
        twist.angular.x = float(state.roll)
        twist.angular.y = float(state.pitch)
        twist.angular.z = float(state.yaw)

        msg.velocity = twist
        msg.button1 = bool(state.buttons[0])
        msg.button2 = bool(state.buttons[14])
        self.publisher_.publish(msg)
        pass

def main(args=None):
    rclpy.init(args=args)

    spacemouse_node = SpaceMouseNode()

    rclpy.spin(spacemouse_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spacemouse_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import time
# import argparse
# import numpy as np
# import rosbag2_py

# from rclpy.serialization import deserialize_message
from gh360_interfaces.msg import SetMotorPositions, SetMotorVelocities, SetPosition, SetVelocity #PortStatus,
# from sensor_msgs.msg import JointState

from gh360_demonstration.rosbag_util import ROSBagUtil

class TimerTest(Node):

    def __init__(self):
        super().__init__('timer_test')


        # timer_period = 0.001  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


        self.last_time = 0
    

    def timer_callback(self):
        while True:
            if self.last_time == 0:
                self.last_time = time.time()
                return
        
            if time.time() - self.last_time >= 0.19:
                self.get_logger().info(f'Time_past: {time.time() - self.last_time}')
                self.last_time = time.time()
            self.spin_once()

def main(args=None):
    rclpy.init(args=args)

    pos_step_pub = TimerTest()

    # rclpy.spin(pos_step_pub)
    pos_step_pub.timer_callback()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pos_step_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
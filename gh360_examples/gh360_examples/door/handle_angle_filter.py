import time
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float64


class DoorHandleAngleFilter(Node):

    def __init__(self):
        super().__init__('door_handle_angle_filter')
        self.publisher_ = self.create_publisher(Float64, 'handle_angle_filtered', 10)

        self.create_subscription(
            Int32,
            'handle_angle',
            self.handle_angle_callback,
            10)
        
        self.data_buffer = np.zeros(50, dtype=float)
        self.filtered_handle_angle = Float64()

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def handle_angle_callback(self, msg):
        self.data_buffer = np.delete(self.data_buffer, 0)
        self.data_buffer = np.append(self.data_buffer,msg.data)
    
    def motor_status_callback(self, msg):
        if not self.first_status:
            self.first_status = True
        self.motor_status = msg.motors[0]

    def timer_callback(self):
        self.filtered_handle_angle.data = (234.31-np.median(self.data_buffer)*(300/1023))*np.pi/180 - 0.1536 + 1.0544
        self.publisher_.publish(self.filtered_handle_angle)


def main(args=None):
    rclpy.init(args=args)
    handle_sensor_filter = DoorHandleAngleFilter()
    rclpy.spin(handle_sensor_filter)
    handle_sensor_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
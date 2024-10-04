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

class PositionStepPublisher(Node):

    def __init__(self):
        super().__init__('position_step_publisher')

        self.declare_parameter('bag_file_path','')
        bag_file_path = self.get_parameter('bag_file_path').get_parameter_value().string_value

        self.rosbag_util = ROSBagUtil(bag_file_path)
        self.vel_goal_steps = self.rosbag_util.get_velocity_goal_steps()

        self.cntr = 0

        self.shoulder_motor_pos_pub = self.create_publisher(SetMotorPositions, '/shoulder/motor_goal_position', 10)
        self.upperarm_motor_pos_pub = self.create_publisher(SetMotorPositions, '/upperarm/motor_goal_position', 10)
        self.lowerarm_motor_pos_pub = self.create_publisher(SetMotorPositions, '/lowerarm/motor_goal_position', 10)

        self.shoulder_motor_vel_pub = self.create_publisher(SetMotorVelocities, '/shoulder/motor_goal_velocity', 10)
        self.upperarm_motor_vel_pub = self.create_publisher(SetMotorVelocities, '/upperarm/motor_goal_velocity', 10)
        self.lowerarm_motor_vel_pub = self.create_publisher(SetMotorVelocities, '/lowerarm/motor_goal_velocity', 10)

        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]

        self.reseted = False
        self.reset_cntr = 0

        self.last_time = 0
    

    def timer_callback(self):
        if not self.reseted:
            self.pos_msg = SetMotorPositions()
            for i in range(1, 14):
                set_pos = SetPosition()
                set_pos.id = i
                set_pos.position = self.robot_reset_pos[i-1]
                self.pos_msg.motor_goal_positions.append(set_pos)

            self.shoulder_motor_pos_pub.publish(self.pos_msg)
            self.upperarm_motor_pos_pub.publish(self.pos_msg)
            self.lowerarm_motor_pos_pub.publish(self.pos_msg)

            self.reset_cntr += 1

            if self.reset_cntr >= 6000:
                self.reseted = True
            return
        if self.last_time == 0:
            self.last_time = time.time()
            print(f"last_time: {self.last_time}")
        self.vel_msg = SetMotorVelocities()
        for i in range(1, 14):
            set_vel = SetVelocity()
            set_vel.id = i
            set_vel.velocity = self.vel_goal_steps[self.cntr][i-1]
            self.vel_msg.motor_goal_velocities.append(set_vel)

        if self.cntr < len(self.vel_goal_steps)-1 and time.time() - self.last_time >= 0.2:
            self.cntr += 1
            self.last_time = time.time()

        self.shoulder_motor_vel_pub.publish(self.vel_msg)
        self.upperarm_motor_vel_pub.publish(self.vel_msg)
        self.lowerarm_motor_vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)

    pos_step_pub = PositionStepPublisher()

    rclpy.spin(pos_step_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pos_step_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
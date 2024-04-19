import time
import rclpy
from rclpy.node import Node
import numpy as np
import os
import csv

from std_msgs.msg import String
from gh360_interfaces.msg import SetMotorCurrents, SetVelocity, PortStatus, MotorStatus, SetMotorVelocities, ArmEncoderStates


class TendonModelGenerator(Node):

    def __init__(self):
        super().__init__('tendon_model_generator')
        self.shoulder_publisher_ = self.create_publisher(SetMotorVelocities, '/shoulder/motor_goal_velocity', 10)
        self.upperarm_publisher_ = self.create_publisher(SetMotorVelocities, '/upperarm/motor_goal_velocity', 10)


        self.joint_vel_data_buffer = np.zeros(50, dtype=float)
        
        self.first_status = False
        self.create_subscription(
            PortStatus,
            '/upperarm/motor_status',
            self.motor_status_callback,
            10
        )
        self.create_subscription(
            PortStatus,
            '/shoulder/motor_status',
            self.motor_status_callback,
            10
        )

        self.create_subscription(
            ArmEncoderStates,
            '/encoder_status',
            self.encoder_callback,
            10
        )

        self.joint_name = "elbow"
        self.motor_ids = [9, 10]
        self.min_angle = -0.7
        self.max_angle = 2.0

        # self.joint_name = "shoulder_roll"
        # self.motor_ids = [4, 3]
        # self.min_angle = -1.4
        # self.max_angle = 1.4

        self.left_motor_status = MotorStatus()
        self.right_motor_status = MotorStatus()

        self.goal_velocity_msg = SetMotorVelocities()
        self.set_velocity_msg = SetVelocity()
        self.set_velocity_msg.id = self.motor_ids[0]
        self.set_velocity_msg.velocity = 0.0
        self.goal_velocity_msg.motor_goal_velocities.append(self.set_velocity_msg)
        self.set_velocity_msg = SetVelocity()
        self.set_velocity_msg.id = self.motor_ids[1]
        self.set_velocity_msg.velocity = 0.0
        self.goal_velocity_msg.motor_goal_velocities.append(self.set_velocity_msg)

        self.status = 0
        self.joint_pos = 0.0
        self.joint_vel = 0.0
        self.joint_time_list = []
        self.joint_pos_list = []
        self.joint_vel_list = []
        self.motor_position_list = []
        self.motor_velocity_list = []
        self.motor_time_list = []

        # self.close_door()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        file_base_dir = '/home/laurenz/phd_project/ros2_gh360_ws/src/gh360/gh360_examples/data/'
        self.elbow_model_file = os.path.join(file_base_dir, 'elbow_tendon_model_data_4.csv')
        
        # robot_state = [timestamp, self.joint_pos, self.ee_pos]
        # robot_state = np.insert(robot_state, 0, timestamp)
        # self.i = 0

    def motor_status_callback(self, msg):
        if not self.first_status:
            self.first_status = True
        # self.motor_status = msg.motors[0]

        for motor in msg.motors:
            if motor.motor_id == self.motor_ids[0]:
                self.motor_time_list.append(time.time())
                self.motor_position_list.append(motor.present_position)
                self.motor_velocity_list.append(motor.present_velocity)
                self.left_motor_status = motor
            elif motor.motor_id == self.motor_ids[1]:
                self.right_motor_status = motor  

    def encoder_callback(self, msg):
        # print("recieved encoder message")
        for joint_msg in msg.current_joint_states:
            if joint_msg.joint_name == self.joint_name:
                self.joint_time_list.append(time.time())
                self.joint_pos_list.append(joint_msg.current_pos)
                self.joint_vel_list.append(joint_msg.current_vel)
                self.joint_pos = joint_msg.current_pos
                self.joint_vel = joint_msg.current_vel
                break

    def setVelocities(self, left_vel, right_vel):
        self.goal_velocity_msg.motor_goal_velocities[0].velocity = left_vel
        self.goal_velocity_msg.motor_goal_velocities[1].velocity = right_vel

    def timer_callback(self):
        if not self.first_status:
            self.get_logger().info("Waiting to get motor status...")
            return
        
        self.upperarm_publisher_.publish(self.goal_velocity_msg)
        self.shoulder_publisher_.publish(self.goal_velocity_msg)

        if self.status == 0:
            self.setVelocities(-0.5, -0.5)
            self.status = 1
            self.get_logger().info("Changing to Satus 1")
        elif self.status == 1:
            # timestamp = time.time()
            # data = np.concatenate((timestamp, self.joint_pos, self.joint_vel, self.right_motor_status.present_position, self.right_motor_status.present_velocity), axis=None)
            # f = open(self.elbow_model_file, 'a')
            # data_writer = csv.writer(f)
            # data_writer.writerow(data)
            # f.close()
        
            if self.joint_pos <= self.min_angle:
                self.status = 2
                self.setVelocities(0.5, 0.5)
                self.get_logger().info("Changing to Satus 2")
        elif self.status == 2:
            timestamp = time.time()
            data = np.concatenate((timestamp, self.joint_pos, self.joint_vel, self.right_motor_status.present_position, self.right_motor_status.present_velocity), axis=None)
            # f = open(self.elbow_model_file, 'a')
            # data_writer = csv.writer(f)
            # data_writer.writerow(data)
            # f.close()

            if self.joint_pos >= self.max_angle:
                self.status = 3
                self.setVelocities(-0.5, -0.5)
                self.get_logger().info("Changing to Satus 3")
        elif self.status == 3 and self.joint_pos <= 0.0:
            self.status = 4
            self.setVelocities(0.0, 0.0)
            self.get_logger().info("Changing to Satus 4")
        # elif self.status == 1 and self.motor_status.present_velocity <= 0.0:
        #     self.status = 3
        #     self.goal_current_msg.motor_goal_currents[0].current = 0.0
        #     self.get_logger().info("Changing to Satus 3")
        # elif self.status == 2 and self.motor_status.present_velocity >= 0.0:
        #     self.status = 3
        #     self.goal_current_msg.motor_goal_currents[0].current = 0.0
        #     self.get_logger().info("Changing to Satus 3")

    #     goal_current_msg = SetMotorCurrents()
    #     set_current_msg = SetCurrent()
    #     set_current_msg.id = 31
    #     set_current_msg.current = 50.0
    #     goal_current_msg.motor_goal_currents.append(set_current_msg)
      
    #     self.publisher_.publish(goal_current_msg)

        # set_current_msg.current = 0
        # goal_current_msg.motor_goal_currents[0] = set_current_msg
        # self.publisher_.publish(goal_current_msg)


def main(args=None):
    rclpy.init(args=args)

    tendon_model_gen = TendonModelGenerator()

    rclpy.spin(tendon_model_gen)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tendon_model_gen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
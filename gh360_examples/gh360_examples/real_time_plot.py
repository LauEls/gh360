import time
import rclpy
from rclpy.node import Node
import numpy as np
import os
import csv
import numpy as np
from matplotlib import pyplot as plt

from std_msgs.msg import String
from gh360_interfaces.msg import SetMotorCurrents, SetVelocity, PortStatus, MotorStatus, SetMotorVelocities, ArmEncoderStates
from sensor_msgs.msg import JointState
from scipy import signal


class RealTimePlot(Node):

    def __init__(self):
        super().__init__('real_time_plot')
        
        self.first_status = False
        self.create_subscription(
            PortStatus,
            '/lowerarm/motor_status',
            self.motor_status_callback,
            10
        )
        # self.create_subscription(
        #     PortStatus,
        #     '/shoulder/motor_status',
        #     self.motor_status_callback,
        #     10
        # )

        # self.create_subscription(
        #     ArmEncoderStates,
        #     '/encoder_status',
        #     self.encoder_callback,
        #     10
        # )

        self.create_subscription(
            JointState,
            '/gh360_joint_states',
            self.joint_states_callback,
            10
        )

        self.joint_name = "forearm_roll"
        self.motor_ids = [11]
        self.min_angle = -0.7
        self.max_angle = 2.0
        # self.motor_to_joint_ratio = 15.6/43.0
        self.motor_to_joint_ratio = 1.0

        self.window_size = 100

        # self.joint_name = "shoulder_roll"
        # self.motor_ids = [4, 3]
        # self.min_angle = -1.4
        # self.max_angle = 1.4
        # self.joint_vel_plot = plt()
        

        self.left_motor_status = MotorStatus()
        self.right_motor_status = MotorStatus()

        self.status = 0
        self.joint_pos = 0.0
        self.joint_vel = 0.0
        self.joint_time_list = np.zeros(self.window_size, dtype=float)
        self.joint_pos_list = []
        self.joint_vel_list = np.zeros(self.window_size, dtype=float)
        # self.motor_position_list = []
        # self.motor_velocity_list = []
        self.motor_velocity_list = np.zeros(int(self.window_size/10), dtype=float)
        # self.motor_time_list = []

        self.start_time = time.time()
        self.joint_time_list *= self.start_time
        self.x = np.linspace(0,1,self.window_size)
        self.x_motor = np.linspace(0,1,int(self.window_size/10))

        self.cut_f = 2
        self._SamplingTime = 0.1
        self.alpha = 0.8
        self.LPF_out = 0.0
        self.joint_vel_lfp_list = np.zeros(int(self.window_size/10), dtype=float)

        plt.ion()
        # self.figure = plt.figure()
        # self.ax = self.figure.add_subplot(111)
        self.figure, ax = plt.subplots(figsize=(10, 8))
        self.line1, = ax.plot(self.x, self.joint_vel_list)
        self.line2, = ax.plot(self.x_motor, self.joint_vel_lfp_list)
        self.line3, = ax.plot(self.x_motor, self.motor_velocity_list)

        plt.xlabel('Time (s)')
        plt.ylabel('Joint Velocity (rad/s)')
        plt.ylim(-2, 2)
        plt.title(self.joint_name+' Joint Velocity')
        plt.legend(['Joint Velocity', 'Joint Velocity LPF', 'Motor Velocity'])

        # self.close_door()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == self.joint_name:
                self.joint_pos = msg.position[i]
                self.joint_vel = msg.velocity[i]
                self.joint_vel_lfp_list = np.delete(self.joint_vel_lfp_list, 0)
                self.joint_vel_lfp_list = np.append(self.joint_vel_lfp_list, self.joint_vel)

    def motor_status_callback(self, msg):
        if not self.first_status:
            self.first_status = True
        # self.motor_status = msg.motors[0]

        for motor in msg.motors:
            if motor.motor_id == self.motor_ids[0]:

                # self.motor_time_list.append(self.start_time - time.time())
                # self.motor_position_list.append(motor.present_position)
                self.motor_velocity_list = np.delete(self.motor_velocity_list, 0)
                self.motor_velocity_list = np.append(self.motor_velocity_list, motor.present_velocity*self.motor_to_joint_ratio)
                self.left_motor_status = motor
            # elif motor.motor_id == self.motor_ids[1]:
            #     self.right_motor_status = motor  

    def encoder_callback(self, msg):
        # print("recieved encoder message")
        for joint_msg in msg.current_joint_states:
            if joint_msg.joint_name == self.joint_name:
                self.joint_time_list = np.delete(self.joint_time_list, 0)
                self.joint_time_list = np.append(self.joint_time_list, time.time() - self.start_time)
                self.joint_vel_list = np.delete(self.joint_vel_list, 0)
                self.joint_vel_list = np.append(self.joint_vel_list, joint_msg.current_vel)

                # print("len joint vel: ", len(self.joint_vel_list))

                # self.joint_vel_lfp_list = np.delete(self.joint_vel_lfp_list, 0)
                # # self.LPF_out += self.cut_f * (joint_msg.current_vel - self.LPF_out) * self._SamplingTime
                # # self.LPF_out += self.alpha * (joint_msg.current_vel - self.LPF_out)
                # self.LPF_out = self.freq_filter(self.joint_vel_list, self.window_size, 10/512)
                # self.median_out = self.median_filter(self.LPF_out, self.window_size)
                # # print("LPF out: ", self.LPF_out)
                # # print("median out: ", self.median_out)
                # # print(len(self.LPF_out))
                # self.joint_vel_lfp_list = np.append(self.joint_vel_lfp_list,self.median_out[int((self.window_size-1)/2)])
                # # self.joint_vel_lfp_list = self.median_out
                # # self.joint_time_list.append(self.start_time - time.time())
                # # self.joint_pos_list.append(joint_msg.current_pos)
                # # self.joint_vel_list.append(joint_msg.current_vel)
                # # self.joint_pos = joint_msg.current_pos
                # # self.joint_vel = joint_msg.current_vel
                break

    # def setVelocities(self, left_vel, right_vel):
    #     self.goal_velocity_msg.motor_goal_velocities[0].velocity = left_vel
    #     self.goal_velocity_msg.motor_goal_velocities[1].velocity = right_vel
    def freq_filter(self, data, f_size, cutoff):
        num_signal=data.shape[0]
        # print("shape ", num_signal)
        f_data=np.zeros(num_signal)
        lpf=signal.firwin(f_size, cutoff, window='hamming')
        f_data=signal.convolve(data, lpf, mode='same')
        # for i in range(num_signal):
            
        return f_data
    
    def median_filter(self, data, f_size):
        lgth=data.shape[0]
        f_data=np.zeros(lgth)
        f_data=signal.medfilt(data, f_size)
        return f_data
    
    def timer_callback(self):
        # print("should plot")
        timestep = time.time() - self.start_time

        self.line1.set_xdata(self.x)
        self.line1.set_ydata(self.joint_vel_list)

        self.line2.set_xdata(self.x_motor)
        self.line2.set_ydata(self.joint_vel_lfp_list)

        self.line3.set_xdata(self.x_motor)
        self.line3.set_ydata(self.motor_velocity_list)

        self.figure.canvas.draw()

        self.figure.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)

    real_time_plot = RealTimePlot()

    rclpy.spin(real_time_plot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    real_time_plot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
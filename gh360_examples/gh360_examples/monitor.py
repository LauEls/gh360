import rclpy
from rclpy.node import Node

import tkinter as tk
import tkinter.ttk as ttk
from tkinter import RIGHT
import threading

from .gui_util import GUIMotor, GUIJoint

from std_msgs.msg import String
from std_srvs.srv import SetBool
from gh360_interfaces.msg import SetMotorPositions, SetPosition, PortStatus, ArmEncoderStates
from gh360_interfaces.srv import MotorPositionStep

class Monitor(Node):

    def __init__(self):
        super().__init__('gh360_monitor')

        ns = "/gh360"

        self.create_subscription(
            PortStatus,
            ns+'/motor_states',
            self.port_callback,
            10)
        
        # self.create_subscription(
        #     PortStatus,
        #     '/upperarm/motor_status',
        #     self.port_callback,
        #     10)
        
        # self.create_subscription(
        #     PortStatus,
        #     '/lowerarm/motor_status',
        #     self.port_callback,
        #     10)
        
        self.create_subscription(
            ArmEncoderStates,
            ns+'/encoder_states',
            self.encoder_callback,
            10
        )
        
        
        self.get_logger().info(ns+'/shoulder/motor_positions_step')
        self.shoulder_client = self.create_client(MotorPositionStep, '/gh360/shoulder/motor_positions_step')
        while not self.shoulder_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('shoulder motor position service not available, waiting again...')

        self.upperarm_client = self.create_client(MotorPositionStep, ns+'/upperarm/motor_positions_step')
        while not self.upperarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('upperarm motor position service not available, waiting again...')

        self.lowerarm_client = self.create_client(MotorPositionStep, ns+'/lowerarm/motor_positions_step')
        while not self.lowerarm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('lowerarm motor position service not available, waiting again...')

        self.shoulder_shutdown_client = self.create_client(SetBool, ns+'/shoulder/motor_set_torque')
        while not self.shoulder_shutdown_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('shoulder motor torque service not available, waiting again...')

        self.upperarm_shutdown_client = self.create_client(SetBool, ns+'/upperarm/motor_set_torque')
        while not self.upperarm_shutdown_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('upperarm motor torque service not available, waiting again...')

        self.lowerarm_shutdown_client = self.create_client(SetBool, ns+'/lowerarm/motor_set_torque')
        while not self.lowerarm_shutdown_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('lowerarm motor torque service not available, waiting again...')
        
        self.window = tk.Tk()
        self.window.title("GH360 Monitor")
        self.window.resizable(width=False, height=False)

        self.test_label_value = "0.0000"

        self.gui_motors = []
        self.gui_joints = []
        self.create_joint(joint_name="Shoulder Yaw", joint_id="shoulder_yaw", port_name="shoulder", row=0, column=0, motor_ids=[1,2])
        self.create_joint(joint_name="Shoulder Roll", joint_id="shoulder_roll", port_name="shoulder", row=1, column=0, motor_ids=[3,4])
        self.create_joint(joint_name="Shoulder Pitch", joint_id="shoulder_pitch", port_name="shoulder", row=2, column=0, motor_ids=[5,6])
        self.create_joint(joint_name="Upperarm Roll", joint_id="upperarm_roll", port_name="upperarm", row=3, column=0, motor_ids=[7,8])
        self.create_joint(joint_name="Elbow", joint_id="elbow", port_name="upperarm", row=0, column=1, motor_ids=[9,10])
        self.create_joint(joint_name="Forearm Roll", joint_id="forearm_roll", port_name="lowerarm", row=1, column=1, motor_ids=[11])
        self.create_joint(joint_name="Wrist Pitch", joint_id="wrist_pitch", port_name="lowerarm", row=2, column=1, motor_ids=[12,13])

        frm_btns = tk.Frame(master=self.window)
        btn_motor_start = tk.Button(
            master=frm_btns,
            text="Start",
            command=lambda: self.set_torque_request(True)
        )
        btn_motor_start.grid(row=0, column=0)

        btn_motor_stop = tk.Button(
            master=frm_btns,
            text="Stop",
            command=lambda: self.set_torque_request(False)
        )
        btn_motor_stop.grid(row=0, column=1)

        frm_btns.grid(row=3, column=1)
        
    def set_torque_request(self, torque_enable):
        shutdown_request = SetBool.Request()
        shutdown_request.data = torque_enable
        self.future = self.shoulder_shutdown_client.call_async(shutdown_request)
        self.future = self.upperarm_shutdown_client.call_async(shutdown_request)
        self.future = self.lowerarm_shutdown_client.call_async(shutdown_request)

    # def lowerarm_callback(self, msg):
    def port_callback(self, msg):
        # self.get_logger().info('Callback loop')
        for motor in msg.motors:
            for gui_motor in self.gui_motors:
                if gui_motor.id == motor.motor_id:
                    gui_motor.present_pos.config(text=self.get_label_str(motor.present_position))
                    gui_motor.present_vel.config(text=self.get_label_str(motor.present_velocity))
                    gui_motor.present_current.config(text=self.get_label_str(motor.present_current))
                    # self.test_label_value = self.get_label_str(motor.present_position)

        self.window.update()

    def encoder_callback(self, msg):
        for joint in msg.current_joint_states:
            for gui_joint in self.gui_joints:
                if gui_joint.joint_name == joint.joint_name:
                    gui_joint.joint_angle.config(text="Joint Angle: "+self.get_label_str(joint.current_pos))

        

    def get_label_str(self, long_value):
        return '%.4f' % long_value


    def send_goal_pos(self, motor_id, port_name, goal_pos):
        print(motor_id)
        self.motor_pos_req = MotorPositionStep.Request()

        set_motor_msg = SetPosition()
        set_motor_msg.id = motor_id
        set_motor_msg.position = goal_pos
        self.motor_pos_req.motor_goal_positions.append(set_motor_msg)

        if port_name == 'shoulder':
            self.future = self.shoulder_client.call_async(self.motor_pos_req)
        elif port_name == 'upperarm':
            self.future = self.upperarm_client.call_async(self.motor_pos_req)
        elif port_name == 'lowerarm':
            self.future = self.lowerarm_client.call_async(self.motor_pos_req)
        else:
            print("Not a valid port name")
            

    def create_joint(self, joint_name, joint_id, port_name, row, column, motor_ids):
        frm_joint = tk.Frame(master=self.window)
        lbl_joint = tk.Label(master=frm_joint, text=joint_name)
        lbl_joint.grid(row=0, column=0, padx=10)
        if joint_id != 'forearm_roll':
            lbl_joint_angle = tk.Label(master=frm_joint, text="Joint Angle: 0.000")
            lbl_joint_angle.grid(row=0, column=1, padx=10, sticky="w")

            new_joint = GUIJoint(_joint_name=joint_id, 
                             _port_name=port_name,
                             _joint_angle=lbl_joint_angle)
            self.gui_joints.append(new_joint)

        if len(motor_ids) == 2:
            self.create_motor(motor_id=motor_ids[0], port_name=port_name, master_frame=frm_joint, row=1)
            self.create_motor(motor_id=motor_ids[1], port_name=port_name, master_frame=frm_joint, row=2)
        else:
            self.create_motor(motor_id=motor_ids[0], port_name=port_name, master_frame=frm_joint, row=1)

        frm_joint.grid(row=row, column=column, padx=10)

    def create_motor(self, motor_id, port_name, master_frame, row):
        frm_motor = tk.Frame(master=master_frame)

        lbl_motor = tk.Label(master=frm_motor, text="Motor "+str(motor_id))
        lbl_motor.grid(row=0, column=0, sticky="w")

        lbl_motor_goal_pos = tk.Label(master=frm_motor, text="Goal Position:")
        lbl_motor_goal_pos.grid(row=1, column=1, sticky="w")

        ent_motor_goal_pos = tk.Entry(master=frm_motor, width=10, justify=RIGHT)
        ent_motor_goal_pos.insert(0, "0.000")
        ent_motor_goal_pos.grid(row=1, column=2, sticky="e")

        btn_motor_goal_pos = tk.Button(
            master=frm_motor,
            text="Send Goal",
            command=lambda: self.send_goal_pos(motor_id=motor_id, port_name=port_name, goal_pos=float(ent_motor_goal_pos.get()))
        )
        btn_motor_goal_pos.grid(row=1, column=3)

        lbl_motor_present_pos = tk.Label(master=frm_motor, text="Present Position:")
        lbl_motor_present_pos.grid(row=2, column=1, sticky="w")
        # self.present_positions.append()

        lbl_motor_present_pos_value = tk.Label(master=frm_motor, text=self.test_label_value)
        lbl_motor_present_pos_value.grid(row=2, column=2, sticky="e")

        lbl_motor_present_vel = tk.Label(master=frm_motor, text="Present Velocity:")
        lbl_motor_present_vel.grid(row=3, column=1, sticky="w")

        lbl_motor_present_vel_value = tk.Label(master=frm_motor, text="0.000")
        lbl_motor_present_vel_value.grid(row=3, column=2, sticky="e")
        
        lbl_motor_present_current = tk.Label(master=frm_motor, text="Present Current:")
        lbl_motor_present_current.grid(row=4, column=1, sticky="w")

        lbl_motor_present_current_value = tk.Label(master=frm_motor, text="0.000")
        lbl_motor_present_current_value.grid(row=4, column=2, sticky="e")

        frm_motor.grid(row=row, column=1, padx=10)

        new_motor = GUIMotor(
            _id=motor_id, 
            _present_pos=lbl_motor_present_pos_value, 
            _present_vel=lbl_motor_present_vel_value,
            _present_current=lbl_motor_present_current_value,
            _port_name=port_name,
        )
        self.gui_motors.append(new_motor)


def main(args=None):
    rclpy.init(args=args)

    gh360_monitor = Monitor()

    rclpy.spin(gh360_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gh360_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
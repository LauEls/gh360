import numpy as np

class SoftJoint:
    def __init__(self,
                 joint_name,
                 port_name,
                 id_right_motor,
                 id_left_motor,
                 max_pos,
                 min_pos,
                 max_current
                 ):

        self.joint_name = joint_name
        self.port_name = port_name
        self.id_right_motor = id_right_motor
        self.id_left_motor = id_left_motor
        self.max_pos = max_pos
        self.min_pos = min_pos
        self.max_current = max_current
        self.min_current = -max_current
        self.right_motor_pos = 0.0
        self.left_motor_pos = 0.0
        self.right_motor_current = 0.0
        self.left_motor_current = 0.0
        self.joint_angle = 0.0
        self.joint_velocity = 0.0
        self.right_motor_safety_check = False
        self.left_motor_safety_check = False
        self.right_motor_moving = False
        self.left_motor_moving = False

        self.window_size = 99
        self.joint_vel_list = np.zeros(self.window_size, dtype=float)

class MotorJoint:
    def __init__(self,
                 joint_name,
                 port_name,
                 id_motor,
                 max_pos,
                 min_pos,
                 max_current
                 ):

        self.joint_name = joint_name
        self.port_name = port_name
        self.id_motor = id_motor
        self.max_pos = max_pos
        self.min_pos = min_pos
        self.max_current = max_current
        self.min_current = -max_current
        self.motor_current = 0.0
        self.joint_angle = 0.0
        self.joint_velocity = 0.0
        self.motor_safety_check = False
        self.motor_moving = False
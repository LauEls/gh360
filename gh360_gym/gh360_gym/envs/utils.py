
class SoftJoint:
    def __init__(self,
                 joint_name,
                 port_name,
                 id_right_motor,
                 id_left_motor,
                 ):

        self.joint_name = joint_name
        self.port_name = port_name
        self.id_right_motor = id_right_motor
        self.id_left_motor = id_left_motor
        self.right_motor_pos = 0.0
        self.left_motor_pos = 0.0
        self.joint_angle = 0.0
        self.joint_velocity = 0.0

class MotorJoint:
    def __init__(self,
                 joint_name,
                 port_name,
                 id_motor,
                 ):

        self.joint_name = joint_name
        self.port_name = port_name
        self.id_motor = id_motor
        self.joint_angle = 0.0
        self.joint_velocity = 0.0


class GUIJoint:
    def __init__(self, _joint_name, _port_name, _joint_angle):
        self.joint_name = _joint_name
        self.joint_angle = _joint_angle
        self.motors = []

        self.port_name = _port_name


class GUIMotor:
    def __init__(self, _id, _present_pos, _present_vel, _present_current, _port_name):
        self.id = _id
        self.port_name = _port_name
        self.present_pos = _present_pos
        self.present_vel = _present_vel
        self.present_current = _present_current
        


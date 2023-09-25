

class GUIJoint:
    def __init__(self, _joint_name):
        self.joint_name = _joint_name
        self.motors = []


class GUIMotor:
    def __init__(self, _id, _present_pos, _present_vel, _present_current):
        self.id = _id
        self.present_pos = _present_pos
        self.present_vel = _present_vel
        self.present_current = _present_current


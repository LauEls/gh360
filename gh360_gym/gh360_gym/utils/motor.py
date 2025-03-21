
class Motor:
    def __init__(self):
        self.motor_id = 0
        self.present_position = 0.0
        self.present_velocity = 0.0
        self.present_current = 0.0
        self.present_temperature = 0.0
        self.safety_check = True
        self.moving = False

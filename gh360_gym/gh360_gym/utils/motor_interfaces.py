
from gh360_interfaces.srv import MotorPositionStep, MotorVelocityStep
from gh360_interfaces.msg import SetVelocity, SetPosition, SetMotorPositions, SetMotorVelocities, MotorStatus, PortStatus, ArmEncoderStates

from gh360_gym.utils.joints import SoftJoint, MotorJoint

def generate_velocities_msg(arm, velocities, srv=False):
    motor_iter = 0
    joint_iter = 0
    if srv:
        motor_vel_req = MotorVelocityStep.Request()
    else:
        motor_vel_req = SetMotorVelocities()

    while motor_iter < len(velocities):
        if type(arm[joint_iter]) == SoftJoint:
            set_motor_msg = SetVelocity()
            set_motor_msg.id = arm[joint_iter].id_right_motor
            set_motor_msg.velocity = velocities[motor_iter]
            motor_vel_req.motor_goal_velocities.append(set_motor_msg)
            motor_iter += 1

            set_motor_msg = SetVelocity()
            set_motor_msg.id = arm[joint_iter].id_left_motor
            set_motor_msg.velocity = velocities[motor_iter]
            motor_vel_req.motor_goal_velocities.append(set_motor_msg)
            motor_iter += 1
        else:
            set_motor_msg = SetVelocity()
            set_motor_msg.id = arm[joint_iter].id_motor
            set_motor_msg.velocity = velocities[motor_iter]
            motor_vel_req.motor_goal_velocities.append(set_motor_msg)
            motor_iter += 1
        joint_iter += 1

    return motor_vel_req

def generate_positions_msg(arm, positions):
    motor_iter = 0
    joint_iter = 0
    motor_pos_req = MotorPositionStep.Request()

    while motor_iter < len(positions):
        if type(arm[joint_iter]) == SoftJoint:
            set_motor_msg = SetPosition()
            set_motor_msg.id = arm[joint_iter].id_right_motor
            set_motor_msg.position = positions[motor_iter]
            motor_pos_req.motor_goal_positions.append(set_motor_msg)
            motor_iter += 1

            set_motor_msg = SetPosition()
            set_motor_msg.id = arm[joint_iter].id_left_motor
            set_motor_msg.position = positions[motor_iter]
            motor_pos_req.motor_goal_positions.append(set_motor_msg)
            motor_iter += 1
        else:
            set_motor_msg = SetPosition()
            set_motor_msg.id = arm[joint_iter].id_motor
            set_motor_msg.position = positions[motor_iter]
            motor_pos_req.motor_goal_positions.append(set_motor_msg)
            motor_iter += 1
        joint_iter += 1

    return motor_pos_req
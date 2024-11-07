import time
import numpy as np
import rclpy
from rclpy.node import Node
import gh360_gym
import gym
from std_msgs.msg import String
from std_srvs.srv import SetBool
from gh360_interfaces.msg import SetMotorCurrents, SetCurrent, PortStatus, MotorStatus, SpaceMouse, SetMotorPositions, SetPosition, SetMotorVelocities, SetVelocity
from sensor_msgs.msg import JointState

# env = None

class ResetRobot(Node):

    def __init__(self):
        super().__init__('reset_robot')

        self.robot_reset_pos = [0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0]
        self.via_point_pos = [0.2, 0.2, 1.5, 1.5, 4.0, 4.0, 4.5, 4.5, 5.5, 5.5, 0.0, 0.0, 0.0]
        self.elbow_pos = 0.0

        self.internal_state = 0
        self.motor_positions = np.zeros(13)
        self.motor_velocities = np.zeros(13)
        self.motor_currents = np.zeros(13)

        self.joint_positions = np.zeros(13)
        
        self.create_subscription(
            SpaceMouse, 
            '/spacemouse', 
            self.spacemouse_callback, 
            10
        )

        self.create_subscription(PortStatus,'/shoulder/motor_status',self.motor_status_callback,10)
        self.create_subscription(PortStatus,'/upperarm/motor_status',self.motor_status_callback,10)
        self.create_subscription(PortStatus,'/lowerarm/motor_status',self.motor_status_callback,10)

        self.pub_goal_velocity_shoulder = self.create_publisher(SetMotorVelocities, '/shoulder/motor_goal_velocity', 10)
        self.pub_goal_velocity_upperarm = self.create_publisher(SetMotorVelocities, '/upperarm/motor_goal_velocity', 10)
        self.pub_goal_velocity_lowerarm = self.create_publisher(SetMotorVelocities, '/lowerarm/motor_goal_velocity', 10)
        self.create_subscription(JointState,'/gh360_joint_states',self.joint_states_callback,10)

        self.reseting = False


        # self.close_door()
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def spacemouse_callback(self, msg):
        btn = msg.button1
        if btn and not self.reseting:
            self.get_logger().info("Resetting Robot")
            self.internal_state = 0
            self.reseting = True
            
            # env.reset()
            # self.reseting = False

    def joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            if msg.name[i] == "elbow":
                self.elbow_pos = msg.position[i]

    def motor_status_callback(self, msg):
        for motor in msg.motors:
            self.motor_positions[motor.motor_id-1] = motor.present_position
            self.motor_velocities[motor.motor_id-1] = motor.present_velocity
            self.motor_currents[motor.motor_id-1] = motor.present_current

    def calc_motor_pos_error(self, target_pos):
        pos_error = []
        pos_error = target_pos - self.motor_positions

        return pos_error

    
    def generate_velocities_msg(self, velocities):
        motor_vel_msg = SetMotorVelocities()

        for i in range(len(velocities)):
            set_motor_msg = SetVelocity()
            set_motor_msg.id = i+1
            set_motor_msg.velocity = velocities[i]
            motor_vel_msg.motor_goal_velocities.append(set_motor_msg)

        return motor_vel_msg

    def timer_callback(self):
        if self.reseting:
            stuck = False
            if self.internal_state == 0:
                pos_error = self.calc_motor_pos_error(self.via_point_pos)
                if self.elbow_pos >= 1.0:
                    pos_error[8] = -1.0
                    pos_error[9] = -1.0
                    stuck = True
                # pos_error = self.calc_motor_effort_error()
            elif self.internal_state == 1:
                pos_error = self.calc_motor_pos_error(self.robot_reset_pos)

            # pos_error = self.calc_motor_pos_error(self.via_point_pos)
            if np.max(np.absolute(pos_error)) <= 0.1 and self.internal_state == 1:
                motor_vel_msg = self.generate_velocities_msg(np.zeros(13))
                self.pub_goal_velocity_shoulder.publish(motor_vel_msg)
                self.pub_goal_velocity_upperarm.publish(motor_vel_msg)
                self.pub_goal_velocity_lowerarm.publish(motor_vel_msg)
                self.reseting = False
                self.get_logger().info(f"final internal state: {self.internal_state}")
                return
            
            # while np.max(np.absolute(pos_error)) > 0.1 or internal_state != 1:
            if self.internal_state == 0 and np.max(np.absolute(pos_error)) < 0.2 and not stuck:
                self.internal_state = 1
            
            pos_error = np.clip(pos_error, -0.8, 0.8)

            motor_vel_msg = self.generate_velocities_msg(pos_error)
            self.pub_goal_velocity_shoulder.publish(motor_vel_msg)
            self.pub_goal_velocity_upperarm.publish(motor_vel_msg)
            self.pub_goal_velocity_lowerarm.publish(motor_vel_msg)
            # rclpy.spin_once(self.node)

            



def main(args=None):
    rclpy.init(args=args)

    reset_robot = ResetRobot()

    rclpy.spin(reset_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reset_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
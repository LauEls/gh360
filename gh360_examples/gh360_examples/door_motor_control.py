import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from gh360_interfaces.msg import SetMotorCurrents, SetCurrent, PortStatus, MotorStatus


class DoorControl(Node):

    def __init__(self):
        super().__init__('door_control')
        self.publisher_ = self.create_publisher(SetMotorCurrents, '/door/motor_goal_current', 10)
        
        self.first_status = False
        self.create_subscription(
            PortStatus,
            '/door/motor_status',
            self.motor_status_callback,
            10)

        self.motor_status = MotorStatus()

        self.goal_current_msg = SetMotorCurrents()
        self.set_current_msg = SetCurrent()
        self.set_current_msg.id = 31
        self.set_current_msg.current = -500.0
        self.goal_current_msg.motor_goal_currents.append(self.set_current_msg)

        self.status = 0


        # self.close_door()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def motor_status_callback(self, msg):
        if not self.first_status:
            self.first_status = True
        self.motor_status = msg.motors[0]

    # def close_door(self):
    #     while self.motor_status.present_current == 0.0:
    #         self.get_logger().info("Waiting to get motor status...")
    #         time.sleep(1)

    #     goal_current_msg = SetMotorCurrents()
    #     set_current_msg = SetCurrent()
    #     set_current_msg.id = 31
    #     set_current_msg.current = 50.0
    #     goal_current_msg.motor_goal_currents.append(set_current_msg)

    #     status = 0

    #     while status != 2:
    #         self.publisher_.publish(goal_current_msg)

    #         if status == 0 and self.motor_status.velocity != 0.0:
    #             status = 1
    #         elif status == 1 and self.motor_status.velocity == 0.0:
    #             status = 2

    #     set_current_msg.current = 0.0
    #     goal_current_msg.motor_goal_currents[0] = set_current_msg
    #     self.publisher_.publish(goal_current_msg)

    def timer_callback(self):
        if not self.first_status:
            self.get_logger().info("Waiting to get motor status...")
            return
        
        self.publisher_.publish(self.goal_current_msg)

        if self.status == 0 and self.motor_status.present_velocity > 0.0:
            self.status = 1
            self.get_logger().info("Changing to Satus 1")
        elif self.status == 0 and self.motor_status.present_velocity < 0.0:
            self.status = 2
            self.get_logger().info("Changing to Satus 2")
        elif self.status == 1 and self.motor_status.present_velocity <= 0.0:
            self.status = 3
            self.goal_current_msg.motor_goal_currents[0].current = 0.0
            self.get_logger().info("Changing to Satus 3")
        elif self.status == 2 and self.motor_status.present_velocity >= 0.0:
            self.status = 3
            self.goal_current_msg.motor_goal_currents[0].current = 0.0
            self.get_logger().info("Changing to Satus 3")

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

    door_control = DoorControl()

    rclpy.spin(door_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    door_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
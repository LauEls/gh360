import time
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from gh360_interfaces.msg import SetMotorCurrents, SetCurrent, PortStatus, MotorStatus, SpaceMouse, SetMotorPositions, SetPosition, SetMotorVelocities, SetVelocity


class DoorControl(Node):

    def __init__(self):
        super().__init__('door_control')
        self.publisher_ = self.create_publisher(SetMotorCurrents, '/door/motor_goal_current', 10)
        self.pos_publisher_ = self.create_publisher(SetMotorPositions, '/door/motor_goal_position', 10)
        self.vel_publisher_ = self.create_publisher(SetMotorVelocities, '/door/motor_goal_velocity', 10)
        
        self.first_status = False
        self.motor_future = None
        self.create_subscription(
            PortStatus,
            '/door/motor_status',
            self.motor_status_callback,
            10
        )
        self.create_subscription(
            SpaceMouse, 
            '/spacemouse', 
            self.spacemouse_callback, 
            10
        )

        self.client_motor_torque = self.create_client(SetBool, '/door/motor_set_torque')
        while not self.client_motor_torque.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
       
        self.motor_status = MotorStatus()

        self.goal_current_msg = SetMotorCurrents()
        self.set_current_msg = SetCurrent()
        self.set_current_msg.id = 31
        self.set_current_msg.current = -500.0
        self.goal_current_msg.motor_goal_currents.append(self.set_current_msg)

        self.door_opening_msg = SetMotorPositions()
        self.set_position_msg = SetPosition()
        self.set_position_msg.id = 31
        self.set_position_msg.position = 2.1
        self.door_opening_msg.motor_goal_positions.append(self.set_position_msg)

        self.goal_velocity_msg = SetMotorVelocities()
        self.set_velocity_msg = SetVelocity()
        self.set_velocity_msg.id = 31
        self.set_velocity_msg.velocity = 0.0
        self.goal_velocity_msg.motor_goal_velocities.append(self.set_velocity_msg)

        self.status = 0
        self.closing_start_pos = 2.2
        self.closing_current = -600.0
        self.close_door = False

        # self.close_door()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def motor_status_callback(self, msg):
        if not self.first_status:
            self.first_status = True
        self.motor_status = msg.motors[0]
        # self.get_logger().info("Motor Position: " + str(self.motor_status.present_position))


    def spacemouse_callback(self, msg):
        btn = msg.button2
        if btn and not self.close_door:
            self.motor_future = self.client_motor_torque.call_async(SetBool.Request(data=True))
            # self.goal_current_msg.motor_goal_currents[0].current = -self.closing_current
            self.status = 0
            self.close_door = True

    def timer_callback(self):
        if not self.first_status:
            self.get_logger().info("Waiting to get motor status...")
            return
        
        # self.get_logger().info("Door status: " + str(self.close_door))
        if self.close_door:
            if not self.motor_future.done():
                return
            # self.client_motor_torque.call_async(SetBool.Request(data=True))
            if self.status == 0: 
                if self.motor_status.present_position < 1.81:
                    self.status = 3
                    self.goal_current_msg.motor_goal_currents[0].current = 0.0
                    self.get_logger().info("Changing to Satus 3")
                elif self.motor_status.present_position >= self.closing_start_pos:
                    self.status = 2
                    self.goal_current_msg.motor_goal_currents[0].current = self.closing_current
                    self.get_logger().info("Changing to Satus 2")
                else:
                    self.status = 1    
                    self.get_logger().info("Changing to Satus 1")

            elif self.status == 1 and self.motor_status.present_velocity <= 0.0 and self.motor_status.present_position >= self.closing_start_pos-0.1:
                self.status = 2
                self.goal_current_msg.motor_goal_currents[0].current = self.closing_current
                self.get_logger().info("Changing to Satus 2")
            elif self.status == 2 and self.motor_status.present_velocity < 0.0:
                self.status = 3
                self.get_logger().info("Changing to Satus 3")
            elif self.status == 3 and self.motor_status.present_velocity >= 0.0:
                self.status = 4
                self.goal_current_msg.motor_goal_currents[0].current = 0.0
                self.client_motor_torque.call_async(SetBool.Request(data=False))
                self.get_logger().info("Changing to Satus 4")
                self.close_door = False

            # self.get_logger().info("Motor Current: " + str(self.goal_current_msg.motor_goal_currents[0].current))
            if self.status == 1:
                self.goal_velocity_msg.motor_goal_velocities[0].velocity = np.clip(self.closing_start_pos - self.motor_status.present_position, 0.0, 0.5)
                # if self.motor_status.present_position >= self.closing_start_pos-0.1:
                #     self.goal_velocity_msg.motor_goal_velocities[0].velocity = 0.0
                self.vel_publisher_.publish(self.goal_velocity_msg)
                # self.pos_pusblisher_.publish(self.door_opening_msg)
            else:
                self.publisher_.publish(self.goal_current_msg)


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
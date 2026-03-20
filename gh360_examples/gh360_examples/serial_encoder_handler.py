import serial
import threading
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class EncoderSerialPublisher(Node):

    def __init__(self):
        super().__init__('serial_encoder_handler')
        
        # self.declare_parameter('serial_port', '')
        self.declare_parameter('serial_port', 'usb-FieldworkRobotics_GH2_Shoulder-MICRO_D36DKY91-if00-port0')
        # self.declare_parameter('serial_port', 'usb-FieldworkRobotics_GH2_Upper-MICRO_D35TXUXS-if00-port0')
        # self.declare_parameter('serial_port', 'usb-FieldworkRobotics_GH2_Lower-MICRO_D361ZNY8-if00-port0')
        self.declare_parameter('baud_rate', 115200)
        # 'usb-FieldworkRobotics_GH2_Shoulder-MICRO_D36DKY91-if00-port0'
        # 'usb-FieldworkRobotics_GH2_Upper-MICRO_D35TXUXS-if00-port0'
        # 'usb-FieldworkRobotics_GH2_Lower-MICRO_D361ZNY8-if00-port0'
    

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        serial_port = '/dev/serial/by-id/'+serial_port

        self.ser = serial.Serial(serial_port, baud_rate, timeout=1) 
        time.sleep(2) # Wait a moment for the connection to establish
        self.ser.flushInput() # Clear the input buffer

        self.stop_event = threading.Event()
        self.serial_thread = threading.Thread(target=self.read_serial_data, args=(self.stop_event,))
        self.serial_thread.start()

        self.publisher_ = self.create_publisher(String, 'encoders', 10)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def read_serial_data(self, stop_event):
        while True:
            raw_data = self.ser.readline()
            if raw_data:
                self.decoded_data = raw_data.decode('utf-8').strip()

    def timer_callback(self):
        if self.decoded_data != "fault":
            msg = String()
            msg.data = self.decoded_data
            self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    encoder_serial_publisher = EncoderSerialPublisher()

    rclpy.spin(encoder_serial_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_serial_publisher.stop_event.set()
    encoder_serial_publisher.serial_thread.join()
    encoder_serial_publisher.ser.close()
    encoder_serial_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
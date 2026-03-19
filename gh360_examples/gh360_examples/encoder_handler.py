import serial
import threading
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # --- Configuration ---
        # Change 'COM4' (Windows) or '/dev/ttyACM0' (Linux/Raspberry Pi) to your port
        SERIAL_PORT = '/dev/serial/by-id/usb-FWR_BOMBUS-SH-E-if00-port0' 
        # Must match the baud rate of your sending device (e.g., Arduino)
        BAUD_RATE = 115200
        # ---------------------

        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) 
        time.sleep(2) # Wait a moment for the connection to establish
        self.ser.flushInput() # Clear the input buffer

        print(f"Opened serial port: {self.ser.name}")

        self.stop_event = threading.Event()

        self.serial_thread = threading.Thread(target=self.read_serial_data, args=(self.stop_event,))
        self.serial_thread.start()

        self.publisher_ = self.create_publisher(String, '/gh360/lowerarm/encoders', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def read_serial_data(self, stop_event):
        # Read a line from the serial port (up to the newline character '\n')
        # The result is a bytes object
        while True:
            raw_data = self.ser.readline()
            
            if raw_data:
                # Decode the bytes to a string and strip leading/trailing whitespace
                self.decoded_data = raw_data.decode('utf-8').strip()
                # if decoded_data:
                #     print(f"Received: {decoded_data}")

        # return decoded_data

    def timer_callback(self):
        # try:
        #     decoded_data = self.read_serial_data()
        # except serial.SerialException as e:
        #     print(f"Serial error: {e}")

        msg = String()
        msg.data = self.decoded_data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.stop_event.set()
    minimal_publisher.serial_thread.join()
    minimal_publisher.ser.close()
    minimal_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
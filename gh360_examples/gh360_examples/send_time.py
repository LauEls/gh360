import sys

from gh360_interfaces.srv import LogTime
from std_msgs.msg import String
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(LogTime, 'erf_log_time')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LogTime.Request()

    def send_request(self, user_name, time):
        username = String()
        # username.data = user_name
        self.req.username = username
        self.req.time = time
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(str(sys.argv[1]), float(sys.argv[2]))
    minimal_client.get_logger().info('Result of erf_log_time: %s' % response.topten)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

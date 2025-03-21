import rclpy
from rclpy.node import Node

# from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import PointStamped

class PointVisual(Node):
    def __init__(self):
        super().__init__('point_visual_node')

        self.point_pub = self.create_publisher(PointStamped,'via_point', 10)
        self.point_msg = PointStamped()
        self.point_msg.point.x = -0.31
        self.point_msg.point.y = 0.46
        self.point_msg.point.z = 1.22
        self.point_msg.header.frame_id = 'base_link'

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def clbk_marker_recognition(self, msg):
        self.marker_id = msg.marker_ids[0]
        self.marker_pose = msg.poses[0]

    def timer_callback(self):
        self.point_pub.publish(self.point_msg)

def main(args=None):
    rclpy.init(args=args)

    point_visual = PointVisual()

    rclpy.spin(point_visual)
    point_visual.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
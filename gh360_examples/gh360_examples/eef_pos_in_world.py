import rclpy
from rclpy.node import Node
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose
from std_msgs.msg import Int64

import tf2_geometry_msgs

class EEFPos(Node):
    def __init__(self):
        super().__init__('EEFPos_node')

        self.eef_pub = self.create_publisher(Pose, '/eef_pose', 10)

        #Initialize a transform listener that is later used for looking up tranformations from one frame to another
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        zero_pose = Pose()
        from_frame_rel = 'eef'
        to_frame_rel = 'base_link'
        marker_id_msg = Int64()

        #Lookup the tranformation from from_frame_rel to to_frame_rel
        try:
            eef_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        eef_pose_msg = Pose()
        eef_pose_msg.position.x = eef_pose_trans.transform.translation.x
        eef_pose_msg.position.y = eef_pose_trans.transform.translation.y
        eef_pose_msg.position.z = eef_pose_trans.transform.translation.z
        eef_pose_msg.orientation = eef_pose_trans.transform.rotation

        self.eef_pub.publish(eef_pose_msg)

def main(args=None):
    rclpy.init(args=args)

    controller = EEFPos()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
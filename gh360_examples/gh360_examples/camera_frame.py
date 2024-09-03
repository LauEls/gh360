from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation


class CameraFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        rot = []
        rot.append([0.42261826174069345,	0.38302221852676516,	0.8213938062574561])
        rot.append([7.415938757451997e-15,	0.9063077885970263,	-0.42261825839446004])
        rot.append([-0.9063077885970263,	0.1802399555017374,	0.3830222218690612])
        sci_rot = Rotation.from_matrix(rot)
        sci_quat = sci_rot.as_quat()
        quat = Quaternion(x=sci_quat[0], y=sci_quat[1], z=sci_quat[2], w=sci_quat[3])
        
        trans = Vector3(x=0.13395628663621322, y=0.07813541504103579, z=0.06372504768866594)

        self.transform  = Transform(rotation=quat,translation=trans)

        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)



    def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'shoulder0'
        t.child_frame_id = 'camera_link'
        t.transform = self.transform
        # t.transform.translation.x = 0.13395628663621322
        # t.transform.translation.y = 0.07813541504103579
        # t.transform.translation.z = 0.06372504768866594
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = CameraFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

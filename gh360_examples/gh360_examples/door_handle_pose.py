#subscribe to marker poses
#generate door pose from marker pose
#generate handle pose from door pose
#publish frames for each pose

import rclpy
from rclpy.node import Node
import numpy as np

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from ros2_aruco_interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Pose, TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Int64
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation

class DoorHandlePose(Node):
    def __init__(self):
        super().__init__('door_handle_pose_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        #Initiate a ROS parameter with a default value and then read the parameters value
        # self.declare_parameter('namespace', 'tb3_5')
        # self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # self.marker_id = 1000
        # self.marker_pose = Pose()
        # self.marker_recieved = False

        # self.marker_recognition_sub = self.create_subscription(ArucoMarkers, '/'+self.namespace+'/aruco_markers', self.clbk_marker_recognition, 10)
        # self.marker_map_pose_pub = self.create_publisher(Pose, '/'+self.namespace+'/marker_map_pose', 10)
        # self.marker_id_pub = self.create_publisher(Int64,'/'+self.namespace+'/marker_id', 10)
        self.create_subscription(ArucoMarkers, '/aruco_markers', self.clbk_marker_recognition, 10)

        #Initialize a transform listener that is later used for looking up tranformations from one frame to another
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.markers = ArucoMarkers()



    def clbk_marker_recognition(self, msg):
        # self.marker_id = msg.marker_ids[0]
        # self.marker_pose = msg.poses[0]
        self.markers = msg

        for i in range(len(msg.marker_ids)):
            self.marker_id = msg.marker_ids[i]
            self.marker_pose = msg.poses[i]

            

    def timer_callback(self):
        for i in range(len(self.markers.marker_ids)):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_color_frame'
            t.child_frame_id = 'marker_'+str(self.markers.marker_ids[i])
            # t.transform = self.transform
            t.transform.translation.x = self.markers.poses[i].position.x
            t.transform.translation.y = self.markers.poses[i].position.y
            t.transform.translation.z = self.markers.poses[i].position.z
            t.transform.rotation = self.markers.poses[i].orientation
            # t.transform.translation.x = 0.13395628663621322
            # t.transform.translation.y = 0.07813541504103579
            # t.transform.translation.z = 0.06372504768866594
            # t.transform.rotation.x = 0.0
            # t.transform.rotation.y = 0.0
            # t.transform.rotation.z = 0.0
            # t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)

            t_door = TransformStamped()
            t_door.header.stamp = self.get_clock().now().to_msg()
            t_door.header.frame_id = 'marker_'+str(self.markers.marker_ids[i])
            t_door.child_frame_id = 'door_'+str(self.markers.marker_ids[i])
            t_handle = TransformStamped()
            t_handle.header.stamp = self.get_clock().now().to_msg()
            t_handle.header.frame_id = 'door_'+str(self.markers.marker_ids[i])
            t_handle.child_frame_id = 'handle_'+str(self.markers.marker_ids[i])
            if self.markers.marker_ids[i] == 1:
                t_door.transform.translation.x = -0.185
                t_door.transform.translation.y = 0.1

                t_handle.transform.translation.x = -(0.22 - (0.14))
                t_handle.transform.translation.y = 0.29-0.263
                t_handle.transform.translation.z = 0.046

                self.tf_broadcaster.sendTransform(t_door)
                self.tf_broadcaster.sendTransform(t_handle)
            elif self.markers.marker_ids[i] == 3:
                t_door.transform.translation.x = 0.1
                t_door.transform.translation.y = -0.19
                self.tf_broadcaster.sendTransform(t_door)

            
        # zero_pose = Pose()
        # from_frame_rel = 'eef'
        # to_frame_rel = 'base_link'
        # marker_id_msg = Int64()

        # #Lookup the tranformation from from_frame_rel to to_frame_rel
        # try:
        #     eef_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        #     return

        # #Tranform a Pose from from_frame_rel to to_frame_rel
        # # eef_base_pose = tf2_geometry_msgs.do_transform_pose(zero_pose, self.trans_eef_base)
        # # eef_pose = tf2_geometry_msgs.do_transform_pose(Pose(), self.trans_eef_base)

        # # if self.marker_id != 1000:
        # #     self.marker_map_pose_pub.publish(marker_map_pose)
        # #     marker_id_msg.data = self.marker_id
        # #     self.marker_id_pub.publish(marker_id_msg)
        # eef_pos = np.array([eef_pose_trans.transform.translation.x, eef_pose_trans.transform.translation.y, eef_pose_trans.transform.translation.z], dtype=np.float64)
        # eef_quat = np.array([eef_pose_trans.transform.rotation.x, eef_pose_trans.transform.rotation.y, eef_pose_trans.transform.rotation.z, eef_pose_trans.transform.rotation.w], dtype=np.float64)
        # self.get_logger().info("EEF Pos: "+str(eef_pos))
        # # self.get_logger().info("EEF Quat in base_link frame: "+str(eef_quat))
        # # self.get_logger().info("EEF Pos in base_link frame: "+str(eef_pose_trans.transform.translation))
        # # self.get_logger().info("EEF Quat in base_link frame: "+str(eef_pose_trans.transform.rotation))

def main(args=None):
    rclpy.init(args=args)

    controller = DoorHandlePose()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
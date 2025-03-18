import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers

class DoorHandlePose(Node):
    def __init__(self):
        super().__init__('door_handle_pose')
        self.markers = ArucoMarkers()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(ArucoMarkers, 'aruco_markers', self.clbk_marker_recognition, 10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_marker_recognition(self, msg):
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
            t.transform.translation.x = self.markers.poses[i].position.x
            t.transform.translation.y = self.markers.poses[i].position.y
            t.transform.translation.z = self.markers.poses[i].position.z
            t.transform.rotation = self.markers.poses[i].orientation

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
                t_door.transform.translation.x = -0.21
                t_door.transform.translation.y = 0.09

                t_handle.transform.translation.x = -(0.22 - (0.14))
                t_handle.transform.translation.y = (0.29-0.263)+0.015
                t_handle.transform.translation.z = 0.046

                self.tf_broadcaster.sendTransform(t_door)
                self.tf_broadcaster.sendTransform(t_handle)
            elif self.markers.marker_ids[i] == 3:
                t_door.transform.translation.x = 0.085
                t_door.transform.translation.y = -0.165

                t_handle.transform.translation.x = (0.29-0.263)+0.015
                t_handle.transform.translation.y = (0.22 - (0.14))
                t_handle.transform.translation.z = 0.046
                self.tf_broadcaster.sendTransform(t_door)
                self.tf_broadcaster.sendTransform(t_handle)

def main(args=None):
    rclpy.init(args=args)

    controller = DoorHandlePose()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
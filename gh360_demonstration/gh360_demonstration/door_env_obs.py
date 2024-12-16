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
from gh360_interfaces.msg import PortStatus
from geometry_msgs.msg import Pose, TransformStamped, Transform, Vector3, Quaternion
from std_msgs.msg import Int64, Float64
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation

from gh360_interfaces.msg import DoorEnv

class DoorEnvObs(Node):
    def __init__(self):
        super().__init__('door_environment_observation_node')
        # self.tf_broadcaster = TransformBroadcaster(self)
        #Initiate a ROS parameter with a default value and then read the parameters value
        # self.declare_parameter('namespace', 'tb3_5')
        # self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # self.marker_id = 1000
        # self.marker_pose = Pose()
        # self.marker_recieved = False

        # self.marker_recognition_sub = self.create_subscription(ArucoMarkers, '/'+self.namespace+'/aruco_markers', self.clbk_marker_recognition, 10)
        # self.marker_map_pose_pub = self.create_publisher(Pose, '/'+self.namespace+'/marker_map_pose', 10)
        # self.marker_id_pub = self.create_publisher(Int64,'/'+self.namespace+'/marker_id', 10)
        # self.create_subscription(ArucoMarkers, '/aruco_markers', self.clbk_marker_recognition, 10)

        #Initialize a transform listener that is later used for looking up tranformations from one frame to another
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.door_env_msg = DoorEnv()
        self.door_env_pub = self.create_publisher(DoorEnv, '/door_env', 10)

        self.create_subscription(Float64, '/door/filtered_handle_angle', self.handle_angle_callback, 10)
        self.create_subscription(
            PortStatus,
            '/door/motor_status',
            self.hinge_callback,
            10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def handle_angle_callback(self, msg):
        self.door_env_msg.handle_angle = msg.data

    def hinge_callback(self, msg):
        motor_pos = msg.motors[0].present_position
        # offset = 3.2505-1.4649
        offset = 1.7809
        fourty_five_deg_pos = 2.5847 - offset
        # max_pos = 3.548 - offset
        # hinge_angle_multi = (17.1887*np.pi/180) / max_pos
        hinge_angle_multi = (np.pi/4) / fourty_five_deg_pos

        self.door_env_msg.hinge_angle = (motor_pos - offset) * hinge_angle_multi

    def timer_callback(self):
        to_frame_rel = 'base_link'
        handle_frames = ['handle_1', 'handle_3']
        handle_pos = []

        for handle_frame in handle_frames:
            try:
                handle_pose_trans = self.tf_buffer.lookup_transform(to_frame_rel, handle_frame, rclpy.time.Time())
                handle_pos.append([handle_pose_trans.transform.translation.x, handle_pose_trans.transform.translation.y, handle_pose_trans.transform.translation.z])
            except TransformException as ex:
                pass
                # self.get_logger().info(f'Could not transform {handle_frame} to {to_frame_rel}: {ex}')
                
        if len(handle_pos) == 0:
            return
        avg_handle_pos = np.mean(handle_pos, axis=0)
        # self.get_logger().info(f"handle pos: {avg_handle_pos}")
        
        self.door_env_msg.handle_position.x = avg_handle_pos[0]
        self.door_env_msg.handle_position.y = avg_handle_pos[1]
        self.door_env_msg.handle_position.z = avg_handle_pos[2]
        # self.door_env_msg.handle_angle = 0.0
        # self.door_env_msg.hinge_angle = 0.0
        self.door_env_pub.publish(self.door_env_msg)


def main(args=None):
    rclpy.init(args=args)

    controller = DoorEnvObs()

    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
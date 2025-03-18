#include "gh360/eef_pose.hpp"

EEFPose::EEFPose() : Node("eef_pose")
{
    this->eef_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("eef_pose", 10);

    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

    this->from_frame = "eef";
    this->to_frame = "base_link";

    this->timer_ = this->create_wall_timer(100ms, std::bind(&EEFPose::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "EEF Pose node started");
}

EEFPose::~EEFPose()
{
}

void EEFPose::timer_callback()
{
    geometry_msgs::msg::Pose eef_pose_msg;
    try
    {
        geometry_msgs::msg::TransformStamped transform = this->tf_buffer_->lookupTransform(this->to_frame, this->from_frame, rclcpp::Time(0));
        eef_pose_msg.position.x = transform.transform.translation.x;
        eef_pose_msg.position.y = transform.transform.translation.y;
        eef_pose_msg.position.z = transform.transform.translation.z;
        eef_pose_msg.orientation = transform.transform.rotation;
        this->eef_pose_publisher_->publish(eef_pose_msg);
    }
    catch (tf2::TransformException &ex)
    {
        // RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto eef_pose_node = std::make_shared<EEFPose>();
    rclcpp::spin(eef_pose_node);
    rclcpp::shutdown();

    return 0;
}
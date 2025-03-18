#include "gh360/camera_frame.hpp"

CameraFrame::CameraFrame() : Node("camera_frame")
{
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


    this->transform = geometry_msgs::msg::TransformStamped();
    this->transform.header.stamp = this->now();
    this->transform.header.frame_id = "shoulder0";
    this->transform.child_frame_id = "camera_link";
    this->transform.transform.translation.x = 0.13395628663621322;
    this->transform.transform.translation.y = 0.07813541504103579;
    this->transform.transform.translation.z = 0.06372504768866594;
    this->transform.transform.rotation.x = 0.18302276459646596;
    this->transform.transform.rotation.y = 0.5245159060608517;
    this->transform.transform.rotation.z = -0.11628237572410241;
    this->transform.transform.rotation.w = 0.8233250518626218;

    this->timer_ = this->create_wall_timer(100ms, std::bind(&CameraFrame::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Camera Frame node started");
}

CameraFrame::~CameraFrame()
{
}

void CameraFrame::timer_callback()
{
    this->transform.header.stamp = this->now();

    this->tf_broadcaster_->sendTransform(this->transform);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto camera_frame_node = std::make_shared<CameraFrame>();
    rclcpp::spin(camera_frame_node);
    rclcpp::shutdown();

    return 0;
}
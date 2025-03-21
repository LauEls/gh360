#ifndef EEF_POSE_HPP_
#define EEF_POSE_HPP_

#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"


using namespace std::chrono_literals;

class EEFPose : public rclcpp::Node
{
    public:
        EEFPose();
        virtual ~EEFPose();

    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr eef_pose_publisher_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string from_frame;
        std::string to_frame;
};

#endif // EEF_POSE_HPP_
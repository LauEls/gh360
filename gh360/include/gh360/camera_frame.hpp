#ifndef CAMERA_FRAME_HPP_
#define CAMERA_FRAME_HPP_

#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class CameraFrame : public rclcpp::Node
{
    public:
        CameraFrame();
        virtual ~CameraFrame();

    private:
        void timer_callback();
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        geometry_msgs::msg::TransformStamped transform;
};

#endif // CAMERA_FRAME_HPP_
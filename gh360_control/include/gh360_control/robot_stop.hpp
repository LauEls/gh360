#ifndef ROBOT_STOP_HPP_
#define ROBOT_STOP_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class RobotStop : public rclcpp::Node
{
    public:
        RobotStop();
        virtual ~RobotStop();

    private:
        void timer_callback();
        void robot_stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr robot_stop_service_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_stop_publisher_;
        bool robot_stop = false;

};
#endif // ROBOT_STOP_HPP_
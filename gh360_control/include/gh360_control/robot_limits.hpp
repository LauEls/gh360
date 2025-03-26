#ifndef ROBOT_LIMITS_HPP_
#define ROBOT_LIMITS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "gh360/util/config_parser.hpp"
#include "gh360_interfaces/srv/set_robot_limits.hpp"
#include "gh360_interfaces/srv/set_joint_limits.hpp"
// #include "gh360_interfaces/msg/joint_limits.hpp"
#include "gh360/joint_types/soft_joint.hpp"

using namespace std::chrono_literals;

class RobotLimits : public rclcpp::Node
{
    public:
        RobotLimits();
        virtual ~RobotLimits();

    private:
        void robot_limits_callback(const std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Request> request, std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Response> response);
            
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<gh360_interfaces::srv::SetRobotLimits>::SharedPtr robot_limits_service_;
        rclcpp::Client<gh360_interfaces::srv::SetJointLimits>::SharedPtr shoulder_limits_client_;
        rclcpp::Client<gh360_interfaces::srv::SetJointLimits>::SharedPtr upperarm_limits_client_;
        rclcpp::Client<gh360_interfaces::srv::SetJointLimits>::SharedPtr lowerarm_limits_client_;
        
        std::vector<Joint*> joints;
        gh360_interfaces::srv::SetJointLimits::Request::SharedPtr joint_limits_request;

        unsigned int joint_cnt;
        unsigned int motor_cnt;
};
#endif // ROBOT_LIMITS_HPP_
#ifndef JOINT_POSITION_HPP_
#define JOINT_POSITION_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// #include "gh360/motor_dictionaries/motor_dict.hpp"
#include "gh360/joint_types/soft_joint.hpp"
#include "gh360/util/config_parser.hpp"
#include "gh360_interfaces/msg/set_motor_velocities.hpp"
#include "gh360_interfaces/msg/port_status.hpp"

using namespace std::chrono_literals;

class JointPosition : public rclcpp::Node
{
    public:
        JointPosition();
        virtual ~JointPosition();

    private:
        void cmd_joint_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);

        rclcpp::Publisher<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr motor_velocity_publisher_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_joint_pos_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_position_subscriber_;
        rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr motor_states_subscriber_;

        std::vector<Joint*> joints;
        double max_motor_vel;
        double joint_pos_accuracy;
        bool joint_states_recieved = false;
        bool motor_states_recieved = false;
        gh360_interfaces::msg::SetMotorVelocities motor_goal_vel_msg;

};

#endif // JOINT_POSITION_HPP_
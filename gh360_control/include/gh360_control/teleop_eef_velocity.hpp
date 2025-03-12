#ifndef TELEOP_EEF_VELOCITY_HPP_
#define TELEOP_EEF_VELOCITY_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "gh360_interfaces/msg/set_motor_positions.hpp"
#include "gh360_interfaces/msg/bool_multi_array.hpp"
#include "gh360_interfaces/msg/port_status.hpp"
#include "gh360/util/config_parser.hpp"

using namespace std::chrono_literals;

class TeleopEEFVelocity : public rclcpp::Node
{
    public:
        TeleopEEFVelocity();
        virtual ~TeleopEEFVelocity();

    private:
        // void timer_callback();
        void teleop_commands_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void teleop_buttons_callback(const gh360_interfaces::msg::BoolMultiArray::SharedPtr msg);
        void motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);

        // rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr eef_velocity_publisher_;
        rclcpp::Publisher<gh360_interfaces::msg::SetMotorPositions>::SharedPtr motor_position_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_commands_subscriber_;
        rclcpp::Subscription<gh360_interfaces::msg::BoolMultiArray>::SharedPtr teleop_buttons_subscriber_;
        rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr motor_states_subscriber_;

        std::vector<Joint*> joints;
        geometry_msgs::msg::Twist desired_eef_velocity;
        gh360_interfaces::msg::SetMotorPositions motor_reset_msg;
        std::vector<double> reset_motor_pos;
        float translation_scaler = 1.0;
        float rotation_scaler = 1.0;
        bool reseting = false;
        bool motor_states_recieved = false;
        // bool motor_moving = false;
};

#endif // TELEOP_EEF_VELOCITY_HPP_
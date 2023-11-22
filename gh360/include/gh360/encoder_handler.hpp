#ifndef ENCODER_HANDLER_HPP_
#define ENCODER_HANDLER_HPP_

#include <cstdio>
#include <memory>
#include <iostream>
#include <vector>
#include <math.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gh360_interfaces/msg/arm_encoder_states.hpp"
#include "gh360_interfaces/msg/joint_encoder_state.hpp"

using namespace std::chrono_literals;

namespace gh360
{
    class EncoderHandler : public rclcpp::Node
    {
        public:
            EncoderHandler();
            virtual ~EncoderHandler();





        private:
            void timer_callback();
            void shoulder_encoder_callback(const std_msgs::msg::String::SharedPtr msg);
            void upperarm_encoder_callback(const std_msgs::msg::String::SharedPtr msg);
            void lowerarm_encoder_callback(const std_msgs::msg::String::SharedPtr msg);

            std::vector<double> strToDoubleVector(std::string s, std::string del = " ");

            rclcpp::TimerBase::SharedPtr timer_;
            // rclcpp::Publisher<gh360_interfaces::msg::PortStatus>::SharedPtr encoder_publisher_;
            rclcpp::Publisher<gh360_interfaces::msg::ArmEncoderStates>::SharedPtr encoder_state_publisher_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shoulder_encoder_subscriber_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr upperarm_encoder_subscriber_;
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lowerarm_encoder_subscriber_;

            std::vector<std::string> joint_names = {"shoulder_yaw", "shoulder_roll", "shoulder_pitch", "upperarm_roll", "elbow", "wrist_pitch"};
            std::vector<std::string> shoulder_joint_names;
            std::vector<std::string> upperarm_joint_names;
            std::vector<std::string> lowerarm_joint_names;
            std::vector<int> shoulder_port_ids;
            std::vector<int> upperarm_port_ids;
            std::vector<int> lowerarm_port_ids;
            std::vector<double> shoulder_offsets;
            std::vector<double> upperarm_offsets;
            std::vector<double> lowerarm_offsets;
            std::vector<int> shoulder_inverters;
            std::vector<int> upperarm_inverters;
            std::vector<int> lowerarm_inverters;
            std::vector<double> shoulder_joint_angles;
            std::vector<double> upperarm_joint_angles;
            std::vector<double> lowerarm_joint_angles;
            std::vector<double> shoulder_joint_vels;
            std::vector<double> upperarm_joint_vels;
            std::vector<double> lowerarm_joint_vels;

            std::chrono::time_point<std::chrono::system_clock> shoulder_prev_time;
            std::chrono::time_point<std::chrono::system_clock> upperarm_prev_time;
            std::chrono::time_point<std::chrono::system_clock> lowerarm_prev_time;
    };   

}

#endif // ENCODER_HANDLER_HPP_
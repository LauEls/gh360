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

#include "joint_types/encoder.hpp"

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
            void encoder_callback(const std_msgs::msg::String::SharedPtr msg, const std::string port_name);

            std::vector<double> strToDoubleVector(std::string s, std::string del = " ");

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<gh360_interfaces::msg::ArmEncoderStates>::SharedPtr encoder_state_publisher_;

            std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> encoder_subscribers;
            std::vector<std::string> joint_names;
            std::vector<std::string> port_names;
            std::vector<Encoder*> encoders;
            std::vector<bool> data_recieved;

            float alpha = 0.1;
    };   

}

#endif // ENCODER_HANDLER_HPP_
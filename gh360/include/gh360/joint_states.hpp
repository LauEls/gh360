#ifndef JOINT_STATES_HPP_
#define JOINT_STATES_HPP_

#include <cstdio>
#include <memory>
#include <iostream>
#include <vector>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "gh360_interfaces/msg/port_status.hpp"
#include "gh360_interfaces/msg/arm_encoder_states.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "joint_types/soft_joint.hpp"
#include "joint_types/motor_joint.hpp"
#include "util/config_parser.hpp"

using namespace std::chrono_literals;

namespace gh360
{
    /**
     * @brief This class handles the joint states of the robot. It subscribes to the motor states and encoder data, and publishes the joint states to a ROS2 topic.
     */
    class JointStates : public rclcpp::Node
    {
        public:
            JointStates();
            virtual ~JointStates();

        private:
            void timer_callback();
            void motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);
            void encoder_callback(const gh360_interfaces::msg::ArmEncoderStates::SharedPtr msg);

            sensor_msgs::msg::JointState create_joint_state_msg();

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr motor_states_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::ArmEncoderStates>::SharedPtr encoder_subscriber_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;

            std::vector<Joint*> joints;
            std::vector<std::string> joint_names;
            bool soft_joint_states_recieved = false;
            bool motor_joint_states_recieved = false;
    };

}

#endif // JOINT_STATES_HPP_
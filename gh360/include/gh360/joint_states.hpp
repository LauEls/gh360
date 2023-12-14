#ifndef JOINT_STATES_HPP_
#define JOINT_STATES_HPP_

#include <cstdio>
#include <memory>
#include <iostream>
#include <vector>
#include <math.h>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "gh360_interfaces/msg/port_status.hpp"
#include "gh360_interfaces/msg/motor_status.hpp"
#include "gh360_interfaces/msg/set_motor_positions.hpp"
#include "gh360_interfaces/msg/set_motor_currents.hpp"
#include "gh360_interfaces/msg/set_position.hpp"
#include "gh360_interfaces/msg/set_velocity.hpp"
#include "gh360_interfaces/msg/set_current.hpp"
#include "gh360_interfaces/srv/motor_position_step.hpp"
#include "gh360_interfaces/srv/motor_velocity_step.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "gh360_interfaces/msg/arm_encoder_states.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include <DynamixelWorkbench.h>
// #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

// #include "motor_dict.hpp"
#include "mx_106_dict.hpp"
#include "mx_64_dict.hpp"
#include "joint.hpp"
#include "soft_joint.hpp"
#include "motor_joint.hpp"

using namespace std::chrono_literals;

namespace gh360
{
    class JointStates : public rclcpp::Node
    {
        public:

            JointStates();
            virtual ~JointStates();

        private:
            void timer_callback();
            void lowerarm_motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);
            void encoder_callback(const gh360_interfaces::msg::ArmEncoderStates::SharedPtr msg);

            sensor_msgs::msg::JointState create_joint_state_msg();

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr lowerarm_motor_states_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::ArmEncoderStates>::SharedPtr encoder_subscriber_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;

            std::vector<Joint*> joints;
            std::vector<std::string> joint_names;
            bool soft_joint_states_recieved = false;
            bool motor_joint_states_recieved = false;
            // std::vector<std::string> upperarm_joint_names;
            // std::vector<std::string> lowerarm_joint_names;
    };

}

#endif // JOINT_STATES_HPP_
#ifndef MOTOR_STATES_HPP_
#define MOTOR_STATES_HPP_

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
     * @brief This class publishes the motor states sorted order from base to end-effector, so that the subscriber does not need to know the motor IDs.
     */
    class MotorStates : public rclcpp::Node
    {
        public:
            MotorStates();
            virtual ~MotorStates();

        private:
            void timer_callback();
            void motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr motor_states_subscriber_;
            rclcpp::Publisher<gh360_interfaces::msg::PortStatus>::SharedPtr motor_states_publisher_;

            std::vector<Joint*> joints;
            bool motor_states_recieved = false;
    };

}

#endif // MOTOR_STATES_HPP_
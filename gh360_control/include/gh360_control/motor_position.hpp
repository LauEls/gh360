#ifndef MOTOR_POSITION_HPP_
#define MOTOR_POSITION_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// #include "gh360/motor_dictionaries/motor_dict.hpp"
#include "gh360/joint_types/soft_joint.hpp"
#include "gh360/util/config_parser.hpp"
#include "gh360_interfaces/msg/set_motor_velocities.hpp"
#include "gh360_interfaces/msg/set_motor_positions.hpp"
#include "gh360_interfaces/msg/port_status.hpp"

using namespace std::chrono_literals;

class MotorPosition : public rclcpp::Node
{
    public:
        MotorPosition();
        virtual ~MotorPosition();

    private:
        // void timer_callback();
        void cmd_motor_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);
        void calculate_motor_goal_velocity(gh360_interfaces::msg::SetMotorPositions msg);

        // rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr motor_velocity_publisher_;
        rclcpp::Publisher<gh360_interfaces::msg::SetMotorPositions>::SharedPtr motor_position_publisher_;
        rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr motor_states_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_motor_pos_subscriber_;

        // std::vector<double> desired_velocity;
        // std::vector<double> current_joint_pos;
        // sensor_msgs::msg::JointState joint_goal_vel_msg;

        std::vector<Joint*> joints;
        double max_motor_vel;
        //motor_goal_vel_msg
        //joint_states_subscriber
        gh360_interfaces::msg::SetMotorVelocities motor_goal_vel_msg;
        gh360_interfaces::msg::SetMotorPositions motor_goal_pos_msg;
        std::string control_mode; //(open loop or closed loop)
        std::string command_interface; //(velocity or position)
        double motor_pos_accuracy;
        bool motor_states_recieved = false;
        // gh360_interfaces::msg::SetMotorPositions motor_goal_pos_msg;
        bool goal_recieved = false;
        bool goal_reached = false;

};

#endif // MOTOR_POSITION_HPP_
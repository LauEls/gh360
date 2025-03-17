#ifndef MOTOR_HANDLER_HPP_
#define MOTOR_HANDLER_HPP_

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
#include "gh360_interfaces/msg/set_motor_velocities.hpp"
#include "gh360_interfaces/msg/set_position.hpp"
#include "gh360_interfaces/msg/set_velocity.hpp"
#include "gh360_interfaces/msg/set_current.hpp"
#include "gh360_interfaces/srv/motor_position_step.hpp"
#include "gh360_interfaces/srv/motor_velocity_step.hpp"
#include "gh360_interfaces/srv/set_robot_limits.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "gh360_interfaces/msg/arm_encoder_states.hpp"
// #include <DynamixelWorkbench.h>
// #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

// #include "motor_dict.hpp"
#include "joint_types/motor.hpp"
#include "motor_dictionaries/mx_106_dict.hpp"
#include "motor_dictionaries/mx_64_dict.hpp"
#include "joint_types/joint.hpp"
#include "joint_types/soft_joint.hpp"
#include "joint_types/motor_joint.hpp"
#include "util/dynamixel.hpp"
#include "util/config_parser.hpp"

using namespace std::chrono_literals;

namespace gh360
{
    class MotorHandler : public rclcpp::Node
    {
        public:
            MotorHandler();
            virtual ~MotorHandler();

            /**
             * @brief Set goal values for all motors in the joints vector. The type of goal value is determined by the template type T.
             * @param motor_goal_positions The goal values for the motors.
             */
            template <typename T>
            void setMotorGoal(std::vector<T> motor_goal_positions);
            
            /**
             * @brief Returns the present position, velocity, current and temperature of the motors
             * @return The present state of the motors as a PortStatus message
             */
            gh360_interfaces::msg::PortStatus getMotorStates();
            
            /**
             * @brief Checks if the present current on all the motors are within the safety limits defined by the motor model.
             * @return True if the current is within the safety limits.
             */
            bool safetyCheck();

            /**
             * @brief Initializes the motor positions to the values defined in the motor_init_pos parameter.
             * @return True if the motor positions are initialized.
             */
            bool initMotorPositions();

            /**
             * @brief Safety check for the movement of the motors during initialization as well as detects if the motors finished moving.
             * @param reference_current If true, the present current of the motors is compared to a reference current.
             * @param reference_joint_angle If true, the present joint angle is compared to a reference joing angle.
             * @return True if the movement has been completed.
             */
            bool initMovementCheck(bool reference_current=false, bool reference_joint_angle=false);

            /**
             * @brief If the motors are in velocity or current mode, this function checks if the goal values are still being updated.
             * If the goal values are not updated for more than 200ms, velocity and current goal values are set to 0.
             */
            void check_goal_alive();

            /**
             * @brief Checks if the state of the robot (joints and motors) are within the limits.
             * If not, the goal velocities and currents are set to 0.
             */
            void check_limits();

        private:
            void timer_callback();
            void motor_goal_positions_callback(const gh360_interfaces::msg::SetMotorPositions::SharedPtr msg);
            void motor_goal_current_callback(const gh360_interfaces::msg::SetMotorCurrents::SharedPtr msg);
            void motor_goal_velocity_callback(const gh360_interfaces::msg::SetMotorVelocities::SharedPtr msg);
            void position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response);
            // void velocity_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorVelocityStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorVelocityStep::Response> response);
            // void delta_position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response);
            void set_torque_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void move_home_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void move_home_sub_callback(const std_msgs::msg::Bool::SharedPtr msg);
            void encoder_callback(const gh360_interfaces::msg::ArmEncoderStates::SharedPtr msg);
            void set_torque_sub_callback(const std_msgs::msg::Bool::SharedPtr msg);
            void set_robot_limits_callback(const std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Request> request, std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Response> response);

            DynamixelHandler* dxl_handler;
            
            int protocol;
            bool torque_start;
            constexpr static int motor_cnt = 2;
            bool multi_motor_models = false;
            MotorDictionary* joints_motor_model;
            bool joint_states_recieved = false;
            bool motors_initiated = false;
            bool emergency_stop = false;
            bool require_encoder_data = true;
         
            std::chrono::time_point<std::chrono::high_resolution_clock> velocity_goal_timestamp;
            std::chrono::time_point<std::chrono::high_resolution_clock> current_goal_timestamp;
            int init_state = 0;

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<gh360_interfaces::msg::PortStatus>::SharedPtr motor_state_publisher_;
            rclcpp::Subscription<gh360_interfaces::msg::SetMotorPositions>::SharedPtr motor_goal_positions_subscriber_;
            rclcpp::Service<gh360_interfaces::srv::MotorPositionStep>::SharedPtr position_step_service_;
            // rclcpp::Service<gh360_interfaces::srv::MotorVelocityStep>::SharedPtr velocity_step_service_;
            // rclcpp::Service<gh360_interfaces::srv::MotorPositionStep>::SharedPtr delta_position_step_service_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_service_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_home_service_;
            rclcpp::Service<gh360_interfaces::srv::SetRobotLimits>::SharedPtr set_robot_limits_service_;
            rclcpp::Subscription<gh360_interfaces::msg::ArmEncoderStates>::SharedPtr encoder_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::SetMotorCurrents>::SharedPtr motor_goal_currents_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr motor_goal_velocities_subscriber_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr move_home_subscriber_;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr set_torque_subscriber_;

            std::vector<std::string> joint_names;
            std::vector<Joint*> joints;
           
    };

}

#endif // MOTOR_HANDLER_HPP_
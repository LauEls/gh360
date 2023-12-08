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
#include "gh360_interfaces/msg/set_position.hpp"
#include "gh360_interfaces/msg/set_velocity.hpp"
#include "gh360_interfaces/msg/set_current.hpp"
#include "gh360_interfaces/srv/motor_position_step.hpp"
#include "gh360_interfaces/srv/motor_velocity_step.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "gh360_interfaces/msg/arm_encoder_states.hpp"
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
    class MotorHandler : public rclcpp::Node
    {
        public:

            // using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
            // using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

            MotorHandler();
            virtual ~MotorHandler();

            // bool addJoint(std::string joint_name);
            // bool initJoints(std::vector<std::string> joint_names);
            bool openPortsAndSetBaudrate();
            MotorDictionary* getMotorModel(int motor_id);
            bool setOperatingMode(Joint* joint, int value);
            bool setPositionControlMode();
            bool setVelocityControlMode();
            bool setCurrentControlMode();
            bool setTorqueEnable(Joint* joint, int value);
            bool setVelocityProfile(Joint* joint, double value);
            bool setAccelerationProfile(Joint* joint, double value);
            // bool setPositionControlMode(uint8_t id);
            // bool setExtendedPositionControlMode(uint8_t id);
            bool readPresentPosition();
            // bool readPresentVelocity();
            // bool readPresentCurrent();
            // bool readPresentTemperature();
            // bool writeGoalPosition();
            bool setMotorGoalPositions(std::vector<gh360_interfaces::msg::SetPosition> motor_goal_positions);
            bool setMotorGoalVelocities(std::vector<gh360_interfaces::msg::SetVelocity> motor_goal_velocities);
            bool setMotorGoalCurrents(std::vector<gh360_interfaces::msg::SetCurrent> motor_goal_currents);
            bool setDeltaMotorGoalPositions(std::vector<gh360_interfaces::msg::SetPosition> delta_motor_goal_positions);
            gh360_interfaces::msg::PortStatus getMotorStatus();
            bool syncRead(uint8_t size, uint8_t address);
            bool syncWrite(uint8_t size, uint8_t address);
            bool writeRegister(uint8_t id, int32_t data, uint8_t data_size, uint8_t address);
            bool safetyCheck();
            bool initMotorPositions();
            bool initMovementCheck(bool reference_current=false, bool reference_joint_angle=false);

        private:
            void timer_callback();
            void motor_goal_positions_callback(const gh360_interfaces::msg::SetMotorPositions::SharedPtr msg);
            void motor_goal_current_callback(const gh360_interfaces::msg::SetMotorCurrents::SharedPtr msg);
            void position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response);
            void velocity_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorVelocityStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorVelocityStep::Response> response);
            void delta_position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response);
            void set_torque_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void move_home_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            void encoder_callback(const gh360_interfaces::msg::ArmEncoderStates::SharedPtr msg);

            dynamixel::PortHandler * portHandler;
            dynamixel::PacketHandler * packetHandler;

            const char* port_name;
            int baud_rate;
            int protocol;
            constexpr static int motor_cnt = 2;
            bool multi_motor_models = false;
            MotorDictionary* joints_motor_model;
            bool joint_states_recieved = false;
            bool motors_initiated = false;
            bool emergency_stop = false;
            bool require_encoder_data = true;
            int init_state = 0;
            // std::vector<double> init_reference_current, init_reference_position;
            std::vector<int> init_motor_side;

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<gh360_interfaces::msg::PortStatus>::SharedPtr motor_state_publisher_;
            rclcpp::Subscription<gh360_interfaces::msg::SetMotorPositions>::SharedPtr motor_goal_positions_subscriber_;
            rclcpp::Service<gh360_interfaces::srv::MotorPositionStep>::SharedPtr position_step_service_;
            rclcpp::Service<gh360_interfaces::srv::MotorVelocityStep>::SharedPtr velocity_step_service_;
            rclcpp::Service<gh360_interfaces::srv::MotorPositionStep>::SharedPtr delta_position_step_service_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_service_;
            rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_home_service_;
            rclcpp::Subscription<gh360_interfaces::msg::ArmEncoderStates>::SharedPtr encoder_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::SetMotorCurrents>::SharedPtr motor_goal_currents_subscriber_;

            std::vector<std::string> joint_names;
            std::vector<Joint*> joints;
            // std::string joint_type;
            // int right_id;
            // int left_id;

            // gh360::MX_106_DICT* motor_test = new gh360::MX_106_DICT(2);

            // uint16_t model_number = 0;
            // uint8_t dxl_id[motor_cnt] = {30, 31};
            // std::vector<MotorDictionary*> motor_dicts;

            // DynamixelWorkbench dxl_wb;
            const char *log;
    };

}

#endif // MOTOR_HANDLER_HPP_
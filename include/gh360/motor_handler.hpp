#ifndef MOTOR_HANDLER_HPP_
#define MOTOR_HANDLER_HPP_

#include <cstdio>
#include <memory>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
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
            bool setTorqueEnable(Joint* joint, int value);
            // bool setPositionControlMode(uint8_t id);
            // bool setExtendedPositionControlMode(uint8_t id);
            bool readPresentPosition();
            // bool writeGoalPosition();
            bool syncRead(uint8_t size, uint8_t address);
            bool syncWrite(uint8_t size, uint8_t address);
            bool writeRegister(uint8_t id, int data, uint8_t data_size, uint8_t address);

        private:
            dynamixel::PortHandler * portHandler;
            dynamixel::PacketHandler * packetHandler;

            const char* port_name;
            int baud_rate;
            int protocol;
            constexpr static int motor_cnt = 2;
            bool multi_motor_models = false;
            MotorDictionary* joints_motor_model;

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
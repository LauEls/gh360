#ifndef PORT_HANDLER_NODE_HPP_
#define PORT_HANDLER_NODE_HPP_

#include <cstdio>
#include <memory>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
// #include <DynamixelWorkbench.h>
// #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
// #include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

// #include "motor_dict.hpp"
// #include "mx_106_dict.hpp"
// #include "mx_64_dict.hpp"
#include "motor_handler.hpp"

namespace gh360
{
    class PortHandlerNode : public rclcpp::Node
    {
        public:
            PortHandlerNode();
            virtual ~PortHandlerNode();

        private:
            std::vector<gh360::MotorHandler*> ports;
            int baud_rate;
            int protocol;
            std::vector<std::string> joint_names;



    };

}

#endif // PORT_HANDLER_NODE_HPP_
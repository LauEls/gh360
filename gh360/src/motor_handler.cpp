#include <memory>
#include <string>

// #include "dynamixel_sdk/dynamixel_sdk.h"
// #include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
// #include "rclcpp/rclcpp.hpp"

// #include <cstdio>
// #include "../include/gh360/motor_handler.hpp"
#include "motor_handler.hpp"
// #include "gh360/motor_dict.hpp"
// #include "gh360/mx_106_dict.hpp"
// #include "gh360/mx_64_dict.hpp"

// Control table address for X series (except XL-320)
// #define ADDR_OPERATING_MODE 11
// #define ADDR_TORQUE_ENABLE 64
// #define ADDR_GOAL_POSITION 116
// #define ADDR_PRESENT_POSITION 132

// // Protocol version
// #define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// // Default setting
// #define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
// #define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// dynamixel::PortHandler * portHandler;
// dynamixel::PacketHandler * packetHandler;

// uint8_t dxl_error = 0;
// uint32_t goal_position = 0;
// int dxl_comm_result = COMM_TX_FAIL;

gh360::MotorHandler::MotorHandler()
: Node("motor_handler")
{
    RCLCPP_INFO(this->get_logger(), "Run motor handler node");

    this->declare_parameter("port_name", "default");
    std::string port_name_string = get_parameter("port_name").as_string();
    // RCLCPP_INFO(this->get_logger(), "Port Name: %s", port_name_string.c_str());
    this->port_name = port_name_string.c_str();
    // RCLCPP_INFO(this->get_logger(), "Port Name: %s", this->port_name);
    this->declare_parameter("baud_rate", 0);
    this->baud_rate = get_parameter("baud_rate").as_int();
    this->declare_parameter("protocol", 0);
    this->protocol = get_parameter("protocol").as_int();

    this->openPortsAndSetBaudrate();

    std::vector<std::string> string_array;
    this->declare_parameter("joint_names", string_array);
    this->joint_names = get_parameter("joint_names").as_string_array();

    // std::type_info& motor_model_type;
    gh360::MotorDictionary* motor_model_type;
    this->multi_motor_models = false;
    for (unsigned int i = 0; i < this->joint_names.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Joint Name: %s", this->joint_names[i].c_str());
        
        std::string joint_name = this->joint_names[i];
        this->declare_parameter(this->joint_names[i]+".joint_type", "default");
        std::string joint_type = get_parameter(this->joint_names[i]+".joint_type").as_string();

        // Joint * new_joint;

        if (joint_type == "soft_joint") {
            this->declare_parameter(joint_name+".right.motor_id", 0);
            this->declare_parameter(joint_name+".left.motor_id", 0);
            this->declare_parameter(joint_name+".right.movement_direction", 0);
            this->declare_parameter(joint_name+".left.movement_direction", 0);

            SoftJoint * new_joint = new SoftJoint();
            new_joint->set_joint_name(joint_name);
            new_joint->set_right_motor_id(get_parameter(joint_name+".right.motor_id").as_int());
            new_joint->set_left_motor_id(get_parameter(joint_name+".left.motor_id").as_int());
            new_joint->set_right_movement_direction(get_parameter(joint_name+".right.movement_direction").as_int());
            new_joint->set_left_movement_direction(get_parameter(joint_name+".left.movement_direction").as_int()); 
            new_joint->set_right_motor_model(this->getMotorModel(new_joint->get_right_motor_id()));
            new_joint->set_left_motor_model(this->getMotorModel(new_joint->get_left_motor_id()));
            this->setOperatingMode(new_joint, 4);
            this->setTorqueEnable(new_joint, 1);
            this->joints.push_back(new_joint);

            if (i == 0) motor_model_type = new_joint->get_right_motor_model();
            else if (typeid(new_joint->get_right_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;

            if (typeid(new_joint->get_left_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;
        }
        else {
            this->declare_parameter(joint_name+".motor_id", 0);
            this->declare_parameter(joint_name+".movement_direction", 0);

            MotorJoint * new_joint = new MotorJoint(); 
            new_joint->set_joint_name(joint_name);
            new_joint->set_motor_id(get_parameter(joint_name+".motor_id").as_int());
            new_joint->set_movement_direction(get_parameter(joint_name+".movement_direction").as_int());
            new_joint->set_motor_model(this->getMotorModel(new_joint->get_motor_id()));
            this->setOperatingMode(new_joint, 3);
            this->setTorqueEnable(new_joint, 1);
            this->joints.push_back(new_joint);

            if (i == 0) motor_model_type = new_joint->get_motor_model();
            else if (typeid(new_joint->get_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;
        }

    }

    if (!this->multi_motor_models) this->joints_motor_model = motor_model_type;

    this->syncRead(this->joints_motor_model->Present_Position.size, this->joints_motor_model->Present_Position.address);
    this->syncRead(this->joints_motor_model->Present_Velocity.size, this->joints_motor_model->Present_Velocity.address);
    this->syncRead(this->joints_motor_model->Present_Current.size, this->joints_motor_model->Present_Current.address);
    this->syncRead(this->joints_motor_model->Present_Temperature.size, this->joints_motor_model->Present_Temperature.address);

    // RCLCPP_INFO(this->get_logger(), "Motor Position: %f", this->joints[0].get_right_motor_present_position());
    // SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[0]);
    // RCLCPP_INFO(this->get_logger(), "Current Right Position: %f", soft_joint->get_right_motor_present_position());
    // RCLCPP_INFO(this->get_logger(), "Current Right Position: %f", soft_joint->get_right_motor_goal_position());
    // this->motor_goal_positions_subscriber_ = this->create_subscription<gh360_interfaces::msg::SetMotorPositions>(
    //   "set_motor_positions", 10, std::bind(&gh360::MotorHandler::motor_goal_positions_callback, this, std::placeholders::_1));

    this->motor_state_publisher_ = this->create_publisher<gh360_interfaces::msg::PortStatus>("motor_status", 10);
    this->timer_ = this->create_wall_timer(100ms, std::bind(&gh360::MotorHandler::timer_callback, this));

    this->position_step_service_ = this->create_service<gh360_interfaces::srv::MotorPositionStep>("motor_positions_step", std::bind(&gh360::MotorHandler::position_step_callback, this, std::placeholders::_1, std::placeholders::_2));

    //CODE THAT SHOULD BE IN PUBLISHER/SUBSCRIBER
    // this->readPresentPosition();
    
    // for (unsigned int i=0; i < this->joints.size(); i++)
    // {
    //     if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
    //     {
    //         // soft_joint->set_right_motor_goal_position(M_PI);
    //         // soft_joint->set_left_motor_goal_position(M_PI);
    //         soft_joint->set_right_motor_goal_position(0.0);
    //         soft_joint->set_left_motor_goal_position(0.0);
    //     }
    //     else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
    //     {
    //         // motor_joint->set_motor_goal_position(M_PI);
    //         motor_joint->set_motor_goal_position(0.0);
    //     }
    // }

    // // bool comm_result;
    // uint8_t address = this->joints_motor_model->Goal_Position.address;
    // uint8_t size = this->joints_motor_model->Goal_Position.size;
    // this->syncWrite(size, address);


    // this->readPresentPosition();



    // RCLCPP_INFO(this->get_logger(), "Joint Name: %s", this->joint_names[4].c_str());
    // RCLCPP_INFO(this->get_logger(), "Joint Type: %s", this->joint_type.c_str());
    // RCLCPP_INFO(this->get_logger(), "Right ID: %ld", this->right_id);
    // RCLCPP_INFO(this->get_logger(), "Left ID: %ld", this->left_id);

    // bool result = false;
    // result = dxl_wb.init(this->port_name, this->baud_rate, &this->log);
    // if (result == false)
    // {
    //     printf("%s\n", log);
    //     printf("Failed to init\n");

    //     return;
    // }
    // else
    //     printf("Succeed to init(%d)\n", baud_rate); 
}

gh360::MotorHandler::~MotorHandler()
{
}

void gh360::MotorHandler::position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response)
{

    this->setMotorGoalPositions(request->motor_goal_positions);

    // this->syncWrite(this->joints_motor_model->Goal_Position.size, this->joints_motor_model->Goal_Position.address);

    // this->syncRead(this->joints_motor_model->Present_Position.size, this->joints_motor_model->Present_Position.address);
    // this->syncRead(this->joints_motor_model->Present_Velocity.size, this->joints_motor_model->Present_Velocity.address);
    // this->syncRead(this->joints_motor_model->Present_Current.size, this->joints_motor_model->Present_Current.address);
    // this->syncRead(this->joints_motor_model->Present_Temperature.size, this->joints_motor_model->Present_Temperature.address);

    // gh360_interfaces::msg::PortStatus port_status_msg = gh360_interfaces::msg::PortStatus();
    gh360_interfaces::msg::MotorStatus motor_status_msg;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = soft_joint->get_right_motor_id();
            motor_status_msg.present_position = soft_joint->get_right_motor_present_position();
            motor_status_msg.present_velocity = soft_joint->get_right_motor_present_velocity();
            motor_status_msg.present_current = soft_joint->get_right_motor_present_current();
            motor_status_msg.present_temperature = soft_joint->get_right_motor_present_temperature();
            // port_status_msg.motors.push_back(motor_status_msg);
            response->motor_status.push_back(motor_status_msg);

            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = soft_joint->get_left_motor_id();
            motor_status_msg.present_position = soft_joint->get_left_motor_present_position();
            motor_status_msg.present_velocity = soft_joint->get_left_motor_present_velocity();
            motor_status_msg.present_current = soft_joint->get_left_motor_present_current();
            motor_status_msg.present_temperature = soft_joint->get_left_motor_present_temperature();
            // port_status_msg.motors.push_back(motor_status_msg);
            response->motor_status.push_back(motor_status_msg);
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = motor_joint->get_motor_id();
            motor_status_msg.present_position = motor_joint->get_motor_present_position();
            motor_status_msg.present_velocity = motor_joint->get_motor_present_velocity();
            motor_status_msg.present_current = motor_joint->get_motor_present_current();
            motor_status_msg.present_temperature = motor_joint->get_motor_present_temperature();
            // port_status_msg.motors.push_back(motor_status_msg);
            response->motor_status.push_back(motor_status_msg);
        }
    }

}

void gh360::MotorHandler::motor_goal_positions_callback(const gh360_interfaces::msg::SetMotorPositions::SharedPtr msg)
{
    // for (unsigned int m=0; m < msg->motor_goal_positions.size(); m++)
    // {
    //     for (unsigned int i=0; i < this->joints.size(); i++)
    //     {
    //         if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
    //         {
    //             if (soft_joint->get_right_motor_id() == msg->motor_goal_positions[m].id) 
    //             {
    //                 soft_joint->set_right_motor_goal_position(msg->motor_goal_positions[m].position);
    //             }
    //             else if (soft_joint->get_left_motor_id() == msg->motor_goal_positions[m].id)
    //             {
    //                 soft_joint->set_left_motor_goal_position(msg->motor_goal_positions[m].position);
    //             }
                
    //         }
    //         else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
    //         {
    //             if (motor_joint->get_motor_id() == msg->motor_goal_positions[m].id) 
    //             {
    //                 motor_joint->set_motor_goal_position(msg->motor_goal_positions[m].position);
    //             }
    //         }
    //     }
    // }

    this->setMotorGoalPositions(msg->motor_goal_positions);

    // uint8_t address = this->joints_motor_model->Goal_Position.address;
    // uint8_t size = this->joints_motor_model->Goal_Position.size;
    


    
}

void gh360::MotorHandler::timer_callback()
{
    // bool comm_result;

    // uint8_t address = this->joints_motor_model->Present_Position.address;
    // uint8_t size = this->joints_motor_model->Present_Position.size;
    // RCLCPP_INFO(this->get_logger(), "Address: %ld", address);
    // RCLCPP_INFO(this->get_logger(), "Size: %ld", size);
    this->syncRead(this->joints_motor_model->Present_Position.size, this->joints_motor_model->Present_Position.address);
    this->syncRead(this->joints_motor_model->Present_Velocity.size, this->joints_motor_model->Present_Velocity.address);
    this->syncRead(this->joints_motor_model->Present_Current.size, this->joints_motor_model->Present_Current.address);
    this->syncRead(this->joints_motor_model->Present_Temperature.size, this->joints_motor_model->Present_Temperature.address);
    this->syncWrite(this->joints_motor_model->Goal_Position.size, this->joints_motor_model->Goal_Position.address);

    gh360_interfaces::msg::PortStatus port_status_msg = gh360_interfaces::msg::PortStatus();
    gh360_interfaces::msg::MotorStatus motor_status_msg;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = soft_joint->get_right_motor_id();
            motor_status_msg.present_position = soft_joint->get_right_motor_present_position();
            motor_status_msg.present_velocity = soft_joint->get_right_motor_present_velocity();
            motor_status_msg.present_current = soft_joint->get_right_motor_present_current();
            motor_status_msg.present_temperature = soft_joint->get_right_motor_present_temperature();
            port_status_msg.motors.push_back(motor_status_msg);

            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = soft_joint->get_left_motor_id();
            motor_status_msg.present_position = soft_joint->get_left_motor_present_position();
            motor_status_msg.present_velocity = soft_joint->get_left_motor_present_velocity();
            motor_status_msg.present_current = soft_joint->get_left_motor_present_current();
            motor_status_msg.present_temperature = soft_joint->get_left_motor_present_temperature();
            port_status_msg.motors.push_back(motor_status_msg);
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = motor_joint->get_motor_id();
            motor_status_msg.present_position = motor_joint->get_motor_present_position();
            motor_status_msg.present_velocity = motor_joint->get_motor_present_velocity();
            motor_status_msg.present_current = motor_joint->get_motor_present_current();
            motor_status_msg.present_temperature = motor_joint->get_motor_present_temperature();
            port_status_msg.motors.push_back(motor_status_msg);
        }
    }

    // port_status_msg.motors.append();
    // if (!comm_result) RCLCPP_ERROR(this->get_logger(), "Address: %ld", address);

    this->motor_state_publisher_->publish(port_status_msg);
}

bool gh360::MotorHandler::setMotorGoalPositions(std::vector<gh360_interfaces::msg::SetPosition> motor_goal_positions)
{
    for (unsigned int m=0; m < motor_goal_positions.size(); m++)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() == motor_goal_positions[m].id) 
                {
                    soft_joint->set_right_motor_goal_position(motor_goal_positions[m].position);
                }
                else if (soft_joint->get_left_motor_id() == motor_goal_positions[m].id)
                {
                    soft_joint->set_left_motor_goal_position(motor_goal_positions[m].position);
                }
                
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                if (motor_joint->get_motor_id() == motor_goal_positions[m].id) 
                {
                    motor_joint->set_motor_goal_position(motor_goal_positions[m].position);
                }
            }
        }
    }
    return true;
}

bool gh360::MotorHandler::readPresentPosition()
{
    bool comm_result;
    if (!this->multi_motor_models)
    {
        uint8_t address = this->joints_motor_model->Present_Position.address;
        uint8_t size = this->joints_motor_model->Present_Position.size;
        // RCLCPP_INFO(this->get_logger(), "Address: %ld", address);
        // RCLCPP_INFO(this->get_logger(), "Size: %ld", size);
        comm_result = this->syncRead(size, address);
        if (!comm_result) return false;
    }

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            RCLCPP_INFO(this->get_logger(), "Current Right Position: %f", soft_joint->get_right_motor_present_position());
            RCLCPP_INFO(this->get_logger(), "Current Left Position: %f", soft_joint->get_left_motor_present_position());
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            RCLCPP_INFO(this->get_logger(), "Current Position: %f", motor_joint->get_motor_present_position());
        }
    }
    
    return true;
}

bool gh360::MotorHandler::syncRead(uint8_t size, uint8_t address)
{
    if (!this->multi_motor_models)
    {
        dynamixel::GroupSyncRead groupSyncRead(this->portHandler, this->packetHandler, address, size);
        
        bool dxl_addparam_result = false; 
        uint8_t motor_id;
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            // if (dynamic_cast<SoftJoint*>(this->joints[i]) != nullptr)
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                motor_id = soft_joint->get_right_motor_id();
                dxl_addparam_result = groupSyncRead.addParam(motor_id);
                if (dxl_addparam_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    return false;
                }

                motor_id = soft_joint->get_left_motor_id();
                dxl_addparam_result = groupSyncRead.addParam(motor_id);
                if (dxl_addparam_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    return false;
                }
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                motor_id = motor_joint->get_motor_id();
                dxl_addparam_result = groupSyncRead.addParam(motor_id);
                if (dxl_addparam_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    return false;
                }
            }
        }

        // Syncread present position
        int dxl_comm_result = groupSyncRead.txRxPacket();
        // if (dxl_comm_result != COMM_SUCCESS) this->packetHandler->printTxRxResult(dxl_comm_result);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to synread.");
            return false;
        } 


        bool dxl_getdata_result = false;
        // int32_t dxl1_present_position = 0, dxl2_present_position = 0;
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                motor_id = soft_joint->get_right_motor_id();
                dxl_getdata_result = groupSyncRead.isAvailable(motor_id, address, size);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->dxl_id[0]);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                    return false;
                }
                // soft_joint->set_right_motor_present_position(groupSyncRead.getData(motor_id, address, size));
                soft_joint->set_right_motor_status(groupSyncRead.getData(motor_id, address, size), address);

                motor_id = soft_joint->get_left_motor_id();
                dxl_getdata_result = groupSyncRead.isAvailable(motor_id, address, size);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->dxl_id[0]);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                    return false;
                }
                // soft_joint->set_left_motor_present_position(groupSyncRead.getData(motor_id, address, size));
                soft_joint->set_left_motor_status(groupSyncRead.getData(motor_id, address, size), address);
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                motor_id = motor_joint->get_motor_id();
                dxl_getdata_result = groupSyncRead.isAvailable(motor_id, address, size);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->dxl_id[0]);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                    return false;
                }
                // motor_joint->set_motor_present_position(groupSyncRead.getData(motor_id, address, size));
                motor_joint->set_motor_status(groupSyncRead.getData(motor_id, address, size), address);
            }
        }
        return true;
    }

    return false;
}

bool gh360::MotorHandler::syncWrite(uint8_t size, uint8_t address)
{
    if (!this->multi_motor_models)
    {
        dynamixel::GroupSyncWrite groupSyncWrite(this->portHandler, this->packetHandler, address, size);

        uint8_t param_goal_position[4];
        int motor_goal;
        uint8_t motor_id;
        bool comm_result;

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                motor_id = soft_joint->get_right_motor_id();
                motor_goal = soft_joint->get_right_motor_goal_int(address);
                // motor_goal_pos = soft_joint->get_right_motor_goal_position_int();

                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));

                comm_result = groupSyncWrite.addParam(motor_id, param_goal_position);
                if (comm_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    return false;
                }

                motor_id = soft_joint->get_left_motor_id();
                // motor_goal_pos = soft_joint->get_left_motor_goal_position_int();
                motor_goal = soft_joint->get_left_motor_goal_int(address);
                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));

                comm_result = groupSyncWrite.addParam(motor_id, param_goal_position);
                if (comm_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    return false;
                }
                
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                motor_id = motor_joint->get_motor_id();
                // motor_goal_pos = motor_joint->get_motor_goal_position_int();
                motor_goal = motor_joint->get_motor_goal_int(address);

                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));

                comm_result = groupSyncWrite.addParam(motor_id, param_goal_position);
                if (comm_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    return false;
                }
            }

        }

        // Syncwrite goal position
        comm_result = groupSyncWrite.txPacket();
        if (comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to sync write to motors.");
            return false;
        } 

        // Clear syncwrite parameter storage
        groupSyncWrite.clearParam();

        return true;
    }
    return false;
}

bool gh360::MotorHandler::openPortsAndSetBaudrate()
{
    this->portHandler = dynamixel::PortHandler::getPortHandler(this->port_name);
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    
    int dxl_comm_result;
    // uint8_t dxl_error;

    // Open Serial Port
    dxl_comm_result = this->portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to open the port!");
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = this->portHandler->setBaudRate(this->baud_rate);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to set the baudrate!");
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Succeeded to set the baudrate.");
    }

    return true;
}

bool gh360::MotorHandler::writeRegister(uint8_t id, int32_t data, uint8_t data_size, uint8_t address)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    switch(data_size) {
        case 1:
            dxl_comm_result = this->packetHandler->write1ByteTxRx(this->portHandler,
                id,
                address,
                data,
                &dxl_error
            );
            break;
        case 2:
            dxl_comm_result = this->packetHandler->write2ByteTxRx(this->portHandler,
                id,
                address,
                data,
                &dxl_error
            );
            break;
        case 4:
            dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler,
                id,
                address,
                data,
                &dxl_error
            );
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Invalid data size for writing to motors!");
    }

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to write to motor.");
        return false;
    } else {
        // RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Succeeded to write to motor.");
        return true;
    }
}

gh360::MotorDictionary* gh360::MotorHandler::getMotorModel(int motor_id)
{
    uint8_t dxl_error;
    uint16_t model_number = 0;
    const char * log;

    int dxl_comm_result = this->packetHandler->ping(this->portHandler, motor_id, &model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to get model number from id: %ld", motor_id);
        return NULL;
    }
    else if (dxl_error != 0)
    {
        log = this->packetHandler->getRxPacketError(dxl_error);
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Log Output: ", log);
        return NULL;
    }
    else {
        // RCLCPP_INFO(this->get_logger(), "Model Number: %ld", this->motor_test->Model_Number);
        // Model Number: 321 = MX-106; 311 = MX-64
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Model Number: %ld", model_number);
        if (model_number == 321)
        {
            int protocol = 2;
            return new MX_106_DICT(protocol);
            // gh360::MX_106_DICT* motor_dict = new MX_106_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
            // RCLCPP_INFO(this->get_logger(), "Test ", log);
        }
        else if (model_number == 320)
        {
            int protocol = 1;
            return new MX_106_DICT(protocol);
            // gh360::MX_106_DICT* motor_dict = new MX_106_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
        }
        else if (model_number == 311)
        {
            int protocol = 2;
            return new MX_64_DICT(protocol);
            // gh360::MX_64_DICT* motor_dict = new MX_64_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
        }
        else if (model_number == 310)
        {
            int protocol = 1;
            return new MX_64_DICT(protocol);
            // gh360::MX_64_DICT* motor_dict = new MX_64_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
        }
        // return true;
    }
    return NULL;

}

bool gh360::MotorHandler::setTorqueEnable(Joint* joint, int value) 
{
    bool comm_result = false;
    uint8_t data_size;
    uint8_t address;
    uint8_t motor_id;

    if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(joint))
    {
        data_size = soft_joint->get_right_motor_model()->Torque_Enable.size;
        address = soft_joint->get_right_motor_model()->Torque_Enable.address;
        motor_id = soft_joint->get_right_motor_id();
        comm_result = this->writeRegister(motor_id, value, data_size, address);

        if (comm_result) 
        {
            // SoftJoint * soft_joint = joint;
            data_size = soft_joint->get_left_motor_model()->Torque_Enable.size;
            address = soft_joint->get_left_motor_model()->Torque_Enable.address;
            motor_id = soft_joint->get_left_motor_id();
            comm_result = this->writeRegister(motor_id, value, data_size, address);
        }
    }
    else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(joint))
    {
        data_size = motor_joint->get_motor_model()->Torque_Enable.size;
        address = motor_joint->get_motor_model()->Torque_Enable.address;
        motor_id = motor_joint->get_motor_id();
        comm_result = this->writeRegister(motor_id, value, data_size, address);
        
    }

    if (comm_result)
    {
        if (value == 0) RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Torque on %s successfully turned off", joint->get_joint_name().c_str());
        else RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Torque on %s successfully turned on", joint->get_joint_name().c_str());
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Changing operation mode on motor %s failed!", joint->get_joint_name().c_str());
        return false;
    }

    
}

bool gh360::MotorHandler::setOperatingMode(Joint* joint, int value)
{
    bool valid_value = false;
    std::string msg;
    switch(value) {
        case 0:
            msg = "Current Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 1:
            msg = "Velocity Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 3:
            msg = "Position Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 4:
            msg = "Extended Position Control on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 5:
            msg = "Current-based Position Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 16:
            msg = "PWM Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        default:
            valid_value = false;
            RCLCPP_ERROR(this->get_logger(), "%d is not a valid operating mode", value);
    }

    

    
    if (valid_value)
    {
        bool comm_result = false;
        uint8_t data_size;
        uint8_t address;
        uint8_t motor_id;
        // if (joint->get_joint_type() == "soft_joint")
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(joint))
        {
            // SoftJoint * soft_joint = joint;
            data_size = soft_joint->get_right_motor_model()->Operating_Mode.size;
            address = soft_joint->get_right_motor_model()->Operating_Mode.address;
            motor_id = soft_joint->get_right_motor_id();
            comm_result = this->writeRegister(motor_id, value, data_size, address);

            if (comm_result) 
            {
                // SoftJoint * soft_joint = joint;
                data_size = soft_joint->get_left_motor_model()->Operating_Mode.size;
                address = soft_joint->get_left_motor_model()->Operating_Mode.address;
                motor_id = soft_joint->get_left_motor_id();
                comm_result = this->writeRegister(motor_id, value, data_size, address);
            }

        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(joint))
        {
            // MotorJoint* motor_joint = joint;
            data_size = motor_joint->get_motor_model()->Operating_Mode.size;
            address = motor_joint->get_motor_model()->Operating_Mode.address;
            motor_id = motor_joint->get_motor_id();
            comm_result = this->writeRegister(motor_id, value, data_size, address);
        }
        // uint8_t data_size = this->motor_dicts[motor_index]->Operating_Mode.size;
        // uint8_t address = this->motor_dicts[motor_index]->Operating_Mode.address;
        
        if (comm_result)
        {
            RCLCPP_INFO(this->get_logger(), msg);
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Changing operation mode on motor %d failed!", motor_id);
            return false;
        }
    }
    return false;
}



// void setupDynamixel(uint8_t dxl_id)
// {
//   // Use Position Control Mode
//   dxl_comm_result = packetHandler->write1ByteTxRx(
//     portHandler,
//     dxl_id,
//     ADDR_OPERATING_MODE,
//     3,
//     &dxl_error
//   );

//   if (dxl_comm_result != COMM_SUCCESS) {
//     RCLCPP_ERROR(rclcpp::get_logger("motor_handler_node"), "Failed to set Position Control Mode.");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("motor_handler_node"), "Succeeded to set Position Control Mode.");
//   }

//   // Enable Torque of DYNAMIXEL
//   dxl_comm_result = packetHandler->write1ByteTxRx(
//     portHandler,
//     dxl_id,
//     ADDR_TORQUE_ENABLE,
//     1,
//     &dxl_error
//   );

//   if (dxl_comm_result != COMM_SUCCESS) {
//     RCLCPP_ERROR(rclcpp::get_logger("motor_handler_node"), "Failed to enable torque.");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("motor_handler_node"), "Succeeded to enable torque.");
//   }
// }

int main(int argc, char * argv[])
{
    // portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    // packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // dxl_comm_result = portHandler->openPort();
    // if (dxl_comm_result == false) {
    //     RCLCPP_ERROR(rclcpp::get_logger("motor_handler_node"), "Failed to open the port!");
    //     return -1;
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("motor_handler_node"), "Succeeded to open the port.");
    // }

    // // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    // dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    // if (dxl_comm_result == false) {
    //     RCLCPP_ERROR(rclcpp::get_logger("motor_handler_node"), "Failed to set the baudrate!");
    //     return -1;
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("motor_handler_node"), "Succeeded to set the baudrate.");
    // }

    // setupDynamixel(BROADCAST_ID);

    rclcpp::init(argc, argv);
    
    // const char* port_name = "/dev/ttyUSB0";
    // int baud_rate = 1000000;
    // int protocol = 2;
    auto motorhandlernode = std::make_shared<gh360::MotorHandler>();
    rclcpp::spin(motorhandlernode);
    rclcpp::shutdown();

    return 0;
}
#include "gh360/util/dynamixel.hpp"

// #include "dynamixel_util.cpp"
// #include "joint_types/soft_joint.hpp"
// #include "joint_types/motor_joint.hpp"

DynamixelHandler::DynamixelHandler(const char* port_name, int baud_rate)
{
    this->port_name = port_name;
    this->baud_rate = baud_rate;

    this->openPortsAndSetBaudrate();
}

DynamixelHandler::~DynamixelHandler()
{

}

bool DynamixelHandler::openPortsAndSetBaudrate()
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



bool DynamixelHandler::setOperatingMode(Joint* joint, int value)
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
            RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "%d is not a valid operating mode", value);
    }
    
    if (valid_value)
    {
        bool comm_result = true;
        for (int i=0; i<joint->get_motor_cnt(); i++) 
        {
            Motor * motor = joint->get_motor(i);
            comm_result = this->writeRegister(motor->get_motor_id(), motor->get_motor_model()->Operating_Mode, value);
            if (!comm_result) break;
        }
        
        if (comm_result)
        {
            joint->set_operating_mode(value);
            RCLCPP_INFO(rclcpp::get_logger("motor_handler"), msg);
            return true;
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Changing operation mode failed!");
            return false;
        }
    }
    return false;
}

bool DynamixelHandler::setControlMode(std::vector<Joint*> joints, std::function<int(Joint*)> get_mode_id)
{
    if (joints[0]->get_operating_mode() == get_mode_id(joints[0])) return true;

    for (unsigned int i=0; i < joints.size(); i++)
    {
        this->setTorqueEnable(joints[i],0);
        this->setOperatingMode(joints[i],get_mode_id(joints[i]));
        this->setTorqueEnable(joints[i],1);
    }

    return true;
}

int DynamixelHandler::getPositionModeID(Joint* joint)
{
    return joint->get_position_mode_id();
}

int DynamixelHandler::getVelocityModeID(Joint* joint)
{
    return joint->get_velocity_mode_id();
}

int DynamixelHandler::getCurrentModeID(Joint* joint)
{
    return joint->get_current_mode_id();
}


bool DynamixelHandler::setTorqueEnable(Joint* joint, int value)
{
    if (this->emergency_stop && value == 1) return false;

    bool comm_result = true;
    bool register_written = false;
    for (int i=0; i<joint->get_motor_cnt(); i++) 
    {
        Motor * motor = joint->get_motor(i);
        if (motor->get_torque_enabled() != value)
        {
            comm_result = this->writeRegister(motor->get_motor_id(), motor->get_motor_model()->Torque_Enable, value);
            register_written = true;
        }
        if (!comm_result) break;
        motor->set_torque_enabled(value);
    }
    
    if (comm_result && register_written)
    {
        if (value == 0) RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Torque on %s successfully turned off", joint->get_joint_name().c_str());
        else RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Torque on %s successfully turned on", joint->get_joint_name().c_str());
        return true;
    }
    else if (register_written && !comm_result)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Changing operation mode failed!");
        return false;
    }
    return true;
}

bool DynamixelHandler::setVelocityProfile(Joint* joint, double value)
{
    int int_value = int(value / 0.229);
    bool comm_result = true;
    
    for (int i=0; i<joint->get_motor_cnt(); i++) 
    {
        Motor * motor = joint->get_motor(i);
        comm_result = this->writeRegister(motor->get_motor_id(), motor->get_motor_model()->Profile_Velocity, int_value);
        if (!comm_result) return false;
    }
    
    return true;
}

bool DynamixelHandler::setAccelerationProfile(Joint* joint, double value)
{
    int int_value = int(value / 214.577);
    bool comm_result = true;
    
    for (int i=0; i<joint->get_motor_cnt(); i++) 
    {
        Motor * motor = joint->get_motor(i);
        comm_result = this->writeRegister(motor->get_motor_id(), motor->get_motor_model()->Profile_Acceleration, int_value);
        if (!comm_result) return false;
    }
    
    return true;
}

MotorDictionary* DynamixelHandler::getMotorModel(int motor_id)
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

bool DynamixelHandler::writeRegister(uint8_t id, int32_t data, uint8_t data_size, uint8_t address)
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

bool DynamixelHandler::writeRegister(uint8_t motor_id, MotorDictionary::motor_dict motor_dict, int32_t data)
{
    uint8_t data_size = motor_dict.size;
    uint8_t address = motor_dict.address;
    bool comm_result = this->writeRegister(motor_id, data, data_size, address);

    return comm_result;
}

bool DynamixelHandler::syncRead(std::vector<Joint*> joints, MotorDictionary::motor_dict motor_dict)
{
    dynamixel::GroupSyncRead groupSyncRead(this->portHandler, this->packetHandler, motor_dict.address, motor_dict.size);
    
    bool dxl_addparam_result = false; 
    uint8_t motor_id;
    for (unsigned int i=0; i < joints.size(); i++)
    {
        for (int j=0; j<joints[i]->get_motor_cnt(); j++)
        {
            motor_id = joints[i]->get_motor(j)->get_motor_id();
            dxl_addparam_result = groupSyncRead.addParam(motor_id);
            if (dxl_addparam_result != true)
            {
                RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                return false;
            }
        }
    }

    int dxl_comm_result = groupSyncRead.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to synread.");
        return false;
    } 

    bool dxl_getdata_result = false;
    for (unsigned int i=0; i < joints.size(); i++)
    {
        for (int j=0; j<joints[i]->get_motor_cnt(); j++)
        {
            motor_id = joints[i]->get_motor(j)->get_motor_id();
            dxl_getdata_result = groupSyncRead.isAvailable(motor_id, motor_dict.address, motor_dict.size);
            if (dxl_getdata_result != true)
            {
                RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                return false;
            }
            joints[i]->get_motor(j)->set_motor_state(groupSyncRead.getData(motor_id, motor_dict.address, motor_dict.size), motor_dict.address);
        }
    }
    return true;
}

bool DynamixelHandler::syncWrite(std::vector<Joint*> joints, MotorDictionary::motor_dict motor_dict)
{
    dynamixel::GroupSyncWrite groupSyncWrite(this->portHandler, this->packetHandler, motor_dict.address, motor_dict.size);

    uint8_t param_motor_goal[4];
    int motor_goal;
    uint8_t motor_id;
    bool comm_result;

    for (unsigned int i=0; i < joints.size(); i++)
    {
        for (int j=0; j<joints[i]->get_motor_cnt(); j++)
        {
            motor_id = joints[i]->get_motor(j)->get_motor_id();
            motor_goal = joints[i]->get_motor(j)->get_motor_goal_int(motor_dict.address);

            if (motor_dict.size == 2)
            {
                param_motor_goal[0] = DXL_LOBYTE(motor_goal);
                param_motor_goal[1] = DXL_HIBYTE(motor_goal);
                param_motor_goal[2] = 0;
                param_motor_goal[3] = 0;
            }
            else if (motor_dict.size == 4)
            {
                param_motor_goal[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_motor_goal[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_motor_goal[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_motor_goal[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));
            }

            comm_result = groupSyncWrite.addParam(motor_id, param_motor_goal);
            if (comm_result != true)
            {
                RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                return false;
            }
        }

    }

    // Syncwrite goal position
    comm_result = groupSyncWrite.txPacket();
    if (comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to sync write to motors.");
        return false;
    } 

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();

    return true;
}

void DynamixelHandler::setEmergencyStop(bool value)
{
    this->emergency_stop = value;
}

bool DynamixelHandler::getEmergencyStop()
{
    return this->emergency_stop;
}

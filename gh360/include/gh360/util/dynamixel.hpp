#ifndef DYNAMIXEL_HANDLER_HPP_
#define DYNAMIXEL_HANDLER_HPP_

#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "gh360/joint_types/joint.hpp"
#include "gh360/motor_dictionaries/mx_106_dict.hpp"
#include "gh360/motor_dictionaries/mx_64_dict.hpp"

class DynamixelHandler
{
    public:
        DynamixelHandler(const char* port_name, int baud_rate);
        virtual ~DynamixelHandler();

        /**
         * @brief Open the serial port and set the baudrate
         * @return True if the port was opened and the baudrate was set successfully
         */
        bool openPortsAndSetBaudrate();    

        /**
         * @brief Set the operating mode of the motor
         * @param joint The joint object
         * @param value The operating mode value
         * @return True if the operating mode was set successfully
         */
        bool setOperatingMode(Joint* joint, int value);

        /**
         * @brief Sets the control mode of all motors on the port
         * @param joints The vector of joint objects
         * @param get_mode_id A function to get the operation mode id of each joint
         * @return True if the control mode was set successfully
         */
        bool setControlMode(std::vector<Joint*> joints, std::function <int(Joint*)> get_mode_id);

        /**
         * @brief Returns the id of the position mode of a given joint
         * @param joint The joint object
         * @return The position mode id
         */
        int getPositionModeID(Joint* joint);

        /**
         * @brief Returns the id of the velocity mode of a given joint
         * @param joint The joint object
         * @return The velocity mode id
         */
        int getVelocityModeID(Joint* joint);

        /**
         * @brief Returns the id of the current operating mode of a given joint
         * @param joint The joint object
         * @return The current mode id
         */
        int getCurrentModeID(Joint* joint);

        /**
         * @brief Enable/Disable the torque of the joint motors
         * @param joint The joint object
         * @param value The torque enable value (1 or 0)
         * @return True if the value was set successfully
         */
        bool setTorqueEnable(Joint* joint, int value);

        /**
         * @brief Set the maximum velocity of the motors
         * @param joint The joint object
         * @param value The maximum velocity value in rad/s
         * @return True if the value was set successfully
         */
        bool setVelocityProfile(Joint* joint, double value);

        /**
         * @brief Set the maximum acceleration of the motors
         * @param joint The joint object
         * @param value The maximum acceleration value in rad/s^2
         * @return True if the value was set successfully
         */
        bool setAccelerationProfile(Joint* joint, double value);

        /**
         * @brief Returns the motor dictionary given the id of the motor
         * @param motor_id The id of the motor
         * @return The motor dictionary
         */
        MotorDictionary* getMotorModel(int motor_id);

        /**
         * @brief Write data to a register of the motor
         * @param id The id of the motor
         * @param data The data to be written to the register
         * @param data_size The size of the data in bits to be written
         * @param address The address of the register
         * @return True if the data was written successfully
         */
        bool writeRegister(uint8_t id, int32_t data, uint8_t data_size, uint8_t address);

        /**
         * @brief Write data to a register of the motor
         * @param motor_id The id of the motor
         * @param motor_dict A motor dictionary object
         * @param data The data to be written to the register
         * @return True if the data was written successfully
         */
        bool writeRegister(uint8_t motor_id, MotorDictionary::motor_dict motor_dict, int32_t data);

        /**
         * @brief Read data from a given address of all motors on the port
         * @param joints The vector of joint objects
         * @param motor_dict A motor dictionary object for the address and size of the data to be read
         * @return True if the data was read successfully
         */
        bool syncRead(std::vector<Joint*> joints, MotorDictionary::motor_dict motor_dict);

        /**
         * @brief Write data to a given address on all motors on the port
         * @param joints The vector of joint objects
         * @param motor_dict A motor dictionary object for the address and size of the data to be written
         * @return True if the data was written successfully
         */
        bool syncWrite(std::vector<Joint*> joints, MotorDictionary::motor_dict motor_dict);

        /**
         * @brief Set the emergency stop state. If this is true the torque on the motors can't be enabled
         * @param value The emergency stop state
         */
        void setEmergencyStop(bool value);

        /**
         * @brief Get the emergency stop state
         * @return The emergency stop state
         */
        bool getEmergencyStop();

    private:
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;
        const char* port_name;
        int baud_rate;
        bool emergency_stop;

    
};

#endif // DYNAMIXEL_HANDLER_HPP_
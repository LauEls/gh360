#ifndef MOTOR_HPP_
#define MOTOR_HPP_

#include <iostream>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "gh360_interfaces/msg/set_position.hpp"
#include "gh360_interfaces/msg/set_velocity.hpp"
#include "gh360_interfaces/msg/set_current.hpp"

#include "gh360/motor_dictionaries/motor_dict.hpp"

class Motor
{
public:
    Motor();
    virtual ~Motor();

    /**
     * @brief Set the motor id
     * @param motor_id The motor id
     */
    void set_motor_id(int motor_id);

    /**
     * @brief Set the movement direction of the motor
     * @param movement_direction The movement direction of the motor (1 or -1)
     */
    void set_movement_direction(int movement_direction);

    /**
     * @brief Set the offset of the motor
     * @param offset The offset of the motor in radians
     */
    void set_offset(double offset);

    /**
     * @brief Set the motor model
     * @param motor_model The motor model type
     */
    void set_motor_model(MotorDictionary *motor_model);

    /**
     * @brief Enable or disable the motor torque
     * @param torque The torque state (true or false)
     */
    void set_torque_enabled(bool torque);

    /**
     * @brief Set the safety check state
     * @param safety_check The safety check state (true or false)
     */
    void set_safety_check(bool safety_check);

    /**
     * @brief Set the motor state
     * @param data The state value
     * @param address The address to identify what state is being set. The address value is defined in the motor dictionary
     */
    void set_motor_state(int data, uint8_t address);

    /**
     * @brief Set the motor goal. The goal type is defined by the current operating mode.
     * @param data The goal value adjusted with offset and movement direction.
     */
    void set_motor_goal_adjusted(double data);

    /**
     * @brief Set the motor goal. The goal type is defined by the address parameter.
     * @param data The goal value adjusted with offset and movement direction.
     * @param address The address to identify what goal is being set. The address value is defined in the motor dictionary
     */
    void set_motor_goal_adjusted(double data, uint8_t address);

    /**
     * @brief Set a motor position goal.
     * @param motor_goal_msg Position goal message
     */
    void set_motor_goal_adjusted(gh360_interfaces::msg::SetPosition motor_goal_msg);

    /**
     * @brief Set the motor velocity goal.
     * @param motor_goal_msg Velocity goal message
     */
    void set_motor_goal_adjusted(gh360_interfaces::msg::SetVelocity motor_goal_msg);

    /**
     * @brief Set a motor current goal.
     * @param motor_goal_msg Current goal message
     */
    void set_motor_goal_adjusted(gh360_interfaces::msg::SetCurrent motor_goal_msg);

    /**
     * @brief Tranforms the given position in integer to double in radians and assigns it to present_position.
     * @param position The present position in integer as given by the motor.
     */
    void set_present_position(int position);

    /**
     * @brief Sets the present position.
     * @param position The present position as double adjusted with offset and movement direction.
     */
    void set_present_position_adjusted(double position);

    /**
     * @brief Tranforms the given velocity in integer to double in rad/s and assigns it to present_velocity.
     * @param velocity The present velocity in integer as given by the motor.
     */
    void set_present_velocity(int velocity);

    /**
     * @brief Sets the present velocity.
     * @param velocity The present velocity as double adjusted with the movement direction.
     */
    void set_present_velocity_adjusted(double velocity);

    /**
     * @brief Tranforms the given current in integer to double in mA and assigns it to present_current.
     * @param current The present current in integer as given by the motor.
     */
    void set_present_current(int current);

    /**
     * @brief Sets the present current.
     * @param current The present current as double adjusted with the movement direction.
     */
    void set_present_current_adjusted(double current);

    /**
     * @brief Assigns the given temperature to present_temperature.
     * @param temperature The present temperature in Celsius.
     */
    void set_present_temperature(int temperature);

    /**
     * @brief Set the moving state of the motor
     * @param moving The moving state (true or false)
     */
    void set_moving(bool moving);

    /**
     * @brief Set the goal position of the motor
     * @param goal_pos The goal position in radians
     */
    void set_goal_position(double goal_pos);

    /**
     * Set the goal position of the motor adjusted with the offset and movement direction.
     * @param goal_pos The goal position in radians
     */
    void set_goal_position_adjusted(double goal_pos);

    /**
     * Set the reference position of the motor. It is used during the initialization of the motor to have an alternative position to go to if the goal position is not suitable.
     * @param reference_pos The reference position in radians
     */
    void set_reference_position(double reference_pos);

    /**
     * Set the reference position of the motor adjusted with the offset and movement direction.
     * @param reference_pos The reference position in radians
     */
    void set_reference_position_adjusted(double reference_pos);

    /**
     * @brief Set the goal velocity of the motor
     * @param goal_vel The goal velocity in rad/s
     */
    void set_goal_velocity(double goal_vel);

    /**
     * Set the goal velocity of the motor adjusted with the movement direction.
     * @param goal_vel The goal velocity in rad/s
     */
    void set_goal_velocity_adjusted(double goal_vel);

    /**
     * @brief Set the goal current of the motor
     * @param goal_current The goal current in mA
     */
    void set_goal_current(double goal_current);

    /**
     * Set the goal current of the motor adjusted with the movement direction.
     * @param goal_current The goal current in mA
     */
    void set_goal_current_adjusted(double goal_current);

    /**
     * Set the reference current of the motor. It is used in the initialization of the motor to compare with the present current.
     * @param ref_current The reference current in mA
     */
    void set_reference_current(double ref_current);

    /**
     * Set the reference current of the motor adjusted with the movement direction.
     * @param ref_current The reference current in mA
     */
    void set_reference_current_adjusted(double ref_current);

    /**
     * @brief Set the operating mode of the motor
     * @param operating_mode The operating mode of the motor as an integer (from the motor dictionary)
     */
    void set_operating_mode(int operating_mode);

    /**
     * @brief Get the motor id
     * @return The motor id
     */
    int get_motor_id();

    /**
     * @brief Get the movement direction of the motor
     * @return The movement direction of the motor (1 or -1)
     */
    int get_movement_direction();

    /**
     * @brief Get the offset of the motor
     * @return The offset of the motor in radians
     */
    double get_offset();

    /**
     * @brief Get the motor model
     * @return The motor model type
     */
    MotorDictionary *get_motor_model();

    /**
     * @brief Query if the torque is enabled
     * @return The torque state (true or false)
     */
    bool get_torque_enabled();

    /**
     * @brief Query if the safety check has been passed.
     * @return The safety check state (true or false)
     */
    bool get_safety_check();

    /**
     * @brief Get the motor state
     * @param address The address to identify what state is being queried. The address value is defined in the motor dictionary
     * @return The state value
     */
    double get_motor_state(uint8_t address);

    /**
     * @brief Get the motor state of the given address adjusted with the offset and movement direction.
     * @param address The address to identify what goal is being queried. The address value is defined in the motor dictionary
     * @return The goal value as a double.
     */
    double get_motor_state_adjusted(uint8_t address);

    /**
     * @brief Get the motor goal of the current operating mode.
     * @return The goal value as a double.
     */
    double get_motor_goal();
    
    /**
     * @brief Get the motor goal of the given address.
     * @param address The address to identify what goal is being queried. The address value is defined in the motor dictionary
     * @return The goal value as a double.
     */
    double get_motor_goal(uint8_t address);

     /**
     * @brief Transforms the double value of the goal into integer and returns it. The type of goal is defined by the current operating mode.
     * @return The goal value as an integer
     */
    int get_motor_goal_int();

    /**
     * @brief Transforms the double value of the goal into integer and returns it. What type of goal is defined by the address parameter.
     * @param address The address to identify what goal is being queried. The address value is defined in the motor dictionary
     * @return The goal value as an integer
     */
    int get_motor_goal_int(uint8_t address);

    /**
     * @brief Get the motor goal of the given address adjusted with the offset and movement direction. The type of goal is defined by the current operating mode.
     * @return The goal value as a double.
     */
    double get_motor_goal_adjusted();


    /**
     * @brief Get the motor goal of the given address adjusted with the offset and movement direction.
     * @param address The address to identify what goal is being queried. The address value is defined in the motor dictionary
     * @return The goal value as a double.
     */
    double get_motor_goal_adjusted(uint8_t address);

    /**
     * @brief Get the present position of the motor
     * @return The present position in radians
     */
    double get_present_position();

    /**
     * @brief Get the present position of the motor adjusted with the offset and movement direction
     * @return The present position in radians
     */
    double get_present_position_adjusted();

    /**
     * @brief Get the present velocity of the motor
     * @return The present velocity in rad/s
     */
    double get_present_velocity();

    /**
     * @brief Get the present velocity of the motor adjusted with the movement direction
     * @return The present velocity in rad/s
     */
    double get_present_velocity_adjusted();

    /**
     * @brief Get the present current of the motor
     * @return The present current in mA
     */
    double get_present_current();

    /**
     * @brief Get the present current of the motor adjusted with the movement direction
     * @return The present current in mA
     */
    double get_present_current_adjusted();

    /**
     * @brief Get the present temperature of the motor
     * @return The present temperature in Celsius
     */
    double get_present_temperature();

    /**
     * @brief Get the moving state of the motor
     * @return The moving state (true or false)
     */
    bool get_moving();

    /**
     * @brief Get the goal position of the motor
     * @return The goal position in radians
     */
    double get_goal_position();

    /**
     * @brief Get the goal position of the motor adjusted with the offset and movement direction
     * @return The goal position in radians
     */
    double get_goal_position_adjusted();

    /**
     * @brief Get the reference position of the motor
     * @return The reference position in radians
     */
    double get_reference_position();

    /**
     * @brief Get the reference position of the motor adjusted with the offset and movement direction
     * @return The reference position in radians
     */
    double get_reference_position_adjusted();

    /**
     * @brief Get the goal velocity of the motor
     * @return The goal velocity in rad/s
     */
    double get_goal_velocity();

    /**
     * @brief Get the goal velocity of the motor adjusted with the movement direction
     * @return The goal velocity in rad/s
     */
    double get_goal_velocity_adjusted();

    /**
     * @brief Get the goal current of the motor
     * @return The goal current in mA
     */
    double get_goal_current();

    /**
     * @brief Get the goal current of the motor adjusted with the movement direction
     * @return The goal current in mA
     */
    double get_goal_current_adjusted();

    /**
     * @brief Get the reference current of the motor
     * @return The reference current in mA
     */
    double get_reference_current();

    /**
     * @brief Get the reference current of the motor adjusted with the movement direction
     * @return The reference current in mA
     */
    double get_reference_current_adjusted();

    /**
     * @return Returns true if the motor has reached the goal position
     */
    bool goal_position_reached();

    /**
     * @return Returns the operating mode of the motor
     */
    int get_operating_mode();


private:
    int motor_id;
    // Movement direction = 1 or -1
    int movement_direction;
    // Offset in rad
    double offset;
    MotorDictionary *motor_model;

    // Motor Position in rad
    double present_position;
    double goal_position;
    double reference_position;
    // Motor Velocity in rad/s
    double present_velocity;
    double goal_velocity;
    // Motor Current in mA
    double present_current;
    double goal_current;
    double reference_current;
    // Motor Temperature in Celsius
    double present_temperature;

    bool moving;
    bool safety_check;
    bool first_pos_value = true;
    bool torque_enabled = false;
    int operating_mode;

    /**
     * @brief Transform motor position from int value (given by the motor) to double value (in radians)
     * @param data The motor position as an integer
     * @return The motor position in radians
     */
    double positionIntToDouble(int data);

    /**
     * @brief Transform motor velocity from int value (given by the motor) to double value (in rad/s)
     * @param data The motor velocity as an integer
     * @return The motor velocity in rad/s
     */
    double velocityIntToDouble(int data);

    /**
     * @brief Transform motor current from int value (given by the motor) to double value (in mA)
     * @param data The motor current as an integer
     * @return The motor current in mA
     */
    double currentIntToDouble(int data);

    /**
     * @brief Transform motor position from double value (in radians) to int value (to be sent to the motor)
     * @param value The motor position in radians
     * @return The motor position as an integer
     */
    int positionDoubleToInt(double value);

    /**
     * @brief Transform motor velocity from double value (in rad/s) to int value (to be sent to the motor)
     * @param value The motor velocity in rad/s
     * @return The motor velocity as an integer
     */
    int velocityDoubleToInt(double value);

    /**
     * @brief Transform motor current from double value (in mA) to int value (to be sent to the motor)
     * @param value The motor current in mA
     * @return The motor current as an integer
     */
    int currentDoubleToInt(double value);
};

#endif // MOTOR_HPP_
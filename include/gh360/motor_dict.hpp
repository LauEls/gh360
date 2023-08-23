#ifndef MOTOR_DICTIONARY_HPP_
#define MOTOR_DICTIONARY_HPP_

namespace gh360
{
    class MotorDictionary
    {
        public:
            
            virtual ~MotorDictionary(){}

            struct motor_dict
            {
                int address;
                int size;
            };

            //Control Table of EEPROM Area
            struct motor_dict Model_Number;
            struct motor_dict Model_Information;
            struct motor_dict Firmware_Version;
            struct motor_dict ID;
            struct motor_dict Baud_Rate;
            struct motor_dict Return_Delay_Time;
            struct motor_dict Drive_Mode;
            struct motor_dict Operating_Mode;
            struct motor_dict Secondary_ID;
            struct motor_dict Protocol_Type;
            struct motor_dict Homing_Offset;
            struct motor_dict Moving_Threshold;
            struct motor_dict Temperature_Limit;
            struct motor_dict Max_Voltage_Limit;
            struct motor_dict Min_Voltage_Limit;
            struct motor_dict PWM_Limit;
            struct motor_dict Current_Limit;
            struct motor_dict Acceleration_Limit;
            struct motor_dict Velocity_Limit;
            struct motor_dict Max_Position_Limit;
            struct motor_dict Min_Position_Limit;
            struct motor_dict Shutdown;
            struct motor_dict CW_Angle_Limit;
            struct motor_dict CCW_Angle_Limit;
            struct motor_dict Max_Torque;
            struct motor_dict Alarm_LED;
            struct motor_dict Multi_Turn_Offset;
            struct motor_dict Resolution_Divider;

            //Control Table of RAM Area
            struct motor_dict Torque_Enable;
            struct motor_dict LED;
            struct motor_dict Status_Return_Level;
            struct motor_dict Registered_Instruction;
            struct motor_dict Hardware_Error_Status;
            struct motor_dict Velocity_I_Gain;
            struct motor_dict Velocity_P_Gain;
            struct motor_dict Position_D_Gain;
            struct motor_dict Position_I_Gain;
            struct motor_dict Position_P_Gain;
            struct motor_dict Feedforward_2nd_Gain;
            struct motor_dict Feedforward_1st_Gain;
            struct motor_dict BUS_Watchdog;
            struct motor_dict Goal_PWM;
            struct motor_dict Goal_Current;
            struct motor_dict Goal_Velocity;
            struct motor_dict Profile_Acceleration;
            struct motor_dict Profile_Velocity;
            struct motor_dict Goal_Position;
            struct motor_dict Realtime_Tick;
            struct motor_dict Moving;
            struct motor_dict Moving_Status;
            struct motor_dict Present_PWM;
            struct motor_dict Present_Current;
            struct motor_dict Present_Velocity;
            struct motor_dict Present_Position;
            struct motor_dict Velocity_Trajectory;
            struct motor_dict Position_Trajectory;
            struct motor_dict Present_Input_Voltage;
            struct motor_dict Present_Temperature;
            struct motor_dict D_Gain;
            struct motor_dict I_Gain;
            struct motor_dict P_Gain;
            struct motor_dict Moving_Speed;
            struct motor_dict Torque_Limit;
            struct motor_dict Present_Speed;
            struct motor_dict Present_Load;
            struct motor_dict Present_Voltage;
            struct motor_dict Registered;
            struct motor_dict Lock;
            struct motor_dict Punch;
            struct motor_dict Current;
            struct motor_dict Torque_Ctrl_Mode_Enable;
            struct motor_dict Goal_Torque;
            struct motor_dict Goal_Acceleration;

            


        protected:
            MotorDictionary(){}


    };
}

#endif // MOTOR_DICTIONARY_HPP_
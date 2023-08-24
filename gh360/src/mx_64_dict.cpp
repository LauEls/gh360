// #include "../include/gh360/mx_64_dict.hpp"
#include "mx_64_dict.hpp"

gh360::MX_64_DICT::MX_64_DICT(int protocol)
{
    if (protocol == 1)
    {
        //Control Table of EEPROM Area
        this->Model_Number.address = 0;
        this->Model_Number.size = 2;
        this->Firmware_Version.address = 2;
        this->Firmware_Version.size = 1;
        this->ID.address = 3;
        this->ID.size = 1;
        this->Baud_Rate.address = 4;
        this->Baud_Rate.size = 1;
        this->Return_Delay_Time.address = 5;
        this->Return_Delay_Time.size = 1;
        this->CW_Angle_Limit.address = 6;
        this->CW_Angle_Limit.size = 2;
        this->CCW_Angle_Limit.address = 8;
        this->CCW_Angle_Limit.size = 2;
        this->Temperature_Limit.address = 11;
        this->Temperature_Limit.size = 1;
        this->Min_Voltage_Limit.address = 12;
        this->Min_Voltage_Limit.size = 1;
        this->Max_Voltage_Limit.address = 13;
        this->Max_Voltage_Limit.size = 1;
        this->Max_Torque.address = 14;
        this->Max_Torque.size = 2;
        this->Status_Return_Level.address = 16;
        this->Status_Return_Level.size = 1;
        this->Alarm_LED.address = 17;
        this->Alarm_LED.size = 1;
        this->Shutdown.address = 18;
        this->Shutdown.size = 1;
        this->Multi_Turn_Offset.address = 20;
        this->Multi_Turn_Offset.size = 2;
        this->Resolution_Divider.address = 22;
        this->Resolution_Divider.size = 1;

        //Control Table of RAM Area
        this->Torque_Enable.address = 24;
        this->Torque_Enable.size = 1;
        this->LED.address = 25;
        this->LED.size = 1;
        this->D_Gain.address = 26;
        this->D_Gain.size = 1;
        this->I_Gain.address = 27;
        this->I_Gain.size = 1;
        this->P_Gain.address = 28;
        this->P_Gain.size = 1;
        this->Goal_Position.address = 30;
        this->Goal_Position.size = 2;
        this->Moving_Speed.address = 32;
        this->Moving_Speed.size = 2;
        this->Torque_Limit.address = 34;
        this->Torque_Limit.size = 2;
        this->Present_Position.address = 36;
        this->Present_Position.size = 2;
        this->Present_Speed.address = 38;
        this->Present_Speed.size = 2;
        this->Present_Load.address = 40;
        this->Present_Load.size = 2;
        this->Present_Voltage.address = 42;
        this->Present_Voltage.size = 1;
        this->Present_Temperature.address = 43;
        this->Present_Temperature.size = 1;
        this->Registered.address = 44;
        this->Registered.size = 1;
        this->Moving.address = 46;
        this->Moving.size = 1;
        this->Lock.address = 47;
        this->Lock.size = 1;
        this->Punch.address = 48;
        this->Punch.size = 2;
        this->Realtime_Tick.address = 50;
        this->Realtime_Tick.size = 2; 
        this->Current.address = 68;
        this->Current.size = 2;
        this->Torque_Ctrl_Mode_Enable.address = 70;
        this->Torque_Ctrl_Mode_Enable.size = 1;
        this->Goal_Torque.address = 71;
        this->Goal_Torque.size = 2;
        this->Goal_Acceleration.address = 73;
        this->Goal_Acceleration.size = 1;
    }
    else if (protocol == 2)
    {
        //Control Table of EEPROM Area
        this->Model_Number.address = 0;
        this->Model_Number.size = 2;
        this->Model_Information.address = 2;
        this->Model_Information.size = 4;
        this->Firmware_Version.address = 6;
        this->Firmware_Version.size = 1;
        this->ID.address = 7;
        this->ID.size = 1;
        this->Baud_Rate.address = 8;
        this->Baud_Rate.size = 1;
        this->Return_Delay_Time.address = 9;
        this->Return_Delay_Time.size = 1;
        this->Drive_Mode.address = 10;
        this->Drive_Mode.size = 1;
        this->Operating_Mode.address = 11;
        this->Operating_Mode.size = 1;
        this->Secondary_ID.address = 12;
        this->Secondary_ID.size = 1;
        this->Protocol_Type.address = 13;
        this->Protocol_Type.size = 1;
        this->Homing_Offset.address = 20;
        this->Homing_Offset.size = 4;
        this->Moving_Threshold.address = 24;
        this->Moving_Threshold.size = 4;
        this->Temperature_Limit.address = 31;
        this->Temperature_Limit.size = 1;
        this->Max_Voltage_Limit.address = 32;
        this->Max_Voltage_Limit.size = 2;
        this->Min_Voltage_Limit.address = 34;
        this->Min_Voltage_Limit.size = 2;
        this->PWM_Limit.address = 36;
        this->PWM_Limit.size = 2;
        this->Current_Limit.address = 38;
        this->Current_Limit.size = 2;
        this->Acceleration_Limit.address = 40;
        this->Acceleration_Limit.size = 4;
        this->Velocity_Limit.address = 44;
        this->Velocity_Limit.size = 4;
        this->Max_Position_Limit.address = 48;
        this->Max_Position_Limit.size = 4;
        this->Min_Position_Limit.address = 52;
        this->Min_Position_Limit.size = 4;
        this->Shutdown.address = 63;
        this->Shutdown.size = 1;

        //Control Table of RAM Area
        this->Torque_Enable.address = 64;
        this->Torque_Enable.size = 1;
        this->LED.address = 65;
        this->LED.size = 1;
        this->Status_Return_Level.address = 68;
        this->Status_Return_Level.size = 1;
        this->Registered_Instruction.address = 69;
        this->Registered_Instruction.size = 1;
        this->Hardware_Error_Status.address = 70;
        this->Hardware_Error_Status.size = 1;
        this->Velocity_I_Gain.address = 76;
        this->Velocity_I_Gain.size = 2;
        this->Velocity_P_Gain.address = 78;
        this->Velocity_P_Gain.size = 2;
        this->Position_D_Gain.address = 80;
        this->Position_D_Gain.size = 2;
        this->Position_I_Gain.address = 82;
        this->Position_I_Gain.size = 2;
        this->Position_P_Gain.address = 84;
        this->Position_P_Gain.size = 2;
        this->Feedforward_2nd_Gain.address = 88;
        this->Feedforward_2nd_Gain.size = 2;
        this->Feedforward_1st_Gain.address = 90;
        this->Feedforward_1st_Gain.size = 2;
        this->BUS_Watchdog.address = 98;
        this->BUS_Watchdog.size = 1;
        this->Goal_PWM.address = 100;
        this->Goal_PWM.size = 2;
        this->Goal_Current.address = 102;
        this->Goal_Current.size = 2;
        this->Goal_Velocity.address = 104;
        this->Goal_Velocity.size = 4;
        this->Profile_Acceleration.address = 108;
        this->Profile_Acceleration.size = 4;
        this->Profile_Velocity.address = 112;
        this->Profile_Velocity.size = 4;
        this->Goal_Position.address = 116;
        this->Goal_Position.size = 4;
        this->Realtime_Tick.address = 120;
        this->Realtime_Tick.size = 2;
        this->Moving.address = 122;
        this->Moving.size = 1;
        this->Moving_Status.address = 123;
        this->Moving_Status.size = 1;
        this->Present_PWM.address = 124;
        this->Present_PWM.size = 2;
        this->Present_Current.address = 126;
        this->Present_Current.size = 2;
        this->Present_Velocity.address = 128;
        this->Present_Velocity.size = 4;
        this->Present_Position.address = 132;
        this->Present_Position.size = 4;
        this->Velocity_Trajectory.address = 136;
        this->Velocity_Trajectory.size = 4;
        this->Position_Trajectory.address = 140;
        this->Position_Trajectory.size = 4;
        this->Present_Input_Voltage.address = 144;
        this->Present_Input_Voltage.size = 2;
        this->Present_Temperature.address = 146;
        this->Present_Temperature.size = 1;
    }
    
    



}

gh360::MX_64_DICT::~MX_64_DICT()
{
    
}
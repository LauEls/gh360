#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include <iostream>
#include <chrono>


class Encoder
{
    public:
        /**
         * @brief Constructor for the Encoder class
         * @param joint_name The name of the joint the encoder is connected to
         * @param port_name The name of the port the encoder is connected to
         * @param port_id The position of this encoder in the message from the port
         * @param offset The offset of the encoder
         * @param inverter The inverter of the encoder (1 or -1)
         */
        Encoder(std::string joint_name, 
            std::string port_name, 
            int port_id, 
            double offset, 
            int inverter);


        virtual ~Encoder();

        /**
         * @brief Sets the angle of the joint and calculates the velocity using the previous angle and the time elapsed
         * @param angle The angle of the joint
         * @param time The time the angle was read
         */
        void set_joint_angle(double angle);

        /**
         * @brief Calculates the velocity of the joint
         * @param time The current timestamp
         * @return The velocity of the joint
         */
        double calc_joint_velocity(std::chrono::time_point<std::chrono::system_clock> time);

        /**
         * @return The name of the joint the encoder is connected to
         */
        std::string get_joint_name();

        /**
         * @return The name of the port the encoder is connected to
         */
        std::string get_port_name();

        /**
         * @return The position of this encoder in the message from the port
         */
        int get_port_id();

        /**
         * @return The offset of the encoder
         */
        double get_offset();

        /**
         * @return The inverter of the encoder (1 or -1)
         */
        int get_inverter();

        /**
         * @return The current angle of the joint
         */
        double get_joint_angle();

        /**
         * @return The previous angle of the joint
         */
        double get_prev_joint_angle();   

        /**
         * @return The velocity of the joint
         */
        double get_joint_velocity();

    private:
        std::string joint_name;
        std::string port_name;
        int port_id;
        double offset;
        int inverter;
        double joint_angle;
        double prev_joint_angle;
        std::chrono::time_point<std::chrono::system_clock> prev_time;
        double joint_velocity;
        float alpha = 0.1;
};

#endif // ENCODER_HPP_
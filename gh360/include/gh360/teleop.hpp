#ifndef TELEOP_HPP_
#define TELEOP_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "gh360_interfaces/msg/space_mouse.hpp"
#include "gh360_interfaces/msg/set_motor_velocities.hpp"
#include "gh360_interfaces/msg/set_velocity.hpp"
#include "gh360_interfaces/msg/set_position.hpp"
#include "gh360_interfaces/msg/set_motor_positions.hpp"
#include "gh360_interfaces/msg/port_status.hpp"


using namespace std::chrono_literals;

namespace gh360
{
    class Teleop : public rclcpp::Node
    {
        public:
            Teleop();
            virtual ~Teleop();

        private:
            void timer_callback();
            void inverse_jacobian_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg);
            void motor_status_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg);

            rclcpp::TimerBase::SharedPtr timer_;
            // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_goal_publisher_;
            // MOTOR VELOCITY PUBLISHERS FOR EACH ARM SECTION
            rclcpp::Publisher<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr shoulder_velocity_publisher_;
            rclcpp::Publisher<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr upperarm_velocity_publisher_;
            rclcpp::Publisher<gh360_interfaces::msg::SetMotorVelocities>::SharedPtr lowerarm_velocity_publisher_;
            rclcpp::Publisher<gh360_interfaces::msg::SetMotorPositions>::SharedPtr shoulder_position_publisher_;
            rclcpp::Publisher<gh360_interfaces::msg::SetMotorPositions>::SharedPtr upperarm_position_publisher_;
            rclcpp::Publisher<gh360_interfaces::msg::SetMotorPositions>::SharedPtr lowerarm_position_publisher_;
            rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr shoulder_motor_status_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr upperarm_motor_status_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::PortStatus>::SharedPtr lowerarm_motor_status_subscriber_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr inverse_jacobian_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::SpaceMouse>::SharedPtr spacemouse_subscriber_;

            sensor_msgs::msg::JointState goal_joint_velocity;
            // sensor_msgs::msg::JointState joint_state_msg;
            gh360_interfaces::msg::SetMotorVelocities set_velocities_msg;
            gh360_interfaces::msg::SetMotorPositions init_motor_msg;
            gh360_interfaces::msg::PortStatus motor_status;
            std::vector<double> init_motor_pos;
            std::vector<double> jnt_to_motor_scaler;
            bool init_flag;
            int init_state;
            double max_motor_vel;
    };
}

#endif // TELEOP_HPP_
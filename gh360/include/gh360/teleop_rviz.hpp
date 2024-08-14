#ifndef TELEOP_RVIZ_HPP_
#define TELEOP_RVIZ_HPP_

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "gh360_interfaces/msg/space_mouse.hpp"

using namespace std::chrono_literals;

namespace gh360
{
    class TeleopRviz : public rclcpp::Node
    {
        public:
            TeleopRviz();
            virtual ~TeleopRviz();

        private:
            void timer_callback();
            void inverse_jacobian_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg);

            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_goal_publisher_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr inverse_jacobian_subscriber_;
            rclcpp::Subscription<gh360_interfaces::msg::SpaceMouse>::SharedPtr spacemouse_subscriber_;

            sensor_msgs::msg::JointState goal_joint_velocity;
            sensor_msgs::msg::JointState joint_state_msg;
            std::vector<double> init_joint_pos;
            // KDL::Chain chain;
            // KDL::ChainFkSolverPos_recursive* fk_solver;
            // KDL::ChainIkSolverVel_pinv* ik_solver;
            // std::string robot_name;
    };
}

#endif // TELEOP_RVIZ_HPP_
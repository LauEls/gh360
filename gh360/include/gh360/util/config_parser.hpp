#ifndef CONFIG_PARSER_HPP_
#define CONFIG_PARSER_HPP_

#include "rclcpp/rclcpp.hpp"

#include "gh360/joint_types/motor_joint.hpp"
#include "gh360/joint_types/soft_joint.hpp"

/**
 * @brief Get the robot joints from the parameter server
 * @param node The ros node to get the parameters from
 * @return A vector of Joint pointers
 */
std::vector<Joint*> get_robot_joints(rclcpp::Node* node);

#endif // CONFIG_PARSER_HPP_
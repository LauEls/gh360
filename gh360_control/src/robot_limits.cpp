#include "robot_limits.hpp"

RobotLimits::RobotLimits(): Node("robot_limits")
{
    this->joints = get_robot_joints(this);
    this->joint_limits_request = std::make_shared<gh360_interfaces::srv::SetJointLimits::Request>();

    this->shoulder_limits_client_ = this->create_client<gh360_interfaces::srv::SetJointLimits>("/gh360/shoulder/set_joint_limits");
    this->upperarm_limits_client_ = this->create_client<gh360_interfaces::srv::SetJointLimits>("/gh360/upperarm/set_joint_limits");
    this->lowerarm_limits_client_ = this->create_client<gh360_interfaces::srv::SetJointLimits>("/gh360/lowerarm/set_joint_limits");

    while ((!this->shoulder_limits_client_->wait_for_service(std::chrono::seconds(1))) || (!this->upperarm_limits_client_->wait_for_service(std::chrono::seconds(1))) || (!this->lowerarm_limits_client_->wait_for_service(std::chrono::seconds(1))))// || this->upperarm_limits_client_->wait_for_service(std::chrono::seconds(1)) || this->lowerarm_limits_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    this->robot_limits_service_ = this->create_service<gh360_interfaces::srv::SetRobotLimits>("set_robot_limits", std::bind(&RobotLimits::robot_limits_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Robot Limits node started");
}

RobotLimits::~RobotLimits()
{
}

void RobotLimits::robot_limits_callback(const std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Request> request, std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Response> response)
{
    bool joint_angle_limits = false;
    bool motor_current_limits = false;

    RCLCPP_INFO(this->get_logger(), "Recieved Robot Limits Request: %s", true ? "true": "false");

    if ((request->max_joint_angles.size() == this->joints.size()) && (request->min_joint_angles.size() == this->joints.size())) joint_angle_limits = true;
    RCLCPP_INFO(this->get_logger(), "Joint Angle Limits: %s", joint_angle_limits ? "true": "false");

    if ((request->max_motor_currents.size() == this->joints.size()) && (request->min_motor_currents.size() == this->joints.size())) motor_current_limits = true;
    RCLCPP_INFO(this->get_logger(), "Motor Current Limits: %s", motor_current_limits ? "true": "false");

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        gh360_interfaces::msg::JointLimits new_joint_limits = gh360_interfaces::msg::JointLimits();
        new_joint_limits.joint_name = this->joints[i]->get_joint_name();
        if (joint_angle_limits)
        {
            new_joint_limits.max_joint_angle = request->max_joint_angles[i];
            new_joint_limits.min_joint_angle = request->min_joint_angles[i];
        }
        if (motor_current_limits)
        {
            new_joint_limits.max_motor_current = request->max_motor_currents[i];
            new_joint_limits.min_motor_current = request->min_motor_currents[i];
        }
       
        this->joint_limits_request->joint_limits.push_back(new_joint_limits);
    }

    auto shoulder_future = this->shoulder_limits_client_->async_send_request(this->joint_limits_request);
    auto upperarm_future = this->upperarm_limits_client_->async_send_request(this->joint_limits_request);
    auto lowerarm_future = this->lowerarm_limits_client_->async_send_request(this->joint_limits_request);

    // rclcpp::spin_until_future_complete(this->get_node_base_interface(), shoulder_future);
    // rclcpp::spin_until_future_complete(this->get_node_base_interface(), upperarm_future);
    // rclcpp::spin_until_future_complete(this->get_node_base_interface(), lowerarm_future);

    response->success = true;

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotLimits>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
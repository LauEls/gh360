#include "robot_stop.hpp"

RobotStop::RobotStop(): Node("robot_stop")
{
    this->robot_stop_service_ = this->create_service<std_srvs::srv::SetBool>("robot_stop", std::bind(&RobotStop::robot_stop_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Robot Stop node started");
    this->robot_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/gh360/set_torque", 10);

    this->timer_ = this->create_wall_timer(100ms, std::bind(&RobotStop::timer_callback, this));
}

RobotStop::~RobotStop()
{
}

void RobotStop::robot_stop_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data == true)
    {
        this->robot_stop = true;
    }
    else
    {
        this->robot_stop = false;
    }

    response->success = true;
}

void RobotStop::timer_callback()
{
    std_msgs::msg::Bool robot_stop_msg;
    robot_stop_msg.data = !this->robot_stop;
    this->robot_stop_publisher_->publish(robot_stop_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
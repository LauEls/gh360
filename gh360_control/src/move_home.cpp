#include "move_home.hpp"

MoveHome::MoveHome(): Node("move_home")
{
    this->publish_msg_cnt = 10;
    this->cntr = 0;
    
    this->move_home_service_ = this->create_service<std_srvs::srv::SetBool>("move_home", std::bind(&MoveHome::move_home_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Move Home node started");
    this->move_home_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/gh360/move_home", 10);

    this->timer_ = this->create_wall_timer(100ms, std::bind(&MoveHome::timer_callback, this));
}

MoveHome::~MoveHome()
{
}

void MoveHome::move_home_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data == true)
    {
        this->move_home = true;
        this->cntr = 0;
    }

    response->success = true;
}

void MoveHome::timer_callback()
{
    if (this->move_home)
    {
        std_msgs::msg::Bool move_home_msg;
        move_home_msg.data = this->move_home;
        this->move_home_publisher_->publish(move_home_msg);
        this->cntr++;
        if (this->cntr >= this->publish_msg_cnt) this->move_home = false;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveHome>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#ifndef MOVE_HOME_HPP_
#define MOVE_HOME_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

class MoveHome : public rclcpp::Node
{
    public:
        MoveHome();
        virtual ~MoveHome();

    private:
        void timer_callback();
        void move_home_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
            
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr move_home_service_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr move_home_publisher_;
        bool move_home = false;
        int publish_msg_cnt;
        int cntr;

};
#endif // MOVE_HOME_HPP_
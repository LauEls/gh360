#include "teleop_rviz.hpp"

gh360::TeleopRviz::TeleopRviz() : Node("teleop_rviz")
{
    this->init_joint_pos = {0.0, 0.0, 0.0, 1.5708, 1.5708, 0.0, 0.0};
    
    this->goal_joint_velocity = sensor_msgs::msg::JointState();
    this->goal_joint_velocity.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    this->joint_state_msg = sensor_msgs::msg::JointState();
    this->joint_state_msg.name = {"shoulder_yaw", "shoulder_roll", "shoulder_pitch", "upperarm_roll", "elbow", "forearm_roll", "wrist_pitch"};
    this->joint_state_msg.position = this->init_joint_pos;
    
    
    this->inverse_jacobian_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/inverse_jacobian", 10, std::bind(&gh360::TeleopRviz::inverse_jacobian_callback, this, std::placeholders::_1));
    this->spacemouse_subscriber_ = this->create_subscription<gh360_interfaces::msg::SpaceMouse>("/spacemouse", 10, std::bind(&gh360::TeleopRviz::spacemouse_callback, this, std::placeholders::_1));
    this->joint_goal_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/gh360_joint_states", 10);
    this->timer_ = this->create_wall_timer(10ms, std::bind(&TeleopRviz::timer_callback, this));
}

gh360::TeleopRviz::~TeleopRviz()
{
}

void gh360::TeleopRviz::spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg)
{
    // this->desired_velocity = msg->velocity;
    if (msg->button1 || msg->button2) {
        this->joint_state_msg.position = this->init_joint_pos;
    }
    // RCLCPP_INFO(this->get_logger(),"spacemouse callback");
}

void gh360::TeleopRviz::inverse_jacobian_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->goal_joint_velocity = *msg;
}

void gh360::TeleopRviz::timer_callback()
{
    for (int i = 0; i < 7; i++) {
        this->joint_state_msg.position[i] += this->goal_joint_velocity.velocity[i];
    }
    // this->joint_state_msg.position += this->goal_joint_velocity.velocity;
    joint_goal_publisher_->publish(this->joint_state_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto teleop_rviz_node = std::make_shared<gh360::TeleopRviz>();
    rclcpp::spin(teleop_rviz_node);
    rclcpp::shutdown();
    return 0;
}
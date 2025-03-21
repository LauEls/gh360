#include "eef_velocity.hpp"

EEFVelocity::EEFVelocity(): Node("eef_velocity")
{
    this->declare_parameter("base_link_name", "base_link");
    std::string base_link_name = get_parameter("base_link_name").as_string();
    RCLCPP_DEBUG(this->get_logger(), "Base link name: %s", base_link_name.c_str());
    this->declare_parameter("eef_link_name", "eef");
    std::string eef_link_name = get_parameter("eef_link_name").as_string();
    RCLCPP_DEBUG(this->get_logger(), "EEF link name: %s", eef_link_name.c_str());
    this->declare_parameter("robot_description", "");
    std::string robot_desc_string = get_parameter("robot_description").as_string();
    this->declare_parameter("joint_names", std::vector<std::string>());
    std::vector<std::string> joint_names = get_parameter("joint_names").as_string_array();

    this->inverse_jacobian = new InverseJacobian(robot_desc_string, base_link_name, eef_link_name);

    this->desired_velocity = std::vector<double>(6, 0.0);
    this->joint_goal_vel_msg = sensor_msgs::msg::JointState();
    for (unsigned int i=0; i < joint_names.size(); i++)
    {
        this->joint_goal_vel_msg.name.push_back(joint_names[i]);
        this->joint_goal_vel_msg.velocity.push_back(0.0);
        this->joint_goal_vel_msg.position.push_back(0.0);
    }

    this->cmd_eef_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_eef_vel", 10, std::bind(&EEFVelocity::cmd_eef_vel_callback, this, std::placeholders::_1));
    this->joint_position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/gh360/joint_states", 10, std::bind(&EEFVelocity::joint_position_callback, this, std::placeholders::_1));
    this->joint_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("cmd_joint_vel", 10);
    
    RCLCPP_INFO(this->get_logger(), "EEF Velocity node started");
}

EEFVelocity::~EEFVelocity()
{
}

void EEFVelocity::cmd_eef_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received EEF velocity command");
    this->desired_velocity[0] = msg->linear.x;
    this->desired_velocity[1] = msg->linear.y;
    this->desired_velocity[2] = msg->linear.z;
    this->desired_velocity[3] = msg->angular.x;
    this->desired_velocity[4] = msg->angular.y;
    this->desired_velocity[5] = msg->angular.z;

    // RCLCPP_INFO(this->get_logger(), "Received EEF velocity command 2");

    if (this->joint_states_recieved)
    {
        // RCLCPP_INFO(this->get_logger(), "Calculating joint velocities");
        std::vector<double> goal_joint_velocities = this->inverse_jacobian->calculate_goal_joint_velocities(this->desired_velocity, this->current_joint_pos);
        this->joint_goal_vel_msg.velocity = goal_joint_velocities;
        // RCLCPP_INFO(this->get_logger(), "Publishing joint velocities");
        this->joint_velocity_publisher_->publish(joint_goal_vel_msg);
    }
}

void EEFVelocity::joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received joint states");
    this->current_joint_pos = msg->position;
    if (!this->joint_states_recieved) this->joint_states_recieved = true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto eef_vel_node = std::make_shared<EEFVelocity>();
    rclcpp::spin(eef_vel_node);
    rclcpp::shutdown();

    return 0;
}
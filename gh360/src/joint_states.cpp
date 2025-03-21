#include "gh360/joint_states.hpp"


gh360::JointStates::JointStates()
: Node("joint_states")
{
    RCLCPP_INFO(this->get_logger(), "Run gh360 joint states node");

    this->joints = get_robot_joints(this);

    this->encoder_subscriber_ = this->create_subscription<gh360_interfaces::msg::ArmEncoderStates>("encoder_states", 10, std::bind(&gh360::JointStates::encoder_callback, this, std::placeholders::_1));
    this->motor_states_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("motor_states", 10, std::bind(&gh360::JointStates::motor_states_callback, this, std::placeholders::_1));

    this->joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    this->timer_ = this->create_wall_timer(100ms, std::bind(&gh360::JointStates::timer_callback, this));

}

gh360::JointStates::~JointStates()
{ 
    
}

void gh360::JointStates::encoder_callback(const gh360_interfaces::msg::ArmEncoderStates::SharedPtr msg)
{
    if (!(this->soft_joint_states_recieved)) this->soft_joint_states_recieved = true;

    for (unsigned int s=0; s < msg->current_joint_states.size(); s++) 
    {
        for (unsigned int j=0; j < this->joints.size(); j++)
        {
            if (msg->current_joint_states[s].joint_name == this->joints[j]->get_joint_name())
            {
                if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[j])) 
                {
                    soft_joint->set_joint_angle(msg->current_joint_states[s].current_pos);
                    soft_joint->set_joint_velocity(msg->current_joint_states[s].current_vel);
                }
            }
        }
    }
}

void gh360::JointStates::motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
{
    if (!(this->motor_joint_states_recieved)) this->motor_joint_states_recieved = true;

    for (unsigned int s=0; s < msg->motors.size(); s++) 
    {
        for (unsigned int i=0; i<this->joints.size(); i++)
        {
            if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                if (msg->motors[s].motor_id == motor_joint->get_motor(0)->get_motor_id())
                {
                    motor_joint->get_motor(0)->set_present_position_adjusted(msg->motors[s].present_position);
                    motor_joint->get_motor(0)->set_present_velocity_adjusted(msg->motors[s].present_velocity);
                }
            }
        }
    }
}

sensor_msgs::msg::JointState gh360::JointStates::create_joint_state_msg()
{
    sensor_msgs::msg::JointState joint_states_msg = sensor_msgs::msg::JointState();
    std::vector<std::string> names(7);
    std::vector<double> positions(7);
    std::vector<double> velocities(7);

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        names[i] = this->joints[i]->get_joint_name();
        positions[i] = this->joints[i]->get_joint_angle();
        velocities[i] = this->joints[i]->get_joint_velocity();
    }
    joint_states_msg.name = names;
    joint_states_msg.position = positions;
    joint_states_msg.velocity = velocities;

    return joint_states_msg;
}

void gh360::JointStates::timer_callback()
{
    if (this->soft_joint_states_recieved && this->motor_joint_states_recieved)
    {
        sensor_msgs::msg::JointState joint_state_msg = this->create_joint_state_msg();
        this->joint_states_publisher_->publish(joint_state_msg);
    }
    else RCLCPP_INFO(this->get_logger(), "Waiting for Joint Position data");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto joint_states_node = std::make_shared<gh360::JointStates>();
    rclcpp::spin(joint_states_node);
    rclcpp::shutdown();

    return 0;
}
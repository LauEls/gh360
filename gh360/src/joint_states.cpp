#include "joint_states.hpp"


gh360::JointStates::JointStates()
: Node("gh360_joint_states")
{
    RCLCPP_INFO(this->get_logger(), "Run gh360 joint states node");
    // this->joint_states_recieved = false;

    SoftJoint* new_joint = new SoftJoint();
    new_joint->set_joint_name("shoulder_yaw");
    new_joint->set_joint_angle(0.0);
    this->joints.push_back(new_joint);
    new_joint = new SoftJoint();
    new_joint->set_joint_name("shouler_roll");
    new_joint->set_joint_angle(0.0);
    this->joints.push_back(new_joint);
    new_joint = new SoftJoint();
    new_joint->set_joint_name("shoulder_pitch");
    new_joint->set_joint_angle(0.0);
    this->joints.push_back(new_joint);
    new_joint = new SoftJoint();
    new_joint->set_joint_name("upperarm_roll");
    new_joint->set_joint_angle(0.0);
    this->joints.push_back(new_joint);
    new_joint = new SoftJoint();
    new_joint->set_joint_name("elbow");
    this->joints.push_back(new_joint);
    new_joint->set_joint_angle(0.0);
    // MotorJoint * new_motor_joint = new MotorJoint();
    new_joint = new SoftJoint();
    new_joint->set_joint_name("forearm_roll");
    new_joint->set_joint_angle(0.0);
    // new_joint->set_motor_id(11);
    this->joints.push_back(new_joint);
    new_joint = new SoftJoint();
    new_joint->set_joint_name("wrist_pitch");
    new_joint->set_joint_angle(0.0);
    this->joints.push_back(new_joint);

    this->encoder_subscriber_ = this->create_subscription<gh360_interfaces::msg::ArmEncoderStates>("/encoder_status", 10, std::bind(&gh360::JointStates::encoder_callback, this, std::placeholders::_1));
    this->lowerarm_motor_states_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("/lowerarm/motor_status", 10, std::bind(&gh360::JointStates::lowerarm_motor_states_callback, this, std::placeholders::_1));

    this->joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/gh360_joint_states", 10);
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
                if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[j])) soft_joint->set_joint_angle(msg->current_joint_states[s].current_pos);
            }
        }
    }
}

void gh360::JointStates::lowerarm_motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
{
    if (!(this->motor_joint_states_recieved)) this->motor_joint_states_recieved = true;
    int motor_joint_iter = 5;
    for (unsigned int s=0; s < msg->motors.size(); s++) 
    {
        if (msg->motors[s].motor_id == 11)
        {   
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[motor_joint_iter])) 
            {   
                soft_joint->set_joint_angle(msg->motors[s].present_position);
            }
        }
    }
    
}

sensor_msgs::msg::JointState gh360::JointStates::create_joint_state_msg()
{
    sensor_msgs::msg::JointState joint_states_msg = sensor_msgs::msg::JointState();
    std::vector<std::string> names(7);
    std::vector<double> positions(7);

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            names[i] = soft_joint->get_joint_name();
            positions[i] = soft_joint->get_joint_angle();
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            names[i] = motor_joint->get_joint_name();
            positions[i] = motor_joint->get_joint_angle();
        }
    }
    joint_states_msg.name = names;
    joint_states_msg.position = positions;

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
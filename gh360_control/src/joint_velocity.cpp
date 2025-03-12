#include "joint_velocity.hpp"

JointVelocity::JointVelocity(): Node("joint_velocity")
{
    this->declare_parameter("control_mode", "open_loop");
    this->control_mode = get_parameter("control_mode").as_string();
    this->declare_parameter("max_motor_vel", 1.0);
    this->max_motor_vel = get_parameter("max_motor_vel").as_double();

    this->joints = get_robot_joints(this);
    
    this->motor_goal_vel_msg = gh360_interfaces::msg::SetMotorVelocities();
    this->cmd_joint_vel_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("cmd_joint_vel", 10, std::bind(&JointVelocity::cmd_joint_vel_callback, this, std::placeholders::_1));
    this->motor_velocity_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorVelocities>("/gh360/motor_goal_velocity", 10);
    // this->timer_ = this->create_wall_timer(10ms, std::bind(&JointVelocity::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Joint Velocity node started");
}

JointVelocity::~JointVelocity()
{
}

void JointVelocity::cmd_joint_vel_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->motor_goal_vel_msg.motor_goal_velocities.clear();
    double max_vel = 0.0;

    for (unsigned int i=0; i < msg->velocity.size(); i++) 
    {
        for (unsigned int j=0; j<this->joints.size(); j++)
        {
            
            if (msg->name[i] == this->joints[j]->get_joint_name())
            {
                double motor_vel = 0.0;
                if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[j]))
                {
                    motor_vel = msg->velocity[i] * (soft_joint->get_radius_passive_pulley() / soft_joint->get_radius_active_pulley());
                }
                else motor_vel = msg->velocity[i];

                if (motor_vel >this->max_motor_vel && motor_vel > max_vel) {
                    max_vel = motor_vel;
                }
                
                for (int k=0; k<this->joints[j]->get_motor_cnt(); k++)
                {
                    gh360_interfaces::msg::SetVelocity new_velocity = gh360_interfaces::msg::SetVelocity();
                    new_velocity.id = this->joints[j]->get_motor(k)->get_motor_id();
                    new_velocity.velocity = motor_vel;
                    this->motor_goal_vel_msg.motor_goal_velocities.push_back(new_velocity);
                }
            }
        }
    }

    if (max_vel > this->max_motor_vel) {
        int m_cntr = this->motor_goal_vel_msg.motor_goal_velocities.size();
        for (int i = 0; i < m_cntr; i++) {
            this->motor_goal_vel_msg.motor_goal_velocities[i].velocity = this->motor_goal_vel_msg.motor_goal_velocities[i].velocity * this->max_motor_vel / max_vel;
        }
    }

    this->motor_velocity_publisher_->publish(this->motor_goal_vel_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointVelocity>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
#include "joint_position.hpp"

JointPosition::JointPosition(): Node("joint_velocity")
{
    this->declare_parameter("max_motor_vel", 1.0);
    this->max_motor_vel = get_parameter("max_motor_vel").as_double();
    this->declare_parameter("joint_pos_accuracy", 0.01);
    this->joint_pos_accuracy = get_parameter("joint_pos_accuracy").as_double();

    this->joints = get_robot_joints(this);
    
    this->motor_goal_vel_msg = gh360_interfaces::msg::SetMotorVelocities();
    this->cmd_joint_pos_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("cmd_joint_pos", 10, std::bind(&JointPosition::cmd_joint_pos_callback, this, std::placeholders::_1));
    this->motor_velocity_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorVelocities>("/gh360/motor_goal_velocity", 10);
    this->joint_position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/gh360/joint_states", 10, std::bind(&JointPosition::joint_position_callback, this, std::placeholders::_1));
    // this->timer_ = this->create_wall_timer(10ms, std::bind(&JointPosition::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Joint Velocity node started");
}

JointPosition::~JointPosition()
{
}

void JointPosition::joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received joint states");
    // this->current_joint_pos = msg->position;
    if (!this->joint_states_recieved) this->joint_states_recieved = true;
    for (unsigned int i=0; i < msg->position.size(); i++) 
    {
        for (unsigned int j=0; j<this->joints.size(); j++)
        {
            if (msg->name[i] == this->joints[j]->get_joint_name())
            {
                if (this->joints[j]->get_motor_cnt() == 1)
                {
                    this->joints[j]->get_motor(0)->set_present_position_adjusted(msg->position[i]);
                }
                else
                {
                    this->joints[j]->set_joint_angle(msg->position[i]);
                }
            }
            // RCLCPP_INFO(this->get_logger(), "Joint %s: %f", this->joints[j]->get_joint_name().c_str(), this->joints[j]->get_joint_angle());
        }
    }
}

void JointPosition::cmd_joint_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!this->joint_states_recieved) return;
    
    this->motor_goal_vel_msg.motor_goal_velocities.clear();

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        double kp = 2.0;
        if (this->joints[i]->get_joint_name() == "shoulder_pitch") kp = 6.0;
        // else if (this->joints[i]->get_joint_name() == "upperarm_roll") kp = 3.0;
        else if (this->joints[i]->get_joint_name() == "forearm_roll") kp = 1.0;
        double joint_pos_error = msg->data[i] - this->joints[i]->get_joint_angle();
        double joint_vel = (joint_pos_error)*kp;
        // RCLCPP_INFO(this->get_logger(), "Joint %s: %f", this->joints[i]->get_joint_name().c_str(), joint_vel);
        double motor_vel = 0.0;
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i])) 
        {
            motor_vel = joint_vel * (soft_joint->get_radius_passive_pulley() / soft_joint->get_radius_active_pulley());
        }
        else motor_vel = joint_vel;

        motor_vel = std::max(-this->max_motor_vel, std::min(motor_vel,this->max_motor_vel));
        if (abs(joint_pos_error) < this->joint_pos_accuracy) motor_vel = 0.0;

        for (int k=0; k < this->joints[i]->get_motor_cnt(); k++)
        {
            gh360_interfaces::msg::SetVelocity new_velocity = gh360_interfaces::msg::SetVelocity();
            new_velocity.id = this->joints[i]->get_motor(k)->get_motor_id();
            new_velocity.velocity = motor_vel;
            this->motor_goal_vel_msg.motor_goal_velocities.push_back(new_velocity);
        }
    }

    // for (unsigned int i=0; i < this->joints.size(); i++)
    // {
    //     for (unsigned int j=0; j < msg->name.size(); j++)
    //     {
    //         if (msg->name[j] == this->joints[i]->get_joint_name())
    //         {
    //             double joint_vel = msg->position[j] - this->joints[i]->get_joint_angle();
    //             // RCLCPP_INFO(this->get_logger(), "Joint %s: %f", this->joints[i]->get_joint_name().c_str(), joint_vel);
    //             double motor_vel = 0.0;
    //             if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i])) 
    //             {
    //                 motor_vel = joint_vel * (soft_joint->get_radius_passive_pulley() / soft_joint->get_radius_active_pulley());
    //             }
    //             else motor_vel = joint_vel;

    //             motor_vel = std::max(-this->max_motor_vel, std::min(motor_vel,this->max_motor_vel));

    //             for (int k=0; k < this->joints[i]->get_motor_cnt(); k++)
    //             {
    //                 gh360_interfaces::msg::SetVelocity new_velocity = gh360_interfaces::msg::SetVelocity();
    //                 new_velocity.id = this->joints[i]->get_motor(k)->get_motor_id();
    //                 new_velocity.velocity = motor_vel;
    //                 this->motor_goal_vel_msg.motor_goal_velocities.push_back(new_velocity);
    //             }
    //         }
    //     }
    // }
    this->motor_velocity_publisher_->publish(this->motor_goal_vel_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointPosition>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
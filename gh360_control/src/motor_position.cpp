#include "motor_position.hpp"

MotorPosition::MotorPosition(): Node("motor_position")
{
    // this->declare_parameter("control_mode", "open_loop");
    // this->control_mode = get_parameter("control_mode").as_string();
    this->declare_parameter("command_interface", "position");
    this->command_interface = get_parameter("command_interface").as_string();
    this->declare_parameter("max_motor_vel", 1.0);
    this->max_motor_vel = get_parameter("max_motor_vel").as_double();
    this->declare_parameter("motor_pos_accuracy", 0.2);
    this->motor_pos_accuracy = get_parameter("motor_pos_accuracy").as_double();

    this->joints = get_robot_joints(this);
    
    this->motor_velocity_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorVelocities>("/gh360/motor_goal_velocity", 10);
    this->motor_position_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorPositions>("/gh360/motor_goal_position", 10);
    this->cmd_motor_pos_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("cmd_motor_pos", 10, std::bind(&MotorPosition::cmd_motor_pos_callback, this, std::placeholders::_1));
    this->motor_states_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("/gh360/motor_states", 10, std::bind(&MotorPosition::motor_states_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Motor Position node started");
}

MotorPosition::~MotorPosition()
{
}

void MotorPosition::motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
{
    if (!(this->motor_states_recieved)) this->motor_states_recieved = true;

    for (unsigned int s=0; s < msg->motors.size(); s++) 
    {
        for (unsigned int i=0; i<this->joints.size(); i++)
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                if (msg->motors[s].motor_id == this->joints[i]->get_motor(j)->get_motor_id())
                {
                    this->joints[i]->get_motor(j)->set_present_position_adjusted(msg->motors[s].present_position);
                    this->joints[i]->get_motor(j)->set_present_velocity_adjusted(msg->motors[s].present_velocity);
                    this->joints[i]->get_motor(j)->set_present_current_adjusted(msg->motors[s].present_current);
                }
            }
        }
    }

    if (this->command_interface == "velocity" && this->goal_recieved)
    {
        this->calculate_motor_goal_velocity(this->motor_goal_pos_msg);
        this->motor_velocity_publisher_->publish(this->motor_goal_vel_msg);

        if (this->goal_reached)
        {
            this->goal_recieved = false;
            this->goal_reached = false;
            RCLCPP_INFO(this->get_logger(), "Goal reached");
        }
    }
}


void MotorPosition::cmd_motor_pos_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    this->motor_goal_vel_msg.motor_goal_velocities.clear();
    this->motor_goal_pos_msg.motor_goal_positions.clear();

    int msg_cntr = 0;
    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        for (int j=0; j < this->joints[i]->get_motor_cnt(); j++)
        {
            Motor * motor = this->joints[i]->get_motor(j);
            gh360_interfaces::msg::SetPosition new_position = gh360_interfaces::msg::SetPosition();
            new_position.id = motor->get_motor_id();
            new_position.position = msg->data[msg_cntr];
            this->motor_goal_pos_msg.motor_goal_positions.push_back(new_position);
            msg_cntr++;
        }
    }

    if (this->command_interface == "position")
    {
        this->motor_position_publisher_->publish(this->motor_goal_pos_msg);
    }
    else if (this->command_interface == "velocity")
    {
        this->goal_recieved = true;
    }
}

void MotorPosition::calculate_motor_goal_velocity(gh360_interfaces::msg::SetMotorPositions msg)
{
    double motor_vel = 0.0;
    this->goal_reached = true;
    
    for (unsigned int m=0; m < msg.motor_goal_positions.size(); m++)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            for (int j=0; j < this->joints[i]->get_motor_cnt(); j++)
            {
                Motor * motor = this->joints[i]->get_motor(j);
                if (motor->get_motor_id() == msg.motor_goal_positions[m].id) 
                {
                    motor_vel = msg.motor_goal_positions[m].position - motor->get_present_position_adjusted();
                    // RCLCPP_INFO(this->get_logger(), "Motor %d: %f", motor->get_motor_id(), motor_vel);
                    motor_vel = std::max(-this->max_motor_vel, std::min(motor_vel,this->max_motor_vel));
                    if (abs(motor_vel) < this->motor_pos_accuracy) motor_vel = 0.0;
                    else this->goal_reached = false;
                    gh360_interfaces::msg::SetVelocity new_velocity = gh360_interfaces::msg::SetVelocity();
                    new_velocity.id = motor->get_motor_id();
                    new_velocity.velocity = motor_vel;
                    this->motor_goal_vel_msg.motor_goal_velocities.push_back(new_velocity);
                }
            }
        }
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorPosition>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
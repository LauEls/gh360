#include "teleop_eef_velocity.hpp"

TeleopEEFVelocity::TeleopEEFVelocity() : Node("teleop_eef_velocity")
{
    this->joints = get_robot_joints(this);
    
    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            this->declare_parameter(soft_joint->get_joint_name()+".right.motor_reset_pos", 0.0);
            this->declare_parameter(soft_joint->get_joint_name()+".left.motor_reset_pos", 0.0);
            this->joints[i]->get_motor(soft_joint->RIGHT)->set_goal_position(get_parameter(soft_joint->get_joint_name()+".right.motor_reset_pos").as_double());
            this->joints[i]->get_motor(soft_joint->LEFT)->set_goal_position(get_parameter(soft_joint->get_joint_name()+".left.motor_reset_pos").as_double());
        }
        else
        {
            this->declare_parameter(this->joints[i]->get_joint_name()+".motor_reset_pos", 0.0);
            this->joints[i]->get_motor(0)->set_goal_position(get_parameter(this->joints[i]->get_joint_name()+".motor_reset_pos").as_double());
        }
        for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
        {
            this->motor_reset_msg.data.push_back(this->joints[i]->get_motor(j)->get_goal_position());
        }
    }

    this->eef_velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_eef_vel", 10);
    this->motor_position_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_motor_pos", 10);
    this->door_reset_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/door/reset", 10);
    this->teleop_commands_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("teleop_eef_velocity", 10, std::bind(&TeleopEEFVelocity::teleop_commands_callback, this, std::placeholders::_1));
    this->teleop_buttons_subscriber_ = this->create_subscription<gh360_interfaces::msg::BoolMultiArray>("teleop_buttons", 10, std::bind(&TeleopEEFVelocity::teleop_buttons_callback, this, std::placeholders::_1));
    this->motor_states_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("/gh360/motor_states", 10, std::bind(&TeleopEEFVelocity::motor_states_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Teleop EEF Velocity node started");
}

TeleopEEFVelocity::~TeleopEEFVelocity()
{
}

void TeleopEEFVelocity::motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
{
    if (!(this->motor_states_recieved)) this->motor_states_recieved = true;

    for (unsigned int k=0; k < msg->motors.size(); k++)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                if (msg->motors[k].motor_id == this->joints[i]->get_motor(j)->get_motor_id())
                {
                    this->joints[i]->get_motor(j)->set_present_position_adjusted(msg->motors[k].present_position);
                    this->joints[i]->get_motor(j)->set_present_velocity_adjusted(msg->motors[k].present_velocity);
                }
            }
        }
    }

}


void TeleopEEFVelocity::teleop_buttons_callback(const gh360_interfaces::msg::BoolMultiArray::SharedPtr msg)
{
    if (msg->data[0] && !this->reseting) {
        this->reseting = true;
        this->desired_eef_velocity.linear.x = 0.0;
        this->desired_eef_velocity.linear.y = 0.0;
        this->desired_eef_velocity.linear.z = 0.0;
        this->desired_eef_velocity.angular.x = 0.0;
        this->desired_eef_velocity.angular.y = 0.0;
        this->desired_eef_velocity.angular.z = 0.0;
        this->eef_velocity_publisher_->publish(this->desired_eef_velocity);
        RCLCPP_INFO(this->get_logger(), "Resetting motors to intial position");
    }
    if (msg->data[1]) {
        std_msgs::msg::Bool door_reset_msg;
        door_reset_msg.data = true;
        this->door_reset_publisher_->publish(door_reset_msg);
        RCLCPP_INFO(this->get_logger(), "Resetting door");
    }

}

void TeleopEEFVelocity::teleop_commands_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (this->reseting) {
        this->motor_position_publisher_->publish(this->motor_reset_msg);

        bool motor_moving = false;
        bool pos_reached = true;

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                double present_position = this->joints[i]->get_motor(j)->get_present_position();
                double reset_position = this->joints[i]->get_motor(j)->get_goal_position();
                if (abs(present_position - reset_position) > 0.2) pos_reached = false;
                if (this->joints[i]->get_motor(j)->get_present_velocity() != 0.0) motor_moving = true;
            }
        }

        if (pos_reached && !motor_moving) {
            RCLCPP_INFO(this->get_logger(), "Motors reset finished");

            this->reseting = false;
        }

        return;
    } 

    this->desired_eef_velocity = *msg;

    this->eef_velocity_publisher_->publish(this->desired_eef_velocity);
    

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto teleop_node = std::make_shared<TeleopEEFVelocity>();
    rclcpp::spin(teleop_node);
    rclcpp::shutdown();
    return 0;
}
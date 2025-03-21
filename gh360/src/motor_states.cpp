#include "gh360/motor_states.hpp"


gh360::MotorStates::MotorStates()
: Node("motor_states")
{
    RCLCPP_INFO(this->get_logger(), "Run gh360 motor states node");

    this->joints = get_robot_joints(this);

    this->motor_states_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("motor_states", 10, std::bind(&gh360::MotorStates::motor_states_callback, this, std::placeholders::_1));
    this->motor_states_publisher_ = this->create_publisher<gh360_interfaces::msg::PortStatus>("motor_states_sorted", 10);
    this->timer_ = this->create_wall_timer(100ms, std::bind(&gh360::MotorStates::timer_callback, this));
}

gh360::MotorStates::~MotorStates()
{ 
    
}

void gh360::MotorStates::motor_states_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
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
                    this->joints[i]->get_motor(j)->set_present_temperature(msg->motors[s].present_temperature);
                    this->joints[i]->get_motor(j)->set_safety_check(msg->motors[s].safety_check);
                    this->joints[i]->get_motor(j)->set_moving(msg->motors[s].moving);
                }
            }
        }
    }
}

void gh360::MotorStates::timer_callback()
{
    if (this->motor_states_recieved)
    {
        gh360_interfaces::msg::PortStatus motor_states_msg = gh360_interfaces::msg::PortStatus();
        for (unsigned int i=0; i<this->joints.size(); i++)
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                gh360_interfaces::msg::MotorStatus motor_state = gh360_interfaces::msg::MotorStatus();
                motor_state.motor_id = this->joints[i]->get_motor(j)->get_motor_id();
                motor_state.present_position = this->joints[i]->get_motor(j)->get_present_position();
                motor_state.present_velocity = this->joints[i]->get_motor(j)->get_present_velocity();
                motor_state.present_current = this->joints[i]->get_motor(j)->get_present_current();
                motor_state.present_temperature = this->joints[i]->get_motor(j)->get_present_temperature();
                motor_state.moving = this->joints[i]->get_motor(j)->get_moving();
                motor_state.safety_check = this->joints[i]->get_motor(j)->get_safety_check();
                motor_states_msg.motors.push_back(motor_state);
            }
        }

        // sensor_msgs::msg::JointState joint_state_msg = this->create_joint_state_msg();
        this->motor_states_publisher_->publish(motor_states_msg);
    }
    else RCLCPP_INFO(this->get_logger(), "Waiting for Joint Position data");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto motor_states_node = std::make_shared<gh360::MotorStates>();
    rclcpp::spin(motor_states_node);
    rclcpp::shutdown();

    return 0;
}
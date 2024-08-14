#include "teleop.hpp"

gh360::TeleopRviz::Teleop() : Node("teleop")
{
    this->init_motor_pos = {0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0};
    this->init_flag = true;
    this->init_motor_msg = gh360_interfaces::msg::SetMotorPositions();
    this->set_velocities_msg = gh360_interfaces::msg::SetMotorVelocities();
    this->motor_status = gh360_interfaces::msg::PortStatus();
    for (int i = 0; i < 13; i++) {
        gh360_interfaces::msg::SetVelocity new_velocity = gh360_interfaces::msg::SetVelocity();
        new_velocity.motor_id = i;
        new_velocity.velocity = 0.0;
        this->set_velocities_msg.motor_goal_velocities.push_back(new_velocity);

        gh360_interfaces::msg::SetPosition new_position = gh360_interfaces::msg::SetPosition();
        new_position.motor_id = i;
        new_position.position = this->init_motor_pos[i];
        this->init_motor_msg.motor_goal_positions.push_back(new_position);

        this->motor_status.motor_status.push_back(gh360_interfaces::msg::MotorStatus());
    }

     this->goal_joint_velocity = sensor_msgs::msg::JointState();
    this->goal_joint_velocity.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    this->inverse_jacobian_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("/inverse_jacobian", 10, std::bind(&gh360::Teleop::inverse_jacobian_callback, this, std::placeholders::_1));
    this->spacemouse_subscriber_ = this->create_subscription<gh360_interfaces::msg::SpaceMouse>("/spacemouse", 10, std::bind(&gh360::Teleop::spacemouse_callback, this, std::placeholders::_1));
    
    this->shoulder_motor_status_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("/shoulder/motor_status", 10, std::bind(&gh360::Teleop::motor_status_callback, this, std::placeholders::_1));
    this->upperarm_motor_status_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("/upperarm/motor_status", 10, std::bind(&gh360::Teleop::motor_status_callback, this, std::placeholders::_1));
    this->lowerarm_motor_status_subscriber_ = this->create_subscription<gh360_interfaces::msg::PortStatus>("/lowerarm/motor_status", 10, std::bind(&gh360::Teleop::motor_status_callback, this, std::placeholders::_1));

    this->shoulder_velocity_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorVelocities>("/shoulder/motor_goal_velocity", 10);
    this->upperarm_velocity_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorVelocities>("/upperarm/motor_goal_velocity", 10);
    this->lowerarm_velocity_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorVelocities>("/lowerarm/motor_goal_velocity", 10);

    this->shoulder_position_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorPositions>("/shoulder/motor_goal_position", 10);
    this->upperarm_position_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorPositions>("/upperarm/motor_goal_position", 10);
    this->lowerarm_position_publisher_ = this->create_publisher<gh360_interfaces::msg::SetMotorPositions>("/lowerarm/motor_goal_position", 10);


    this->timer_ = this->create_wall_timer(10ms, std::bind(&Teleop::timer_callback, this));
}

gh360::TeleopRviz::~Teleop()
{
}

void gh360::Teleop::motor_status_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
{
    int msg_len = sizeof(msg->motor_status) / sizeof(msg->motor_status[0]);
    for (int i = 0; i < msg_len; i++) {
        this->motor_status.motor_status[msg->motor_status[i].motor_id] = msg->motor_status[i];
    }
}

void gh360::Teleop::spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg)
{
    if (msg->button1 || msg->button2) {
        // this->joint_state_msg.position = this->init_joint_pos;
        this->init_flag = true;
    }
}

void gh360::Teleop::inverse_jacobian_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->goal_joint_velocity = *msg;
}

void gh360::Teleop::timer_callback()
{
    if (this->init_flag) {
        // for (int i = 0; i < 13; i++) {
        //     this->set_velocities_msg.motor_goal_velocities[i].velocity = 0.0;
        // }
        // this->init_flag = false;
        this->shoulder_position_publisher_->publish(this->init_motor_msg);
        this->upperarm_position_publisher_->publish(this->init_motor_msg);
        this->lowerarm_position_publisher_->publish(this->init_motor_msg);

        bool moving = false;
        for (int i = 0; i < 13; i++) {
            if (this->motor_status.motor_status[i].moving) {
                moving = true;
                break;
            }
        }

        if (!moving) {
            this->init_flag = false;
        }
    }
    else {
        int m_cntr = 0;
        for (int i = 0; i < 7; i++) {
            this->set_velocities_msg.motor_goal_velocities[m_cntr] += this->goal_joint_velocity.velocity[i];
            m_cntr++;
            if (i != 5) {
                this->set_velocities_msg.motor_goal_velocities[m_cntr] += this->goal_joint_velocity.velocity[i];
                m_cntr++;
            }
        }
        // this->joint_state_msg.position += this->goal_joint_velocity.velocity;
        // joint_goal_publisher_->publish(this->joint_state_msg);
        this->shoulder_velocity_publisher_->publish(this->set_velocities_msg);
        this->upperarm_velocity_publisher_->publish(this->set_velocities_msg);
        this->lowerarm_velocity_publisher_->publish(this->set_velocities_msg);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto teleop_node = std::make_shared<gh360::Teleop>();
    rclcpp::spin(teleop_node);
    rclcpp::shutdown();
    return 0;
}
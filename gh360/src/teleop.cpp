#include "teleop.hpp"

gh360::Teleop::Teleop() : Node("teleop")
{
    this->init_motor_pos = {0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 2.5, 2.5, 6.28, 6.28, 0.0, 0.0, 0.0};
    this->init_flag = true;
    this->init_state = 0;
    this->init_motor_msg = gh360_interfaces::msg::SetMotorPositions();
    this->set_velocities_msg = gh360_interfaces::msg::SetMotorVelocities();
    this->motor_status = gh360_interfaces::msg::PortStatus();
    for (int i = 0; i < 13; i++) {
        gh360_interfaces::msg::SetVelocity new_velocity = gh360_interfaces::msg::SetVelocity();
        new_velocity.id = i+1;
        new_velocity.velocity = 0.0;
        this->set_velocities_msg.motor_goal_velocities.push_back(new_velocity);

        gh360_interfaces::msg::SetPosition new_position = gh360_interfaces::msg::SetPosition();
        new_position.id = i+1;
        new_position.position = this->init_motor_pos[i];
        this->init_motor_msg.motor_goal_positions.push_back(new_position);

        this->motor_status.motors.push_back(gh360_interfaces::msg::MotorStatus());
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

    std::vector<double> active_pulleys = {18,18,18,18,18,18,15.6,15.6,15.6,15.6,1,15.6,15.6};
    std::vector<double> passive_pulleys = {74.5,74.5,50.5,50.5,50,50,43,43,40,40,1,30,30};
    for (int i = 0; i < 13; i++) {
        this->jnt_to_motor_scaler.push_back(passive_pulleys[i]/active_pulleys[i]);
    }
    // this->jnt_to_motor_scaler = passive_pulleys/active_pulleys;
    this->max_motor_vel = 10.0;


    this->timer_ = this->create_wall_timer(10ms, std::bind(&Teleop::timer_callback, this));
}

gh360::Teleop::~Teleop()
{
}

void gh360::Teleop::motor_status_callback(const gh360_interfaces::msg::PortStatus::SharedPtr msg)
{
    // int msg_len = sizeof(msg->motors) / sizeof(msg->motors[0]);
    for (unsigned int i = 0; i < msg->motors.size(); i++) {
        this->motor_status.motors[msg->motors[i].motor_id-1] = msg->motors[i];
    }
}

void gh360::Teleop::spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg)
{
    if (msg->button1 and this->init_flag == false) {
        // this->joint_state_msg.position = this->init_joint_pos;
        this->init_flag = true;
        RCLCPP_INFO(this->get_logger(),"init flag true");
        this->init_state = 0;
    }
}

void gh360::Teleop::inverse_jacobian_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->goal_joint_velocity = *msg;
}

void gh360::Teleop::timer_callback()
{
    
    if (this->init_flag) {
        // RCLCPP_INFO(this->get_logger(),"timer callback");
        // for (int i = 0; i < 13; i++) {
        //     this->set_velocities_msg.motor_goal_velocities[i].velocity = 0.0;
        // }
        // this->init_flag = false;
        this->shoulder_position_publisher_->publish(this->init_motor_msg);
        this->upperarm_position_publisher_->publish(this->init_motor_msg);
        this->lowerarm_position_publisher_->publish(this->init_motor_msg);

        bool moving = false;
        bool pos_reached = true;
        for (int i = 0; i < 13; i++) {
            // RCLCPP_INFO(this->get_logger(),"Motor %d moving: %d", i+1, this->motor_status.motors[i].moving);
            if (this->motor_status.motors[i].moving) {
                moving = true;
                break;
            }

            if (this->motor_status.motors[i].present_position < this->init_motor_pos[i]-0.1 || this->motor_status.motors[i].present_position > this->init_motor_pos[i]+0.1) {
                pos_reached = false;
            }
        }
        // if (moving && this->init_state == 0) {
        //     this->init_state = 1;
        //     RCLCPP_INFO(this->get_logger(),"Init state 1");
        // }

        if (pos_reached) {
            this->init_state = 1;
            RCLCPP_INFO(this->get_logger(),"init state 1");
        }
        

        if (!moving && this->init_state == 1) {
            this->init_flag = false;
            this->shoulder_velocity_publisher_->publish(this->set_velocities_msg);
            this->upperarm_velocity_publisher_->publish(this->set_velocities_msg);
            this->lowerarm_velocity_publisher_->publish(this->set_velocities_msg);
            RCLCPP_INFO(this->get_logger(),"init flag false");
        }
    }
    else {
        double max_vel = 0.0;
        double motor_vel = 0.0;
        int m_cntr = 0;
        for (int i = 0; i < 7; i++) {
            motor_vel = this->goal_joint_velocity.velocity[i]*this->jnt_to_motor_scaler[m_cntr];
            this->set_velocities_msg.motor_goal_velocities[m_cntr].velocity = motor_vel;
            if (motor_vel >this->max_motor_vel && motor_vel > max_vel) {
                max_vel = motor_vel;
            }
            m_cntr++;
            if (i != 5) {
                motor_vel = this->goal_joint_velocity.velocity[i]*this->jnt_to_motor_scaler[m_cntr];
                this->set_velocities_msg.motor_goal_velocities[m_cntr].velocity = motor_vel;
                if (motor_vel >this->max_motor_vel && motor_vel > max_vel) {
                    max_vel = motor_vel;
                }
                m_cntr++;
            }
        }
        // this->joint_state_msg.position += this->goal_joint_velocity.velocity;
        // joint_goal_publisher_->publish(this->joint_state_msg);
        if (max_vel > this->max_motor_vel) {
            for (int i = 0; i < 13; i++) {
                this->set_velocities_msg.motor_goal_velocities[i].velocity = this->set_velocities_msg.motor_goal_velocities[i].velocity * this->max_motor_vel / max_vel;
            }
        }

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
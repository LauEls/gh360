// #include <memory>
// #include <string>
#include "gh360/motor_handler.hpp"


gh360::MotorHandler::MotorHandler()
: Node("motor_handler")
{
    RCLCPP_INFO(this->get_logger(), "Run motor handler node");
    this->joint_states_recieved = false;

    this->declare_parameter("port_name", "default");
    std::string port_name_string = get_parameter("port_name").as_string();
    const char* port_name = port_name_string.c_str();
    this->declare_parameter("baud_rate", 0);
    int baud_rate = get_parameter("baud_rate").as_int();
    this->declare_parameter("protocol", 0);
    this->protocol = get_parameter("protocol").as_int();
    this->declare_parameter("start_with_torque", true);
    this->torque_start = get_parameter("start_with_torque").as_bool();

    this->dxl_handler = new DynamixelHandler(port_name, baud_rate);
    this->joints = get_robot_joints(this);
    RCLCPP_INFO(this->get_logger(), "Number of joints: %d", this->joints.size());

    // this->declare_parameter("joint_names", std::vector<std::string>());
    // this->joint_names = get_parameter("joint_names").as_string_array();

    MotorDictionary* motor_model_type = nullptr;
    this->multi_motor_models = false;
    this->require_encoder_data = false;
    this->motors_initiated = true;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        this->declare_parameter(this->joints[i]->get_joint_name()+".initialize", true);
        this->motors_initiated = !(get_parameter(this->joints[i]->get_joint_name()+".initialize").as_bool());
        RCLCPP_INFO(this->get_logger(), "Got parameter %s", this->joints[i]->get_joint_name().c_str());

        for (int j=0; j< this->joints[i]->get_motor_cnt(); j++)
        {
            Motor * motor = this->joints[i]->get_motor(j);

            motor->set_motor_model(this->dxl_handler->getMotorModel(motor->get_motor_id()));
            if (motor_model_type == nullptr) motor_model_type = motor->get_motor_model();
            else if (typeid(motor->get_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;
        }

        RCLCPP_INFO(this->get_logger(), "set motor model");

        if (this->joints[i]->get_motor_cnt() == 2) 
        {
            this->dxl_handler->setOperatingMode(this->joints[i], 4);
            this->require_encoder_data = true;
        }
        else this->dxl_handler->setOperatingMode(this->joints[i], 3);

        if (this->torque_start) this->dxl_handler->setTorqueEnable(this->joints[i], 1);
    }

    if (!this->multi_motor_models) this->joints_motor_model = motor_model_type;
    else RCLCPP_ERROR(this->get_logger(), "Current implementation doesn't allow for different motor models to be used on the same port!");

    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Position);
    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Velocity);
    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Current);
    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Temperature);

    
    this->init_state = 0;
    std::string ns = this->get_namespace();
    size_t slash_pos = ns.find('/', 1);
    if (slash_pos != std::string::npos) ns = ns.substr(0, slash_pos);
    RCLCPP_INFO(this->get_logger(), "Namespace: %s", ns.c_str());
    this->encoder_subscriber_ = this->create_subscription<gh360_interfaces::msg::ArmEncoderStates>(ns+"/encoder_states", 10, std::bind(&gh360::MotorHandler::encoder_callback, this, std::placeholders::_1));
    this->motor_state_publisher_ = this->create_publisher<gh360_interfaces::msg::PortStatus>(ns+"/motor_states", 10);
    this->timer_ = this->create_wall_timer(100ms, std::bind(&gh360::MotorHandler::timer_callback, this));

    this->position_step_service_ = this->create_service<gh360_interfaces::srv::MotorPositionStep>("motor_positions_step", std::bind(&gh360::MotorHandler::position_step_callback, this, std::placeholders::_1, std::placeholders::_2));
    // this->velocity_step_service_ = this->create_service<gh360_interfaces::srv::MotorVelocityStep>("motor_velocities_step", std::bind(&gh360::MotorHandler::velocity_step_callback, this, std::placeholders::_1, std::placeholders::_2));
    // this->delta_position_step_service_ = this->create_service<gh360_interfaces::srv::MotorPositionStep>("motor_delta_positions_step", std::bind(&gh360::MotorHandler::delta_position_step_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    this->set_torque_service_ = this->create_service<std_srvs::srv::SetBool>("motor_set_torque", std::bind(&gh360::MotorHandler::set_torque_callback, this, std::placeholders::_1, std::placeholders::_2));
    this->move_home_service_ = this->create_service<std_srvs::srv::SetBool>("motor_move_home", std::bind(&gh360::MotorHandler::move_home_callback, this, std::placeholders::_1, std::placeholders::_2));
    this->set_robot_limits_service_ = this->create_service<gh360_interfaces::srv::SetRobotLimits>("set_robot_limits", std::bind(&gh360::MotorHandler::set_robot_limits_callback, this, std::placeholders::_1, std::placeholders::_2));

    this->motor_goal_positions_subscriber_ = this->create_subscription<gh360_interfaces::msg::SetMotorPositions>(ns+"/motor_goal_position", 10, std::bind(&gh360::MotorHandler::motor_goal_positions_callback, this, std::placeholders::_1));
    this->motor_goal_currents_subscriber_ = this->create_subscription<gh360_interfaces::msg::SetMotorCurrents>(ns+"/motor_goal_current", 10, std::bind(&gh360::MotorHandler::motor_goal_current_callback, this, std::placeholders::_1));
    this->motor_goal_velocities_subscriber_ = this->create_subscription<gh360_interfaces::msg::SetMotorVelocities>(ns+"/motor_goal_velocity", 10, std::bind(&gh360::MotorHandler::motor_goal_velocity_callback, this, std::placeholders::_1));
 
    this->move_home_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(ns+"/move_home", 10, std::bind(&gh360::MotorHandler::move_home_sub_callback, this, std::placeholders::_1));
    this->set_torque_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(ns+"/set_torque", 10, std::bind(&gh360::MotorHandler::set_torque_sub_callback, this, std::placeholders::_1));
}



gh360::MotorHandler::~MotorHandler()
{ 
    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        this->dxl_handler->setTorqueEnable(this->joints[i], 0);
    }
}

bool gh360::MotorHandler::initMotorPositions()
{   
    if (this->init_state == 0)
    {
        if (!(this->joint_states_recieved) && this->require_encoder_data) RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for joint encoder data...");
        else this->init_state = 1;

        return false;
    }
    else if (this->init_state == 1)
    {
        double present_position, closest_pos, closest_pos_compare, reference_pos, closest_pos_middle, init_pos;   
        int rotations;
        RCLCPP_INFO(this->get_logger(), "Moving to closest init position");
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            this->dxl_handler->setVelocityProfile(this->joints[i], 10.0);
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                present_position = this->joints[i]->get_motor(j)->get_present_position_adjusted();
                init_pos = this->joints[i]->get_motor_init_pos();
                rotations = trunc((present_position-init_pos)/(M_PI*2));
                closest_pos_middle = init_pos + rotations*M_PI*2;

                if (closest_pos_middle < present_position) closest_pos_compare = init_pos + (rotations+1)*M_PI*2;
                else closest_pos_compare = init_pos + (rotations-1)*M_PI*2;

                if (abs(present_position-closest_pos_middle) < abs(present_position-closest_pos_compare)) 
                {
                    closest_pos = closest_pos_middle;
                    reference_pos = closest_pos_compare;
                }
                else
                {
                    closest_pos = closest_pos_compare;
                    reference_pos = closest_pos_middle;
                }

                this->joints[i]->get_motor(j)->set_reference_position_adjusted(reference_pos);
                this->joints[i]->get_motor(j)->set_goal_position_adjusted(closest_pos);
            }
        }

        this->init_state = 2;
    }
    else if (this->init_state == 2)
    {
        bool positions_reached = this->initMovementCheck();

        if (positions_reached) 
        {
            // sleep(1);
            bool current_check = true;
            bool angle_check = true;
            for (unsigned int i=0; i < this->joints.size(); i++)
            {
                for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
                {
                    if (abs(this->joints[i]->get_motor(j)->get_present_current_adjusted()) > 500) current_check = false;
                }
                if (abs(this->joints[i]->get_joint_angle()) > 0.5) angle_check = false;
            }

            if (current_check && angle_check) this->init_state = 7;
            else this->init_state = 3;
        } 
    }
    else if (this->init_state == 3)
    {   
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                RCLCPP_INFO(this->get_logger(), "Ajusting cocontraction: "+soft_joint->get_joint_name());

                Motor * right_motor = soft_joint->get_motor(soft_joint->RIGHT);
                Motor * left_motor = soft_joint->get_motor(soft_joint->LEFT);

                right_motor->set_reference_current(right_motor->get_present_current());
                left_motor->set_reference_current(left_motor->get_present_current());
                right_motor->set_reference_position(right_motor->get_present_position());
                left_motor->set_reference_position(left_motor->get_present_position());

                if (abs(right_motor->get_present_current()) > abs(left_motor->get_present_current()))
                {
                    if (right_motor->get_present_current_adjusted() > 0.0) right_motor->set_goal_position(right_motor->get_goal_position() - M_PI*2);
                    else right_motor->set_goal_position(right_motor->get_goal_position() + M_PI*2);
                    RCLCPP_INFO(this->get_logger(), "Moving right motor to: %f", right_motor->get_goal_position_adjusted());
                   
                }
                else 
                {
                    if (left_motor->get_present_current_adjusted() > 0.0) left_motor->set_goal_position(left_motor->get_goal_position() - M_PI*2);
                    else left_motor->set_goal_position(left_motor->get_goal_position() + M_PI*2);
                    RCLCPP_INFO(this->get_logger(), "Moving left motor to: %f", left_motor->get_goal_position_adjusted());
                }
            }
        }

        this->init_state = 4;
    }
    else if (this->init_state == 4)
    {
        bool reference_current = true;
        bool reference_joint_angle = false;
        bool positions_reached = this->initMovementCheck(reference_current, reference_joint_angle);

        if (positions_reached) 
        {
            this->init_state = 5;
            sleep(1);
        }
    }
    else if (this->init_state == 5)
    {
        double present_joint_angle, goal_adjustment;

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                RCLCPP_INFO(this->get_logger(), "Finding base equilibrium point position: "+soft_joint->get_joint_name());

                Motor * right_motor = soft_joint->get_motor(soft_joint->RIGHT);
                Motor * left_motor = soft_joint->get_motor(soft_joint->LEFT);

                right_motor->set_reference_position(right_motor->get_present_position());
                left_motor->set_reference_position(left_motor->get_present_position());
                present_joint_angle = soft_joint->get_joint_angle();
                soft_joint->set_reference_joint_angle(present_joint_angle);

                if (present_joint_angle > 0.0) goal_adjustment = -M_PI*2;
                else if (present_joint_angle < 0.0) goal_adjustment = M_PI*2;
                right_motor->set_goal_position_adjusted(right_motor->get_present_position_adjusted() + goal_adjustment);
                left_motor->set_goal_position_adjusted(left_motor->get_present_position_adjusted() + goal_adjustment);
            }
        }

        this->init_state = 6;

    }
    else if (this->init_state == 6)
    {
        bool reference_current = false;
        bool reference_joint_angle = true;
        bool positions_reached = this->initMovementCheck(reference_current, reference_joint_angle);

        if (positions_reached) 
        {
            this->init_state = 7;
            sleep(1);
        }
    }
    else if (this->init_state == 7)
    {
        double new_offset;

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                for (int j=0; j<soft_joint->get_motor_cnt(); j++)
                {
                    new_offset = soft_joint->get_motor(j)->get_offset() + soft_joint->get_motor(j)->get_present_position_adjusted() - soft_joint->get_motor_init_pos();
                    soft_joint->get_motor(j)->set_offset(new_offset);
                }
            }
        }

        return true;
    }
    
    return false;
}

bool gh360::MotorHandler::initMovementCheck(bool reference_current/*=false*/, bool reference_joint_angle/*=false*/)
{
    bool positions_reached = true;

    double present_current_sum;
    double reference_current_sum;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            std::vector<bool> swap_to_reference = {false, false};
            Motor * right_motor = soft_joint->get_motor(soft_joint->RIGHT);
            Motor * left_motor = soft_joint->get_motor(soft_joint->LEFT);

            //Check current limit
            if (abs(right_motor->get_present_current_adjusted() - left_motor->get_present_current_adjusted()) > 1000)
            {
                if (abs(right_motor->get_present_current()) > abs(left_motor->get_present_current())) swap_to_reference[soft_joint->RIGHT] = true;
                else swap_to_reference[soft_joint->LEFT] = true;
            }

            //Check if max angle is reached
            else if (soft_joint->get_joint_angle() >= soft_joint->get_max_joint_angle())
            {      
                if (right_motor->get_goal_position() > right_motor->get_present_position()) swap_to_reference[soft_joint->RIGHT] = true;
                if (left_motor->get_goal_position() > left_motor->get_present_position()) swap_to_reference[soft_joint->LEFT] = true;
                RCLCPP_INFO(this->get_logger(), "Joint reached max joint angle. Moving to reference position");
            }

            //Check if min angle is reached
            else if ((soft_joint->get_joint_angle() <= soft_joint->get_min_joint_angle()) && (soft_joint->get_joint_name() != "shoulder_pitch"))
            {
                if (right_motor->get_goal_position() < right_motor->get_present_position()) swap_to_reference[soft_joint->RIGHT] = true;
                if (left_motor->get_goal_position() < left_motor->get_present_position()) swap_to_reference[soft_joint->LEFT] = true;
                RCLCPP_INFO(this->get_logger(), "Joint reached min joint angle. Moving to reference position");
            }

            //If true check summed reference currents
            else if (reference_current)
            {
                present_current_sum = abs(right_motor->get_present_current()) + abs(left_motor->get_present_current());
                reference_current_sum = abs(right_motor->get_reference_current()) + abs(left_motor->get_reference_current());

                if (present_current_sum > reference_current_sum)
                {
                    swap_to_reference[soft_joint->RIGHT] = true;
                    swap_to_reference[soft_joint->LEFT] = true;
                    RCLCPP_INFO(this->get_logger(), "Motor exceeded absolute reference current. Moving to reference position");
                }
            }

            //If true check reference_joint_angle
            else if (reference_joint_angle)
            {
                if (abs(soft_joint->get_joint_angle()) > abs(soft_joint->get_reference_joint_angle()))
                {
                    swap_to_reference[soft_joint->RIGHT] = true;
                    swap_to_reference[soft_joint->LEFT] = true;
                    RCLCPP_INFO(this->get_logger(), "Joint exceeded absolute reference joint angle. Moving to reference motor position");
                }
            }

            if (swap_to_reference[soft_joint->RIGHT]) right_motor->set_goal_position_adjusted(right_motor->get_reference_position_adjusted());
            if (swap_to_reference[soft_joint->LEFT]) left_motor->set_goal_position_adjusted(left_motor->get_reference_position_adjusted());

            //Check goal reached
            if ((!(soft_joint->get_motor(soft_joint->RIGHT)->goal_position_reached())) || (!(soft_joint->get_motor(soft_joint->LEFT)->goal_position_reached()))) 
            {
                positions_reached = false;
            }
        }
    }

    return positions_reached;
}

template <typename T>
void gh360::MotorHandler::setMotorGoal(std::vector<T> motor_goal_msg)
{
    for (unsigned int m=0; m < motor_goal_msg.size(); m++)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            for (int j=0; j < this->joints[i]->get_motor_cnt(); j++)
            {
                Motor * motor = this->joints[i]->get_motor(j);
                if (motor->get_motor_id() == motor_goal_msg[m].id) 
                {
                    motor->set_motor_goal_adjusted(motor_goal_msg[m]);
                }
            }
        }
    }
}

gh360_interfaces::msg::PortStatus gh360::MotorHandler::getMotorStates()
{
    gh360_interfaces::msg::PortStatus port_status_msg = gh360_interfaces::msg::PortStatus();
    gh360_interfaces::msg::MotorStatus motor_status_msg;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        for (int j=0; j< this->joints[i]->get_motor_cnt(); j++)
        {
            Motor * motor = this->joints[i]->get_motor(j);
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = motor->get_motor_id();
            motor_status_msg.present_position = motor->get_present_position_adjusted();
            motor_status_msg.present_velocity = motor->get_present_velocity_adjusted();
            motor_status_msg.present_current = motor->get_present_current_adjusted();
            motor_status_msg.present_temperature = motor->get_present_temperature();
            motor_status_msg.safety_check = motor->get_safety_check();
            motor_status_msg.moving = motor->get_moving();
            port_status_msg.motors.push_back(motor_status_msg);
        }
    }

    return port_status_msg;
}

bool gh360::MotorHandler::safetyCheck()
{
    bool safety_check = true;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
        {
            Motor * motor = this->joints[i]->get_motor(j);
            if (abs(motor->get_present_current()) > motor->get_motor_model()->CURRENT_LIMIT)
            {
                this->dxl_handler->setTorqueEnable(this->joints[i], 0);
                motor->set_safety_check(false);
                safety_check = false;
            }
            else
            {
                motor->set_safety_check(true);
            }
        }
    }

    return safety_check;
}

void gh360::MotorHandler::check_goal_alive()
{
    auto current_time = std::chrono::high_resolution_clock::now();
    auto velocity_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - this->velocity_goal_timestamp);
    auto current_duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - this->current_goal_timestamp);
    bool goal_alive = true;

    if (this->joints[0]->get_operating_mode() == this->joints[0]->get_velocity_mode_id())
    {
        if (velocity_duration.count() > 200) goal_alive = false;
    }
    else if (this->joints[0]->get_operating_mode() == this->joints[0]->get_current_mode_id())
    {
        if (current_duration.count() > 200) goal_alive = false;
    }
    
    if (!goal_alive)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                Motor * motor = this->joints[i]->get_motor(j);
                motor->set_goal_velocity(0.0);
                motor->set_goal_current(0.0);
            }
        }
    }
}

void gh360::MotorHandler::check_limits()
{
    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        bool limit_reached = false;
        if (this->joints[i]->get_joint_angle() >= this->joints[i]->get_max_joint_angle())
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                Motor * motor = this->joints[i]->get_motor(j);
                if (motor->get_goal_position_adjusted() > motor->get_present_position_adjusted()) motor->set_goal_position(motor->get_present_position());
                if (motor->get_goal_velocity_adjusted() > 0.0) motor->set_goal_velocity(0.0);
                if (motor->get_goal_current_adjusted() > 0.0) motor->set_goal_current(0.0);
            }
        }
        else if (this->joints[i]->get_joint_angle() <= this->joints[i]->get_min_joint_angle())
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                Motor * motor = this->joints[i]->get_motor(j);
                if (motor->get_goal_position_adjusted() < motor->get_present_position_adjusted()) motor->set_goal_position(motor->get_present_position());
                if (motor->get_goal_velocity_adjusted() < 0.0) motor->set_goal_velocity(0.0);
                if (motor->get_goal_current_adjusted() < 0.0) motor->set_goal_current(0.0);
            }
        }
        for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
        {
            Motor * motor = this->joints[i]->get_motor(j);
            if (motor->get_present_current_adjusted() >= motor->get_max_current())
            {
                if (motor->get_goal_position_adjusted() > motor->get_present_position_adjusted()) limit_reached = true;
                if (motor->get_goal_velocity_adjusted() > 0.0) limit_reached = true;
                if (motor->get_goal_current_adjusted() > 0.0) limit_reached = true;
            }
            else if (motor->get_present_current() <= motor->get_min_current())
            {
                if (motor->get_goal_position_adjusted() < motor->get_present_position_adjusted()) limit_reached = true;
                if (motor->get_goal_velocity_adjusted() < 0.0) limit_reached = true;
                if (motor->get_goal_current_adjusted() < 0.0) limit_reached = true;
            }
        }

        if (limit_reached)
        {
            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                Motor * motor = this->joints[i]->get_motor(j);
                motor->set_goal_position(motor->get_present_position());
                motor->set_goal_velocity(0.0);
                motor->set_goal_current(0.0);
            }
        }
    }
}

void gh360::MotorHandler::motor_goal_positions_callback(const gh360_interfaces::msg::SetMotorPositions::SharedPtr msg)
{
    this->dxl_handler->setControlMode(this->joints, [this](Joint* joint) {
        return this->dxl_handler->getPositionModeID(joint);
    });

    this->setMotorGoal(msg->motor_goal_positions);
}

void gh360::MotorHandler::motor_goal_current_callback(const gh360_interfaces::msg::SetMotorCurrents::SharedPtr msg)
{
    this->dxl_handler->setControlMode(this->joints, [this](Joint* joint) {
        return this->dxl_handler->getCurrentModeID(joint);
    });

    this->current_goal_timestamp = std::chrono::high_resolution_clock::now();

    this->setMotorGoal(msg->motor_goal_currents);
}

void gh360::MotorHandler::motor_goal_velocity_callback(const gh360_interfaces::msg::SetMotorVelocities::SharedPtr msg)
{
    this->dxl_handler->setControlMode(this->joints, [this](Joint* joint) {
        return this->dxl_handler->getVelocityModeID(joint);
    });

    this->velocity_goal_timestamp = std::chrono::high_resolution_clock::now();

    this->setMotorGoal(msg->motor_goal_velocities);
}


void gh360::MotorHandler::encoder_callback(const gh360_interfaces::msg::ArmEncoderStates::SharedPtr msg)
{
    if (!(this->joint_states_recieved)) this->joint_states_recieved = true;

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

void gh360::MotorHandler::position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response)
{
    this->dxl_handler->setControlMode(this->joints, [this](Joint* joint) {
        return this->dxl_handler->getPositionModeID(joint);
    });
    this->setMotorGoal(request->motor_goal_positions);
    
    
    response->motor_status = this->getMotorStates().motors;

}

void gh360::MotorHandler::set_robot_limits_callback(const std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Request> request, std::shared_ptr<gh360_interfaces::srv::SetRobotLimits::Response> response)
{
    bool max_joint_angles_set = false;
    bool min_joint_angles_set = false;
    bool max_motor_currents_set = false;
    bool min_motor_currents_set = false;
    bool max_motor_velocities_set = false;
    bool min_motor_velocities_set = false;

    unsigned int motor_cnt = 0;
    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        motor_cnt += this->joints[i]->get_motor_cnt();
    }

    if (request->max_joint_angles.size() == this->joints.size()) max_joint_angles_set = true;
    if (request->min_joint_angles.size() == this->joints.size()) min_joint_angles_set = true;
    if (request->max_motor_currents.size() == motor_cnt) max_motor_currents_set = true;
    if (request->min_motor_currents.size() == motor_cnt) min_motor_currents_set = true;
    if (request->max_motor_velocities.size() == motor_cnt) max_motor_velocities_set = true;
    if (request->min_motor_velocities.size() == motor_cnt) min_motor_velocities_set = true;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (max_joint_angles_set) this->joints[i]->set_max_joint_angle(request->max_joint_angles[i]);
        if (min_joint_angles_set) this->joints[i]->set_min_joint_angle(request->min_joint_angles[i]);
        for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
        {
            if (max_motor_currents_set) this->joints[i]->get_motor(j)->set_max_current(request->max_motor_currents[i*this->joints[i]->get_motor_cnt()+j]);
            if (min_motor_currents_set) this->joints[i]->get_motor(j)->set_min_current(request->min_motor_currents[i*this->joints[i]->get_motor_cnt()+j]);
            if (max_motor_velocities_set) this->joints[i]->get_motor(j)->set_max_velocity(request->max_motor_velocities[i*this->joints[i]->get_motor_cnt()+j]);
            if (min_motor_velocities_set) this->joints[i]->get_motor(j)->set_min_velocity(request->min_motor_velocities[i*this->joints[i]->get_motor_cnt()+j]);
        }
    }

    response->success = true;
}

void gh360::MotorHandler::set_torque_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int set_torque;
    if (request->data == true)
    {
        set_torque = 1;
        this->dxl_handler->setEmergencyStop(false);
    }
    else
    {
        set_torque = 0;
        this->dxl_handler->setEmergencyStop(true);
    }

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        this->dxl_handler->setTorqueEnable(this->joints[i], set_torque);
    }

    RCLCPP_INFO(this->get_logger(), "Torque set to: %d", set_torque);

    response->success = true;
}

void gh360::MotorHandler::set_torque_sub_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    int set_torque;
    if (msg->data == true)
    {
        set_torque = 1;
        this->dxl_handler->setEmergencyStop(false);
    }
    else
    {
        set_torque = 0;
        this->dxl_handler->setEmergencyStop(true);
    }

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        this->dxl_handler->setTorqueEnable(this->joints[i], set_torque);
    }

    // RCLCPP_INFO(this->get_logger(), "Torque set to: %d", set_torque);
}

void gh360::MotorHandler::move_home_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    if (request->data == true)
    {
        double init_pos;
        this->dxl_handler->setEmergencyStop(false);
        this->dxl_handler->setControlMode(this->joints, [this](Joint* joint) {
            return this->dxl_handler->getPositionModeID(joint);
        });
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (!this->joints[i]->get_motor(0)->get_torque_enabled()) this->dxl_handler->setTorqueEnable(this->joints[i],1);

            this->dxl_handler->setVelocityProfile(this->joints[i], 10.0);
            init_pos = this->joints[i]->get_motor_init_pos();

            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                this->joints[i]->get_motor(j)->set_goal_position_adjusted(init_pos);
            }
        }
    }

    response->success = true;
}

void gh360::MotorHandler::move_home_sub_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data == true)
    {
        double init_pos;
        this->dxl_handler->setEmergencyStop(false);
        this->dxl_handler->setControlMode(this->joints, [this](Joint* joint) {
            return this->dxl_handler->getPositionModeID(joint);
        });
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (!this->joints[i]->get_motor(0)->get_torque_enabled()) this->dxl_handler->setTorqueEnable(this->joints[i],1);
            init_pos = this->joints[i]->get_motor_init_pos();

            for (int j=0; j<this->joints[i]->get_motor_cnt(); j++)
            {
                this->joints[i]->get_motor(j)->set_goal_position_adjusted(init_pos);
            }
        }
    }
}

void gh360::MotorHandler::timer_callback()
{
    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Position);
    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Velocity);
    this->dxl_handler->syncRead(this->joints, this->joints_motor_model->Present_Current);

    if (!(this->motors_initiated))
    {
        this->motors_initiated = this->initMotorPositions();
    }
    
    bool safety_check = this->safetyCheck();
    if (safety_check == true) 
    {
        this->check_goal_alive();
        this->check_limits();
        if (this->joints[0]->get_operating_mode() == this->joints[0]->get_position_mode_id())
        {
            this->dxl_handler->syncWrite(this->joints, this->joints_motor_model->Goal_Position);
        }
        else if (this->joints[0]->get_operating_mode() == this->joints[0]->get_velocity_mode_id())
        {
            this->dxl_handler->syncWrite(this->joints, this->joints_motor_model->Goal_Velocity);
        }
        else if (this->joints[0]->get_operating_mode() == this->joints[0]->get_current_mode_id())
        {
            this->dxl_handler->syncWrite(this->joints, this->joints_motor_model->Goal_Current);
        }

    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "The safety check was not successful!");
    }

    gh360_interfaces::msg::PortStatus port_status_msg = gh360_interfaces::msg::PortStatus();
    port_status_msg = this->getMotorStates();
    this->motor_state_publisher_->publish(port_status_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto motorhandlernode = std::make_shared<gh360::MotorHandler>();
    rclcpp::spin(motorhandlernode);
    rclcpp::shutdown();

    // motorhandlernode->~MotorHandler();

    return 0;
}
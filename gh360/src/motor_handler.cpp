#include <memory>
#include <string>
#include "motor_handler.hpp"


gh360::MotorHandler::MotorHandler()
: Node("motor_handler")
{
    RCLCPP_INFO(this->get_logger(), "Run motor handler node");
    this->joint_states_recieved = false;

    this->declare_parameter("port_name", "default");
    std::string port_name_string = get_parameter("port_name").as_string();
    // RCLCPP_INFO(this->get_logger(), "Port Name: %s", port_name_string.c_str());
    this->port_name = port_name_string.c_str();
    // RCLCPP_INFO(this->get_logger(), "Port Name: %s", this->port_name);
    this->declare_parameter("baud_rate", 0);
    this->baud_rate = get_parameter("baud_rate").as_int();
    this->declare_parameter("protocol", 0);
    this->protocol = get_parameter("protocol").as_int();

    this->openPortsAndSetBaudrate();

    std::vector<std::string> string_array;
    this->declare_parameter("joint_names", string_array);
    this->joint_names = get_parameter("joint_names").as_string_array();

    // std::type_info& motor_model_type;
    gh360::MotorDictionary* motor_model_type;
    this->multi_motor_models = false;
    for (unsigned int i = 0; i < this->joint_names.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "Joint Name: %s", this->joint_names[i].c_str());
        
        std::string joint_name = this->joint_names[i];
        this->declare_parameter(this->joint_names[i]+".joint_type", "default");
        std::string joint_type = get_parameter(this->joint_names[i]+".joint_type").as_string();

        this->declare_parameter(this->joint_names[i]+".min_joint_angle", 0.0);
        this->declare_parameter(this->joint_names[i]+".max_joint_angle", 0.0);

        if (joint_type == "soft_joint") {
            this->declare_parameter(joint_name+".right.motor_id", 0);
            this->declare_parameter(joint_name+".left.motor_id", 0);
            this->declare_parameter(joint_name+".right.movement_direction", 0);
            this->declare_parameter(joint_name+".left.movement_direction", 0);
            this->declare_parameter(joint_name+".right.offset", 0.0);
            this->declare_parameter(joint_name+".left.offset", 0.0);

            SoftJoint * new_joint = new SoftJoint();
            new_joint->set_joint_name(joint_name);
            new_joint->set_min_joint_angle(get_parameter(joint_name+".min_joint_angle").as_double());
            new_joint->set_max_joint_angle(get_parameter(joint_name+".max_joint_angle").as_double());
            new_joint->set_right_motor_id(get_parameter(joint_name+".right.motor_id").as_int());
            new_joint->set_left_motor_id(get_parameter(joint_name+".left.motor_id").as_int());
            new_joint->set_right_movement_direction(get_parameter(joint_name+".right.movement_direction").as_int());
            new_joint->set_left_movement_direction(get_parameter(joint_name+".left.movement_direction").as_int());
            new_joint->set_right_offset(get_parameter(joint_name+".right.offset").as_double());
            new_joint->set_left_offset(get_parameter(joint_name+".left.offset").as_double());
            new_joint->set_right_motor_model(this->getMotorModel(new_joint->get_right_motor_id()));
            new_joint->set_left_motor_model(this->getMotorModel(new_joint->get_left_motor_id()));
            this->setOperatingMode(new_joint, 4);
            this->setTorqueEnable(new_joint, 1);
            this->joints.push_back(new_joint);

            if (i == 0) motor_model_type = new_joint->get_right_motor_model();
            else if (typeid(new_joint->get_right_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;

            if (typeid(new_joint->get_left_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;
        }
        else {
            this->declare_parameter(joint_name+".motor_id", 0);
            this->declare_parameter(joint_name+".movement_direction", 0);
            this->declare_parameter(joint_name+".offset", 0.0);

            MotorJoint * new_joint = new MotorJoint(); 
            new_joint->set_joint_name(joint_name);
            new_joint->set_min_joint_angle(get_parameter(joint_name+".min_joint_angle").as_double());
            new_joint->set_max_joint_angle(get_parameter(joint_name+".max_joint_angle").as_double());
            new_joint->set_motor_id(get_parameter(joint_name+".motor_id").as_int());
            new_joint->set_movement_direction(get_parameter(joint_name+".movement_direction").as_int());
            new_joint->set_offset(get_parameter(joint_name+".offset").as_double());
            new_joint->set_motor_model(this->getMotorModel(new_joint->get_motor_id()));
            this->setOperatingMode(new_joint, 3);
            this->setTorqueEnable(new_joint, 1);
            this->joints.push_back(new_joint);

            if (i == 0) motor_model_type = new_joint->get_motor_model();
            else if (typeid(new_joint->get_motor_model())!= typeid(motor_model_type)) this->multi_motor_models = true;
        }

    }

    if (!this->multi_motor_models) this->joints_motor_model = motor_model_type;

    this->syncRead(this->joints_motor_model->Present_Position.size, this->joints_motor_model->Present_Position.address);
    this->syncRead(this->joints_motor_model->Present_Velocity.size, this->joints_motor_model->Present_Velocity.address);
    this->syncRead(this->joints_motor_model->Present_Current.size, this->joints_motor_model->Present_Current.address);
    this->syncRead(this->joints_motor_model->Present_Temperature.size, this->joints_motor_model->Present_Temperature.address);

    this->encoder_subscriber_ = this->create_subscription<gh360_interfaces::msg::ArmEncoderStates>("/encoder_status", 10, std::bind(&gh360::MotorHandler::encoder_callback, this, std::placeholders::_1));

    this->motors_initiated = false;
    this->init_state = 0;
    // this->init_reference_current = std::vector<double>(this->joints.size());
    // this->init_reference_position = std::vector<double>(this->joints.size());
    this->init_motor_side = std::vector<int>(this->joints.size());

    this->motor_state_publisher_ = this->create_publisher<gh360_interfaces::msg::PortStatus>("motor_status", 10);
    this->timer_ = this->create_wall_timer(100ms, std::bind(&gh360::MotorHandler::timer_callback, this));

    this->position_step_service_ = this->create_service<gh360_interfaces::srv::MotorPositionStep>("motor_positions_step", std::bind(&gh360::MotorHandler::position_step_callback, this, std::placeholders::_1, std::placeholders::_2));
    this->delta_position_step_service_ = this->create_service<gh360_interfaces::srv::MotorPositionStep>("motor_delta_positions_step", std::bind(&gh360::MotorHandler::delta_position_step_callback, this, std::placeholders::_1, std::placeholders::_2));
    this->set_torque_service_ = this->create_service<std_srvs::srv::SetBool>("motor_set_torque", std::bind(&gh360::MotorHandler::set_torque_callback, this, std::placeholders::_1, std::placeholders::_2));   

   
}



gh360::MotorHandler::~MotorHandler()
{
    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        this->setTorqueEnable(this->joints[i], 0);
    }
}

bool gh360::MotorHandler::initMotorPositions()
{
    //FOR GOAL POSITIONS CHECK ALSO CHECK VELOCITY!!!!!!!!!!!!!!!!!!!!!!!!
    
    // RCLCPP_INFO_ONCE(this->get_logger(), "init_state: %i", this->init_state);

    if (this->init_state == 0)
    {
    // while (!(this->joint_states_recieved))
    // {
        if (!(this->joint_states_recieved))
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for joint encoder data...");
            // RCLCPP_INFO(this->get_logger(), "Joint States Recieved: %s", this->joint_states_recieved ? "true" : "false");
        // this->spin_once();
        // sleep(1);
        }
        else
        {
            this->init_state = 1;
            // return true;
        }

        return false;
    }
    else if (this->init_state == 1)
    {
        // RCLCPP_INFO(this->get_logger(), "Starting Motor Calibration!");
        double right_present_position, left_present_position, right_closest_pos, left_closest_pos; 
        //left_present_current, right_present_current,
        
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() != 13) continue;
                RCLCPP_INFO(this->get_logger(), "Starting Motor Calibration!");
                this->setVelocityProfile(soft_joint, 10.0);

                right_present_position = soft_joint->get_right_motor_present_position();
                // right_present_current = soft_joint->get_right_motor_present_current();

                left_present_position = soft_joint->get_left_motor_present_position();
                // left_present_current = soft_joint->get_left_motor_present_current();

                right_closest_pos = round(right_present_position/(M_PI*2))*(M_PI*2);
                left_closest_pos = round(left_present_position/(M_PI*2))*(M_PI*2);

                RCLCPP_INFO(this->get_logger(), "present right pos: %f", right_present_position);
                RCLCPP_INFO(this->get_logger(), "closest right pos: %f", right_closest_pos);
                RCLCPP_INFO(this->get_logger(), "present left pos: %f", left_present_position);
                RCLCPP_INFO(this->get_logger(), "closest left pos: %f", left_closest_pos);

                soft_joint->set_right_motor_goal_position(right_closest_pos);
                soft_joint->set_left_motor_goal_position(left_closest_pos);

                RCLCPP_INFO(this->get_logger(), "Set Goal Pos");
            }
        }

        this->init_state = 2;
    }
    else if (this->init_state == 2)
    {
        bool positions_reached = this->initMovementCheck();

        // for (unsigned int i=0; i < this->joints.size(); i++)
        // {
        //     if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        //     {
        //         if (soft_joint->get_right_motor_id() != 13) continue;

        //         RCLCPP_INFO(this->get_logger(), "Waiting to reach goal pos");

        //         //CHECK IF CURRENT LIMIT HAS BEEN VIOLATED IF SO, MOVE THAT MOTOR TO THE OTHER 2*PI POSITION

        //         if (!(soft_joint->right_motor_goal_reached())) 
        //         {
        //             positions_reached = false;
        //         }
        //         else if (!(soft_joint->left_motor_goal_reached()))
        //         {
        //             positions_reached = false;
        //         }
                
        //     }
        // }

        if (positions_reached) 
        {
            sleep(1);
            this->init_state = 3;
        } 
    }
    else if (this->init_state == 3)
    {
        double right_present_position, right_present_current, left_present_position, left_present_current;
        
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() != 13) continue;

                RCLCPP_INFO(this->get_logger(), "Set Goal Pos 2");

                right_present_position = soft_joint->get_right_motor_present_position();
                left_present_position = soft_joint->get_left_motor_present_position();
                right_present_current = soft_joint->get_right_motor_present_current();
                left_present_current = soft_joint->get_left_motor_present_current();

                RCLCPP_INFO(this->get_logger(), "present right pos: %f", right_present_position);
                RCLCPP_INFO(this->get_logger(), "present right current: %f", right_present_current);
                RCLCPP_INFO(this->get_logger(), "present left pos: %f", left_present_position);
                RCLCPP_INFO(this->get_logger(), "present left current: %f", left_present_current);

                soft_joint->set_right_reference_current(right_present_current);
                soft_joint->set_right_reference_position(right_present_position);
                soft_joint->set_left_reference_current(left_present_current);
                soft_joint->set_left_reference_position(left_present_position);

                if (abs(right_present_current) > abs(left_present_current))
                {
                    // soft_joint->set_right_reference_current(right_present_current);
                    // this->init_reference_current[i] = right_present_current;
                    // this->init_reference_position[i] = right_present_position;
                    
                    // this->init_motor_side[i] = 0;
                    if (right_present_current > 0.0)
                    {
                        soft_joint->set_right_motor_goal_position(right_present_position - M_PI*2);
                    }
                    else 
                    {
                        soft_joint->set_right_motor_goal_position(right_present_position + M_PI*2);
                    }
                    RCLCPP_INFO(this->get_logger(), "Moving right motor to: %f", soft_joint->get_right_motor_goal_position());
                   
                }
                else 
                {
                    // soft_joint->set_left_reference_current(left_present_current);
                    // this->init_reference_current[i] = left_present_current;
                    // this->init_reference_position[i] = left_present_position;
                    // soft_joint->set_left_reference_current(left_present_current);
                    // soft_joint->set_left_reference_position(left_present_position);
                    // this->init_motor_side[i] = 1;
                    if (left_present_current > 0.0)
                    {
                        soft_joint->set_left_motor_goal_position(left_present_position - M_PI*2);
                    }
                    else 
                    {
                        soft_joint->set_left_motor_goal_position(left_present_position + M_PI*2);
                    }
                    RCLCPP_INFO(this->get_logger(), "Moving left motor to: %f", soft_joint->get_left_motor_goal_position());
                }
            }
        }
        
        // for (unsigned int k=0; k<this->init_reference_position.size(); k++)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Right reference position: %f", this->init_reference_position[k]);
        // }
        

        this->init_state = 4;
    }
    else if (this->init_state == 4)
    {
        bool reference_current = false;
        bool reference_joint_angle = false;
        bool positions_reached = this->initMovementCheck(reference_current, reference_joint_angle);

        // for (unsigned int k=0; k<this->init_reference_position.size(); k++)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Right reference position: %f", this->init_reference_position[k]);
        // }
        // copy(reference_position.begin(), reference_position.end(), std::ostream_iterator<double>(std::cout, " "));

        // for (unsigned int i=0; i < this->joints.size(); i++)
        // {
        //     if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        //     {
        //         if (soft_joint->get_right_motor_id() != 13) continue;

        //         RCLCPP_INFO(this->get_logger(), "Waiting to reach goal pos 2");

        //         if (this->init_motor_side[i] == 0)
        //         {
        //             //if present current is bigger than reference current move back to reference position

        //             if (!(soft_joint->right_motor_goal_reached())) 
        //             {
        //                 positions_reached = false;
        //             }
        //             // if (soft_joint->get_right_motor_present_current() > this->init_reference_current[i])
        //             if (abs(soft_joint->get_right_motor_present_current()) > abs(soft_joint->get_right_reference_current()))
        //             {
        //                 // RCLCPP_INFO(this->get_logger(), "Right current higher than reference current. Moving back to: %f", this->init_reference_position[i]);
        //                 // soft_joint->set_right_motor_goal_position(this->init_reference_position[i]);

        //                 RCLCPP_INFO(this->get_logger(), "Right current higher than reference current. Moving back to: %f", soft_joint->get_right_reference_position());
        //                 soft_joint->set_right_motor_goal_position(soft_joint->get_right_reference_position());
        //             }
        //         }
        //         else if (this->init_motor_side[i] == 1)
        //         {
        //             //if present current is bigger than reference current move back to reference position

        //             if (!(soft_joint->left_motor_goal_reached()))
        //             {
        //                 positions_reached = false;
        //             }
        //             // if (soft_joint->get_left_motor_present_current() > this->init_reference_current[i])
        //             if (abs(soft_joint->get_left_motor_present_current()) > abs(soft_joint->get_right_reference_current()))
        //             {
        //                 // soft_joint->set_left_motor_goal_position(this->init_reference_position[i]);
        //                 // RCLCPP_INFO(this->get_logger(), "Left current higher than reference current. Moving back to: %f", this->init_reference_position[i]);

        //                 RCLCPP_INFO(this->get_logger(), "Left current higher than reference current. Moving back to: %f", soft_joint->get_left_reference_position());
        //                 soft_joint->set_right_motor_goal_position(soft_joint->get_left_reference_position());
        //             }
        //         }
        //     }
        // }

        if (positions_reached) this->init_state = 5;
    }
    else if (this->init_state == 5)
    {
        //------------------------
        //EQ POSITION CALIBARTION
        //------------------------
        RCLCPP_INFO(this->get_logger(), "Finding base equilibrium point position...");
        double right_present_position, left_present_position, present_joint_angle;
        //  left_present_current, right_present_current,

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() != 13) continue;

                right_present_position = soft_joint->get_right_motor_present_position();
                left_present_position = soft_joint->get_left_motor_present_position();
                // right_present_current = soft_joint->get_right_motor_present_current();
                // left_present_current = soft_joint->get_left_motor_present_current();
                present_joint_angle = soft_joint->get_joint_angle();

                soft_joint->set_right_reference_position(right_present_position);
                soft_joint->set_left_reference_position(left_present_position);
                soft_joint->set_reference_joint_angle(present_joint_angle);

                if (present_joint_angle > 0.0)
                {
                    soft_joint->set_right_motor_goal_position(right_present_position - M_PI*2);
                    soft_joint->set_left_motor_goal_position(left_present_position - M_PI*2);

                }
                else if (present_joint_angle < 0.0)
                {
                    soft_joint->set_right_motor_goal_position(right_present_position + M_PI*2);
                    soft_joint->set_left_motor_goal_position(left_present_position + M_PI*2);

                }
            }
        }
        //check if joint angle is bigger or smaller than 0
        // if bigger move to current pos - 2*pi
        // if smaller move to current pos + 2*pi

        //while loop as long as goal pos not reached and safety check not violated
            //if movement direction is positive
                //if joint angle is smaller than min or bigger than max joint angle move back to reference position
                //if joint angle is further away from 0 then reference angle move back to reference motor position

        
        //set current motor position as 0 - reference offset.
        this->init_state = 6;

    }
    else if (this->init_state == 6)
    {
        bool positions_reached = true;

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() != 13) continue;

                RCLCPP_INFO(this->get_logger(), "Waiting to reach goal pos 3");

                if ((!(soft_joint->right_motor_goal_reached())) || (!(soft_joint->left_motor_goal_reached()))) 
                {
                    positions_reached = false;
                }
                // if (soft_joint->get_right_motor_present_current() > this->init_reference_current[i])
                if (abs(soft_joint->get_joint_angle()) > abs(soft_joint->get_reference_joint_angle()))
                {
                    RCLCPP_INFO(this->get_logger(), "Present joing angle further away from 0 than reference angle. Moving back.");
                    // soft_joint->set_right_motor_goal_position(this->init_reference_position[i]);

                    // RCLCPP_INFO(this->get_logger(), "Right current higher than reference current. Moving back to: %f", soft_joint->get_right_reference_position());
                    soft_joint->set_right_motor_goal_position(soft_joint->get_right_reference_position());
                    soft_joint->set_left_motor_goal_position(soft_joint->get_left_reference_position());
                }
            }
        }

        if (positions_reached) return true;
    }
    
    return false;
}

void gh360::MotorHandler::set_torque_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    int set_torque;
    if (request->data == true)
    {
        set_torque = 1;
    }
    else
    {
        set_torque = 0;
    }

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        this->setTorqueEnable(this->joints[i], set_torque);
    }

    response->success = true;
}

void gh360::MotorHandler::position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response)
{

    this->setMotorGoalPositions(request->motor_goal_positions);

    response->motor_status = this->getMotorStatus().motors;

}

void gh360::MotorHandler::delta_position_step_callback(const std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Request> request, std::shared_ptr<gh360_interfaces::srv::MotorPositionStep::Response> response)
{
    
    this->setDeltaMotorGoalPositions(request->motor_goal_positions);

    response->motor_status = this->getMotorStatus().motors;

}

void gh360::MotorHandler::motor_goal_positions_callback(const gh360_interfaces::msg::SetMotorPositions::SharedPtr msg)
{
    this->setMotorGoalPositions(msg->motor_goal_positions);
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

void gh360::MotorHandler::timer_callback()
{
    // RCLCPP_INFO(this->get_logger(), "timer_callback...");

    if (!(this->motors_initiated))
    {
        // RCLCPP_INFO(this->get_logger(), "Initiating Motors");
        this->motors_initiated = this->initMotorPositions();
    }
    

    this->syncRead(this->joints_motor_model->Present_Position.size, this->joints_motor_model->Present_Position.address);
    this->syncRead(this->joints_motor_model->Present_Velocity.size, this->joints_motor_model->Present_Velocity.address);
    this->syncRead(this->joints_motor_model->Present_Current.size, this->joints_motor_model->Present_Current.address);
    this->syncRead(this->joints_motor_model->Present_Temperature.size, this->joints_motor_model->Present_Temperature.address);
    bool safety_check = this->safetyCheck();
    if (safety_check == true) 
    {
        this->syncWrite(this->joints_motor_model->Goal_Position.size, this->joints_motor_model->Goal_Position.address);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "The safety check was not successful!");
    }

    gh360_interfaces::msg::PortStatus port_status_msg = gh360_interfaces::msg::PortStatus();
    port_status_msg = this->getMotorStatus();
    this->motor_state_publisher_->publish(port_status_msg);
}

bool gh360::MotorHandler::initMovementCheck(bool reference_current/*=false*/, bool reference_joint_angle/*=false*/)
{
    bool positions_reached = true;

    double present_current_sum;
    double reference_current_sum;

    // for (unsigned int k=0; k<this->init_reference_position.size(); k++)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Right reference position: %f", this->init_reference_position[k]);
    // }
    // copy(reference_position.begin(), reference_position.end(), std::ostream_iterator<double>(std::cout, " "));

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            if (soft_joint->get_right_motor_id() != 13) continue;

            //Check current limit
            if (abs(soft_joint->get_right_motor_present_current()) > this->joints_motor_model->CURRENT_LIMIT-100)
            {
                //update goal to reference position
                soft_joint->set_right_motor_goal_position(soft_joint->get_right_reference_position());
                RCLCPP_INFO(this->get_logger(), "Right Motor reached current limit. Moving to reference position: %f", soft_joint->get_right_motor_goal_position());
            }
            else if (abs(soft_joint->get_left_motor_present_current()) > this->joints_motor_model->CURRENT_LIMIT-100)
            {
                //update goal to reference position
                soft_joint->set_left_motor_goal_position(soft_joint->get_left_reference_position());
                RCLCPP_INFO(this->get_logger(), "Left Motor reached current limit. Moving to reference position: %f", soft_joint->get_left_motor_goal_position());
            }

            //Check min/max angle
            else if (soft_joint->get_joint_angle() > soft_joint->get_max_joint_angle())
            {
                soft_joint->set_right_motor_goal_position(soft_joint->get_right_motor_goal_position()-M_PI*2);
                soft_joint->set_left_motor_goal_position(soft_joint->get_left_motor_goal_position()-M_PI*2);
                RCLCPP_INFO(this->get_logger(), "Joint reached min joint angle. Moving to: %f, %f", soft_joint->get_right_motor_goal_position(), soft_joint->get_left_motor_goal_position());
            }
            else if (soft_joint->get_joint_angle() < soft_joint->get_min_joint_angle())
            {
                soft_joint->set_right_motor_goal_position(soft_joint->get_right_motor_goal_position()+M_PI*2);
                soft_joint->set_left_motor_goal_position(soft_joint->get_left_motor_goal_position()+M_PI*2);
                RCLCPP_INFO(this->get_logger(), "Joint reached max joint angle. Moving to: %f, %f", soft_joint->get_right_motor_goal_position(), soft_joint->get_left_motor_goal_position());
            }

            //If true check summed reference currents
            else if (reference_current)
            {
                present_current_sum = abs(soft_joint->get_right_motor_present_current()) + abs(soft_joint->get_left_motor_present_current());
                reference_current_sum = abs(soft_joint->get_right_reference_current()) + abs(soft_joint->get_left_reference_current());

                if (present_current_sum > reference_current_sum)
                {
                    soft_joint->set_right_motor_goal_position(soft_joint->get_right_reference_position());
                    soft_joint->set_left_motor_goal_position(soft_joint->get_left_reference_position());
                    RCLCPP_INFO(this->get_logger(), "Motor exceeded absolute reference current. Moving to: %f, %f", soft_joint->get_right_motor_goal_position(), soft_joint->get_left_motor_goal_position());
                }
            }

            //If true check reference_joint_angle
            else if (reference_joint_angle)
            {
                if (abs(soft_joint->get_joint_angle()) > abs(soft_joint->get_reference_joint_angle()))
                {
                    soft_joint->set_right_motor_goal_position(soft_joint->get_right_reference_position());
                    soft_joint->set_left_motor_goal_position(soft_joint->get_left_reference_position());
                    RCLCPP_INFO(this->get_logger(), "Joint exceeded absolute reference joint angle. Moving to: %f, %f", soft_joint->get_right_motor_goal_position(), soft_joint->get_left_motor_goal_position());
                }
            }

            //Check goal reached
            if ((!(soft_joint->right_motor_goal_reached())) || (!(soft_joint->left_motor_goal_reached()))) 
            {
                positions_reached = false;
            }
        }
    }

    return positions_reached;
}

bool gh360::MotorHandler::safetyCheck()
{
    bool safety_check = true;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            if ((abs(soft_joint->get_right_motor_present_current()) > this->joints_motor_model->CURRENT_LIMIT) || (abs(soft_joint->get_left_motor_present_current()) > this->joints_motor_model->CURRENT_LIMIT))
            {
                this->setTorqueEnable(soft_joint, 0);
                safety_check = false;
            }
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            // RCLCPP_INFO(this->get_logger(), "Current Limit: %f", this->joints_motor_model->CURRENT_LIMIT);
            // RCLCPP_INFO(this->get_logger(), "Present Current: %f", abs(motor_joint->get_motor_present_current()));
            if (abs(motor_joint->get_motor_present_current()) > this->joints_motor_model->CURRENT_LIMIT)
            {
                this->setTorqueEnable(motor_joint, 0);
                safety_check = false;
            }
        }
    }

    return safety_check;
}

gh360_interfaces::msg::PortStatus gh360::MotorHandler::getMotorStatus()
{
    gh360_interfaces::msg::PortStatus port_status_msg = gh360_interfaces::msg::PortStatus();
    gh360_interfaces::msg::MotorStatus motor_status_msg;

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = soft_joint->get_right_motor_id();
            motor_status_msg.present_position = soft_joint->get_right_motor_present_position();
            motor_status_msg.present_velocity = soft_joint->get_right_motor_present_velocity();
            motor_status_msg.present_current = soft_joint->get_right_motor_present_current();
            motor_status_msg.present_temperature = soft_joint->get_right_motor_present_temperature();
            port_status_msg.motors.push_back(motor_status_msg);
            // response->motor_status.push_back(motor_status_msg);

            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = soft_joint->get_left_motor_id();
            motor_status_msg.present_position = soft_joint->get_left_motor_present_position();
            motor_status_msg.present_velocity = soft_joint->get_left_motor_present_velocity();
            motor_status_msg.present_current = soft_joint->get_left_motor_present_current();
            motor_status_msg.present_temperature = soft_joint->get_left_motor_present_temperature();
            port_status_msg.motors.push_back(motor_status_msg);
            // response->motor_status.push_back(motor_status_msg);
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            motor_status_msg = gh360_interfaces::msg::MotorStatus();
            motor_status_msg.motor_id = motor_joint->get_motor_id();
            motor_status_msg.present_position = motor_joint->get_motor_present_position();
            motor_status_msg.present_velocity = motor_joint->get_motor_present_velocity();
            motor_status_msg.present_current = motor_joint->get_motor_present_current();
            motor_status_msg.present_temperature = motor_joint->get_motor_present_temperature();
            port_status_msg.motors.push_back(motor_status_msg);
            // response->motor_status.push_back(motor_status_msg);
        }
    }

    return port_status_msg;
}

bool gh360::MotorHandler::setMotorGoalPositions(std::vector<gh360_interfaces::msg::SetPosition> motor_goal_positions)
{
    for (unsigned int m=0; m < motor_goal_positions.size(); m++)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() == motor_goal_positions[m].id) 
                {
                    soft_joint->set_right_motor_goal_position(motor_goal_positions[m].position);
                }
                else if (soft_joint->get_left_motor_id() == motor_goal_positions[m].id)
                {
                    soft_joint->set_left_motor_goal_position(motor_goal_positions[m].position);
                }
                
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                if (motor_joint->get_motor_id() == motor_goal_positions[m].id) 
                {
                    motor_joint->set_motor_goal_position(motor_goal_positions[m].position);
                }
            }
        }
    }
    return true;
}

bool gh360::MotorHandler::setDeltaMotorGoalPositions(std::vector<gh360_interfaces::msg::SetPosition> delta_motor_goal_positions)
{
    double current_goal = 0.0;

    for (unsigned int m=0; m < delta_motor_goal_positions.size(); m++)
    {
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                if (soft_joint->get_right_motor_id() == delta_motor_goal_positions[m].id) 
                {
                    current_goal = soft_joint->get_right_motor_goal_position();
                    // if (delta_motor_goal_positions[m].id == 12) RCLCPP_INFO(this->get_logger(), "New Right Goal: %f", (current_goal + delta_motor_goal_positions[m].position));
                    soft_joint->set_right_motor_goal_position(current_goal + delta_motor_goal_positions[m].position);
                }
                else if (soft_joint->get_left_motor_id() == delta_motor_goal_positions[m].id)
                {
                    current_goal = soft_joint->get_left_motor_goal_position();

                    // if (delta_motor_goal_positions[m].id == 12) RCLCPP_INFO(this->get_logger(), "New Left Goal: %f", (current_goal + delta_motor_goal_positions[m].position));

                    soft_joint->set_left_motor_goal_position(current_goal + delta_motor_goal_positions[m].position);
                }
                
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                if (motor_joint->get_motor_id() == delta_motor_goal_positions[m].id) 
                {
                    current_goal = motor_joint->get_motor_goal_position();
                    // RCLCPP_INFO(this->get_logger(), "New Motor Goal: %f", (current_goal + delta_motor_goal_positions[m].position));
                    motor_joint->set_motor_goal_position(current_goal + delta_motor_goal_positions[m].position);
                }
            }
        }
    }
    return true;
}

bool gh360::MotorHandler::readPresentPosition()
{
    bool comm_result;
    if (!this->multi_motor_models)
    {
        uint8_t address = this->joints_motor_model->Present_Position.address;
        uint8_t size = this->joints_motor_model->Present_Position.size;
        // RCLCPP_INFO(this->get_logger(), "Address: %ld", address);
        // RCLCPP_INFO(this->get_logger(), "Size: %ld", size);
        comm_result = this->syncRead(size, address);
        if (!comm_result) return false;
    }

    for (unsigned int i=0; i < this->joints.size(); i++)
    {
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
        {
            RCLCPP_INFO(this->get_logger(), "Current Right Position: %f", soft_joint->get_right_motor_present_position());
            RCLCPP_INFO(this->get_logger(), "Current Left Position: %f", soft_joint->get_left_motor_present_position());
        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
        {
            RCLCPP_INFO(this->get_logger(), "Current Position: %f", motor_joint->get_motor_present_position());
        }
    }
    
    return true;
}

bool gh360::MotorHandler::syncRead(uint8_t size, uint8_t address)
{
    if (!this->multi_motor_models)
    {
        dynamixel::GroupSyncRead groupSyncRead(this->portHandler, this->packetHandler, address, size);
        
        bool dxl_addparam_result = false; 
        uint8_t motor_id;
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            // if (dynamic_cast<SoftJoint*>(this->joints[i]) != nullptr)
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                motor_id = soft_joint->get_right_motor_id();
                dxl_addparam_result = groupSyncRead.addParam(motor_id);
                if (dxl_addparam_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    return false;
                }

                motor_id = soft_joint->get_left_motor_id();
                dxl_addparam_result = groupSyncRead.addParam(motor_id);
                if (dxl_addparam_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    return false;
                }
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                motor_id = motor_joint->get_motor_id();
                dxl_addparam_result = groupSyncRead.addParam(motor_id);
                if (dxl_addparam_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead addparam failed", motor_id);
                    return false;
                }
            }
        }

        // Syncread present position
        int dxl_comm_result = groupSyncRead.txRxPacket();
        // if (dxl_comm_result != COMM_SUCCESS) this->packetHandler->printTxRxResult(dxl_comm_result);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to synread.");
            return false;
        } 


        bool dxl_getdata_result = false;
        // int32_t dxl1_present_position = 0, dxl2_present_position = 0;
        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                motor_id = soft_joint->get_right_motor_id();
                dxl_getdata_result = groupSyncRead.isAvailable(motor_id, address, size);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->dxl_id[0]);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                    return false;
                }
                // soft_joint->set_right_motor_present_position(groupSyncRead.getData(motor_id, address, size));
                soft_joint->set_right_motor_status(groupSyncRead.getData(motor_id, address, size), address);

                motor_id = soft_joint->get_left_motor_id();
                dxl_getdata_result = groupSyncRead.isAvailable(motor_id, address, size);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->dxl_id[0]);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                    return false;
                }
                // soft_joint->set_left_motor_present_position(groupSyncRead.getData(motor_id, address, size));
                soft_joint->set_left_motor_status(groupSyncRead.getData(motor_id, address, size), address);
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                motor_id = motor_joint->get_motor_id();
                dxl_getdata_result = groupSyncRead.isAvailable(motor_id, address, size);
                if (dxl_getdata_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->dxl_id[0]);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncRead getdata failed", motor_id);
                    return false;
                }
                // motor_joint->set_motor_present_position(groupSyncRead.getData(motor_id, address, size));
                motor_joint->set_motor_status(groupSyncRead.getData(motor_id, address, size), address);
            }
        }
        return true;
    }

    return false;
}

bool gh360::MotorHandler::syncWrite(uint8_t size, uint8_t address)
{
    if (!this->multi_motor_models)
    {
        dynamixel::GroupSyncWrite groupSyncWrite(this->portHandler, this->packetHandler, address, size);

        uint8_t param_goal_position[4];
        int motor_goal;
        uint8_t motor_id;
        bool comm_result;

        for (unsigned int i=0; i < this->joints.size(); i++)
        {
            if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(this->joints[i]))
            {
                motor_id = soft_joint->get_right_motor_id();
                motor_goal = soft_joint->get_right_motor_goal_int(address);
                // motor_goal_pos = soft_joint->get_right_motor_goal_position_int();

                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));

                comm_result = groupSyncWrite.addParam(motor_id, param_goal_position);
                if (comm_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    return false;
                }

                motor_id = soft_joint->get_left_motor_id();
                // motor_goal_pos = soft_joint->get_left_motor_goal_position_int();
                motor_goal = soft_joint->get_left_motor_goal_int(address);
                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));

                comm_result = groupSyncWrite.addParam(motor_id, param_goal_position);
                if (comm_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    return false;
                }
                
            }
            else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(this->joints[i]))
            {
                motor_id = motor_joint->get_motor_id();
                // motor_goal_pos = motor_joint->get_motor_goal_position_int();
                motor_goal = motor_joint->get_motor_goal_int(address);

                param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(motor_goal));
                param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(motor_goal));
                param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(motor_goal));

                comm_result = groupSyncWrite.addParam(motor_id, param_goal_position);
                if (comm_result != true)
                {
                    // fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    RCLCPP_ERROR(this->get_logger(), "[ID:%03d] groupSyncWrite addparam failed", motor_id);
                    return false;
                }
            }

        }

        // Syncwrite goal position
        comm_result = groupSyncWrite.txPacket();
        if (comm_result != COMM_SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to sync write to motors.");
            return false;
        } 

        // Clear syncwrite parameter storage
        groupSyncWrite.clearParam();

        return true;
    }
    return false;
}

bool gh360::MotorHandler::openPortsAndSetBaudrate()
{
    this->portHandler = dynamixel::PortHandler::getPortHandler(this->port_name);
    this->packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

    
    int dxl_comm_result;
    // uint8_t dxl_error;

    // Open Serial Port
    dxl_comm_result = this->portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to open the port!");
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = this->portHandler->setBaudRate(this->baud_rate);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to set the baudrate!");
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Succeeded to set the baudrate.");
    }

    return true;
}

bool gh360::MotorHandler::writeRegister(uint8_t id, int32_t data, uint8_t data_size, uint8_t address)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    switch(data_size) {
        case 1:
            dxl_comm_result = this->packetHandler->write1ByteTxRx(this->portHandler,
                id,
                address,
                data,
                &dxl_error
            );
            break;
        case 2:
            dxl_comm_result = this->packetHandler->write2ByteTxRx(this->portHandler,
                id,
                address,
                data,
                &dxl_error
            );
            break;
        case 4:
            dxl_comm_result = this->packetHandler->write4ByteTxRx(this->portHandler,
                id,
                address,
                data,
                &dxl_error
            );
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Invalid data size for writing to motors!");
    }

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to write to motor.");
        return false;
    } else {
        // RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Succeeded to write to motor.");
        return true;
    }
}

gh360::MotorDictionary* gh360::MotorHandler::getMotorModel(int motor_id)
{
    uint8_t dxl_error;
    uint16_t model_number = 0;
    const char * log;

    int dxl_comm_result = this->packetHandler->ping(this->portHandler, motor_id, &model_number, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Failed to get model number from id: %ld", motor_id);
        return NULL;
    }
    else if (dxl_error != 0)
    {
        log = this->packetHandler->getRxPacketError(dxl_error);
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Log Output: ", log);
        return NULL;
    }
    else {
        // RCLCPP_INFO(this->get_logger(), "Model Number: %ld", this->motor_test->Model_Number);
        // Model Number: 321 = MX-106; 311 = MX-64
        RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Model Number: %ld", model_number);
        if (model_number == 321)
        {
            int protocol = 2;
            return new MX_106_DICT(protocol);
            // gh360::MX_106_DICT* motor_dict = new MX_106_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
            // RCLCPP_INFO(this->get_logger(), "Test ", log);
        }
        else if (model_number == 320)
        {
            int protocol = 1;
            return new MX_106_DICT(protocol);
            // gh360::MX_106_DICT* motor_dict = new MX_106_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
        }
        else if (model_number == 311)
        {
            int protocol = 2;
            return new MX_64_DICT(protocol);
            // gh360::MX_64_DICT* motor_dict = new MX_64_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
        }
        else if (model_number == 310)
        {
            int protocol = 1;
            return new MX_64_DICT(protocol);
            // gh360::MX_64_DICT* motor_dict = new MX_64_DICT(protocol);
            // this->motor_dicts.push_back(motor_dict);
        }
        // return true;
    }
    return NULL;

}

bool gh360::MotorHandler::setVelocityProfile(Joint* joint, double value)
{
    bool comm_result = false;
    uint8_t data_size;
    uint8_t address;
    uint8_t motor_id;

    int int_value = int(value / 0.229);

    if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(joint))
    {
        data_size = soft_joint->get_right_motor_model()->Profile_Velocity.size;
        address = soft_joint->get_right_motor_model()->Profile_Velocity.address;
        motor_id = soft_joint->get_right_motor_id();
        comm_result = this->writeRegister(motor_id, int_value, data_size, address);

        if (comm_result) 
        {
            // SoftJoint * soft_joint = joint;
            data_size = soft_joint->get_left_motor_model()->Profile_Velocity.size;
            address = soft_joint->get_left_motor_model()->Profile_Velocity.address;
            motor_id = soft_joint->get_left_motor_id();
            comm_result = this->writeRegister(motor_id, int_value, data_size, address);
        }
    }
    else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(joint))
    {
        data_size = motor_joint->get_motor_model()->Profile_Velocity.size;
        address = motor_joint->get_motor_model()->Profile_Velocity.address;
        motor_id = motor_joint->get_motor_id();
        comm_result = this->writeRegister(motor_id, int_value, data_size, address);
        
    }
    
    return true;
}

bool gh360::MotorHandler::setAccelerationProfile(Joint* joint, double value)
{
    bool comm_result = false;
    uint8_t data_size;
    uint8_t address;
    uint8_t motor_id;

    int int_value = int(value / 214.577);

    if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(joint))
    {
        data_size = soft_joint->get_right_motor_model()->Profile_Acceleration.size;
        address = soft_joint->get_right_motor_model()->Profile_Acceleration.address;
        motor_id = soft_joint->get_right_motor_id();
        comm_result = this->writeRegister(motor_id, int_value, data_size, address);

        if (comm_result) 
        {
            // SoftJoint * soft_joint = joint;
            data_size = soft_joint->get_left_motor_model()->Profile_Acceleration.size;
            address = soft_joint->get_left_motor_model()->Profile_Acceleration.address;
            motor_id = soft_joint->get_left_motor_id();
            comm_result = this->writeRegister(motor_id, int_value, data_size, address);
        }
    }
    else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(joint))
    {
        data_size = motor_joint->get_motor_model()->Profile_Acceleration.size;
        address = motor_joint->get_motor_model()->Profile_Acceleration.address;
        motor_id = motor_joint->get_motor_id();
        comm_result = this->writeRegister(motor_id, int_value, data_size, address);
        
    }

    return true;
}

bool gh360::MotorHandler::setTorqueEnable(Joint* joint, int value) 
{
    bool comm_result = false;
    uint8_t data_size;
    uint8_t address;
    uint8_t motor_id;

    if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(joint))
    {
        data_size = soft_joint->get_right_motor_model()->Torque_Enable.size;
        address = soft_joint->get_right_motor_model()->Torque_Enable.address;
        motor_id = soft_joint->get_right_motor_id();
        comm_result = this->writeRegister(motor_id, value, data_size, address);

        if (comm_result) 
        {
            // SoftJoint * soft_joint = joint;
            data_size = soft_joint->get_left_motor_model()->Torque_Enable.size;
            address = soft_joint->get_left_motor_model()->Torque_Enable.address;
            motor_id = soft_joint->get_left_motor_id();
            comm_result = this->writeRegister(motor_id, value, data_size, address);
        }
    }
    else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(joint))
    {
        data_size = motor_joint->get_motor_model()->Torque_Enable.size;
        address = motor_joint->get_motor_model()->Torque_Enable.address;
        motor_id = motor_joint->get_motor_id();
        comm_result = this->writeRegister(motor_id, value, data_size, address);
        
    }

    if (comm_result)
    {
        if (value == 0) RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Torque on %s successfully turned off", joint->get_joint_name().c_str());
        else RCLCPP_INFO(rclcpp::get_logger("motor_handler"), "Torque on %s successfully turned on", joint->get_joint_name().c_str());
        return true;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_handler"), "Changing operation mode on motor %s failed!", joint->get_joint_name().c_str());
        return false;
    }

    
}

bool gh360::MotorHandler::setOperatingMode(Joint* joint, int value)
{
    bool valid_value = false;
    std::string msg;
    switch(value) {
        case 0:
            msg = "Current Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 1:
            msg = "Velocity Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 3:
            msg = "Position Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 4:
            msg = "Extended Position Control on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 5:
            msg = "Current-based Position Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        case 16:
            msg = "PWM Control Mode on "+joint->get_joint_name()+" enabled";
            valid_value = true;
            break;
        default:
            valid_value = false;
            RCLCPP_ERROR(this->get_logger(), "%d is not a valid operating mode", value);
    }

    

    
    if (valid_value)
    {
        bool comm_result = false;
        uint8_t data_size;
        uint8_t address;
        uint8_t motor_id;
        // if (joint->get_joint_type() == "soft_joint")
        if (SoftJoint* soft_joint = dynamic_cast<SoftJoint*>(joint))
        {
            // SoftJoint * soft_joint = joint;
            data_size = soft_joint->get_right_motor_model()->Operating_Mode.size;
            address = soft_joint->get_right_motor_model()->Operating_Mode.address;
            motor_id = soft_joint->get_right_motor_id();
            comm_result = this->writeRegister(motor_id, value, data_size, address);

            if (comm_result) 
            {
                // SoftJoint * soft_joint = joint;
                data_size = soft_joint->get_left_motor_model()->Operating_Mode.size;
                address = soft_joint->get_left_motor_model()->Operating_Mode.address;
                motor_id = soft_joint->get_left_motor_id();
                comm_result = this->writeRegister(motor_id, value, data_size, address);
            }

        }
        else if (MotorJoint* motor_joint = dynamic_cast<MotorJoint*>(joint))
        {
            // MotorJoint* motor_joint = joint;
            data_size = motor_joint->get_motor_model()->Operating_Mode.size;
            address = motor_joint->get_motor_model()->Operating_Mode.address;
            motor_id = motor_joint->get_motor_id();
            comm_result = this->writeRegister(motor_id, value, data_size, address);
        }
        // uint8_t data_size = this->motor_dicts[motor_index]->Operating_Mode.size;
        // uint8_t address = this->motor_dicts[motor_index]->Operating_Mode.address;
        
        if (comm_result)
        {
            RCLCPP_INFO(this->get_logger(), msg);
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Changing operation mode on motor %d failed!", motor_id);
            return false;
        }
    }
    return false;
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
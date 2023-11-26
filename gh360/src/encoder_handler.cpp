#include <memory>
#include <string>
#include "encoder_handler.hpp"

gh360::EncoderHandler::EncoderHandler()
: Node("encoder_handler")
{
    RCLCPP_INFO(this->get_logger(), "Run encoder handler node");

    this->shoulder_data_recieved = false;
    this->upperarm_data_recieved = false;
    this->lowerarm_data_recieved = false;

    this->shoulder_encoder_subscriber_ = this->create_subscription<std_msgs::msg::String>("/arm1/Shoulder_Encoders", 10, std::bind(&gh360::EncoderHandler::shoulder_encoder_callback, this, std::placeholders::_1));
    this->upperarm_encoder_subscriber_ = this->create_subscription<std_msgs::msg::String>("/arm1/UpperArm_Encoders", 10, std::bind(&gh360::EncoderHandler::upperarm_encoder_callback, this, std::placeholders::_1));
    this->lowerarm_encoder_subscriber_ = this->create_subscription<std_msgs::msg::String>("/arm1/LowerArm_Encoders", 10, std::bind(&gh360::EncoderHandler::lowerarm_encoder_callback, this, std::placeholders::_1));

    for (uint i=0; i<this->joint_names.size(); i++)
    {
        this->declare_parameter(this->joint_names[i]+".port_name", "default");
        this->declare_parameter(this->joint_names[i]+".port_id", 1);
        this->declare_parameter(this->joint_names[i]+".offset", 0.0);
        this->declare_parameter(this->joint_names[i]+".inverter", 1);
        std::string port_name = get_parameter(this->joint_names[i]+".port_name").as_string();
        if (port_name == "shoulder")
        {
            this->shoulder_joint_names.push_back(this->joint_names[i]);
            this->shoulder_port_ids.push_back(get_parameter(this->joint_names[i]+".port_id").as_int());
            this->shoulder_offsets.push_back(get_parameter(this->joint_names[i]+".offset").as_double());
            this->shoulder_inverters.push_back(get_parameter(this->joint_names[i]+".inverter").as_int());
            this->shoulder_joint_angles.push_back(0.0);
            this->shoulder_joint_vels.push_back(0.0);
        }
        else if (port_name == "upperarm")
        {
            this->upperarm_joint_names.push_back(this->joint_names[i]);
            this->upperarm_port_ids.push_back(get_parameter(this->joint_names[i]+".port_id").as_int());
            this->upperarm_offsets.push_back(get_parameter(this->joint_names[i]+".offset").as_double());
            this->upperarm_inverters.push_back(get_parameter(this->joint_names[i]+".inverter").as_int());
            this->upperarm_joint_angles.push_back(0.0);
            this->upperarm_joint_vels.push_back(0.0);
        }
        else if (port_name == "lowerarm")
        {
            this->lowerarm_joint_names.push_back(this->joint_names[i]);
            this->lowerarm_port_ids.push_back(get_parameter(this->joint_names[i]+".port_id").as_int());
            this->lowerarm_offsets.push_back(get_parameter(this->joint_names[i]+".offset").as_double());
            this->lowerarm_inverters.push_back(get_parameter(this->joint_names[i]+".inverter").as_int());
            this->lowerarm_joint_angles.push_back(0.0);
            this->lowerarm_joint_vels.push_back(0.0);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), port_name+" is not a valid port name. Valid port names are: shoulder, upperarm, lowerarm");
        }
    }

    // publisher definition
    this->encoder_state_publisher_ = this->create_publisher<gh360_interfaces::msg::ArmEncoderStates>("encoder_status", 10);
    this->timer_ = this->create_wall_timer(100ms, std::bind(&gh360::EncoderHandler::timer_callback, this));


}

gh360::EncoderHandler::~EncoderHandler()
{
}

void gh360::EncoderHandler::shoulder_encoder_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Ecoder Message: "+msg->data);
    // this->tokenize(msg->data, ";");

    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;
    double new_joint_angle;
    double new_joint_vel;

    //check if shoulder_prev_time is on it's default init value
    if (this->shoulder_prev_time == std::chrono::time_point<std::chrono::system_clock>())
    {
        this->shoulder_prev_time = current_time;
    }
    else
    {
        elapsed_seconds = current_time - this->shoulder_prev_time;
        this->shoulder_prev_time = current_time;
    }

    std::vector<double> joint_angles = this->strToDoubleVector(msg->data, ";");
    for (uint i=0; i<this->shoulder_joint_names.size(); i++)
    {
        new_joint_angle = joint_angles[this->shoulder_port_ids[i]-1] * this->shoulder_inverters[i] - this->shoulder_offsets[i] ;
        new_joint_vel = (new_joint_angle - this->shoulder_joint_angles[i]) / elapsed_seconds.count()    ;

        this->shoulder_joint_angles[i] = new_joint_angle;
        this->shoulder_joint_vels[i] = new_joint_vel;
    }
    
    if (!(this->shoulder_data_recieved)) this->shoulder_data_recieved = true;
}

void gh360::EncoderHandler::upperarm_encoder_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Ecoder Message: "+msg->data); -0.454 2.83
    // this->strToDoubleVector(msg->data, ";");

    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;
    double new_joint_angle;
    double new_joint_vel;

    //check if upperarm_prev_time is on it's default init value
    if (this->upperarm_prev_time == std::chrono::time_point<std::chrono::system_clock>())
    {
        this->upperarm_prev_time = current_time;
    }
    else
    {
        elapsed_seconds = current_time - this->upperarm_prev_time;
        this->upperarm_prev_time = current_time;
    }

    std::vector<double> joint_angles = this->strToDoubleVector(msg->data, ";");

    for (uint i=0; i<this->upperarm_joint_names.size(); i++)
    {
        new_joint_angle = joint_angles[this->upperarm_port_ids[i]-1] * this->upperarm_inverters[i] - this->upperarm_offsets[i];
        new_joint_vel = (new_joint_angle - this->upperarm_joint_angles[i]) / elapsed_seconds.count()    ;

        this->upperarm_joint_angles[i] = new_joint_angle;
        this->upperarm_joint_vels[i] = new_joint_vel;
    }

    if (!(this->upperarm_data_recieved)) this->upperarm_data_recieved = true;
}

void gh360::EncoderHandler::lowerarm_encoder_callback(const std_msgs::msg::String::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Ecoder Message: "+msg->data);
    // this->strToDoubleVector(msg->data, ";");

    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;
    double new_joint_angle;
    double new_joint_vel;

    //check if lowerarm_prev_time is on it's default init value
    if (this->lowerarm_prev_time == std::chrono::time_point<std::chrono::system_clock>())
    {
        this->lowerarm_prev_time = current_time;
    }
    else
    {
        elapsed_seconds = current_time - this->lowerarm_prev_time;
        this->lowerarm_prev_time = current_time;
    }

    std::vector<double> joint_angles = this->strToDoubleVector(msg->data, ";");

    for (uint i=0; i<this->lowerarm_joint_names.size(); i++)
    {
        new_joint_angle = joint_angles[this->lowerarm_port_ids[i]-1] * this->lowerarm_inverters[i] - this->lowerarm_offsets[i];
        new_joint_vel = (new_joint_angle - this->lowerarm_joint_angles[i]) / elapsed_seconds.count()    ;

        this->lowerarm_joint_angles[i] = new_joint_angle;
        this->lowerarm_joint_vels[i] = new_joint_vel;
    }

    if (!(this->lowerarm_data_recieved)) this->lowerarm_data_recieved = true;
}

std::vector<double> gh360::EncoderHandler::strToDoubleVector(std::string s, std::string del)
{
    std::vector<double> encoder_angles(3);
    int start, end = -1*del.size();
    for(int i=0; i<3; i++)
    {
        start = end + del.size();
        end = s.find(del, start);
        //std::cout << s.substr(start, end - start) << std::endl;
        encoder_angles[i] = stod(s.substr(start, end - start));
        encoder_angles[i] = encoder_angles[i] / 16384 * 6.283185;
        //std::cout << encoder_angles[i] << std::endl;
    }

    //+/- 8191 -> 360 degrees???   5.6 2.4
    return encoder_angles;
}

void gh360::EncoderHandler::timer_callback()
{
    //  RCLCPP_INFO(this->get_logger(), "Publishing Loop");
    if (this->shoulder_data_recieved && this->upperarm_data_recieved && this->lowerarm_data_recieved)
    {
        gh360_interfaces::msg::ArmEncoderStates arm_encoder_msg = gh360_interfaces::msg::ArmEncoderStates();
        gh360_interfaces::msg::JointEncoderState joint_encoder_msg;

        for (uint i=0; i<this->shoulder_joint_names.size(); i++)
        {
            joint_encoder_msg = gh360_interfaces::msg::JointEncoderState();
            joint_encoder_msg.joint_name = this->shoulder_joint_names[i];
            joint_encoder_msg.current_pos = this->shoulder_joint_angles[i];
            joint_encoder_msg.current_vel = this->shoulder_joint_vels[i];
            
            arm_encoder_msg.current_joint_states.push_back(joint_encoder_msg);
        }

        for (uint i=0; i<this->upperarm_joint_names.size(); i++)
        {
            joint_encoder_msg = gh360_interfaces::msg::JointEncoderState();
            joint_encoder_msg.joint_name = this->upperarm_joint_names[i];
            joint_encoder_msg.current_pos = this->upperarm_joint_angles[i];
            joint_encoder_msg.current_vel = this->upperarm_joint_vels[i];
            
            arm_encoder_msg.current_joint_states.push_back(joint_encoder_msg);
        }

        for (uint i=0; i<this->lowerarm_joint_names.size(); i++)
        {
            joint_encoder_msg = gh360_interfaces::msg::JointEncoderState();
            joint_encoder_msg.joint_name = this->lowerarm_joint_names[i];
            joint_encoder_msg.current_pos = this->lowerarm_joint_angles[i];
            joint_encoder_msg.current_vel = this->lowerarm_joint_vels[i];
            
            arm_encoder_msg.current_joint_states.push_back(joint_encoder_msg);
        }

        this->encoder_state_publisher_->publish(arm_encoder_msg);
    }
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for joint encoder data...");
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto encoderhandlernode = std::make_shared<gh360::EncoderHandler>();
    rclcpp::spin(encoderhandlernode);
    rclcpp::shutdown();

    return 0;
}
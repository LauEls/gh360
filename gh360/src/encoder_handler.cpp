// #include <memory>
// #include <string>
#include "gh360/encoder_handler.hpp"

gh360::EncoderHandler::EncoderHandler()
: Node("encoder_handler")
{
    RCLCPP_INFO(this->get_logger(), "Run encoder handler node");

    this->declare_parameter("joint_names", std::vector<std::string>());
    this->joint_names = get_parameter("joint_names").as_string_array();
    this->declare_parameter("port_names", std::vector<std::string>());
    this->port_names = get_parameter("port_names").as_string_array();

    for (uint i=0; i<this->port_names.size(); i++)
    {
        std::function<void(const std_msgs::msg::String::SharedPtr)> fcn = std::bind(&gh360::EncoderHandler::encoder_callback, this, std::placeholders::_1, this->port_names[i]);
        this->encoder_subscribers.push_back(this->create_subscription<std_msgs::msg::String>(this->port_names[i], 10, fcn));
        this->data_recieved.push_back(false);
    }   

    for (unsigned int i=0; i < this->joint_names.size(); i++)
    {
        this->declare_parameter(this->joint_names[i]+".port_name", "default");
        this->declare_parameter(this->joint_names[i]+".port_id", 1);
        this->declare_parameter(this->joint_names[i]+".offset", 0.0);
        this->declare_parameter(this->joint_names[i]+".inverter", 1);

        std::string port_name = get_parameter(this->joint_names[i]+".port_name").as_string();
        int port_id = get_parameter(this->joint_names[i]+".port_id").as_int();
        double offset = get_parameter(this->joint_names[i]+".offset").as_double();
        int inverter = get_parameter(this->joint_names[i]+".inverter").as_int();

        if (port_name == "default") 
        {
            RCLCPP_ERROR(this->get_logger(), "Port name not specified for joint %s", this->joint_names[i].c_str());
            continue;
        }

        Encoder * new_encoder = new Encoder(this->joint_names[i], port_name, port_id, offset, inverter);
        this->encoders.push_back(new_encoder);
    }

    RCLCPP_INFO(this->get_logger(), "Number of encoders: %d", this->encoders.size());

    this->encoder_state_publisher_ = this->create_publisher<gh360_interfaces::msg::ArmEncoderStates>("encoder_states", 10);
    this->timer_ = this->create_wall_timer(10ms, std::bind(&gh360::EncoderHandler::timer_callback, this));
}

gh360::EncoderHandler::~EncoderHandler()
{
}

void gh360::EncoderHandler::encoder_callback(const std_msgs::msg::String::SharedPtr msg, const std::string port_name)
{
    std::vector<double> joint_angles = this->strToDoubleVector(msg->data, ";");

    for (uint i=0; i<this->encoders.size(); i++)
    {
        if (this->encoders[i]->get_port_name() == port_name)
        {
            this->encoders[i]->set_joint_angle(joint_angles[this->encoders[i]->get_port_id()-1]);
        }
    }

    if (!std::all_of(this->data_recieved.begin(), this->data_recieved.end(), [](bool value) {return value;}))
    {
        for (uint i=0; i<this->port_names.size(); i++)
        {
            if (this->port_names[i] == port_name)
            {
                this->data_recieved[i] = true;
            }
        }
    }
}

std::vector<double> gh360::EncoderHandler::strToDoubleVector(std::string s, std::string del)
{
    std::vector<double> encoder_angles(3);
    int start, end = -1*del.size();
    for(int i=0; i<3; i++)
    {
        start = end + del.size();
        end = s.find(del, start);
        encoder_angles[i] = stod(s.substr(start, end - start));
        encoder_angles[i] = encoder_angles[i] / 16384 * 6.283185;
    }

    return encoder_angles;
}

void gh360::EncoderHandler::timer_callback()
{
    if (std::all_of(this->data_recieved.begin(), this->data_recieved.end(), [](bool value) {return value;}))
    {
        gh360_interfaces::msg::ArmEncoderStates arm_encoder_msg = gh360_interfaces::msg::ArmEncoderStates();
        gh360_interfaces::msg::JointEncoderState joint_encoder_msg;

        std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();

        for (uint i=0; i<this->encoders.size(); i++)
        {
            joint_encoder_msg = gh360_interfaces::msg::JointEncoderState();
            joint_encoder_msg.joint_name = this->encoders[i]->get_joint_name();
            joint_encoder_msg.current_pos = this->encoders[i]->get_joint_angle();
            joint_encoder_msg.current_vel = this->encoders[i]->calc_joint_velocity(current_time);
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
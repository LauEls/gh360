#include "port_handler.hpp"



gh360::PortHandlerNode::PortHandlerNode(): Node("port_handler")
{
    // RCLCPP_INFO(this->get_logger(), "Test");
    // std::vector<std::string> string_array;
    // string_array.push_back("test1");

    // this->declare_parameter("joint_names", string_array);
    // this->joint_names = get_parameter("joint_names").as_string_array();
    // this->declare_parameter("baud_rate", 0);
    // this->baud_rate = get_parameter("baud_rate").as_int();
    // this->declare_parameter("protocol", 0);
    // this->protocol = get_parameter("protocol").as_int();
    // bool exists = false;
    // std::string curr_port_name;
    // gh360::MotorHandlerNode * curr_port;
    // for (unsigned int i = 0; i < this->joint_names.size(); i++) {
    //     exists = false;
    //     curr_port_name = get_parameter(this->joint_names[i]+".port_name").as_string();

    //     for (int x = 0; x < this->ports.size(); x++) {
    //         if (this->ports[x].port_name == curr_port_name)
    //         {
    //             exists = true;
    //             curr_port = this->ports[x];
    //             // this->ports[x].addJoint();
    //             break;
    //         }
    //     }

    //     if (exists == false) {
    //         curr_port = new gh360::MotorHandlerNode(curr_port_name, this->baud_rate, this->protocol)

    //     }

    //     curr_port.addJoint(this->joint_names[i]);
    // }
}

gh360::PortHandlerNode::~PortHandlerNode()
{

}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // const char* port_name = "/dev/ttyUSB0";
    // int baud_rate = 1000000;
    // int protocol = 2;
    auto porthandlernode = std::make_shared<gh360::PortHandlerNode>();
    // rclcpp::spin(porthandlernode);
    // rclcpp::shutdown();

    return 0;
}
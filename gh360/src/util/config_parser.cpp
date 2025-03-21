#include "gh360/util/config_parser.hpp"

std::vector<Joint*> get_robot_joints(rclcpp::Node* node)
{
    RCLCPP_INFO(node->get_logger(), "Parsing Robot Config");
    // this->joint_states_recieved = false;
    std::vector<Joint*> joints;

    node->declare_parameter("joint_names", std::vector<std::string>());
    std::vector<std::string> joint_names = node->get_parameter("joint_names").as_string_array();

    for (unsigned int i = 0; i < joint_names.size(); i++) {
        // RCLCPP_INFO(node->get_logger(), "Joint Name: %s", joint_names[i].c_str());
        
        std::string joint_name = joint_names[i];
        node->declare_parameter(joint_name+".joint_type", "default");
        std::string joint_type = node->get_parameter(joint_name+".joint_type").as_string();
        if (joint_type == "default") {
            RCLCPP_ERROR(node->get_logger(), "Joint type not specified for joint %s", joint_name.c_str());
            continue;
        }

        node->declare_parameter(joint_name+".min_joint_angle", 0.0);
        node->declare_parameter(joint_name+".max_joint_angle", 0.0);
        node->declare_parameter(joint_name+".motor_init_pos", 0.0);
        // node->declare_parameter(joint_name+".initialize", true);
        if (joint_type == "soft_joint") {
            node->declare_parameter(joint_name+".right.motor_id", 0);
            node->declare_parameter(joint_name+".left.motor_id", 0);
            node->declare_parameter(joint_name+".right.movement_direction", 1);
            node->declare_parameter(joint_name+".left.movement_direction", 1);
            node->declare_parameter(joint_name+".right.offset", 0.0);
            node->declare_parameter(joint_name+".left.offset", 0.0);
            node->declare_parameter(joint_name+".radius_active_pulley", 0.0);
            node->declare_parameter(joint_name+".radius_passive_pulley", 0.0);
            
            SoftJoint * new_joint = new SoftJoint();
            new_joint->set_joint_name(joint_name);
            new_joint->set_min_joint_angle(node->get_parameter(joint_name+".min_joint_angle").as_double());
            new_joint->set_max_joint_angle(node->get_parameter(joint_name+".max_joint_angle").as_double());
            new_joint->set_motor_init_pos(node->get_parameter(joint_name+".motor_init_pos").as_double());
            new_joint->set_radius_active_pulley(node->get_parameter(joint_name+".radius_active_pulley").as_double());
            new_joint->set_radius_passive_pulley(node->get_parameter(joint_name+".radius_passive_pulley").as_double());
            
            Motor * right_motor = new_joint->get_motor(new_joint->RIGHT);
            Motor * left_motor = new_joint->get_motor(new_joint->LEFT);
            right_motor->set_motor_id(node->get_parameter(joint_name+".right.motor_id").as_int());
            left_motor->set_motor_id(node->get_parameter(joint_name+".left.motor_id").as_int());
            right_motor->set_movement_direction(node->get_parameter(joint_name+".right.movement_direction").as_int());
            left_motor->set_movement_direction(node->get_parameter(joint_name+".left.movement_direction").as_int());
            right_motor->set_offset(node->get_parameter(joint_name+".right.offset").as_double());
            left_motor->set_offset(node->get_parameter(joint_name+".left.offset").as_double());
            

            joints.push_back(new_joint);
        }
        else {
            node->declare_parameter(joint_name+".motor_id", 0);
            node->declare_parameter(joint_name+".movement_direction", 1);
            node->declare_parameter(joint_name+".offset", 0.0);

            MotorJoint * new_joint = new MotorJoint(); 
            new_joint->set_joint_name(joint_name);
            new_joint->set_min_joint_angle(node->get_parameter(joint_name+".min_joint_angle").as_double());
            new_joint->set_max_joint_angle(node->get_parameter(joint_name+".max_joint_angle").as_double());
            new_joint->set_motor_init_pos(node->get_parameter(joint_name+".motor_init_pos").as_double());
            Motor * motor = new_joint->get_motor(0);
            motor->set_motor_id(node->get_parameter(joint_name+".motor_id").as_int());
            motor->set_movement_direction(node->get_parameter(joint_name+".movement_direction").as_int());
            motor->set_offset(node->get_parameter(joint_name+".offset").as_double());

            joints.push_back(new_joint);
        }
        
    }

    return joints;
}
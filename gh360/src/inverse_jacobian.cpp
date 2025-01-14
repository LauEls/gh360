#include "inverse_jacobian.hpp"

gh360::InverseJacobian::InverseJacobian()
: Node("inverse_jacobian")
{
    KDL::Tree kdl_tree;
    // KDL::Chain chain;

    // if (!kdl_parser::treeFromXml(xml_root, my_tree)){
    //    ROS_ERROR("Failed to construct kdl tree");
    //    return false;
    // }
    this->declare_parameter("tcp_link_name", "eef");
    this->tcp_link_name = get_parameter("tcp_link_name").as_string();
    this->declare_parameter("robot_description", "");
    std::string robot_desc_string = get_parameter("robot_description").as_string();
    this->declare_parameter("joint_position_topic_name", "/gh360_joint_states");
    std::string joint_position_topic = get_parameter("joint_position_topic_name").as_string();

    // RCLCPP_INFO(this->get_logger(),robot_desc_string);

    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
    // if (!kdl_parser::treeFromFile("/home/laurenz/phd_project/ros2_gh360_ws/src/gh360/gh360/urdf/gh360.urdf", kdl_tree)){
        RCLCPP_ERROR(this->get_logger(),"Failed to construct kdl tree");
    }

    // if (this->robot_name == "panda") {
    //     RCLCPP_INFO(this->get_logger(),"Using panda robot");
    //     kdl_tree.getChain("base_link", "panda_hand", this->chain);
    // }
    // else if (this->robot_name == "gh360") {
    //     RCLCPP_INFO(this->get_logger(),"Using gh360 robot");
    //     kdl_tree.getChain("base_link", "eef", this->chain);
    // }
    
    kdl_tree.getChain("base_link", this->tcp_link_name, this->chain);
    // RCLCPP_INFO(this->get_logger(),std::to_string(this->chain.getNrOfJoints()));
    this->num_joints = this->chain.getNrOfJoints();

    this->fk_solver = new KDL::ChainFkSolverPos_recursive(this->chain);
    this->ik_solver = new KDL::ChainIkSolverVel_pinv(this->chain);

    this->joint_vel_msg = sensor_msgs::msg::JointState();
    this->current_joint_pos = sensor_msgs::msg::JointState();
    for (unsigned int i = 0; i < this->chain.getNrOfSegments(); i++) {
        // this->joint_vel_msg.name.push_back(this->chain.getSe);
        // this->joint_vel_msg.velocity.push_back(0.0);
        if (this->chain.getSegment(i).getJoint().getType() != KDL::Joint::None) {
            this->joint_vel_msg.name.push_back(this->chain.getSegment(i).getJoint().getName());
            this->joint_vel_msg.velocity.push_back(0.0);
            this->joint_vel_msg.position.push_back(0.0);
            // RCLCPP_INFO(this->get_logger(),this->chain.getSegment(i).getJoint().getName());
            // RCLCPP_INFO(this->get_logger(),this->chain.getSegment(i).getJoint().getTypeName());
        }
    }
    // if (this->robot_name == "panda") {
    //     this->joint_state_msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    //     this->joint_state_msg.position = {0.0, 0.0, 0.0, -1.578, 0.0, 1.578, 0.0};
    // }
    // else if (this->robot_name == "gh360") {
    this->current_joint_pos.name = {"shoulder_yaw", "shoulder_roll", "shoulder_pitch", "upperarm_roll", "elbow", "forearm_roll", "wrist_pitch"};
    this->current_joint_pos.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // }
    // this->joint_state_msg.name = {"shoulder_yaw", "shoulder_roll", "shoulder_pitch", "upperarm_roll", "elbow", "forearm_roll", "wrist_pitch"};
    // this->joint_state_msg.position = {0.0, 0.0, 0.0, 1.578, 1.578, 0.0, 0.0};
    // this->joint_state_msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    // this->joint_state_msg.position = {0.0, 0.0, 0.0, -1.578, 0.0, 1.578, 0.0};
    this->desired_velocity = geometry_msgs::msg::Twist();

    // this->spacemouse_subscriber_ = this->create_subscription<gh360_interfaces::msg::SpaceMouse>("/spacemouse", 10, std::bind(&gh360::InverseJacobian::spacemouse_callback, this, std::placeholders::_1));
    this->spacemouse_subscriber_ = this->create_subscription<gh360_interfaces::msg::SpaceMouse>("/cmd_eef_vel", 10, std::bind(&gh360::InverseJacobian::spacemouse_callback, this, std::placeholders::_1));
    this->joint_position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_position_topic, 10, std::bind(&gh360::InverseJacobian::joint_position_callback, this, std::placeholders::_1));

    
    // RCLCPP_INFO(this->get_logger(),std::to_string(sizeof(this->current_joint_pos.position)/sizeof(int)));

    this->joint_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/inverse_jacobian", 10);
    
    
    this->timer_ = this->create_wall_timer(10ms, std::bind(&gh360::InverseJacobian::timer_callback, this));
    
}

gh360::InverseJacobian::~InverseJacobian()
{
}

void gh360::InverseJacobian::spacemouse_callback(const gh360_interfaces::msg::SpaceMouse::SharedPtr msg)
{
    this->desired_velocity = msg->velocity;
    // RCLCPP_INFO(this->get_logger(),"spacemouse callback");
}

void gh360::InverseJacobian::joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // for (int i = 0; i < this->num_joints; i++) {
    //     this->current_joint_pos.position[i] = msg->position[i];
    // }
    this->current_joint_pos = *msg;
}

void gh360::InverseJacobian::timer_callback()
{
    KDL::JntArray q_current(this->num_joints);
    KDL::JntArray new_q(this->num_joints);

    // RCLCPP_INFO(this->get_logger(),std::to_string(sizeof(this->current_joint_pos.position)/sizeof(this->current_joint_pos.position[0])));

    for (int i = 0; i < this->num_joints; i++) {
        q_current(i) = this->current_joint_pos.position[i];
        // RCLCPP_INFO(this->get_logger(),std::to_string(q_current(i)));
    }

    // q_current = this->joint_vel_msg.position;
    float translation_scale = 0.1;
    float rotation_scale = 0.5;
    // KDL::Vector pos_vel(-this->desired_velocity.linear.z*translation_scale,
    //             this->desired_velocity.linear.y*translation_scale,
    //             this->desired_velocity.linear.x*translation_scale);
    KDL::Vector pos_vel(this->desired_velocity.linear.x*translation_scale,
                this->desired_velocity.linear.y*translation_scale,
                this->desired_velocity.linear.z*translation_scale);
    // KDL::Vector rot_vel(-this->desired_velocity.angular.z*translation_scale,        
    //             this->desired_velocity.angular.y*translation_scale,
    //             this->desired_velocity.angular.x*translation_scale);
    KDL::Vector rot_vel(-this->desired_velocity.angular.y*rotation_scale,
                this->desired_velocity.angular.x*rotation_scale,
                -this->desired_velocity.angular.z*rotation_scale);
    KDL::Twist T_desired = KDL::Twist(pos_vel, rot_vel);

    KDL::Frame F_current;
    int ret_fk = this->fk_solver->JntToCart(q_current,F_current);
    if(ret_fk < 0) {
        RCLCPP_INFO(this->get_logger(),"FK not solved!\n");
        assert(false);
    }

    // KDL::Frame F_at_hand = F_current;
    // F_at_hand.M = KDL::Rotation::Identity();
    // T_desired = F_at_hand * T_desired;

    // T_desired = F_current * T_desired;

    int ret_ik = this->ik_solver->CartToJnt(q_current,T_desired, new_q);
    if(ret_ik >= 0) {
        
        // for (int i = 0; i < num_joints_; i++) {
        //     this->joint_vel_msg.position[i] = new_q(i);
        // }
        for (int i = 0; i < this->num_joints; i++) {
            // this->joint_vel_msg.position[i] += new_q(i);
            this->joint_vel_msg.velocity[i] = new_q(i);
            // RCLCPP_INFO(this->get_logger(),std::to_string(q_current(i)));
        }

        // assert(false);
    }
    else {
        RCLCPP_INFO(this->get_logger(),"IK not solved!\n");
        assert(false);
    }

    this->joint_velocity_publisher_->publish(this->joint_vel_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto inverse_jacobian_node = std::make_shared<gh360::InverseJacobian>();
    rclcpp::spin(inverse_jacobian_node);
    rclcpp::shutdown();

    return 0;
}
#include "inverse_jacobian.hpp"

gh360::InverseJacobian::InverseJacobian(): Node("inverse_jacobian")
{
    KDL::Tree kdl_tree;
  
    this->declare_parameter("tcp_link_name", "eef");
    this->tcp_link_name = get_parameter("tcp_link_name").as_string();
    this->declare_parameter("robot_description", "");
    std::string robot_desc_string = get_parameter("robot_description").as_string();
    this->declare_parameter("joint_position_topic_name", "/gh360_joint_states");
    std::string joint_position_topic = get_parameter("joint_position_topic_name").as_string();


    if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)){
        RCLCPP_ERROR(this->get_logger(),"Failed to construct kdl tree");
    }
    
    kdl_tree.getChain("base_link", this->tcp_link_name, this->chain);

    this->num_joints = this->chain.getNrOfJoints();

    this->fk_solver = new KDL::ChainFkSolverPos_recursive(this->chain);
    this->ik_solver = new KDL::ChainIkSolverVel_pinv(this->chain);

    this->joint_vel_msg = sensor_msgs::msg::JointState();
    this->current_joint_pos = sensor_msgs::msg::JointState();
    for (unsigned int i = 0; i < this->chain.getNrOfSegments(); i++) {
        if (this->chain.getSegment(i).getJoint().getType() != KDL::Joint::None) {
            this->joint_vel_msg.name.push_back(this->chain.getSegment(i).getJoint().getName());
            this->joint_vel_msg.velocity.push_back(0.0);
            this->joint_vel_msg.position.push_back(0.0);
        }
    }
    
    this->current_joint_pos.name = {"shoulder_yaw", "shoulder_roll", "shoulder_pitch", "upperarm_roll", "elbow", "forearm_roll", "wrist_pitch"};
    this->current_joint_pos.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    this->desired_velocity = geometry_msgs::msg::Twist();

    // this->spacemouse_subscriber_ = this->create_subscription<gh360_interfaces::msg::SpaceMouse>("/spacemouse", 10, std::bind(&gh360::InverseJacobian::spacemouse_callback, this, std::placeholders::_1));
    this->cmd_eef_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_eef_vel", 10, std::bind(&gh360::InverseJacobian::cmd_eef_vel_callback, this, std::placeholders::_1));
    this->joint_position_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_position_topic, 10, std::bind(&gh360::InverseJacobian::joint_position_callback, this, std::placeholders::_1));

    this->joint_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/inverse_jacobian", 10);
    
    
    this->timer_ = this->create_wall_timer(10ms, std::bind(&gh360::InverseJacobian::timer_callback, this));
    
}

gh360::InverseJacobian::~InverseJacobian()
{
}

void gh360::InverseJacobian::cmd_eef_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    this->desired_velocity = *msg;
}

void gh360::InverseJacobian::joint_position_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    this->current_joint_pos = *msg;
}

void gh360::InverseJacobian::timer_callback()
{
    KDL::JntArray q_current(this->num_joints);
    KDL::JntArray new_q(this->num_joints);

    for (int i = 0; i < this->num_joints; i++) {
        q_current(i) = this->current_joint_pos.position[i];
    }

    float translation_scale = 0.1;
    float rotation_scale = 0.5;
    
    KDL::Vector pos_vel(this->desired_velocity.linear.x*translation_scale,
                this->desired_velocity.linear.y*translation_scale,
                this->desired_velocity.linear.z*translation_scale);
    KDL::Vector rot_vel(-this->desired_velocity.angular.y*rotation_scale,
                this->desired_velocity.angular.x*rotation_scale,
                -this->desired_velocity.angular.z*rotation_scale);
    KDL::Twist T_desired = KDL::Twist(pos_vel, rot_vel);

    // KDL::Frame F_current;
    // int ret_fk = this->fk_solver->JntToCart(q_current,F_current);
    // if(ret_fk < 0) {
    //     RCLCPP_INFO(this->get_logger(),"FK not solved!\n");
    //     assert(false);
    // }

    int ret_ik = this->ik_solver->CartToJnt(q_current,T_desired, new_q);
    if(ret_ik >= 0) {
        for (int i = 0; i < this->num_joints; i++) {
            this->joint_vel_msg.velocity[i] = new_q(i);
        }
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
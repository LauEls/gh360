#include "inverse_jacobian.hpp"

InverseJacobian::InverseJacobian(std::string robot_description, std::string base_link_name, std::string eef_link_name)
{
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree)){
        RCLCPP_ERROR(rclcpp::get_logger("eef_velocity"),"Failed to construct kdl tree");
    }
    
    kdl_tree.getChain(base_link_name, eef_link_name, this->chain);

    this->num_joints = this->chain.getNrOfJoints();
    this->ik_solver = new KDL::ChainIkSolverVel_pinv(this->chain);
}

InverseJacobian::~InverseJacobian()
{
}

std::vector<double> InverseJacobian::calculate_goal_joint_velocities(std::vector<double> desired_eef_velocity, std::vector<double> current_joint_pos)
{
    KDL::JntArray q_current(this->num_joints);
    KDL::JntArray new_q_vel(this->num_joints);
    std::vector<double> joint_goal_velocities;

    for (int i = 0; i < this->num_joints; i++) {
        q_current(i) = current_joint_pos[i];
    }
    
    KDL::Vector pos_vel(desired_eef_velocity[0],
                        desired_eef_velocity[1],
                        desired_eef_velocity[2]);
    KDL::Vector rot_vel(desired_eef_velocity[3],
                        desired_eef_velocity[4],
                        desired_eef_velocity[5]);
    KDL::Twist T_desired = KDL::Twist(pos_vel, rot_vel);
    int ret_ik = this->ik_solver->CartToJnt(q_current,T_desired, new_q_vel);
    if(ret_ik >= 0) {
        for (int i = 0; i < this->num_joints; i++) {
            joint_goal_velocities.push_back(new_q_vel(i));
        }
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("eef_velocity"),"IK not solved!\n");
        assert(false);
    }

    return joint_goal_velocities;
}
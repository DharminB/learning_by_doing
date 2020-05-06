#include <algorithm>
#include <math.h>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
/* #include <youbot_arm_kinematics/inverse_kinematics.h> */

class Kinematics
{
    public:
        Kinematics(KDL::Chain &chain);

    private:
        KDL::Chain my_chain;
        sensor_msgs::JointState joint_state_msg;
        std::vector<std::string> joint_names;
        std::vector<double> joint_lower_limits;
        std::vector<double> joint_upper_limits;

        KDL::ChainFkSolverPos_recursive fk_solver;
        KDL::ChainIkSolverVel_wdls ik_solver_vel;

        ros::NodeHandle nh;
        void initialiseJoints();
};

Kinematics::Kinematics(KDL::Chain &chain):
    nh("~"),
    my_chain(chain),
    fk_solver(chain),
    ik_solver_vel(chain)
{
    /* get the names and limits of all joints */
    initialiseJoints();

    KDL::JntArray q_min(this->my_chain.getNrOfJoints());
    KDL::JntArray q_max(this->my_chain.getNrOfJoints());
    for (int i = 0; i < this->my_chain.getNrOfJoints(); ++i)
    {
        q_min(i) = this->joint_lower_limits[i];    
        q_max(i) = this->joint_upper_limits[i];    
    }

    KDL::ChainIkSolverPos_NR_JL ik_solver(my_chain, q_min, q_max, fk_solver, ik_solver_vel);

    KDL::JntArray q_init(this->my_chain.getNrOfJoints());
    KDL::JntArray q_target(this->my_chain.getNrOfJoints());
    KDL::Rotation r = KDL::Rotation::RPY(3.14, 0.5, 3.14);
    KDL::Vector v = KDL::Vector(0.65, 0.0, 0.1);
    KDL::Frame target_frame(r, v);
    std::cout << "before ik" << std::endl;
    int stat = ik_solver.CartToJnt(q_init, target_frame, q_target);
    std::cout << "after ik" << std::endl;
    std::cout << stat << std::endl;
    for (int i = 0; i < this->my_chain.getNrOfJoints(); ++i) {
        std::cout << q_target(i) << std::endl;
    }
    KDL::Frame reached_frame;
    fk_solver.JntToCart(q_target, reached_frame);
    bool same = KDL::Equal(reached_frame, target_frame, 0.001);
    std::cout << "Same: " << same << std::endl;
    if (!same)
    {
        std::cout << reached_frame.p.x() << " " << reached_frame.p.y() << " " << reached_frame.p.z() << std::endl;
        double r, p, y;
        reached_frame.M.GetRPY(r, p, y);
        std::cout << r << " " << p << " " << y << std::endl;
    }

}

void Kinematics::initialiseJoints()
{
    for (KDL::Segment seg : this->my_chain.segments)
    {
        KDL::Joint joint = seg.getJoint();
        if (joint.getType() == KDL::Joint::JointType::None) continue;
        this->joint_names.push_back(joint.getName());
    }
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
        exit(1);
    }
    boost::shared_ptr<const urdf::Joint> joint;
    for (std::string joint_name : this->joint_names)
    {
        joint = model.getJoint(joint_name);
        if (!joint)
        {
            ROS_ERROR_STREAM("Failed to get joint " << joint_name << " from urdf");
            exit(1);
        }
        this->joint_lower_limits.push_back(joint->limits->lower);
        this->joint_upper_limits.push_back(joint->limits->upper);
    }
    /* for debugging */
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        std::cout << this->joint_names[i] << ": " 
                  << this->joint_lower_limits[i] << " - "
                  << this->joint_upper_limits[i] << std::endl;
    }

}

/* ======================================================================== */
/* ======================================================================== */

KDL::Chain getChainFromParam(std::string param_name)
{
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam(param_name, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree. Exiting.");
        exit(1);
    }

    /* create chain from tree */
    std::string base_name = "base_link";
    std::string ee_name = "gripper_static_grasp_link";

    KDL::Chain my_chain;
    if (!my_tree.getChain(base_name, ee_name, my_chain))
    {
        ROS_ERROR("Failed to construct chain from tree. Exiting.");
        exit(1);
    }

    ROS_INFO_STREAM("Successfully created kdl chain");
    std::cout << my_chain.getNrOfJoints() << " joints found." << std::endl;
    return my_chain;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "youbot_kinematics");
    KDL::Chain my_chain = getChainFromParam("/robot_description");
    Kinematics kinematics(my_chain);

    std::cout << "Exiting." << std::endl;

    return 0;
}

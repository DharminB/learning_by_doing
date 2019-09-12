#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_datatypes.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <geometry_msgs/Pose.h>

int main(int argc, char *argv[])
{
    KDL::Tree my_tree;
    KDL::Chain my_chain;

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");

    if (!kdl_parser::treeFromParam("/robot_description", my_tree))
    {
        ROS_ERROR("failed to construct kdl tree");
        return 1;
    }

    if (!my_tree.getChain("base_link", "end_effector", my_chain))
    {
        ROS_ERROR("failed to construct chain from tree");
        return 1;
    }

    ROS_INFO_STREAM("Successfully created kdl chain");
    std::cout << my_chain.getNrOfJoints() << " joints found in chain" << std::endl;

    KDL::JntArray joint_pos(my_chain.getNrOfJoints());
    for (int i = 0; i < joint_pos.rows(); i++)
    {
        std::cout << joint_pos(i) << std::endl;
    }
    joint_pos(1) = -3.0;
    KDL::Frame cart_pos;
    KDL::ChainFkSolverPos_recursive fk_solver(my_chain);
    fk_solver.JntToCart(joint_pos, cart_pos);

    geometry_msgs::Pose pose;
    tf::poseKDLToMsg(cart_pos, pose);
    ROS_INFO_STREAM(pose);

    KDL::ChainIkSolverPos_LMA ik_solver(my_chain);
    KDL::JntArray init_joint_pos(my_chain.getNrOfJoints());
    KDL::JntArray final_joint_pos(my_chain.getNrOfJoints());
    KDL::Frame ee_pose(KDL::Vector(0.84, 0.0, 0.98));
    int status = ik_solver.CartToJnt(init_joint_pos, ee_pose, final_joint_pos);
    std::cout << status << std::endl;
    for (int i = 0; i < final_joint_pos.rows(); i++)
    {
        std::cout << final_joint_pos(i) << std::endl;
    }


    ros::spinOnce();
    return 0;
}

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>

int main(int argc, char *argv[])
{
    KDL::Tree my_tree;
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh("~");
    std::string robot_desc_string;
    nh.param("/robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, my_tree))
    {
        ROS_ERROR("failed to construct kdl tree");
        return 1;
    }
    ros::spin();
    return 0;
}

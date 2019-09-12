#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>
#include <urdf/model.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

class ArmController
{
    private:
        int control_rate;
        ros::Publisher joint_state_pub;
        ros::Subscriber position_command_sub;
        KDL::Chain my_chain;
        sensor_msgs::JointState joint_state_msg;
        std::vector<std::string> joint_names;
        std::vector<double> joint_lower_limits;
        std::vector<double> joint_upper_limits;

        std::vector<double> target_joint_positions;
        std::vector<double> target_joint_velocities;
        std::vector<double> current_joint_positions;
        std::vector<double> current_joint_velocities;

        /* pid related variables */
        double proportional_factor = 0.5;
        double integral_factor = 0.1;
        double differential_factor = 0.1;
        std::vector<double> integral_value;
        std::vector<double> differential_value;
        ros::NodeHandle nh;

        /*
         * get the name of the segment which has no children
         * Assumption: manipulator has single chain. No segment has more than 1 child
         */
        std::string getEndEffector(KDL::Tree tree);

        void positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void fillJointStateMsg();

        void initialiseJoints();

        double positionPController(double current, double target);

    public:
        ArmController(int control_rate);
        void update();
};

ArmController::ArmController(int control_rate): nh("~")
{
    this->control_rate = control_rate;
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam("/robot_description", my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree. Exiting.");
        exit(1);
    }

    /* create chain from tree */
    std::string base_name = my_tree.getRootSegment()->first;
    std::string ee_name = getEndEffector(my_tree);

    if (!my_tree.getChain(base_name, ee_name, my_chain))
    {
        ROS_ERROR("Failed to construct chain from tree. Exiting.");
        exit(1);
    }

    ROS_INFO_STREAM("Successfully created kdl chain");
    std::cout << my_chain.getNrOfJoints() << " joints found." << std::endl;

    /* get the names and limits of all joints */
    initialiseJoints();

    /* create template message with zero joint angles */
    joint_state_msg.name = joint_names;
    for (int i = 0; i < joint_names.size(); i++)
    {
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.velocity.push_back(0.0);
    }

    /* initialise publisher and subscribers */
    joint_state_pub = nh.advertise<sensor_msgs::JointState> ("/joint_states", 1);
    position_command_sub = nh.subscribe<sensor_msgs::ChannelFloat32>
                                        ("position_command", 1,
                                         &ArmController::positionCommandCb, this);
}

void ArmController::update()
{
    std::cout << "Target position: ";
    for (double joint_position : this->target_joint_positions)
        std::cout << joint_position << " ";
    std::cout << std::endl;
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        this->current_joint_velocities[i] = this->positionPController(
                this->current_joint_positions[i], this->target_joint_positions[i]);
        this->current_joint_positions[i] += this->current_joint_velocities[i] / this->control_rate;
    }
    std::cout << "current position: ";
    for (double joint_position : this->current_joint_positions)
        std::cout << joint_position << " ";
    std::cout << std::endl;
    std::cout << "current velocities: ";
    for (double joint_position : this->current_joint_velocities)
        std::cout << joint_position << " ";
    std::cout << std::endl;
    this->fillJointStateMsg();
    ROS_INFO_STREAM(this->joint_state_msg);
    
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_pub.publish(joint_state_msg);
}

double ArmController::positionPController(double current, double target)
{
    double error = target - current;
    std::cout << error << std::endl;
    std::cout << fabs(error) << std::endl;
    if (fabs(error) < 0.001)
        return 0.0;
    double velocity = error * this->proportional_factor;
    std::cout << velocity << std::endl;
    return velocity;
}

void ArmController::positionCommandCb
                    (const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    ROS_INFO_STREAM(*msg);
    /* check for number of values provided */
    if (msg->values.size() != this->joint_names.size())
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << this->joint_names.size() << " values.");
        return;
    }
    /* check for joint limits */
    for (int i = 0; i < msg->values.size(); i++)
    {
        if (msg->values[i] < this->joint_lower_limits[i] 
                || msg->values[i] > this->joint_upper_limits[i])
        {
            ROS_WARN_STREAM("Joint value for " << this->joint_names[i]
                            << " is outside limit! Expecting values between "
                            << this->joint_lower_limits[i] << " and "
                            << this->joint_upper_limits[i] << ".");
            return;
        }
    }
    /* set target joint positions and reset target joint vel */
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        this->target_joint_positions[i] = msg->values[i];
        this->target_joint_velocities[i] = 0.0;
    }
}

void ArmController::fillJointStateMsg()
{
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        this->joint_state_msg.position[i] = this->current_joint_positions[i];
        this->joint_state_msg.velocity[i] = this->current_joint_velocities[i];
    }
}

void ArmController::initialiseJoints()
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

    /* initialising all joints and targets to zero */
    for (std::string joint_name : this->joint_names)
    {
        this->current_joint_positions.push_back(0.0);
        this->current_joint_velocities.push_back(0.0);
        this->target_joint_positions.push_back(0.0);
        this->target_joint_velocities.push_back(0.0);
        this->integral_value.push_back(0.0);
        this->differential_value.push_back(0.0);
    }
}

std::string ArmController::getEndEffector(KDL::Tree tree)
{
    std::string end_effector_name;
    KDL::SegmentMap sm = tree.getSegments();
    for (auto segment : sm)
    {
        auto children = segment.second.children;
        if (children.size() == 0)
        {
            end_effector_name = segment.first;
            break;
        }
    }
    return end_effector_name;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "arm_controller");
    int control_rate = 10;
    ArmController arm_controller(control_rate);

    ros::Rate rate(control_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        arm_controller.update();
        rate.sleep();
    }
    std::cout << "Exiting." << std::endl;

    return 0;
}

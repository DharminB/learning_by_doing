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
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include <pid_controller.cpp>
#include <kinematics.cpp>

class ArmController
{
    private:
        int control_rate;

        KDL::Chain my_chain;
        sensor_msgs::JointState joint_state_msg;
        std::vector<std::string> joint_names;
        std::vector<double> joint_lower_limits;
        std::vector<double> joint_upper_limits;
        /* KDL::ChainFkSolverPos_recursive *fk_solver; */
        /* KDL::ChainIkSolverPos_NR_JL *ik_solver; */
        Kinematics kinematics;

        std::vector<double> target_joint_positions;
        std::vector<double> target_joint_velocities;
        std::vector<double> current_joint_positions;
        std::vector<double> current_joint_velocities;
        std::vector<PIDController> position_controllers;

        /* pid related variables */
        double proportional_factor = 1.5;
        double integral_factor = 0.005;
        double differential_factor = 0.1;
        double position_threshold = 0.001;
        double velocity_threshold = 0.001;
        double max_vel = 0.5;
        double min_vel = 0.001;

        ros::NodeHandle nh;
        std::vector<ros::Publisher> joint_vel_pubs;
        ros::Subscriber point_command_sub;
        ros::Subscriber position_command_sub;
        ros::Subscriber velocity_command_sub;
        ros::Subscriber joint_state_sub;

        /*
         * get the name of the segment which has no children
         * Assumption: manipulator has single chain. No segment has more than 1 child
         */
        /* std::string getEndEffector(KDL::Tree tree); */

        void pointCommandCb(const geometry_msgs::PointStamped::ConstPtr& msg);

        void positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void velocityCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

        /* bool getIK(double x, double y, double z, std::vector<double> &joint_positions); */

        void publishJointVelocity(int joint_index, double joint_vel);

        void initialiseJoints();

    public:
        ArmController(KDL::Chain &chain, int control_rate);
        void publishZeroVelocity();
        void update();
};

ArmController::ArmController(KDL::Chain &chain, int control_rate):
    nh("~"),
    my_chain(chain),
    kinematics(chain)
{
    this->control_rate = control_rate;

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
    for (int i = 0; i < joint_names.size(); i++)
    {
        std::string topic_name = "/joint" + std::to_string(i+1) + "_velocity/command";
        ros::Publisher joint_vel_pub = nh.advertise<std_msgs::Float64> (topic_name, 1);
        this->joint_vel_pubs.push_back(joint_vel_pub);
    }
    joint_state_sub = nh.subscribe<sensor_msgs::JointState>
                                        ("/joint_states", 1,
                                         &ArmController::jointStateCb, this);
    point_command_sub = nh.subscribe<geometry_msgs::PointStamped>
                                        ("point_command", 1,
                                         &ArmController::pointCommandCb, this);
    position_command_sub = nh.subscribe<sensor_msgs::ChannelFloat32>
                                        ("position_command", 1,
                                         &ArmController::positionCommandCb, this);
    velocity_command_sub = nh.subscribe<sensor_msgs::ChannelFloat32>
                                        ("velocity_command", 1,
                                         &ArmController::velocityCommandCb, this);

    this->kinematics.setJointLimits(this->joint_lower_limits, this->joint_upper_limits);
    /* std::vector<double> joints({0.0, 1.0, 0.0}); */
    /* double x, y, z; */
    /* this->kinematics.findFK(joints, x, y, z); */
    /* std::cout << x << " " << y << " " << z << std::endl; */
}

void ArmController::update()
{
    bool moved_arm_using_pos_cmd = false;
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        /* std::cout << this->current_joint_positions[i] << std::endl; */
        if (this->target_joint_positions.size() > 0)
        {
            double vel = this->position_controllers[i].control(this->current_joint_positions[i],
                                                        this->target_joint_positions[i]);
            double safe_vel;
            if (vel != 0.0)
            {
                int sign = (vel > 0.0) ? 1 : -1;
                safe_vel = sign * std::max(this->min_vel, std::min(this->max_vel, fabs(vel)));
                this->publishJointVelocity(i, safe_vel);
                moved_arm_using_pos_cmd = true;
            }
        }
        if (this->target_joint_velocities.size() > 0)
            this->publishJointVelocity(i, this->target_joint_velocities[i]);

        /* safety condition */
        double future_position = this->current_joint_positions[i] + this->current_joint_velocities[i] / this->control_rate;
        if (future_position < this->joint_lower_limits[i] 
                || future_position > this->joint_upper_limits[i])
        {
            ROS_WARN_STREAM("Joint value for " << this->joint_names[i]
                            << " is outside limit! Stopping motion.");
            this->publishJointVelocity(i, 0.0);
        }
    }
    if (this->target_joint_velocities.size() > 0)
        this->target_joint_velocities.clear();

    if (this->target_joint_positions.size() > 0 && !moved_arm_using_pos_cmd)
    {
        this->target_joint_positions.clear();
        std::cout << "Reached target position" << std::endl;
    }

    /* ROS_INFO_STREAM(this->joint_state_msg); */
}

void ArmController::jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    /* ROS_INFO_STREAM(*msg); */
    if (msg->name.size() != this->joint_names.size())
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << this->joint_names.size() << " values.");
        return;
    }
    for (int i = 0; i < msg->position.size(); i++)
    {
        this->current_joint_positions[i] = msg->position[i];
        this->current_joint_velocities[i] = msg->velocity[i];
    }
}

void ArmController::pointCommandCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO_STREAM(*msg);
    if (msg->header.frame_id != "base_link")
    {
        ROS_WARN_STREAM("Frame should be base_link. Ignoring.");
        return;
    }
    /* check for ground */
    if (msg->point.z < 0.05)
    {
        ROS_WARN_STREAM("Target point below ground. Ignoring.");
        return;
    }
    std::vector<double> joint_pos;
    bool success = this->kinematics.findNearestIK(msg->point.x, msg->point.y, msg->point.z,
                                                  this->current_joint_positions,
                                                  joint_pos);
    std::cout << success << std::endl;
    if (success)
    {
        this->target_joint_positions.clear();
        for (double i : joint_pos)
        {
            this->target_joint_positions.push_back(i);
            std::cout << i << std::endl;
        }
    }
    else
        ROS_WARN_STREAM("Could not find a valid IK solution!");
}

void ArmController::positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
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
    this->target_joint_positions.clear();
    this->target_joint_velocities.clear();
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        this->target_joint_positions.push_back(msg->values[i]);
        this->position_controllers[i].reset();
    }
}

void ArmController::velocityCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
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
        if (msg->values[i] < -1 * this->max_vel
                || msg->values[i] > this->max_vel)
        {
            ROS_WARN_STREAM("Joint velocity for " << this->joint_names[i]
                            << " is outside limit! Expecting values between "
                            << -1 * this->max_vel << " and "
                            << this->max_vel << ".");
            return;
        }
    }
    /* set target joint positions and reset target joint vel */
    this->target_joint_positions.clear();
    this->target_joint_velocities.clear();
    for (int i = 0; i < this->joint_names.size(); i++)
    {
        this->target_joint_velocities.push_back(msg->values[i]);
        this->position_controllers[i].reset();
    }
}

void ArmController::publishZeroVelocity()
{
    for (int i = 0; i < this->my_chain.getNrOfJoints(); ++i) {
        this->publishJointVelocity(i, 0.0);
    }
}

void ArmController::publishJointVelocity(int joint_index, double joint_vel)
{
    std_msgs::Float64 msg;
    msg.data = joint_vel;
    this->joint_vel_pubs[joint_index].publish(msg);
    /* std::cout << joint_index << " " << joint_vel << std::endl; */
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
        /* this->target_joint_positions.push_back(0.0); */
        /* this->target_joint_velocities.push_back(0.0); */
        PIDController pid(this->proportional_factor, this->integral_factor,
                          this->differential_factor, this->control_rate,
                          this->position_threshold);
        this->position_controllers.push_back(pid);
    }
}

std::string getEndEffector(KDL::Tree tree)
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

KDL::Chain getChainFromParam(std::string param_name)
{
    KDL::Tree my_tree;
    if (!kdl_parser::treeFromParam(param_name, my_tree))
    {
        ROS_ERROR("Failed to construct kdl tree. Exiting.");
        exit(1);
    }

    /* create chain from tree */
    std::string base_name = my_tree.getRootSegment()->first;
    std::string ee_name = getEndEffector(my_tree);

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
    ros::init(argc, argv, "arm_controller");
    KDL::Chain my_chain = getChainFromParam("/robot_description");
    int control_rate = 10;
    ArmController arm_controller(my_chain, control_rate);

    ros::Rate rate(control_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        arm_controller.update();
        rate.sleep();
    }
    arm_controller.publishZeroVelocity();
    std::cout << "Exiting." << std::endl;

    return 0;
}

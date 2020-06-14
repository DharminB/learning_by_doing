#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/PointStamped.h>

#include <tf/transform_datatypes.h>
#include <urdf/model.h>

#include <pid_controller.cpp>
#include <my_simple_manipulator/kinematics.h>

class ArmController
{
    public:
        ArmController();
        void publishZeroVelocity();
        void update();

    private:
        int control_rate;

        size_t num_of_joints_;
        sensor_msgs::JointState joint_state_msg;
        std::vector<std::string> joint_names;
        std::vector<double> joint_lower_limits;
        std::vector<double> joint_upper_limits;
        Kinematics kinematics;
        int cart_vel_countdown, cart_vel_countdown_start = 10;

        std::vector<double> target_joint_positions;
        std::vector<double> target_joint_velocities;
        std::vector<double> current_joint_positions;
        std::vector<double> current_joint_velocities;
        std::vector<PIDController> position_controllers;

        /* pid related variables */
        double proportional_factor = 2.0;
        double integral_factor = 0.001;
        double differential_factor = 0.2;
        double i_clamp = 2.0;
        double position_threshold = 0.001;
        double max_vel = 0.5;
        double min_vel = 0.0001;

        ros::NodeHandle nh;
        std::vector<ros::Publisher> joint_vel_pubs;
        ros::Subscriber point_command_sub;
        ros::Subscriber position_command_sub;
        ros::Subscriber cart_vel_command_sub;
        ros::Subscriber velocity_command_sub;
        ros::Subscriber joint_state_sub;

        void CartVelCommandCb(const geometry_msgs::Vector3::ConstPtr& msg);

        void pointCommandCb(const geometry_msgs::PointStamped::ConstPtr& msg);

        void positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void velocityCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

        void publishJointVelocity(int joint_index, double joint_vel);

        void initialiseJoints();
};
#endif

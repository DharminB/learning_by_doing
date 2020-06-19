#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>

#include <urdf/model.h>

#include <my_simple_manipulator/pid_controller.h>
#include <my_simple_manipulator/kinematics.h>

class ArmController
{
    public:
        ArmController();
        void publishZeroVelocity();
        void update();

    private:
        float control_rate_;

        size_t num_of_joints_;
        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;
        boost::shared_ptr<Kinematics> kinematics_;
        ros::Time last_cart_vel_msg_time_;
        ros::Duration cart_vel_valid_msg_duration_;
        bool cart_vel_set_;

        std::vector<double> target_joint_positions_;
        std::vector<double> target_joint_velocities_;
        std::vector<double> current_joint_positions_;
        std::vector<double> current_joint_velocities_;
        std::vector<PIDController> position_controllers_;

        float max_vel_;
        float min_vel_;

        ros::NodeHandle nh_;
        std::vector<ros::Publisher> joint_vel_pubs_;
        ros::Subscriber point_command_sub_;
        ros::Subscriber position_command_sub_;
        ros::Subscriber cart_vel_command_sub_;
        ros::Subscriber velocity_command_sub_;
        ros::Subscriber joint_state_sub_;
        ros::Subscriber cancel_sub_;

        void CartVelCommandCb(const geometry_msgs::Vector3::ConstPtr& msg);

        void pointCommandCb(const geometry_msgs::PointStamped::ConstPtr& msg);

        void positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void velocityCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg);

        void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg);

        void cancelCb(const std_msgs::Empty::ConstPtr& msg);

        void publishJointVelocity(int joint_index, double joint_vel);
};
#endif

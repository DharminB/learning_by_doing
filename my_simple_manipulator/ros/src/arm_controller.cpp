#include <my_simple_manipulator/arm_controller.h>

#include <math.h>
#include <std_msgs/Float64.h>

ArmController::ArmController():
    nh_("~")
{
    /* read ros params */
    nh_.param<float>("control_rate", control_rate_, 10.0);
    int num_joints;
    nh_.param<int>("num_of_joints", num_joints, 3);
    num_of_joints_ = (size_t)num_joints;
    nh_.param<float>("proportional_factor", proportional_factor_, 2.0);
    nh_.param<float>("integral_factor", integral_factor_, 0.001);
    nh_.param<float>("differential_factor", differential_factor_, 0.2);
    nh_.param<float>("i_clamp", i_clamp_, 2.0);
    nh_.param<float>("position_tolerance", position_tolerance_, 0.001);
    nh_.param<float>("max_vel", max_vel_, 0.5);
    nh_.param<float>("min_vel", min_vel_, 0.0001);
    float link_length_1, link_length_2, link_length_3;
    nh_.param<float>("link_length_1", link_length_1, 0.4);
    nh_.param<float>("link_length_2", link_length_2, 0.6);
    nh_.param<float>("link_length_3", link_length_3, 0.4);

    kinematics_.reset(new Kinematics(link_length_1, link_length_2, link_length_3));

    /* get the names and limits of all joints and initialise PID controller for
     * each joint */
    initialiseJoints();

    /* initialise publisher and subscribers */
    for (int i = 0; i < num_of_joints_; i++)
    {
        std::string topic_name = "/joint" + std::to_string(i+1) + "_velocity/command";
        ros::Publisher joint_vel_pub = nh_.advertise<std_msgs::Float64> (topic_name, 1);
        joint_vel_pubs_.push_back(joint_vel_pub);
    }
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states",
            1, &ArmController::jointStateCb, this);
    point_command_sub_ = nh_.subscribe<geometry_msgs::PointStamped> ("point_command",
            1, &ArmController::pointCommandCb, this);
    position_command_sub_ = nh_.subscribe<sensor_msgs::ChannelFloat32>("position_command",
            1, &ArmController::positionCommandCb, this);
    cart_vel_command_sub_ = nh_.subscribe<geometry_msgs::Vector3>("cart_vel_command",
            1, &ArmController::CartVelCommandCb, this);
    velocity_command_sub_ = nh_.subscribe<sensor_msgs::ChannelFloat32>("velocity_command",
            1, &ArmController::velocityCommandCb, this);

    kinematics_->setJointLimits(joint_lower_limits_, joint_upper_limits_);
}

void ArmController::update()
{
    bool moved_arm_using_pos_cmd = false;
    for (int i = 0; i < num_of_joints_; i++)
    {
        if (target_joint_positions_.size() > 0)
        {
            float vel = position_controllers_[i].control(current_joint_positions_[i],
                                                        target_joint_positions_[i]);
            float safe_vel;
            if (vel != 0.0)
            {
                int sign = (vel > 0.0) ? 1 : -1;
                safe_vel = sign * std::max(min_vel_, std::min(max_vel_, (float)fabs(vel)));
                publishJointVelocity(i, safe_vel);
                moved_arm_using_pos_cmd = true;
            }
        }
        if (target_joint_velocities_.size() > 0)
        {
            publishJointVelocity(i, target_joint_velocities_[i]);
        }

        /* safety condition */
        double future_position = current_joint_positions_[i]
                                 + current_joint_velocities_[i] / control_rate_;
        if (future_position < joint_lower_limits_[i] 
                || future_position > joint_upper_limits_[i])
        {
            ROS_WARN_STREAM("Joint value for joint " << i+1
                            << " is out of limit! Stopping motion.");
            publishJointVelocity(i, 0.0);
        }
    }
    if (target_joint_velocities_.size() > 0)
    {
        if (cart_vel_countdown_ <= 0)
        {
            target_joint_velocities_.clear();
            publishZeroVelocity();
        }
        else
        {
            cart_vel_countdown_--;
        }
    }

    if (target_joint_positions_.size() > 0 && !moved_arm_using_pos_cmd)
    {
        target_joint_positions_.clear();
        ROS_INFO("Reached target position");
        publishZeroVelocity();
    }
}

void ArmController::jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->name.size() != num_of_joints_)
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << num_of_joints_ << " values.");
        return;
    }
    for (int i = 0; i < num_of_joints_; i++)
    {
        current_joint_positions_[i] = msg->position[i];
        current_joint_velocities_[i] = msg->velocity[i];
    }
}

void ArmController::pointCommandCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
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
    bool success = kinematics_->findNearestIK(msg->point.x,
                                                  msg->point.y,
                                                  msg->point.z, 
                                                  current_joint_positions_,
                                                  joint_pos);
    if (success)
    {
        /* set target joint positions and reset target joint vel */
        target_joint_positions_.clear();
        target_joint_velocities_.clear();

        std::cout << "IK solution:";
        for (double i = 0; i < num_of_joints_; ++i)
        {
            target_joint_positions_.push_back(joint_pos[i]);
            position_controllers_[i].reset();
            std::cout << " " << joint_pos[i];
        }
        std::cout << std::endl;
    }
    else
    {
        ROS_WARN_STREAM("Could not find a valid IK solution!");
    }
}

void ArmController::positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    /* check for number of values provided */
    if (msg->values.size() != num_of_joints_)
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << num_of_joints_ << " values.");
        return;
    }
    /* check for joint limits */
    for (int i = 0; i < num_of_joints_; i++)
    {
        if (msg->values[i] < joint_lower_limits_[i] 
                || msg->values[i] > joint_upper_limits_[i])
        {
            ROS_WARN_STREAM("Joint value for joint " << i+1
                            << " is out of limit! Expecting values between "
                            << joint_lower_limits_[i] << " and "
                            << joint_upper_limits_[i] << ".");
            return;
        }
    }
    /* set target joint positions and reset target joint vel */
    target_joint_positions_.clear();
    target_joint_velocities_.clear();
    for (int i = 0; i < num_of_joints_; i++)
    {
        target_joint_positions_.push_back(msg->values[i]);
        position_controllers_[i].reset();
    }
}

void ArmController::CartVelCommandCb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    std::cout << *msg;
    std::vector<double> out_vel;
    kinematics_->findIKVel(msg->x, msg->y, msg->z, current_joint_positions_, out_vel);

    /* filter vel */
    std::cout << "Calculated vel:";
    for (double i = 0; i < out_vel.size(); ++i)
    {
        std::cout << " " << out_vel[i];
        if (fabs(out_vel[i]) < min_vel_)
            out_vel[i] = 0.0;
        if (fabs(out_vel[i]) > max_vel_)
        {
            ROS_WARN_STREAM("Calculated velocity exceeded max vel. Ignoring.");
            return;
        }
    }
    std::cout << std::endl;

    /* set target joint positions and reset target joint vel */
    target_joint_positions_.clear();
    target_joint_velocities_.clear();

    for (double i = 0; i < out_vel.size(); ++i)
    {
            target_joint_velocities_.push_back(out_vel[i]);
            position_controllers_[i].reset();
    }
    cart_vel_countdown_ = cart_vel_countdown_start_;
}

void ArmController::velocityCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    /* check for number of values provided */
    if (msg->values.size() != num_of_joints_)
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << num_of_joints_ << " values.");
        return;
    }
    /* check for joint limits */
    for (int i = 0; i < num_of_joints_; i++)
    {
        if (msg->values[i] < -1 * max_vel_
                || msg->values[i] > max_vel_)
        {
            ROS_WARN_STREAM("Joint velocity for joint " << i+1
                            << " is outside limit! Expecting values between "
                            << -1 * max_vel_ << " and "
                            << max_vel_ << ".");
            return;
        }
    }
    /* set target joint positions and reset target joint vel */
    target_joint_positions_.clear();
    target_joint_velocities_.clear();
    for (int i = 0; i < num_of_joints_; i++)
    {
        target_joint_velocities_.push_back(msg->values[i]);
        position_controllers_[i].reset();
    }
}

void ArmController::publishZeroVelocity()
{
    for (int i = 0; i < num_of_joints_; ++i)
    {
        publishJointVelocity(i, 0.0);
    }
}

void ArmController::publishJointVelocity(int joint_index, double joint_vel)
{
    std_msgs::Float64 msg;
    msg.data = joint_vel;
    joint_vel_pubs_[joint_index].publish(msg);
}

void ArmController::initialiseJoints()
{
    urdf::Model model;
    if (!model.initParam("/robot_description"))
    {
        ROS_ERROR("Failed to parse urdf file");
        exit(1);
    }
    boost::shared_ptr<const urdf::Joint> joint;
    for (int i = 0; i < num_of_joints_; ++i)
    {
        joint = model.getJoint("joint"+std::to_string(i+1));
        if (!joint)
        {
            ROS_ERROR_STREAM("Failed to get joint joint" << i+1 << " from urdf");
            exit(1);
        }
        joint_lower_limits_.push_back(joint->limits->lower);
        joint_upper_limits_.push_back(joint->limits->upper);
    }
    /* for debugging */
    std::cout << "Joint limits:" << std::endl;
    for (int i = 0; i < num_of_joints_; i++)
    {
        std::cout << "joint" << i+1 << ": " 
                  << joint_lower_limits_[i] << " - "
                  << joint_upper_limits_[i] << std::endl;
    }

    /* initialising all joints and targets to zero */
    for (int i = 0; i < num_of_joints_; i++)
    {
        current_joint_positions_.push_back(0.0);
        current_joint_velocities_.push_back(0.0);
        PIDController pid(proportional_factor_, integral_factor_,
                          differential_factor_, control_rate_,
                          position_tolerance_, i_clamp_);
        position_controllers_.push_back(pid);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    int control_rate = 10;
    ArmController arm_controller;

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

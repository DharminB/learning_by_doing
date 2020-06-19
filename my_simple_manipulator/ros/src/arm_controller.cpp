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
    float link_length_1, link_length_2, link_length_3;
    nh_.param<float>("link_length_1", link_length_1, 0.4);
    nh_.param<float>("link_length_2", link_length_2, 0.6);
    nh_.param<float>("link_length_3", link_length_3, 0.4);
    nh_.param<float>("max_vel", max_vel_, 0.5);
    nh_.param<float>("min_vel", min_vel_, 0.0001);
    float cart_vel_duration;
    nh_.param<float>("cart_vel_valid_msg_duration", cart_vel_duration, 0.5);
    cart_vel_valid_msg_duration_ = ros::Duration(cart_vel_duration);
    last_cart_vel_msg_time_ = ros::Time::now();
    nh_.getParam("lower_joint_limits", joint_lower_limits_);
    nh_.getParam("upper_joint_limits", joint_upper_limits_);

    kinematics_.reset(new Kinematics(link_length_1, link_length_2, link_length_3));
    kinematics_->setJointLimits(joint_lower_limits_, joint_upper_limits_);

    /* initialise PID controller for each joint */
    float proportional_factor, integral_factor, differential_factor, i_clamp, position_tolerance;
    nh_.param<float>("proportional_factor", proportional_factor, 2.0);
    nh_.param<float>("integral_factor", integral_factor, 0.001);
    nh_.param<float>("differential_factor", differential_factor, 0.2);
    nh_.param<float>("i_clamp", i_clamp, 2.0);
    nh_.param<float>("position_tolerance", position_tolerance, 0.001);
    for (int i = 0; i < num_of_joints_; i++)
    {
        PIDController pid(proportional_factor, integral_factor, differential_factor,
                control_rate_, position_tolerance, i_clamp);
        position_controllers_.push_back(pid);
    }

    /* initialising all joints positions and velocities to zero */
    for (int i = 0; i < num_of_joints_; i++)
    {
        current_joint_positions_.push_back(0.0);
        current_joint_velocities_.push_back(0.0);
    }

    cart_vel_set_ = false;

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
    cancel_sub_ = nh_.subscribe<std_msgs::Empty>("cancel",
            1, &ArmController::cancelCb, this);

    ROS_INFO("Arm Controller Initialised");
}

void ArmController::update()
{
    bool moved_arm_using_pos_cmd = false;
    for (int i = 0; i < num_of_joints_; i++)
    {
        float safe_velocity = 0.0;
        if (target_joint_positions_.size() > 0)
        {
            float vel = position_controllers_[i].control(current_joint_positions_[i],
                                                        target_joint_positions_[i]);
            if (vel != 0.0)
            {
                int sign = (vel > 0.0) ? 1 : -1;
                safe_velocity = sign * std::max(min_vel_, std::min(max_vel_, (float)fabs(vel)));
                moved_arm_using_pos_cmd = true;
            }
        }
        if (target_joint_velocities_.size() > 0)
        {
            safe_velocity = target_joint_velocities_[i];
        }

        /* safety condition */
        double future_position = current_joint_positions_[i]
                                 + safe_velocity / control_rate_;
        if ( fabs(safe_velocity) > min_vel_ &&
             ( future_position < joint_lower_limits_[i] 
               || future_position > joint_upper_limits_[i]) )
        {
            std::cout << safe_velocity << std::endl;
            std::cout << i << " " << future_position << std::endl;
            ROS_WARN_STREAM("Joint value for joint " << i+1
                            << " is out of limit! Stopping motion.");
            publishZeroVelocity();
            target_joint_velocities_.clear();
            target_joint_positions_.clear();
            return;
        }
        else
        {
            publishJointVelocity(i, safe_velocity);
        }
    }

    if ( cart_vel_set_ &&
         target_joint_velocities_.size() > 0 &&
         ros::Time::now() - last_cart_vel_msg_time_ > cart_vel_valid_msg_duration_ )
    {
        target_joint_velocities_.clear();
        publishZeroVelocity();
        cart_vel_set_ = false;
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
    last_cart_vel_msg_time_ = ros::Time::now();
    cart_vel_set_ = true;
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

void ArmController::cancelCb(const std_msgs::Empty::ConstPtr& msg)
{
    if ( target_joint_positions_.size() > 0 || target_joint_velocities_.size() > 0 )
    target_joint_positions_.clear();
    target_joint_velocities_.clear();
    for (int i = 0; i < num_of_joints_; i++)
    {
        position_controllers_[i].reset();
    }
    publishZeroVelocity();
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ArmController arm_controller;

    ros::NodeHandle nh("~");

    float control_rate;
    nh.param<float>("control_rate", control_rate, 10.0);
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

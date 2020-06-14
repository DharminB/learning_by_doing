#include <my_simple_manipulator/arm_controller.h>

#include <algorithm>
#include <math.h>
#include <std_msgs/Float64.h>

ArmController::ArmController():
    nh("~"),
    /* my_chain(chain), */
    kinematics(0.4, 0.6, 0.4)
{
    this->control_rate = 10;
    num_of_joints_ = 3;

    /* get the names and limits of all joints */
    initialiseJoints();

    /* create template message with zero joint angles */
    joint_state_msg.name = joint_names;
    for (int i = 0; i < num_of_joints_; i++)
    {
        joint_state_msg.position.push_back(0.0);
        joint_state_msg.velocity.push_back(0.0);
    }

    /* initialise publisher and subscribers */
    for (int i = 0; i < num_of_joints_; i++)
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
    cart_vel_command_sub = nh.subscribe<geometry_msgs::Vector3>
                                        ("cart_vel_command", 1,
                                         &ArmController::CartVelCommandCb, this);
    velocity_command_sub = nh.subscribe<sensor_msgs::ChannelFloat32>
                                        ("velocity_command", 1,
                                         &ArmController::velocityCommandCb, this);

    this->kinematics.setJointLimits(this->joint_lower_limits, this->joint_upper_limits);
}

void ArmController::update()
{
    bool moved_arm_using_pos_cmd = false;
    for (int i = 0; i < num_of_joints_; i++)
    {
        /* std::cout << this->current_joint_positions[i] << std::endl; */
        if (this->target_joint_positions.size() > 0)
        {
            /* std::cout << i << std::endl; */
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
        {
            this->publishJointVelocity(i, this->target_joint_velocities[i]);
        }

        /* safety condition */
        double future_position = this->current_joint_positions[i] + this->current_joint_velocities[i] / this->control_rate;
        if (future_position < this->joint_lower_limits[i] 
                || future_position > this->joint_upper_limits[i])
        {
            ROS_WARN_STREAM("Joint value for " << this->joint_names[i]
                            << " is out of limit! Stopping motion.");
            std::cerr << "Joint value out of limit. Stopping motion" << std::endl;
            this->publishJointVelocity(i, 0.0);
        }
    }
    if (this->target_joint_velocities.size() > 0)
    {
        if (this->cart_vel_countdown <= 0)
        {
            this->target_joint_velocities.clear();
            this->publishZeroVelocity();
        }
        else
        {
            this->cart_vel_countdown--;
        }
    }

    if (this->target_joint_positions.size() > 0 && !moved_arm_using_pos_cmd)
    {
        this->target_joint_positions.clear();
        std::cout << "Reached target position" << std::endl;
        this->publishZeroVelocity();
    }

    /* ROS_INFO_STREAM(this->joint_state_msg); */
}

void ArmController::jointStateCb(const sensor_msgs::JointState::ConstPtr& msg)
{
    /* ROS_INFO_STREAM(*msg); */
    if (msg->name.size() != num_of_joints_)
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << num_of_joints_ << " values.");
        std::cerr << "Number of joint mismatch" << std::endl;
        return;
    }
    for (int i = 0; i < num_of_joints_; i++)
    {
        this->current_joint_positions[i] = msg->position[i];
        this->current_joint_velocities[i] = msg->velocity[i];
    }
}

void ArmController::pointCommandCb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    std::cout << *msg;
    if (msg->header.frame_id != "base_link")
    {
        ROS_WARN_STREAM("Frame should be base_link. Ignoring.");
        std::cerr << "Frame shoud be base_link" << std::endl;
        return;
    }
    /* check for ground */
    if (msg->point.z < 0.05)
    {
        ROS_WARN_STREAM("Target point below ground. Ignoring.");
        std::cerr << "Target point is below ground. Ignoring." << std::endl;
        return;
    }
    std::vector<double> joint_pos;
    bool success = this->kinematics.findNearestIK(msg->point.x,
                                                  msg->point.y,
                                                  msg->point.z, 
                                                  this->current_joint_positions,
                                                  joint_pos);
    /* std::cout << success << std::endl; */
    if (success)
    {
        /* set target joint positions and reset target joint vel */
        this->target_joint_positions.clear();
        this->target_joint_velocities.clear();

        std::cout << "IK solution:";
        for (double i = 0; i < num_of_joints_; ++i)
        {
            this->target_joint_positions.push_back(joint_pos[i]);
            this->position_controllers[i].reset();
            std::cout << " " << joint_pos[i];
        }
        std::cout << std::endl;
    }
    else
    {
        ROS_WARN_STREAM("Could not find a valid IK solution!");
        std::cerr << "Could not find a valid IK solution" << std::endl;
    }
}

void ArmController::positionCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    std::cout << *msg;
    /* check for number of values provided */
    if (msg->values.size() != num_of_joints_)
    {
        ROS_WARN_STREAM("Number of joints mismatch! Expecting " 
                        << num_of_joints_ << " values.");
        std::cerr << "Number of joint mismatch" << std::endl;
        return;
    }
    /* check for joint limits */
    for (int i = 0; i < num_of_joints_; i++)
    {
        if (msg->values[i] < this->joint_lower_limits[i] 
                || msg->values[i] > this->joint_upper_limits[i])
        {
            ROS_WARN_STREAM("Joint value for " << this->joint_names[i]
                            << " is out of limit! Expecting values between "
                            << this->joint_lower_limits[i] << " and "
                            << this->joint_upper_limits[i] << ".");
            std::cerr << "Joint value out of limit" << std::endl;
            return;
        }
    }
    /* set target joint positions and reset target joint vel */
    this->target_joint_positions.clear();
    this->target_joint_velocities.clear();
    for (int i = 0; i < num_of_joints_; i++)
    {
        this->target_joint_positions.push_back(msg->values[i]);
        this->position_controllers[i].reset();
    }
}

void ArmController::CartVelCommandCb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    std::cout << *msg;
    std::vector<double> out_vel;
    this->kinematics.findIKVel(msg->x, msg->y, msg->z, this->current_joint_positions, out_vel);

    /* filter vel */
    std::cout << "Calculated vel:";
    for (double i = 0; i < out_vel.size(); ++i)
    {
        std::cout << " " << out_vel[i];
        if (fabs(out_vel[i]) < this->min_vel)
            out_vel[i] = 0.0;
        if (fabs(out_vel[i]) > this->max_vel)
        {
            std::cout << "Calculated velocity exceeded max vel. Ignoring." << std::endl;
            return;
        }
    }
    std::cout << std::endl;

    /* set target joint positions and reset target joint vel */
    this->target_joint_positions.clear();
    this->target_joint_velocities.clear();

    for (double i = 0; i < out_vel.size(); ++i)
    {
            this->target_joint_velocities.push_back(out_vel[i]);
            this->position_controllers[i].reset();
    }
    this->cart_vel_countdown = this->cart_vel_countdown_start;
}

void ArmController::velocityCommandCb(const sensor_msgs::ChannelFloat32::ConstPtr& msg)
{
    std::cout << *msg;
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
    for (int i = 0; i < num_of_joints_; ++i) {
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
    for (int i = 0; i < num_of_joints_; ++i)
    {
        this->joint_names.push_back("joint"+std::to_string(i+1));
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
                          this->position_threshold, this->i_clamp);
        this->position_controllers.push_back(pid);
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

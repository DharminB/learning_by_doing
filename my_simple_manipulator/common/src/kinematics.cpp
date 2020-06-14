#include <my_simple_manipulator/kinematics.h>
#include <math.h>

Kinematics::Kinematics(double link_length_1, double link_length_2, double link_length_3):
    l1_(link_length_1),
    l2_(link_length_2),
    l3_(link_length_3)
{
}

void Kinematics::setJointLimits(std::vector<double> &lower, std::vector<double> &upper)
{
    joint_lower_limits_ = lower;
    joint_upper_limits_ = upper;
}

bool Kinematics::findFK(std::vector<double> joint_positions, double &x, double &y, double &z)
{
    if ( joint_positions.size() != 3 || !withinJointLimit(joint_positions))
    {
        return false;
    }
    double j1 = joint_positions[0];
    double j2 = joint_positions[1] + M_PI/2;
    double j3 = joint_positions[2];
    x = ((l3_*cos(j3) + l2_)*(cos(j1)*cos(j2))) + ((l3_*sin(j3))*(-cos(j1)*sin(j2)));
    y = ((l3_*cos(j3) + l2_)*(sin(j1)*cos(j2))) + ((l3_*sin(j3))*(-sin(j1)*sin(j2)));
    z = ((l3_*cos(j3) + l2_)*(sin(j2))) + ((l3_*sin(j3))*(cos(j2))) + l1_+0.05;// because link0 is 5 cm above base_link
    return true;
}

void Kinematics::findIKVel(double vel_x, double vel_y, double vel_z,
               std::vector<double> current_joint_positions,
               std::vector<double> &target_joint_velocities)
{
    target_joint_velocities.clear();

    float max_vel_modifier = 10.0;
    double pos_x, pos_y, pos_z;
    findFK(current_joint_positions, pos_x, pos_y, pos_z);
    /* std::cout << pos_x << " " << pos_y << " " << pos_z << std::endl; */
    pos_x += vel_x/max_vel_modifier;
    pos_y += vel_y/max_vel_modifier;
    pos_z += vel_z/max_vel_modifier;
    /* std::cout << pos_x << " " << pos_y << " " << pos_z << std::endl; */
    std::vector<double> future_joint_positions;
    bool success = findNearestIK(pos_x, pos_y, pos_z,
                                       current_joint_positions,
                                       future_joint_positions);
    if (!success)
        return;
    double future_x, future_y, future_z;
    findFK(future_joint_positions, future_x, future_y, future_z);
    /* std::cout << future_x << " " << future_y << " " << future_z << std::endl; */
    for (double i = 0; i < future_joint_positions.size(); ++i) {
        double diff = future_joint_positions[i] - current_joint_positions[i];
        if (fabs(diff) < 0.001)
        {
            target_joint_velocities.push_back(0.0);
        }
        else
        {
            target_joint_velocities.push_back(diff*max_vel_modifier);
        }
    }
}

bool Kinematics::findIK(double x, double y, double z, std::vector<std::vector<double>> &possible_solutions)
{
    double d = sqrt(pow(x, 2) + pow(y, 2));
    /* for joint 1 */
    double joint1 = atan2(y, x);
    double joint1_prime = (joint1 > 0.0) ? joint1 - M_PI : joint1 + M_PI;
    std::vector<double> possible_joint_1({joint1, joint1_prime});

    /* for joint 2 */
    double theta_prime = atan2(z - l1_, d);
    double d_prime = sqrt(pow(d, 2) + pow(z-l1_, 2));
    double cos_ratio = (pow(d_prime, 2) + pow(l2_, 2) - pow(l3_, 2)) / (2 * d_prime * l2_);
    cos_ratio = fmax(-1.0, fmin(1.0, cos_ratio));
    double theta_double_prime = acos(cos_ratio);
    double joint2 = M_PI/2.0 - (theta_prime + theta_double_prime);
    double joint2_prime = -1.0 * joint2;
    std::vector<double> possible_joint_2({joint2, joint2_prime});

    /* for joint 3 */
    double ratio_2 = (pow(l2_, 2) + pow(l3_, 2) - pow(d_prime, 2)) / (2 * l2_ * l3_);
    ratio_2 = fmax(-1.0, fmin(1.0, ratio_2));
    double theta_triple_prime = acos(ratio_2);
    double joint3 = M_PI - theta_triple_prime;
    double joint3_prime = -1.0 * joint3;
    std::vector<double> possible_joint_3({joint3, joint3_prime});

    for (double joint_1 : possible_joint_1)
    {
        for (double joint_2 : possible_joint_2)
        {
            for (double joint_3 : possible_joint_3)
            {
                std::vector<double> joint_pos({joint_1, joint_2, joint_3});
                double fk_x, fk_y, fk_z;
                findFK(joint_pos, fk_x, fk_y, fk_z);
                if (fabs(fk_x - x) < ik_threshold_ &&
                    fabs(fk_y - y) < ik_threshold_ &&
                    fabs(fk_z - z) < ik_threshold_ &&
                    withinJointLimit(joint_pos))
                {
                    possible_solutions.push_back(joint_pos);
                }
            }
        }
    }
    return (possible_solutions.size() > 0) ? true : false;
}

bool Kinematics::findNearestIK(double x, double y, double z,
                   std::vector<double> current_joint_positions,
                   std::vector<double> &dest_joint_positions)
{
    std::vector<std::vector<double>> possible_solutions;
    bool success = findIK(x, y, z-0.05, possible_solutions);// because link0 is 5 cm above base_link
    if (!success)
    {
        return false;
    }

    if (x == 0.0 && y == 0.0)
    {
        int initial_size = possible_solutions.size();
        for (int i = 0; i < initial_size; ++i) {
            std::vector<double> joint_pos({current_joint_positions[0],
                                           possible_solutions[i][1],
                                           possible_solutions[i][2]});
            possible_solutions.push_back(joint_pos);
        }
    }
    /* std::cout << possible_solutions.size() << std::endl; */
    /* for (std::vector<double> solution: possible_solutions) */
    /*     std::cout << solution[0] << " " << solution[1] << " " << solution[2] << std::endl; */

    double min_dist = 10000.0;
    int min_dist_index = 0;
    for (int i = 0; i < possible_solutions.size(); ++i)
    {
        /* distance in joint space */
        double dist = sqrt(pow(current_joint_positions[0] - possible_solutions[i][0], 2)
                           + pow(current_joint_positions[1] - possible_solutions[i][1], 2)
                           + pow(current_joint_positions[2] - possible_solutions[i][2], 2));
        if (dist < min_dist)
        {
            min_dist = dist;
            min_dist_index = i;
        }
    }
    dest_joint_positions = possible_solutions[min_dist_index];
    return true;

}

bool Kinematics::withinJointLimit(const std::vector<double> &joint_pos)
{
    /* joint limits not available. Solution valid. */
    if (joint_lower_limits_.size() != joint_pos.size())
    {
        return true;
    }

    for (int i = 0; i < joint_pos.size(); ++i)
    {
        if (joint_pos[i] < joint_lower_limits_[i] 
            || joint_pos[i] > joint_upper_limits_[i])
        {
            return false;
        }
    }
    return true;
}

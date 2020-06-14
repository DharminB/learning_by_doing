#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <vector>

class Kinematics
{
    private:
        double l1_, l2_, l3_;
        std::vector<double> joint_lower_limits_;
        std::vector<double> joint_upper_limits_;
        double ik_threshold_ = 0.1;

    public:
        Kinematics(double link_length_1, double link_length_2, double link_length_3);

        void setJointLimits(std::vector<double> &lower, std::vector<double> &upper);

        bool findFK(std::vector<double> joint_positions, double &x, double &y, double &z);

        void findIKVel(double vel_x, double vel_y, double vel_z,
                       std::vector<double> current_joint_positions,
                       std::vector<double> &target_joint_velocities);

        bool findIK(double x, double y, double z,
                    std::vector<std::vector<double>> &possible_solutions);

        bool findNearestIK(double x, double y, double z,
                           std::vector<double> current_joint_positions,
                           std::vector<double> &dest_joint_positions);

        bool withinJointLimit(const std::vector<double> &joint_pos);
};
#endif

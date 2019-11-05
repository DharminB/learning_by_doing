#include <math.h>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#define PI 3.14159265

class Kinematics
{
    private:
        KDL::Chain my_chain;
        double l1, l2, l3;
        std::vector<double> joint_lower_limits;
        std::vector<double> joint_upper_limits;
        KDL::ChainFkSolverPos_recursive fk_solver;
        double ik_threshold = 0.1;

    public:
        Kinematics(KDL::Chain &chain): my_chain(chain), fk_solver(chain)
        {
            /* hacky solution for this particular arm to get link length */
            std::vector<double> link_length;
            bool useful = false;
            for (KDL::Segment seg : this->my_chain.segments)
            {
                KDL::Joint joint = seg.getJoint();
                if (useful)
                {
                    /* std::cout << seg.getName() << std::endl; */
                    KDL::Frame frame = seg.getFrameToTip();
                    if (frame.p.x() != 0.0)
                    {
                        link_length.push_back(frame.p.x());
                        continue;
                    }
                    if (frame.p.y() != 0.0)
                    {
                        link_length.push_back(frame.p.y());
                        continue;
                    }
                    if (frame.p.z() != 0.0)
                    {
                        link_length.push_back(frame.p.z());
                        continue;
                    }
                }
                if (joint.getType() != KDL::Joint::JointType::None) useful = true;
                else useful = false;
            }
            if (link_length.size() != 3)
            {
                std::cout << "Expected 3 link manipulator. Exiting." << std::endl;
                exit(1);
            }
            this->l1 = link_length[0];
            this->l2 = link_length[1];
            this->l3 = link_length[2];
        };

        void setJointLimits(std::vector<double> &lower, std::vector<double> &upper)
        {
            this->joint_lower_limits = lower;
            this->joint_upper_limits = upper;
        };

        void findFK(std::vector<double> joint_positions, double &x, double &y, double &z)
        {
            KDL::JntArray joint_pos(this->my_chain.getNrOfJoints());
            for (int i = 0; i < joint_positions.size(); ++i)
                joint_pos(i) = joint_positions[i];    
            KDL::Frame cart_pose;
            this->fk_solver.JntToCart(joint_pos, cart_pose);
            x = cart_pose.p.x();
            y = cart_pose.p.y();
            z = cart_pose.p.z();
        };

        bool findIK(double x, double y, double z, std::vector<std::vector<double>> &possible_solutions)
        {
            double d = sqrt(pow(x, 2) + pow(y, 2));
            /* for joint 1 */
            double joint1 = atan2(y, x);
            double joint1_prime = (joint1 > 0.0) ? joint1 - PI : joint1 + PI;
            std::vector<double> possible_joint_1({joint1, joint1_prime});

            /* for joint 2 */
            double theta_prime = atan2(z - this->l1, d);
            double d_prime = sqrt(pow(d, 2) + pow(z-this->l1, 2));
            double cos_ratio = (pow(d_prime, 2) + pow(this->l2, 2) - pow(l3, 2)) / (2 * d_prime * l2);
            cos_ratio = fmax(-1.0, fmin(1.0, cos_ratio));
            double theta_double_prime = acos(cos_ratio);
            double joint2 = PI/2.0 - (theta_prime + theta_double_prime);
            double joint2_prime = -1.0 * joint2;
            std::vector<double> possible_joint_2({joint2, joint2_prime});

            /* for joint 3 */
            double ratio_2 = (pow(this->l2, 2) + pow(this->l3, 2) - pow(d_prime, 2)) / (2 * this->l2 * this->l3);
            ratio_2 = fmax(-1.0, fmin(1.0, ratio_2));
            double theta_triple_prime = acos(ratio_2);
            double joint3 = PI - theta_triple_prime;
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
                        this->findFK(joint_pos, fk_x, fk_y, fk_z);
                        if (fabs(fk_x - x) < this->ik_threshold &&
                            fabs(fk_y - y) < this->ik_threshold &&
                            fabs(fk_z - z) < this->ik_threshold)
                            possible_solutions.push_back(joint_pos);
                    }
                }
            }
            /* std::cout << possible_solutions.size() << std::endl; */
            /* for (std::vector<double> solution: possible_solutions) */
            /*     std::cout << solution[0] << " " << solution[1] << " " << solution[2] << std::endl; */
            return (possible_solutions.size() > 0) ? true : false;
        };

        bool findNearestIK(double x, double y, double z,
                           std::vector<double> current_joint_positions,
                           std::vector<double> &dest_joint_positions)
        {
            std::vector<std::vector<double>> possible_solutions;
            bool success = this->findIK(x, y, z, possible_solutions);
            if (!success)
                return false;

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

        };

};

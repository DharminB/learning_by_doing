#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolverpos_recursive.hpp>

class Kinematics
{
    private:
        KDL::Chain my_chain;
        std::vector<std::string> joint_names;
        std::vector<double> joint_lower_limits;
        std::vector<double> joint_upper_limits;
        KDL::ChainFkSolverPos_recursive fk_solver;

    public:
        Kinematics(KDL::Chain &chain): my_chain(chain), fk_solver(chain) { };

        bool findIK(double x, double y, double z, std::vector<double> &joint_positions)
        {
            return true;
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
};

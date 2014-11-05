#ifndef GENERALIK_HPP
#define GENERALIK_HPP

#include "robot.hpp"

namespace Move3D {

class GeneralIK
{
public:
    GeneralIK(Move3D::Robot* robot);

    //! Initialize with a set of joints and Joint for the end effector
    bool initialize( const std::vector<Move3D::Joint*>& joints, Move3D::Joint* eef );

    //! Solve for a given task (6Dof)
    bool solve(const Eigen::VectorXd& xdes);

    //! single step, Jacobian transpose (6Dof)
    Eigen::VectorXd single_step( const Eigen::VectorXd& xdes );

    //! single step with joint limits (6Dof)
    Eigen::VectorXd single_step_joint_limits( const Eigen::VectorXd& xdes );

private:
    Move3D::Robot* robot_;
    Move3D::Joint* eef_;
    std::vector<Move3D::Joint*> active_joints_;
    std::vector<int> active_dofs_;
    bool limits_;
    double magnitude_;
    bool check_joint_limits_;
    int nb_steps_;
};

}

#endif // GENERALIK_HPP

#ifndef GENERALIK_HPP
#define GENERALIK_HPP

#include "robot.hpp"

namespace Move3D {

class GeneralIK
{
public:
    GeneralIK(Move3D::Robot* robot);
    bool initialize( const std::vector<Move3D::Joint*>& joints, Move3D::Joint* eef );
    bool solve(int steps, const Eigen::VectorXd& xdes);
    Eigen::VectorXd single_step( const Eigen::VectorXd& xdes );
    Eigen::VectorXd single_step_joint_limits( const Eigen::VectorXd& xdes );

private:
    Move3D::Robot* robot_;
    Move3D::Joint* eef_;
    std::vector<Move3D::Joint*> active_joints_;
    std::vector<int> active_dofs_;
    bool limits_;
    double magnitude_;

};

}

#endif // GENERALIK_HPP

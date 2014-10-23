#include "generalik.hpp"
#include "utils/misc_functions.hpp"

using namespace Move3D;
using std::cout;
using std::endl;

GeneralIK::GeneralIK(Move3D::Robot* robot) : robot_(robot)
{

}

bool GeneralIK::initialize( const std::vector<Move3D::Joint*>& joints, Move3D::Joint* eef )
{
    bool succeed = true;
    eef_ = eef;
    active_joints_ = joints;
    active_dofs_.clear();
    for( int i=0; i<int(joints.size()); i++) {
        if( joints[i]->getNumberOfDof() > 1 )
            succeed = false;
        active_dofs_.push_back( joints[i]->getIndexOfFirstDof() );
    }
    magnitude_ = 0.05;
    return succeed;
}

bool GeneralIK::solve(int steps, const Eigen::VectorXd& xdes)
{
    Move3D::confPtr_t q_proj = robot_->getCurrentPos();
    robot_->setAndUpdate(*q_proj);

    Move3D::confPtr_t q_cur = robot_->getCurrentPos();
    double dist=.0;
    bool succeed = false;
    for( int i=0; i<steps; i++) // IK LOOP
    {
        Eigen::VectorXd q = q_cur->getEigenVector( active_dofs_ );
        Eigen::VectorXd q_new = q + single_step_joint_limits(xdes);
        q_cur->setFromEigenVector( q_new, active_dofs_ );
        robot_->setAndUpdate( *q_cur );
        dist = ( xdes - eef_->getXYZPose() ).norm();
//        cout << "diff = " << dist << endl;
        if( dist < 0.001 ){
            cout << "success (" << i << "), diff = " << dist << endl;
            succeed = true;
            break;
        }
    }

    return succeed;
}

Eigen::VectorXd GeneralIK::single_step( const Eigen::VectorXd& xdes )
{
    Eigen::MatrixXd J = robot_->getJacobian( active_joints_, eef_, true );
    Eigen::MatrixXd Jt = move3d_pinv( J, 1e-3 );
    // cout << "Jt : " << endl << Jt << endl;
    Eigen::VectorXd x_error = ( xdes - eef_->getXYZPose() );
    Eigen::VectorXd dq = Jt * magnitude_ * x_error;
    return dq;
}

Eigen::VectorXd GeneralIK::single_step_joint_limits( const Eigen::VectorXd& xdes )
{
    std::vector<int> badjointinds;
    bool limit = false;
    Eigen::VectorXd q_s = robot_->getCurrentPos()->getEigenVector( active_dofs_ );
    Eigen::MatrixXd J = robot_->getJacobian( active_joints_, eef_, true );
    Eigen::VectorXd dq;
    Eigen::VectorXd x_error = ( xdes - eef_->getXYZPose() );

    do
    {
        Eigen::VectorXd q_s_old = q_s;

        // eliminate bad joint columns from the Jacobian
        for(int j = 0; j < badjointinds.size(); j++)
            for(int k = 0; k < xdes.size(); k++)
                J( k, badjointinds[j] ) = 0;

        Eigen::MatrixXd Reg(Eigen::MatrixXd::Zero(J.rows(), J.rows()));
        Reg.diagonal() = 0.0001 * Eigen::VectorXd::Ones(Reg.diagonal().size());
//        cout << "Reg : " << endl << Reg << endl;
        Eigen::MatrixXd M = (J*J.transpose())+Reg;
        Eigen::MatrixXd Jplus = J.transpose()*M.inverse();

        dq = Jplus * magnitude_ * x_error;

        // add step
        q_s = q_s_old + dq;

        limit = false;
        for(int j = 0; j<active_joints_.size(); j++)
        {
            if( active_joints_[j]->isJointDofCircular(0))
                continue;

            double lowerLimit, upperLimit;
            active_joints_[j]->getDofRandBounds( 0, lowerLimit, upperLimit );
            if( q_s[j] < lowerLimit || q_s[j] > upperLimit )
            {
                badjointinds.push_back(j); // note this will never add the same joint twice, even if bClearBadJoints = false
                limit = true;
            }
        }

        // move back to previous point if any joint limits
        if(limit)
            q_s = q_s_old;

    } while(limit);

    return dq;
}

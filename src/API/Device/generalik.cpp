/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */

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
    magnitude_ = 0.1;
    nb_steps_ = 100;
    check_joint_limits_ = true;
    return succeed;
}

bool GeneralIK::solve( const Eigen::VectorXd& xdes ) const
{
    Move3D::confPtr_t q_proj = robot_->getCurrentPos();
    robot_->setAndUpdate(*q_proj);

    Move3D::confPtr_t q_cur = robot_->getCurrentPos();
    double dist=.0;
    bool succeed = false;
    Eigen::VectorXd q_new;

    bool with_rotations = (xdes.size() == 6);

    for( int i=0; i<nb_steps_; i++) // IK LOOP
    {
        Eigen::VectorXd q = q_cur->getEigenVector( active_dofs_ );

        if( check_joint_limits_ )
            q_new = q + single_step_joint_limits( xdes );
        else
            q_new = q + single_step( xdes );

        q_cur->setFromEigenVector( q_new, active_dofs_ );
        robot_->setAndUpdate( *q_cur );
        dist =  ( xdes - ( with_rotations ? eef_->getXYZPose() : Eigen::VectorXd( eef_->getVectorPos().head(xdes.size())))).norm();
//        cout << "diff = " << dist << endl;

        if( dist < 0.001 ){
//            cout << "success (" << i << "), diff = " << dist << endl;
            succeed = true;
            break;
        }
    }

    return succeed;
}

Eigen::VectorXd GeneralIK::single_step( const Eigen::VectorXd& xdes ) const
{
    bool with_rotation  = xdes.size() == 6;
    bool with_height    = xdes.size() == 3;

    Eigen::VectorXd x_error = xdes - ( with_rotation ? eef_->getXYZPose() : Eigen::VectorXd( eef_->getVectorPos() ) ).head(xdes.size());

    Eigen::MatrixXd J = robot_->getJacobian( active_joints_, eef_, with_rotation, with_height );
    Eigen::MatrixXd Jplus = move3d_pinv( J, 1e-3 );
    // cout << "Jt : " << endl << Jt << endl;
    Eigen::VectorXd dq = Jplus * magnitude_ * x_error;
    return dq;
}

Eigen::VectorXd GeneralIK::single_step_joint_limits( const Eigen::VectorXd& xdes ) const
{
    bool with_rotation  = xdes.size() == 6;
    bool with_height    = xdes.size() == 3;

    Eigen::VectorXd x_error = xdes - ( with_rotation ? eef_->getXYZPose() : Eigen::VectorXd( eef_->getVectorPos() ) ).head( xdes.size() );

    std::vector<int> badjointinds;
    bool limit = false;
    Eigen::VectorXd q_s = robot_->getCurrentPos()->getEigenVector( active_dofs_ );
    Eigen::MatrixXd J = robot_->getJacobian( active_joints_, eef_, with_rotation, with_height );
    Eigen::VectorXd dq;

    // cout << "J : " << endl << J << endl;

    do
    {
        Eigen::VectorXd q_s_old = q_s;

        // eliminate bad joint columns from the Jacobian
        for(int j = 0; j < badjointinds.size(); j++)
            for(int k = 0; k < xdes.size(); k++)
                J( k, badjointinds[j] ) = 0;

        /*
        // Damped Least-Squares (DLS)
        // Jplus = Jt * [(Jt * J ) + lambda * diag(I)]^{-1}
        Eigen::MatrixXd Reg(Eigen::MatrixXd::Zero(J.rows(), J.rows()));
        Reg.diagonal() = 0.0001 * Eigen::VectorXd::Ones(Reg.diagonal().size());
        Eigen::MatrixXd M = (J*J.transpose())+Reg;
        Eigen::MatrixXd Jplus = J.transpose()*M.inverse();
        */

        Eigen::MatrixXd Jplus = move3d_pinv( J, 1e-3 ).block(0, 0, active_joints_.size(), xdes.size() );

        dq = Jplus * magnitude_ * x_error;

        // add step
        q_s = q_s_old + dq;

        limit = false;
        for(int j =0; j<active_joints_.size(); j++)
        {
            if( active_joints_[j]->isJointDofCircular(0) )
                continue;

            double lowerLimit, upperLimit;
            active_joints_[j]->getDofRandBounds( 0, lowerLimit, upperLimit );

            if( q_s[j] < lowerLimit || q_s[j] > upperLimit )
            {
                badjointinds.push_back(j); // note this will never add the same joint twice, even if bClearBadJoints = false
                limit = true;
                // cout << "does not respect joint limits : " << active_joints_[j]->getName() << endl;
            }
        }

        // move back to previous point if any joint limits
        if(limit)
            q_s = q_s_old;

    } while(limit);

    return dq;
}

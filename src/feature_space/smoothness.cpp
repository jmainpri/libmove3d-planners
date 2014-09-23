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

#include "smoothness.hpp"

#include "planner/planEnvironment.hpp"
#include "utils/misc_functions.hpp"

#include <libmove3d/include/Util-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

//----------------------------------------------------------------------
//----------------------------------------------------------------------

LengthFeature::LengthFeature()
{
    w_ = WeightVect::Ones( 1 ); // Sets the number of feature in the base class
    scaling_ = 1;
    is_config_dependent_ = true;
    name_ = "Length";
}

//FeatureVect LengthFeature::getFeatureCount( const Move3D::Trajectory& t )
//{
//    FeatureVect feature_count = scaling_ * t.getParamMax() * FeatureVect::Ones( 1 );
////    cout << "length : " << feature_count.transpose() << endl;
//    return feature_count;
//}

FeatureVect LengthFeature::getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs )
{
    return FeatureVect::Zero( 1 );
//    return scaling_ * FeatureVect::Ones( 1 );
}

FeatureVect LengthFeature::getFeatureCount( const Move3D::Trajectory& t )
{
    Eigen::MatrixXd traj_smooth = getSmoothedTrajectory( t );

    // num of collums is the dimension of state space
    // num of rows is the length of the trajectory
    std::vector<Eigen::VectorXd> control_costs( traj_smooth.rows() );

    if( traj_smooth.cols() == 0 || traj_smooth.rows() == 0 || traj_smooth.cols() == 1 )
    {
//        cout << "ERROR IN MAT" << endl;
        return FeatureVect::Zero( 1 );
    }

    int diff_rule_length = control_cost_.getDiffRuleLength();

    // this measures the velocity and squares them
    for ( int d=0; d<traj_smooth.rows(); ++d )
    {
        Eigen::VectorXd params_all = traj_smooth.row(d);
        Eigen::VectorXd acc_all    = Eigen::VectorXd::Zero( traj_smooth.cols() );

        for (int i=diff_rule_length; i<params_all.size()-diff_rule_length; i++)
        {
            if( i < params_all.size()-1 ) // Check for jumps on circular joints (MAY ACTIVATE FOR TRANSLATION JOINTS (diff > 3.14)
            {
                double diff = params_all[i] - params_all[i+1];
                double error = std::fabs( diff_angle( params_all[i+1] , params_all[i] ) - diff );
                if( error > 1e-6 ){
                    cout << "control cost breaks at row "  << d << " and col  " << i << " by " << error << " in " << __PRETTY_FUNCTION__ << endl;
                }
            }

//            acc_all[i] = ( std::pow( params_all[i], 2. ) - std::pow( params_all[i+1], 2. ) );
            acc_all[i] = std::abs( params_all[i] ) - std::abs( params_all[i+1] );
        }

        control_costs[d] = acc_all;

//        cout << "acc_all : " << std::scientific << acc_all.transpose() << endl;

    }

    double length = 0.0;
    for (int i=diff_rule_length; i<traj_smooth.cols()-diff_rule_length; i++)
    {
        double tmp = 0.0;

        for ( int d=0; d<traj_smooth.rows(); ++d )
            tmp += control_costs[d][i];

        length += std::sqrt(tmp); // Sum of squarred lengths
    }

    return WeightVect::Ones( 1 ) * length; // scaling
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

VelocitySmoothness::VelocitySmoothness()
{
    control_cost_.setType( 0 );
    name_ = "Velocity";
}

void VelocitySmoothness::setWeights( const WeightVect& w )
{
    w_ = w;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

AccelerationSmoothness::AccelerationSmoothness()
{
    control_cost_.setType( 1 );
    name_ = "Acceleration";
}

void AccelerationSmoothness::setWeights( const WeightVect& w )
{
    w_ = w;
    PlanEnv->setDouble( PlanParam::trajOptimSmoothWeight, w(0) );
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

JerkSmoothness::JerkSmoothness()
{
    control_cost_.setType( 2 );
    name_ = "Jerk";
}

void JerkSmoothness::setWeights( const WeightVect& w )
{
    w_ = w;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost

TrajectorySmoothness::TrajectorySmoothness() : Feature("Smoothness")
{
    w_ = WeightVect::Ones( 1 ); // Sets the number of feature in the base class
    is_config_dependent_ = false;
    buffer_is_filled_ = false;
}

Eigen::MatrixXd TrajectorySmoothness::getSmoothedTrajectory( const Move3D::Trajectory& t )
{
    Robot* robot = t.getRobot();

//    cout << "active_dofs_ : ";
//    for(int i=0;i<active_dofs_.size();i++) cout << active_dofs_[i] << " ";
//    cout << endl;

    int rows = active_dofs_.size();
    int cols = t.getNbOfViaPoints();

    //    cout << "cols : " << cols << endl;

    int diff_rule_length = control_cost_.getDiffRuleLength();

    Eigen::VectorXd q_init = t.getBegin()->getEigenVector( active_dofs_ );
    Eigen::VectorXd q_goal = t.getEnd()->getEigenVector( active_dofs_ );

    Eigen::MatrixXd mat1 = t.getEigenMatrix( active_dofs_ );
    //    cout << "motion matrix 1" << endl;
    //    cout.precision(2);
    //    cout << mat1 << endl;

    Eigen::MatrixXd mat2( rows, cols + 2*(diff_rule_length-1) );
    mat2.block( 0, diff_rule_length-1, rows, cols ) = mat1;

//    cout << "motion matrix 2" << endl;
//    cout.precision(2);
//    cout << mat2 << endl;

    if( buffer_is_filled_ )
        control_cost_.fillTrajectoryWithBuffer( q_goal, mat2 );
    else
        control_cost_.fillTrajectory( q_init, q_goal, mat2 );

    // Smooth circular curves before cost computation
    // must be after the mat2 construction
    int r=0;
    for( int i=0; i<robot->getNumberOfJoints(); i++ )
    {
        Joint* joint = robot->getJoint(i);

        for( int k=0; k<joint->getNumberOfDof(); k++ ) {

            if( std::find( active_dofs_.begin(), active_dofs_.end(), joint->getIndexOfFirstDof() + k ) != active_dofs_.end() )
            {
                if( joint->isJointDofCircular(k) )
                {
                    Eigen::VectorXd row = mat2.row( r );
                    //                    cout << "dof : " << r << " is circula , " << joint->getName() << endl;
                    move3d_smooth_circular_parameters( row );
                    mat2.row( r ) = row;
                }
                r++;
            }
        }
    }

    return mat2;
}

FeatureVect TrajectorySmoothness::getFeatureCount( const Move3D::Trajectory& t )
{
    FeatureVect f( Eigen::VectorXd::Zero( 1 ) );

    Eigen::MatrixXd mat2 = getSmoothedTrajectory( t );
//    cout << "motion matrix 3" << endl;
//    cout.precision(4);
//    cout << mat2 << endl;

    double dt = t.getUseTimeParameter() && t.getUseConstantTime() ? t.getDeltaTime() : 0.0;

//    cout << "dt : " << dt << endl;


    // Compute the squared profile of quantities
    std::vector<Eigen::VectorXd> control_cost = control_cost_.getSquaredQuantities( mat2, dt );

//    printControlCosts( control_cost );

    double smoothness_factor = PlanEnv->getDouble( PlanParam::trajOptimSmoothFactor ); // * 1000.0; // for IOC, scale the features between 0.1
    double smoothness_cost = control_cost_.cost( control_cost );
    f[0] = smoothness_factor * smoothness_cost;

//    cout.precision(6);
//    cout << "smoothness_cost : " << smoothness_cost << " , smoothness_factor : " << PlanEnv->getDouble( PlanParam::trajOptimSmoothFactor ) << endl;

//    cout.precision(6);
//    cout << "size (" << mat2.rows() << ", " << mat2.cols() << ") , control cost : "  << f << endl;

    return f;
}

void TrajectorySmoothness::setWeights( const WeightVect& w )
{
    w_ = w;
    PlanEnv->setDouble( PlanParam::trajOptimSmoothWeight, w(0) );
}

void TrajectorySmoothness::printControlCosts( const std::vector<Eigen::VectorXd>& control_cost )
{
    if( control_cost.empty() )
        return;

    int diff_rule_length = control_cost_.getDiffRuleLength();
    int size = control_cost[0].size() - 2*(diff_rule_length-1);

    FeatureVect f_tmp = FeatureVect::Zero( size );

    for( int i=0; i<int(control_cost.size()); i++ )
    {
        f_tmp += control_cost[i].segment( diff_rule_length-1, size );
    }

    cout.precision(6);
    cout << f_tmp.transpose() << endl;
}

FeatureVect TrajectorySmoothness::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
//    cout << "Get feature smoothness" << endl;
    return FeatureVect::Zero( 1 );
}


//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost


SmoothnessFeature::SmoothnessFeature() : Feature("SmoothnessAll")
{
    w_ = FeatureVect::Zero(4);
}

void SmoothnessFeature::setWeights( const WeightVect& w )
{
    w_ = w;
    PlanEnv->setDouble( PlanParam::trajOptimSmoothWeight, w(2) );
}

FeatureVect SmoothnessFeature::getFeatureCount( const Move3D::Trajectory& t )
{
    FeatureVect phi( FeatureVect::Zero( 4 ) );
    phi[0] = length_.getFeatureCount(t)[0] * 1e-01;
    phi[1] = velocity_.getFeatureCount(t)[0] * 1e-05;
    phi[2] = acceleration_.getFeatureCount(t)[0] * 1e-10;
    phi[3] = jerk_.getFeatureCount(t)[0] * 1e-15;
    return phi;
}

FeatureVect SmoothnessFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
//    cout << "Get feature smoothness" << endl;
    return FeatureVect::Zero( 4 );
}

void SmoothnessFeature::setActiveDoFs( const std::vector<int>& active_dofs )
{
    length_.setActiveDoFs( active_dofs );
    velocity_.setActiveDoFs( active_dofs );
    acceleration_.setActiveDoFs( active_dofs );
    jerk_.setActiveDoFs( active_dofs );
}

void SmoothnessFeature::setBuffer(const Eigen::MatrixXd& buffer)
{
    velocity_.setBuffer( buffer );
    acceleration_.setBuffer( buffer );
    jerk_.setBuffer( buffer );
}

void SmoothnessFeature::clearBuffer()
{
    velocity_.clearBuffer();
    acceleration_.clearBuffer();
    jerk_.clearBuffer();
}

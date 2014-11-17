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

double LengthFeature::getControlCosts(const Eigen::MatrixXd& traj_smooth, std::vector<Eigen::VectorXd>& control_costs)
{
    // num of collums is the dimension of state space
    // num of rows is the length of the trajectory

    if( traj_smooth.cols() == 0 || traj_smooth.rows() == 0 || traj_smooth.cols() == 1 )
    {
        cout << "ERROR IN MAT" << endl;
        return 0.0;
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

            acc_all[i] = std::pow( params_all[i]  - params_all[i+1], 2. );
//            acc_all[i] = std::abs( params_all[i] ) - std::abs( params_all[i+1] );
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

        length += tmp;
//        length += std::sqrt(tmp); // Sum of squarred lengths
    }

    // Set inner segments
    for ( int d=0; d<int(control_costs.size()); ++d )
        control_costs[d] = control_cost_.getInnerSegment( control_costs[d] );

    return length;
}

FeatureVect LengthFeature::getFeatureCount( const Move3D::Trajectory& t )
{
    Eigen::MatrixXd traj_smooth = getSmoothedTrajectory( t );
    std::vector<Eigen::VectorXd> control_costs( traj_smooth.rows() );
    double length = getControlCosts( traj_smooth, control_costs);
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

void TrajectorySmoothness::setBuffer(const std::vector<Eigen::VectorXd>& buffer)
{
    control_cost_.setBuffer(buffer);
    buffer_is_filled_=true;
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

double TrajectorySmoothness::getControlCosts(const Eigen::MatrixXd& traj_smooth, std::vector<Eigen::VectorXd>& control_costs, double dt)
{
    // Compute the squared profile of quantities
    control_costs = control_cost_.getSquaredQuantities( traj_smooth, dt );

//    printControlCosts( control_costs );
    double cost = control_cost_.cost( control_costs );

    // Set inner segments
    for ( int d=0; d<int(control_costs.size()); ++d )
        control_costs[d] = control_cost_.getInnerSegment( control_costs[d] );

    return cost;
}

FeatureVect TrajectorySmoothness::getFeatureCount( const Move3D::Trajectory& t )
{
    FeatureVect f( Eigen::VectorXd::Zero( 1 ) );

    Eigen::MatrixXd traj_smooth = getSmoothedTrajectory( t );
//    cout << "motion matrix 3" << endl;
//    cout.precision(4);
//    cout << traj_smooth << endl;

    std::vector<Eigen::VectorXd> control_costs( traj_smooth.rows() );

    double dt = t.getUseTimeParameter() && t.getUseConstantTime() ? t.getDeltaTime() : 0.0;
//    cout << "dt : " << dt << endl;

    double smoothness_factor = PlanEnv->getDouble( PlanParam::trajOptimSmoothFactor ); // * 1000.0; // for IOC, scale the features between 0.1
    double smoothness_cost = getControlCosts( traj_smooth, control_costs, dt );
    f[0] = smoothness_factor * smoothness_cost;

    //    cout.precision(6);
    //    cout << "smoothness_cost : " << smoothness_cost << " , smoothness_factor : " << PlanEnv->getDouble( PlanParam::trajOptimSmoothFactor ) << endl;

    //    cout.precision(6);
    //    cout << "size (" << mat2.rows() << ", " << mat2.cols() << ") , control cost : "  << f << endl;

    //    control_cost_.saveProfiles( mat2, "/home/jmainpri/Dropbox/move3d/move3d-launch/launch_files/" , dt );

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


//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------

TaskSmoothnessFeature::TaskSmoothnessFeature( Move3D::Robot* robot ) : robot_(robot)
{
     name_ = "TaskSmoothness";

//    human_active_joints_.push_back( human_active_->getJoint("Pelvis") );
//    human_active_joints_.push_back( human_active_->getJoint("rShoulderX") );
//    human_active_joints_.push_back( human_active_->getJoint("rElbowZ") );
     task_joints_.clear();
     task_joints_.push_back( robot_->getJoint("rWristX") );

     for( int i=0;i<task_joints_.size();i++)
         veclocity_joint_ids_.push_back( task_joints_[i]->getId() );
}


Eigen::VectorXd TaskSmoothnessFeature::getTaskPose( Move3D::confPtr_t q )
{
    robot_->setAndUpdate(*q);
    Eigen::VectorXd pose = task_joints_[0]->getVectorPos();
    return pose;
}
//! Returns a matrix m with m.col() (i.e., number of collumns)
//! being the number of configuration in the trajectory
Eigen::MatrixXd TaskSmoothnessFeature::getTaskTrajectory( const Move3D::Trajectory& t )
{
    //    cout << "active_dofs_ : ";
    //    for(int i=0;i<active_dofs_.size();i++) cout << active_dofs_[i] << " ";
    //    cout << endl;

    int rows = 3 * task_joints_.size();
    int cols = t.getNbOfViaPoints();

    //    cout << "cols : " << cols << endl;

    int diff_rule_length = control_cost_.getDiffRuleLength();

    Eigen::VectorXd x_init = getTaskPose(t.getBegin());
    Eigen::VectorXd x_goal = getTaskPose(t.getEnd());

    Eigen::MatrixXd mat1( rows, cols );


    for( int i=0; i<cols; i++){
        mat1.col(i) = getTaskPose( t[i] );
    }

//    cout << "motion matrix 1" << endl;
//    cout.precision(2);
//    cout << mat1 << endl;

    Eigen::MatrixXd mat2( rows, cols + 2*(diff_rule_length-1) );
    mat2.block( 0, diff_rule_length-1, rows, cols ) = mat1;

    if( buffer_is_filled_ )
        control_cost_.fillTrajectoryWithBuffer( x_goal, mat2 );
    else
        control_cost_.fillTrajectory( x_init, x_goal, mat2 );

//    if( !PlanEnv->getBool(PlanParam::trajStompNoPrint))
//    {
//        cout << "motion matrix 2" << endl;
//        cout.precision(5);
//        cout << mat2 << endl;
//    }

    return mat2;
}

double TaskSmoothnessFeature::getDist( const Eigen::MatrixXd& mat, Eigen::VectorXd& costs )
{

}

double TaskSmoothnessFeature::getVelocity( const Eigen::MatrixXd& mat, Eigen::VectorXd& costs, double dt )
{
    control_cost_.setType(0);

    std::vector<Eigen::VectorXd> control_cost = control_cost_.getSquaredQuantities( mat, dt );
    costs = getControlCosts( control_cost );
    return control_cost_.cost( control_cost );
}

double TaskSmoothnessFeature::getAcceleration( const Eigen::MatrixXd& mat, Eigen::VectorXd& costs, double dt )
{
    control_cost_.setType(1);

    std::vector<Eigen::VectorXd> control_cost = control_cost_.getSquaredQuantities( mat, dt );
    costs = getControlCosts( control_cost );
    return control_cost_.cost( control_cost );
}

double TaskSmoothnessFeature::getJerk( const Eigen::MatrixXd& mat, Eigen::VectorXd& costs, double dt )
{
    control_cost_.setType(2);

    std::vector<Eigen::VectorXd> control_cost = control_cost_.getSquaredQuantities( mat, dt );
    costs = getControlCosts( control_cost );
    return control_cost_.cost( control_cost );
}

Eigen::VectorXd TaskSmoothnessFeature::getControlCosts( std::vector<Eigen::VectorXd>& control_cost ) const
{
    if( control_cost.empty() )
        return Eigen::VectorXd::Zero(0);

    Eigen::VectorXd costs = Eigen::VectorXd::Zero( control_cost_.getInnerSegmentSize( control_cost[0] ) );

    for( int d=0; d<control_cost.size(); d ++)
    {
        Eigen::VectorXd control_cost_tmp = control_cost_.getInnerSegment( control_cost[d] );
        for( int i=0; i<control_cost_tmp.size(); i ++){
            costs[i] += control_cost_tmp[i];
        }
    }
    return costs;
}

// Compute velocity between two configurations
double TaskSmoothnessFeature::getDist( const Move3D::Trajectory& t, Eigen::VectorXd& control_costs )
{
//    std::vector<Eigen::Vector3d> dist(veclocity_joint_ids_.size());

//    std::vector<Eigen::Vector3d> pos_0(veclocity_joint_ids_.size());
//    std::vector<Eigen::Vector3d> pos_1(veclocity_joint_ids_.size());

//    human_active_->setAndUpdate(*t[i_q-1]);

//    for( int i=0;i<veclocity_joint_ids_.size();i++)
//        pos_0[i] = human_active_joints_[i]->getVectorPos();

//    human_active_->setAndUpdate(*t[i_q]);

//    for( int i=0;i<veclocity_joint_ids_.size();i++)
//        pos_1[i] = human_active_joints_[i]->getVectorPos();

    double dist=0.0;

    Eigen::VectorXd x_0, x_1;

    control_costs = Eigen::VectorXd::Zero( t.getNbOfViaPoints() );

    for( int i=0;i<t.getNbOfPaths();i++)
    {
        x_0 = getTaskPose( t[i] );
        x_1 = getTaskPose( t[i+1] );

        Eigen::VectorXd delta = ( x_0 - x_1 );
        Eigen::VectorXd costs = delta.cwise() * delta;

        control_costs[i] = costs.sum();

        dist += control_costs[i];
    }

    return dist;
}

// Compute velocity between two configurations
double TaskSmoothnessFeature::getVelocity(const Move3D::Trajectory& t, Eigen::VectorXd& costs )
{
    double dt = t.getUseTimeParameter() && t.getUseConstantTime() ? t.getDeltaTime() : 0.0;

    Eigen::MatrixXd mat = getTaskTrajectory( t );

    // Compute the squared profile of quantities
   double cost = getVelocity( mat, costs, dt );

//    cout << "number of via points : " << t.getNbOfViaPoints() << endl;
//    cout << "control_cost[0].size() : " << control_cost[0].size() << endl;
//    cout << "costs.size() : " << costs.size() << endl;


//    control_cost_.saveProfiles( mat, "/home/jmainpri/Dropbox/move3d/move3d-launch/launch_files/task_vel/" , dt );

    return cost;
}

double TaskSmoothnessFeature::getAcceleration(const Move3D::Trajectory& t, Eigen::VectorXd& costs )
{
    double dt = t.getUseTimeParameter() && t.getUseConstantTime() ? t.getDeltaTime() : 0.0;

    Eigen::MatrixXd mat = getTaskTrajectory( t );

    // Compute the squared profile of quantities
    double cost = getAcceleration( mat, costs, dt );
    return cost;
}

double TaskSmoothnessFeature::getJerk( const Move3D::Trajectory& t, Eigen::VectorXd& costs )
{
    double dt = t.getUseTimeParameter() && t.getUseConstantTime() ? t.getDeltaTime() : 0.0;

    Eigen::MatrixXd mat = getTaskTrajectory( t );

    // Compute the squared profile of quantities
    double cost = getJerk( mat, costs, dt );
    return cost;
}

void TaskSmoothnessFeature::setBuffer(const std::vector<Eigen::VectorXd>& buffer )
{
    std::vector<Eigen::VectorXd> x_buffer( buffer.size() );

    Move3D::confPtr_t q = robot_->getCurrentPos();

    for( int i=0; i<buffer.size(); i++)
    {
        q->setFromEigenVector( buffer[i], active_dofs_ );
        x_buffer[i] = getTaskPose( q );
    }

//    if( !PlanEnv->getBool(PlanParam::trajStompNoPrint))
//    {
//        for( int i=0; i<buffer.size(); i++)
//            cout << "x_buffer[" << i << "] : " << x_buffer[i].transpose() << endl;
//    }

    control_cost_.setBuffer( x_buffer );
    buffer_is_filled_=true;
}

void TaskSmoothnessFeature::draw()
{
    //    for( int i=0;i<veclocity_joint_ids_.size();i++)
    //        pos_0[i] = human_active_joints_[i]->getVectorPos();
    //    q_last_
}

FeatureVect TaskSmoothnessFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
    FeatureVect count;
    return count;
}

FeatureVect TaskSmoothnessFeature::getFeatureCount(const Move3D::Trajectory& traj)
{
    FeatureVect count( veclocity_joint_ids_.size() * 4 );

    Eigen::VectorXd costs;

    count[0] = getDist( traj, costs );
    count[1] = getVelocity( traj, costs );
    count[2] = getAcceleration( traj, costs );
    count[3] = getJerk( traj, costs );

    return count;
}


//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Smoothness cost


SmoothnessFeature::SmoothnessFeature(Robot* robot) : Feature("SmoothnessAll"), task_features_(robot)
{
    w_ = FeatureVect::Zero(8);
}

void SmoothnessFeature::setWeights( const WeightVect& w )
{
    w_ = w;
    PlanEnv->setDouble( PlanParam::trajOptimSmoothWeight, w(2) );
}

FeatureVect SmoothnessFeature::getFeatureCount( const Move3D::Trajectory& t )
{
    FeatureVect phi( FeatureVect::Zero( 8 ) );
    phi[0] = length_.getFeatureCount(t)[0] * 1e-01;
    phi[1] = velocity_.getFeatureCount(t)[0] * 1e-05;
    phi[2] = acceleration_.getFeatureCount(t)[0] * 1e-10;
    phi[3] = jerk_.getFeatureCount(t)[0] * 1e-15;

    FeatureVect phi_task = task_features_.getFeatureCount( t );
    phi[4] = phi_task[0] * 1e+01;
    phi[5] = phi_task[1] * 1e-03;
    phi[6] = phi_task[2] * 1e-09;
    phi[7] = phi_task[3] * 1e-13;

    return phi;
}

FeatureVect SmoothnessFeature::getFeatures(const Configuration& q, std::vector<int> active_dofs)
{
//    cout << "Get feature smoothness" << endl;
    return FeatureVect::Zero( 8 );
}

void SmoothnessFeature::setActiveDoFs( const std::vector<int>& active_dofs )
{
    length_.setActiveDoFs( active_dofs );
    velocity_.setActiveDoFs( active_dofs );
    acceleration_.setActiveDoFs( active_dofs );
    jerk_.setActiveDoFs( active_dofs );
    task_features_.setActiveDoFs( active_dofs );
}

void SmoothnessFeature::setBuffer(const std::vector<Eigen::VectorXd>& buffer)
{
    velocity_.setBuffer( buffer );
    acceleration_.setBuffer( buffer );
    jerk_.setBuffer( buffer );
    task_features_.setBuffer( buffer );
 }

void SmoothnessFeature::clearBuffer()
{
    velocity_.clearBuffer();
    acceleration_.clearBuffer();
    jerk_.clearBuffer();
    task_features_.clearBuffer();
}

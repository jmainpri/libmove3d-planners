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
#include "cost_computation.hpp"

#include "cost_space.hpp"

using namespace stomp_motion_planner;
using namespace Move3D;

using std::cout;
using std::endl;

costComputation::costComputation(Robot* robot,
                                 const CollisionSpace *collision_space,
                                 const ChompPlanningGroup* planning_group,
                                 std::vector< ChompCost > joint_costs,
                                 ChompTrajectory group_trajectory,
                                 double obstacle_weight,
                                 bool use_costspace)
{
    robot_model_ = robot;

    cout << "new cost computation with robot : " << robot_model_->getName() << endl;

    collision_space_ = collision_space;
    planning_group_ = planning_group;
    joint_costs_ = joint_costs;

    group_trajectory_ = group_trajectory;

    obstacle_weight_ = obstacle_weight;
    use_costspace_ = use_costspace;
    hack_tweek_ = 1.0;

    iteration_ = 0;

    num_vars_free_ = group_trajectory_.getNumFreePoints();
    num_vars_all_ = group_trajectory_.getNumPoints();
    num_joints_ = group_trajectory_.getNumJoints();
    free_vars_start_ = group_trajectory_.getStartIndex();
    free_vars_end_ = group_trajectory_.getEndIndex();
    num_collision_points_ = planning_group_->collision_points_.size();

    joint_axis_eigen_.resize(num_vars_all_);
    joint_pos_eigen_.resize(num_vars_all_);
    collision_point_pos_eigen_.resize(num_vars_all_);
    collision_point_vel_eigen_.resize(num_vars_all_);
    collision_point_acc_eigen_.resize(num_vars_all_);

    segment_frames_.resize( num_vars_all_ );

    state_is_in_collision_.resize(num_vars_all_);
    general_cost_potential_.resize(num_vars_all_);

    for(int i=0; i<num_vars_all_;i++)
    {
        segment_frames_[i].resize(planning_group_->num_dofs_);
        joint_pos_eigen_[i].resize(planning_group_->num_dofs_);
    }

    if (num_collision_points_ > 0)
    {
        collision_point_potential_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_vel_mag_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

        for(int i=0; i<num_vars_all_;i++)
        {
            joint_axis_eigen_[i].resize(num_collision_points_);
            joint_pos_eigen_[i].resize(num_collision_points_);
            collision_point_pos_eigen_[i].resize(num_collision_points_);
            collision_point_vel_eigen_[i].resize(num_collision_points_);
            collision_point_acc_eigen_[i].resize(num_collision_points_);
        }
    }
}

costComputation::~costComputation()
{
    cout << "destroy cost computation" << endl;
}

bool costComputation::handleJointLimits(  ChompTrajectory& group_traj  )
{
    bool succes_joint_limits = true;
    bool joint_limits_violation_ = 0;

    for (int joint=0; joint<num_joints_; joint++)
    {
        if (!planning_group_->chomp_dofs_[joint].has_joint_limits_)
            continue;

        // Added by jim for pr2 free flyer
        if( planning_group_->robot_->getName() == "PR2_ROBOT" )
        {
            int index = planning_group_->chomp_dofs_[joint].move3d_dof_index_;

            if( index == 8 || index == 9 || index == 10 ) {
                group_traj.getFreeJointTrajectoryBlock(joint) = Eigen::VectorXd::Zero(num_vars_free_);
                continue;
            }
        }

        double joint_max = planning_group_->chomp_dofs_[joint].joint_limit_max_;
        double joint_min = planning_group_->chomp_dofs_[joint].joint_limit_min_;

        int count = 0;
        bool violation = false;
        int count_violation = false;

        do
        {
            double max_abs_violation =  1e-6;
            double max_violation = 0.0;
            int max_violation_index = 0;
            violation = false;
            count_violation = false;
            for (int i=free_vars_start_; i<=free_vars_end_; i++)
            {
                double amount = 0.0;
                double absolute_amount = 0.0;
                if ( group_traj(i, joint) > joint_max )
                {
                    amount = joint_max - group_traj(i, joint);
                    absolute_amount = fabs(amount);
                }
                else if ( group_traj(i, joint) < joint_min )
                {
                    amount = joint_min - group_traj(i, joint);
                    absolute_amount = fabs(amount);
                }
                if (absolute_amount > max_abs_violation)
                {
                    max_abs_violation = absolute_amount;
                    max_violation = amount;
                    max_violation_index = i;
                    violation = true;

                    if ( i != free_vars_start_ && i != free_vars_end_ ) {
                        count_violation = true;
                    }
                }
            }

            if (violation)
            {
                // Only count joint limits violation for
                // interpolated configurations
                if( count_violation )
                {
                    joint_limits_violation_++;
                }
                // cout << "Violation of Limits (joint) : " <<  joint << endl;
                int free_var_index = max_violation_index - free_vars_start_;
                double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);

                group_traj.getFreeJointTrajectoryBlock(joint) +=
                        multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
            }
            if (++count > 10)
            {
                succes_joint_limits = false;
                break;
            }
        }
        while(violation);
    }


    return succes_joint_limits;
}

void costComputation::getFrames( int segment, const Eigen::VectorXd& joint_array, Configuration& q )
{
    q = *robot_model_->getCurrentPos();

    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

    // Set the configuration to the joint array value
    for(int j=0; j<planning_group_->num_dofs_;j++)
    {
        int dof = joints[j].move3d_dof_index_;

        if ( !std::isnan(joint_array[j]) )
        {
            q[dof]= joint_array[j];
        }
        else {
            cout << "q[" << dof << "] is nan" << endl;
            q[dof]= 0;
        }
    }

    robot_model_->setAndUpdate( q );

    if( collision_space_ )
    {
        // Get the collision point position
        for(int j=0; j<planning_group_->num_dofs_;j++)
        {
            Eigen::Transform3d t = robot_model_->getJoint( joints[j].move3d_joint_->getId() )->getMatrixPos();

//            std::vector<double> vect;
//            eigenTransformToStdVector( t, vect );

            segment_frames_[segment][j]  = t;
            joint_pos_eigen_[segment][j] = t.translation();

            joint_axis_eigen_[segment][j](0) = t(0,2);
            joint_axis_eigen_[segment][j](1) = t(1,2);
            joint_axis_eigen_[segment][j](2) = t(2,2);
        }
    }
}

bool costComputation::getConfigObstacleCost(int segment, int coll_point, Configuration& q )
{
    bool colliding = false;

    if( collision_space_ /*&& (use_costspace_==false)*/ )
    {
        int i= segment; int j= coll_point; double distance;

        planning_group_->collision_points_[j].getTransformedPosition( segment_frames_[i], collision_point_pos_eigen_[i][j] );

        // To fix in collision space
        // The joint 1 is allways colliding
        colliding = collision_space_->getCollisionPointPotentialGradient( planning_group_->collision_points_[j],
                                                                          collision_point_pos_eigen_[i][j],
                                                                          distance,
                                                                          collision_point_potential_(i,j),
                                                                          collision_point_potential_gradient_[i][j]);
    }

    return colliding;
}

bool costComputation::performForwardKinematics( const ChompTrajectory& group_traj )
{
    double invTime = 1.0 / group_traj.getDiscretization();
    double invTimeSq = invTime*invTime;

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
    if ( iteration_==0 )
    {
        start = 0;
        end = num_vars_all_-1;
    }

    is_collision_free_ = true;

    Eigen::VectorXd joint_array;

    Configuration q( robot_model_ );

    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        group_traj.getTrajectoryPointP3d( group_traj.getFullTrajectoryIndex(i), joint_array );

        getFrames( i, joint_array, q );

        state_is_in_collision_[i] = false;

        if( use_costspace_ )
        {
            general_cost_potential_[i] = global_costSpace->cost(q);
            // cout << "compute cost : " << general_cost_potential_[i] << endl;
        }

        if( collision_space_ )
        {
            // calculate the position of every collision point:
            for (int j=0; j<num_collision_points_; j++)
            {
                bool colliding = getConfigObstacleCost( i, j, q );

                if ( colliding )
                {
                    // This is the function that discards joints too close to the base
                    if( planning_group_->collision_points_[j].getSegmentNumber() > 1 )
                    {
                        state_is_in_collision_[i] = true;
                    }
                }
            }
        }

        if( use_costspace_ && (collision_space_==NULL) )
        {
            state_is_in_collision_[i] = false;
        }

        if ( state_is_in_collision_[i] )
        {
            is_collision_free_ = false;
        }
    }

    if( collision_space_ )
    {
        // now, get the vel and acc for each collision point (using finite differencing)
        for (int i=free_vars_start_; i<=free_vars_end_; i++)
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
                collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();

                for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
                {
                    collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) *
                            collision_point_pos_eigen_[i+k][j];
                    collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) *
                            collision_point_pos_eigen_[i+k][j];
                }
                // get the norm of the velocity:
                collision_point_vel_mag_(i,j) = collision_point_vel_eigen_[i][j].norm();
            }
        }
    }

    return is_collision_free_;
}

bool costComputation::getCost(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, int iteration )
{
//    cout << __PRETTY_FUNCTION__ << endl;
    iteration_ = iteration;

    // copy the parameters into group_trajectory_:
    for (int d=0; d<num_joints_; ++d) {
        group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
    }
    //    group_trajectory_.print();

    // respect joint limits:
    succeded_joint_limits_ = handleJointLimits( group_trajectory_ );
    //succeded_joint_limits_ = true;

    // do forward kinematics:
    performForwardKinematics( group_trajectory_ );

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;
        double state_general_cost = 0.0;
        double cumulative = 0.0;

        if( collision_space_ )
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
                state_collision_cost += cumulative;
            }
        }
        if( use_costspace_ )
        {
            state_general_cost = ( pow( general_cost_potential_(i) , hack_tweek_ ) /* ( q_f - q_i ).norm() */) ;
        }

        costs(i-free_vars_start_) = obstacle_weight_ * ( state_collision_cost + state_general_cost );
    }

    // cout << costs.transpose() << endl;
    // cout << "return with joint_limits" << endl;

    // copy the parameters into group_trajectory_:
    for (int d=0; d<num_joints_; ++d) {
        parameters[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
    }

    return true;
}

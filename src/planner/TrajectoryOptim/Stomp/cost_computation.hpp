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
#ifndef COST_COMPUTATION_HPP
#define COST_COMPUTATION_HPP

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"
#include "planner/TrajectoryOptim/Stomp/stompParameters.hpp"

#include "collision_space/collision_space.hpp"

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

namespace stomp_motion_planner
{

class costComputation
{
public:
    costComputation(Move3D::Robot* robot,
                    const Move3D::CollisionSpace *collision_space,
                    const Move3D::ChompPlanningGroup* planning_group,
                    std::vector< Move3D::ChompCost > joint_costs,
                    Move3D::ChompTrajectory group_trajectory,
                    double obstacle_weight,
                    bool use_costspace,
                    Move3D::confPtr_t q_source,
                    Move3D::confPtr_t q_target,
                    bool use_external_collision_space,
                    int collision_space_id,
                    const stomp_motion_planner::StompParameters* stomp_parameters);

    ~costComputation();

    bool getCost(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number, bool joint_limits, bool resample, bool is_rollout );
    bool handleJointLimits( Move3D::ChompTrajectory& group_traj );
    double getCollisionSpaceCost( const Move3D::Configuration& q );
    bool getConfigObstacleCost( Move3D::Robot* robot, int i, Eigen::MatrixXd& collision_point_potential, std::vector< std::vector<Eigen::Vector3d> >& collision_point_pos );
    bool getCollisionPointObstacleCost( int segment, int coll_point, double& collion_point_potential, Eigen::Vector3d& pos );
    void getFrames(int segment, const Eigen::VectorXd& joint_array, Move3D::Configuration& q);
    bool performForwardKinematics( const Move3D::ChompTrajectory& group_traj, bool is_rollout );

    bool getJointLimitViolationSuccess() const { return succeded_joint_limits_; }

    const std::vector< std::vector< Eigen::Transform3d, Eigen::aligned_allocator<Eigen::Transform3d> > > & getSegmentFrames() const { return segment_frames_; }

    const std::vector< std::vector<Eigen::Vector3d> >& getJointPosEigen() const { return joint_pos_eigen_; }
    const std::vector< std::vector<Eigen::Vector3d> >& getJointAxisEigen() const { return joint_axis_eigen_; }
    const std::vector< std::vector<Eigen::Vector3d> >& getCollisionPointPosEigen() const { return collision_point_pos_eigen_; }
    const std::vector< std::vector<Eigen::Vector3d> >& getCollisionPointVelEigen() const { return collision_point_vel_eigen_; }
    const std::vector< std::vector<Eigen::Vector3d> >& getCollisionPointAccEigen() const { return collision_point_acc_eigen_; }
    const std::vector< std::vector<Eigen::Vector3d> >& getCollisionPointPotentialGradient() const { return collision_point_potential_gradient_; }

    const Eigen::MatrixXd& getCollisionPointPotential() const { return collision_point_potential_; }
    const Eigen::MatrixXd& getCollisionVelMag() const { return collision_point_vel_mag_; }
    const Eigen::VectorXd& getCollisionCostPotential() const { return general_cost_potential_; }
    const Eigen::VectorXd& getDts() const { return dt_; }

    const Move3D::ChompTrajectory& getGroupTrajectory() const { return group_trajectory_; }

private:

    Move3D::Robot* robot_model_;
    const Move3D::ChompPlanningGroup* planning_group_;

    double resampleParameters(std::vector<Eigen::VectorXd>& parameters);

    Move3D::ChompTrajectory group_trajectory_;

    bool is_collision_free_;
    bool succeded_joint_limits_;

    bool allow_end_configuration_motion_;
    bool use_costspace_;
    std::vector<int> state_is_in_collision_;
    std::vector< std::vector< Eigen::Transform3d, Eigen::aligned_allocator<Eigen::Transform3d> > > segment_frames_;
    std::vector< std::vector<Eigen::Vector3d> >  joint_axis_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  joint_pos_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_pos_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_vel_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_acc_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_potential_gradient_;
    Eigen::MatrixXd collision_point_potential_;
    Eigen::MatrixXd collision_point_vel_mag_;
    Eigen::VectorXd general_cost_potential_;
    Eigen::VectorXd dt_;
    std::vector< Move3D::ChompCost > joint_costs_;

    int free_vars_start_;
    int free_vars_end_;
    int num_vars_all_;
    int num_vars_free_;
    int id_fixed_;
    int iteration_;
    int num_collision_points_;
    int num_joints_;

    double hack_tweek_;
    double obstacle_weight_;

    double last_move3d_cost_;

    Move3D::confPtr_t source_;
    Move3D::confPtr_t target_;

    const Move3D::CollisionSpace *move3d_collision_space_;
    bool use_external_collision_space_;
    int collision_space_id_;

    const stomp_motion_planner::StompParameters* stomp_parameters_;
};

}

void move3d_set_api_functions_collision_space( bool use_move3d_fct );
bool move3d_use_api_functions_collision_space();
void move3d_set_fct_get_nb_collision_points( boost::function<int(Move3D::Robot*)> fct, int id=0 );
void move3d_set_fct_get_config_collision_cost( boost::function<bool( Move3D::Robot* robot, int i, Eigen::MatrixXd&, std::vector< std::vector<Eigen::Vector3d> >& )> fct, int id=0 ) ;

#endif // COST_COMPUTATION_HPP

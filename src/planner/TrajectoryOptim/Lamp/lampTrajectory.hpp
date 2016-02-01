/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI, MPI
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
#ifndef LAMP_HPP
#define LAMP_HPP

#include "API/Trajectory/trajectory.hpp"

#include <vector>

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/vector_trajectory.hpp"

#include "collision_space/collision_space.hpp"

#include <boost/shared_ptr.hpp>

#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"
#include "planner/TrajectoryOptim/Stomp/policy_improvement_loop.hpp"
#include "planner/TrajectoryOptim/Stomp/stompStatistics.hpp"
#include "planner/TrajectoryOptim/Stomp/cost_computation.hpp"

namespace Move3D {
class ConfGenerator;
}

//#include <motion_planning_msgs/Constraints.h>
//#include "constraint_evaluator.hpp"
//#include <stomp_motion_planner/STOMPStatistics.h>
////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

namespace Move3D {

//! Sampler of noisy trajectories
class LampSampler {
 public:
  LampSampler(Move3D::Robot* robot) { robot_model_ = robot; }
  LampSampler(int num_var_free, int num_dofs);
  ~LampSampler() { delete planning_group_; }

  //! Initializes the different data structures
  //! Initializes a coviarant trajectory policy
  //! Computes the control cost vector for the multivariate gaussian sampler
  void initialize(const std::vector<double>& derivative_costs, int nb_points);

  //! Samples a noisy trajectory
  Eigen::VectorXd sample(double std_dev = 1.0);

  //! Return sampled trajectories
  std::vector<VectorTrajectory> sampleTrajectories(
      int nb_trajectories, const Move3D::VectorTrajectory& current_trajectory);

  //! PLanning group
  Move3D::ChompPlanningGroup* planning_group_;

  //! Trajectory policy
  stomp_motion_planner::CovariantTrajectoryPolicy policy_;

  Eigen::MatrixXd control_;
  Eigen::MatrixXd precision_;
  Eigen::MatrixXd covariance_;

 private:
  Eigen::MatrixXd getOneDofBlockOfPrecisionMatrix(int dof);

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
  getConfigurationBlockOfPrecisionMatrix(int var);

  void setOneDofBlockOfPrecisionMatrix(int dof, Eigen::MatrixXd matrix);
  void setTimeStepPrecisionMatrix(int time_step, Eigen::MatrixXd matrix);
  bool addHessianToPrecisionMatrix(const Move3D::VectorTrajectory& traj);

  /**< [num_dimensions] num_parameters x num_parameters */
  std::vector<Eigen::MatrixXd> control_costs_;

  /**< [num_dimensions] num_parameters x num_parameters */
  std::vector<Eigen::MatrixXd> inv_control_costs_;

  //! Allocate a multivariate gaussian sampler
  //! the sampler produces one dimenssional noisy trajectories
  bool preAllocateMultivariateGaussianSampler();

  /**< objects that generate noise for each dimension */
  std::vector<MultivariateGaussian> noise_generators_;
  Eigen::VectorXd tmp_noise_;

  int num_vars_free_;
  int num_dofs_;

  double min_eigen_value_;
  double max_eigen_value_;

  // Planning group
  Move3D::Robot* robot_model_;
};
}

void lamp_sample_trajectories();

#endif

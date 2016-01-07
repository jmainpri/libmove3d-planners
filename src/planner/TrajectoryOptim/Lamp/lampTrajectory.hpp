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
#ifndef LAMP_HPP
#define LAMP_HPP

#include "API/Trajectory/trajectory.hpp"

#include <vector>

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
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

//! Trajectory structure
struct VectorTrajectory {
  VectorTrajectory() : discretization_(0.0) {}
  VectorTrajectory(int nb_joints, int nb_var, double duration);

  //!  \brief Gets the number of points in the trajectory
  int getNumPoints() const { return num_vars_free_; }

  //!  \brief Gets the number of points (that are free to be optimized) in the
  //trajectory
  int getNumFreePoints() const { return num_vars_free_; }

  //!  \brief Gets the number of joints in each trajectory point
  int getNumDofs() const { return num_dofs_; }

  //! \brief Gets the discretization time interval of the trajectory
  double getDiscretization() const { return discretization_; }

  //!  \brief Gets the start index
  int getStartIndex() const { return 0; }

  //!  \brief Gets the end index
  int getEndIndex() const { return num_vars_free_ - 1; }

  //! Get use time
  bool getUseTime() const { return use_time_; }

  //! Get index for traj point and dof
  int getVectorIndex(int traj_point, int dof);

  //!  \brief Get the value at a given point for a given dof
  double& operator()(int traj_point, int dof);

  //!  \brief Get the value at a given point for a given dof
  double operator()(int traj_point, int dof) const;

  //!  \brief Get dof cost value
  double& dof_cost(int traj_point, int dof);

  //!  \brief Get dof cost value
  double dof_cost(int traj_point, int dof) const;

  //! Set one dof trajectory
  void setDofTrajectoryBlock(int dof, const Eigen::VectorXd traj);

  //! Set one dof trajectory
  void addToDofTrajectoryBlock(int dof, const Eigen::VectorXd traj);

  //! Get trajectory paramters
  Eigen::VectorXd getDofTrajectoryBlock(int dof) const;

  //! Get trajectory paramters
  bool getParameters(std::vector<Eigen::VectorXd>& parameters) const;

  //! Get trajectory paramters
  bool getFreeParameters(std::vector<Eigen::VectorXd>& parameters) const;

  //! Get trajectory paramters
  Eigen::VectorXd getTrajectoryPoint(int i) const;

  //! Planning group
  Move3D::ChompPlanningGroup* planning_group_;

  //! Interpolation between two vector
  Eigen::VectorXd interpolate(const Eigen::VectorXd& a,
                              const Eigen::VectorXd& b,
                              double u) const;

  //! Sets the interpolated trajectory
  Eigen::VectorXd getSraightLineTrajectory();

  //! Return move3d configuration
  Move3D::confPtr_t getMove3DConfiguration(int i) const;

  //! Returns the move3d trajectory
  Move3D::Trajectory getMove3DTrajectory() const;

  //! Set from move3d trajectory
  void setFromMove3DTrajectory(const Move3D::Trajectory& T);

  //! Get configuration at particular point
  void getTrajectoryPointP3d(int traj_point, Eigen::VectorXd& jnt_array) const;

  //! Return the index along the trajectory
  int getFullTrajectoryIndex(int i) const { return i; }

  /**< [num_dimensions] num_parameters */
  Eigen::VectorXd trajectory_;

  /**< [num_dimensions] num_parameters */
  Eigen::VectorXd dof_costs_;

  /**< num_time_steps */
  Eigen::VectorXd state_costs_;
  int num_vars_free_; /**< nb_vars_ */
  int num_dofs_;      /**< nb_joints_ */

  /**< Wether the rollout is violating dof limits */
  bool out_of_bounds_;
  double discretization_; /**< time discretization */
  double duration_;       /**< duration */
  bool use_time_;
  double total_cost_;
  double total_smoothness_cost_;
};

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

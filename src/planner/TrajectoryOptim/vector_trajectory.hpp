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
#ifndef VECTOR_TRAJECORY_HPP
#define VECTOR_TRAJECORY_HPP

#include "API/Trajectory/trajectory.hpp"

#include <vector>

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/vector_trajectory.hpp"

namespace Move3D {

//! Trajectory structure
struct VectorTrajectory {
  VectorTrajectory() : discretization_(0.0) {}
  VectorTrajectory(int nb_dofs, int nb_var, double duration);

  //!  \brief Gets the number of points in the trajectory
  int getNumPoints() const { return num_vars_free_; }

  //!  \brief Gets the number of points (that are free to be optimized) in the
  //! trajectory
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

  //! Planning group
  Move3D::ChompPlanningGroup* planning_group_;

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

}

#endif

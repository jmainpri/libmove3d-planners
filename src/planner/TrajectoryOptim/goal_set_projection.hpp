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

#ifndef GOAL_SET_PROJECTION_HPP_
#define GOAL_SET_PROJECTION_HPP_

#include "eiquadprog.hpp"
#include "API/Device/robot.hpp"
#include "planner/TrajectoryOptim/vector_trajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include <iostream>

// This class solves the joint limit projection using a QP
// the problem is of the form :
//
// min 0.5 * xi^t A xi + g0^t xi
// s.t.
//    CE^t x + ce0 = 0
//    CI^t xi + ci0 >= 0
//
// The matrix and vectors dimensions are as follows:
//    xi.size() : n
//    A.size()  : n * n
//    g0.size() : n
//    CE.size() : n * 1
//    ce0       : 1
//    CI.size() : n * m
//    ci0       : m
//
// here m is equal to 2 because we have upper and lower limits
// g0 sets an offest because it is of the original problem is of the form
//
// min 0.5 * (x-xi)^t A (x-xi)
// s.t.
//    CI^t xi + ci0 >= 0
//
// this leads to the following offset: g0 = ( -1 * x^t * A )^t
// => xi^t A xi + ( -1 * x^t * A )^t xi  = (x-xi)^t A (x-xi)
class TrajOptGoalSet {
 public:
  TrajOptGoalSet();

  // Setup the goal set projector
  bool Initialize(const Move3D::ChompPlanningGroup* pg,
                  const std::vector<Eigen::MatrixXd>& control_cost);

  // task space goal position
  bool ProjectGoalSetProblem(const Eigen::VectorXd& x_goal_pos,
                             std::vector<Eigen::VectorXd>& parameters);

  // end effector
  Move3D::Joint* end_effector() { return eef_; }
  void set_print_convergence(bool v) { print_convergence_ = v; }

 private:
  // Run the QP problem
  bool Initialize(const Eigen::MatrixXd& quadric);
  double Project(Eigen::VectorXd& parameters,
                 const Eigen::MatrixXd& quadric,
                 const Eigen::VectorXd& linear,
                 const Eigen::MatrixXd& equality_coeff,
                 const Eigen::VectorXd& equality_value);
  bool QPProject(const Eigen::VectorXd& h_k,
                 const Eigen::VectorXd& g_h_k,
                 Eigen::VectorXd& xi);
  Eigen::MatrixXd in_select_;  // this is set internaly
  Eigen::VectorXd in_const_;   //
  Eigen::MatrixXd eq_select_;
  Eigen::VectorXd eq_const_;
  double c1_;
  Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> chol_;
  double c2_;
  Eigen::MatrixXd J_;
  double lambda_;

  // Manipulator Jacobian functions
  Eigen::MatrixXd GetJacobian(const Eigen::VectorXd& x_error,
                              const Eigen::VectorXd& q_end,
                              bool check_joint_limits) const;
  bool CheckViolateJointLimits(const Eigen::VectorXd& q,
                               std::vector<int>& badjointinds) const;
  bool alpha_;
  bool has_free_flyer_;

  // Trajectory constraints
  Eigen::VectorXd upper_;     // Upper joint limit bounds
  Eigen::VectorXd lower_;     // Lower joint limit bounds
  Eigen::MatrixXd dynamics_;  // Control cost
  Move3D::VectorTrajectory traj_cur_;

  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd Ainv_col_block_;
  Eigen::MatrixXd Ainv_last_block_;
  Eigen::MatrixXd Ainv_last_block_without_free_flyer_;

  // Setup problem
  void SetOneDofBlockOfDynamicsMatrix(int dof, const Eigen::MatrixXd& matrix);
  void SetDynamicsMatrix(const std::vector<Eigen::MatrixXd>& control_cost);
  void SetDofLimits();

  // Robot Structures
  Move3D::Robot* robot_;
  Move3D::Joint* eef_;
  const Move3D::ChompPlanningGroup* planning_group_;
  std::vector<Move3D::Joint*> active_joints_;
  std::vector<int> active_dofs_;
  Move3D::VectorTrajectory trajectory_;



  bool print_convergence_;
};

#endif

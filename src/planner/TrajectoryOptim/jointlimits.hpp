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

#ifndef TRAJECTORY_JOINT_LIMITS_HPP_
#define TRAJECTORY_JOINT_LIMITS_HPP_

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "eiquadprog.hpp"
#include <iostream>

namespace Move3D {

// This class solves the joint limit projection using a QP
// the problem is of the form :
//
// min 0.5 * xi^t A xi + g0^t xi
// s.t.
//    CI^t xi + ci0 >= 0
//
// The matrix and vectors dimensions are as follows:
//    xi.size() : n
//    A.size()  : n * n
//    g0.size() : n
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
class TrajOptJointLimit {
 public:
  TrajOptJointLimit() {}

  bool initialize() {
    if (upper_.size() != lower_.size()) return false;
    if (upper_.size() != dynamics_.rows()) return false;
    if (upper_.size() != dynamics_.cols()) return false;

    int nb_params = upper_.size();
    selection_ = Eigen::MatrixXd::Zero(nb_params, 2 * nb_params);
    selection_.block(0, 0, nb_params, nb_params) =
        Eigen::MatrixXd::Identity(nb_params, nb_params);
    selection_.block(0, nb_params, nb_params, nb_params) =
        -1. * Eigen::MatrixXd::Identity(nb_params, nb_params);

    constraints_ = Eigen::VectorXd::Zero(2 * nb_params);
    constraints_.segment(0, nb_params) = -1. * lower_;
    constraints_.segment(nb_params, nb_params) = upper_;

    chol_ = Eigen::LLT<Eigen::MatrixXd, Eigen::Lower>(dynamics_.cols());

    /* compute the trace of the original matrix G */
    c1_ = dynamics_.trace();

    /* decompose the matrix G in the form LL^T */
    chol_.compute(dynamics_);

    /* compute the inverse of the factorized matrix G^-1, this is the initial
     * value for H */
    // J = L^-T
    J_ = Eigen::MatrixXd::Identity(dynamics_.rows(), dynamics_.rows());
    J_ = chol_.matrixU().solve(J_);

    /* c1 * c2 is an estimate for cond(G) */
    c2_ = J_.trace();

    return true;
  }

  double project(Eigen::VectorXd& parameters) const {
    if (parameters.size() != upper_.size()) return 0.0;

    Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> chol = chol_;
    Eigen::VectorXd offset =
        -1. * (parameters.transpose() * dynamics_).transpose();
    Eigen::MatrixXd J = J_;
    Eigen::VectorXd lagrange_mult;

    double cost = Eigen::solve_quadprog2<Eigen::VectorXd,
                                         Eigen::MatrixXd,
                                         Eigen::VectorXd,
                                         Eigen::MatrixXd,
                                         Eigen::VectorXd,
                                         Eigen::VectorXd>(chol,
                                                          c1_,
                                                          J,
                                                          c2_,
                                                          offset,
                                                          eq_select_,
                                                          eq_const_,
                                                          selection_,
                                                          constraints_,
                                                          parameters,
                                                          lagrange_mult);

    return cost;
  }

  friend bool CreateJointLimitProjectors(
      std::vector<TrajOptJointLimit>& joint_limits_projector,
      const Move3D::ChompPlanningGroup& planning_group,
      const std::vector<Eigen::MatrixXd>& control_costs,
      int num_var,
      int init_offset,
      int id_fixed = 0) {
    bool succeded = true;
    joint_limits_projector.resize(planning_group.chomp_dofs_.size());

    for (size_t d = 0; d < planning_group.chomp_dofs_.size(); d++) {
      double max_limit = planning_group.chomp_dofs_[d].joint_limit_max_ - 1e-5;
      double min_limit = planning_group.chomp_dofs_[d].joint_limit_min_ + 1e-5;

      joint_limits_projector[d].dynamics_ = control_costs[d].block(
          init_offset, init_offset, num_var - id_fixed, num_var - id_fixed);
      joint_limits_projector[d].upper_ =
          max_limit * Eigen::VectorXd::Ones(num_var - id_fixed);
      joint_limits_projector[d].lower_ =
          min_limit * Eigen::VectorXd::Ones(num_var - id_fixed);

      if (!joint_limits_projector[d].initialize()) {
        std::cout << "Error: could not initialize joint limits in "
                  << __PRETTY_FUNCTION__ << " for joint : "
                  << planning_group.chomp_dofs_[d].joint_name_ << std::endl;
        succeded = false;
      }
    }
    return succeded;
  }

  // TODO move privte
  Eigen::MatrixXd dynamics_;  // control cost matrix
  Eigen::VectorXd upper_;     // Upper joint limit bounds
  Eigen::VectorXd lower_;     // Lower joint limit bounds

 private:
  Eigen::MatrixXd selection_;    // this is set internaly
  Eigen::VectorXd constraints_;  //

  Eigen::MatrixXd eq_select_;
  Eigen::VectorXd eq_const_;

  double c1_;
  Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> chol_;

  double c2_;
  Eigen::MatrixXd J_;
};
}

#endif

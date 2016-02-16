/*
 * Copyright (c) 2010-2015 LAAS/CNRS, WPI
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
 *                                                 Jim Mainprice Mon 25 Jan 2016
 */

#include "goal_set_projection.hpp"
#include "planEnvironment.hpp"
#include <iomanip>
using std::cout;
using std::endl;

TrajOptGoalSet::TrajOptGoalSet()
    : has_free_flyer_(false), robot_(NULL), print_convergence_(false) {}

bool TrajOptGoalSet::Initialize(
    const Move3D::ChompPlanningGroup* pg,
    const std::vector<Eigen::MatrixXd>& control_cost) {
  planning_group_ = pg;
  robot_ = planning_group_->robot_;
  active_dofs_ = planning_group_->getActiveDofs();
  active_joints_ = planning_group_->getActiveJoints();
  eef_ = robot_->getJoint(
      PlanEnv->getString(PlanParam::end_effector_joint));  // plannar J4

  if (robot_ == NULL) {
    return false;
  }
  if (eef_ == NULL) {
    return false;
  }
  if (active_dofs_.empty() || active_joints_.empty()) {
    return false;
  }

  cout << "eef joint name : " << eef_->getName() << endl;
  // cout << "control_cost[0].rows() : " << control_cost[0].rows() << endl;
  // cout << "control_cost[0].cols() : " << control_cost[0].cols() << endl;

  // Set trajectory with the correct dimensions
  Move3D::VectorTrajectory traj_tmp(
      active_dofs_.size(), control_cost[0].rows(), 0.0);
  trajectory_ = traj_tmp;

  has_free_flyer_ = false;
  for (size_t i = 0; i < planning_group_->chomp_dofs_.size(); i++) {
    if (planning_group_->chomp_dofs_[i].is_free_flyer_) {
      // has_free_flyer_ = true;
    }
  }

  // Set the control cost
  SetDynamicsMatrix(control_cost);
  SetDofLimits();

  // Parameter for the Jacobian gradient descent
  alpha_ = .01;
  return true;
}

bool TrajOptGoalSet::Initialize(const Eigen::MatrixXd& quadric) {
  if (upper_.size() != lower_.size()) return false;
  if (upper_.size() != quadric.rows()) return false;
  if (upper_.size() != quadric.cols()) return false;
  if (upper_.size() != traj_cur_.trajectory_.size()) return false;

  int nb_params = upper_.size();

  const bool with_joint_limits = true;
  if (with_joint_limits) {
    // Set Inequality selection
    in_select_ = Eigen::MatrixXd::Zero(nb_params, 2 * nb_params);
    in_select_.block(0, 0, nb_params, nb_params) =
        Eigen::MatrixXd::Identity(nb_params, nb_params);
    in_select_.block(0, nb_params, nb_params, nb_params) =
        -1. * Eigen::MatrixXd::Identity(nb_params, nb_params);

    // Set Inequality constraint
    in_const_ = Eigen::VectorXd::Zero(2 * nb_params);
    in_const_.segment(0, nb_params) = -1. * lower_ - traj_cur_.trajectory_;
    in_const_.segment(nb_params, nb_params) = upper_ + traj_cur_.trajectory_;
  }

  chol_ = quadric.cols();

  /* compute the trace of the original matrix G */
  c1_ = quadric.trace();

  /* decompose the matrix G in the form LL^T */
  chol_.compute(quadric);

  /* compute the inverse of the factorized matrix G^-1,
   * this is the initial value for H */
  // J = L^-T
  J_ = Eigen::MatrixXd::Identity(quadric.rows(), quadric.rows());
  J_ = chol_.matrixU().solve(J_);

  /* c1 * c2 is an estimate for cond(G) */
  c2_ = J_.trace();

  return true;
}

double TrajOptGoalSet::Project(Eigen::VectorXd& parameters,
                               const Eigen::MatrixXd& quadric,
                               const Eigen::VectorXd& linear,
                               const Eigen::MatrixXd& equality_coeff,
                               const Eigen::VectorXd& equality_value) {
  if (parameters.size() != upper_.size()) {
    cout << "Error : parameter does not match limtis size" << endl;
    return -1;
  }

  if (!Initialize(quadric)) {
    cout << "Error : initialize" << endl;
    return -1;
  }

  Eigen::LLT<Eigen::MatrixXd, Eigen::Lower> chol = chol_;
  Eigen::MatrixXd J = J_;
  Eigen::VectorXd g0 = linear;
  Eigen::VectorXd lagrange_mult;

  //  double cost =
  Eigen::solve_quadprog2<Eigen::VectorXd,
                         Eigen::MatrixXd,
                         Eigen::VectorXd,
                         Eigen::MatrixXd,
                         Eigen::VectorXd,
                         Eigen::VectorXd>(chol,
                                          c1_,
                                          J,
                                          c2_,
                                          g0,
                                          equality_coeff,
                                          equality_value,
                                          in_select_,
                                          in_const_,
                                          parameters,
                                          lagrange_mult);

  // Get the lagrange multiplier
  // cout << "lagrange mult.size() : " << lagrange_mult.size() << endl;
  lambda_ = lagrange_mult[0];

  return 0;
}

void TrajOptGoalSet::SetDofLimits() {
  int num_var = trajectory_.getNumFreePoints();
  int num_dofs = trajectory_.getNumDofs();
  upper_.resize(num_var * num_dofs);
  lower_.resize(num_var * num_dofs);

  for (int i = 0; i < num_var; i++) {     // traj_point
    for (int d = 0; d < num_dofs; d++) {  // dof
      double max_limit =
          planning_group_->chomp_dofs_[d].joint_limit_max_ - 1e-5;
      double min_limit =
          planning_group_->chomp_dofs_[d].joint_limit_min_ + 1e-5;
      upper_[i * num_dofs + d] = max_limit;
      lower_[i * num_dofs + d] = min_limit;
    }
  }

  // cout << "upper : " << upper_.transpose() << endl;
  // cout << "lower : " << lower_.transpose() << endl;
}

void TrajOptGoalSet::SetOneDofBlockOfDynamicsMatrix(
    int dof, const Eigen::MatrixXd& matrix) {
  int num_dofs = trajectory_.getNumDofs();
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      if ((i * num_dofs + dof) < dynamics_.rows() &&
          (j * num_dofs + dof) < dynamics_.cols()) {
        dynamics_(i * num_dofs + dof, j * num_dofs + dof) += matrix(i, j);
      }
    }
  }
}

void TrajOptGoalSet::SetDynamicsMatrix(
    const std::vector<Eigen::MatrixXd>& control_cost) {
  int num_var = trajectory_.getNumFreePoints();
  int num_dofs = trajectory_.getNumDofs();

  cout << __PRETTY_FUNCTION__ << endl;
  cout << "num_var_free : " << num_var << endl;
  cout << "num_dofs : " << num_dofs << endl;

  dynamics_ = Eigen::MatrixXd::Zero(num_var * num_dofs, num_var * num_dofs);

  cout << "dynamics.rows() : " << dynamics_.rows() << endl;
  cout << "dynamics.cols() : " << dynamics_.cols() << endl;

  cout << "control_costs.size() : " << control_cost.size() << endl;
  for (int d = 0; d < num_dofs; d++) {
    SetOneDofBlockOfDynamicsMatrix(d, control_cost[d]);
    // cout << endl
    //     << "control_cost[" << j << "] : " << control_cost[j] << endl;
  }

  std::stringstream ss;
  ss.str("");
  ss << std::string(getenv("HOME_MOVE3D")) + "/../move3d-launch/launch_files/";
  ss << "control_matrices/A_inverse.data";

  const bool compute_a_inverse = true;
  if (compute_a_inverse) {
    cout << "cash A.inverse()" << endl;
    Ainv_ = dynamics_.inverse();
    //matrix_write_binary(ss.str(), Ainv_);
  } else {
    cout << "load A.inverse()" << endl;
    matrix_read_binary(ss.str(), Ainv_);
  }

  Ainv_col_block_ =
      Ainv_.block(0, Ainv_.cols() - num_dofs, Ainv_.rows(), num_dofs);

  Ainv_last_block_ = Ainv_.block(
      Ainv_.rows() - num_dofs, Ainv_.cols() - num_dofs, num_dofs, num_dofs);

  if (has_free_flyer_) {
    // For this matrix we remove the block that corresponds to the 6 FF dofs
    Ainv_last_block_without_free_flyer_ =
        Ainv_.block(Ainv_.rows() - num_dofs + 6,
                    Ainv_.cols() - num_dofs + 6,
                    num_dofs - 6,
                    num_dofs - 6);
  }

  // cout << "dynamics : " << endl;
  // cout << std::fixed << std::setw(2) << std::setprecision(2) << dynamics_
  //     << endl;
}

bool TrajOptGoalSet::QPProject(const Eigen::VectorXd& h_k,
                               const Eigen::VectorXd& g_h_k,
                               Eigen::VectorXd& xi) {
  int num_var = trajectory_.getNumFreePoints();
  int num_dofs = trajectory_.getNumDofs();

  // Get full gradient of the goal set constraint
  Eigen::MatrixXd g_h_k_full = Eigen::MatrixXd::Zero(num_var * num_dofs, 1);
  g_h_k_full.block(g_h_k_full.rows() - num_dofs, 0, num_dofs, 1) = g_h_k;

  // Get hessian of the lagrangian using the guass-newton approximation
  // to the Hessian Lambda is the lagrange multiplier
  Eigen::MatrixXd H_h_k = lambda_ * (g_h_k * g_h_k.transpose());

  // Only set the last block the rest is 0
  Eigen::MatrixXd H_h_k_full =
      Eigen::MatrixXd::Zero(num_var * num_dofs, num_var * num_dofs);
  int last_block = H_h_k_full.rows() - num_dofs;
  H_h_k_full.block(last_block, last_block, num_dofs, num_dofs) = H_h_k;

  // Get hessian of the root problem
  Eigen::MatrixXd hessian_f_k = 2. * dynamics_ + H_h_k_full;

  // Get gradient of the objective
  Eigen::VectorXd g_f_k =
      2. * dynamics_ * (traj_cur_.trajectory_ - trajectory_.trajectory_);

  const bool print_debug = false;
  if (print_debug) {
    // cout << "h_k : " << h_k << endl;
    // cout << "x_goal_pos : " << x_goal_pos.transpose() << endl;
    cout << "eef_ : " << eef_->getVectorPos().transpose() << endl;
    // cout << "x_error : " << x_error.transpose() << endl;
    cout << "g_h_k : " << g_h_k.transpose() << endl;
    cout << "g_h_k_full : " << g_h_k_full.transpose() << endl;
    cout << "g_f_k.size() : " << g_f_k.size() << endl;
    cout << "dynamics.rows() : " << dynamics_.rows() << endl;
    cout << "dynamics.cols() : " << dynamics_.cols() << endl;
    cout << "H_h_k.rows() : " << H_h_k.rows() << endl;
    cout << "H_h_k.cols() : " << H_h_k.cols() << endl;
    cout << "H_h_k_full.rows() : " << H_h_k_full.rows() << endl;
    cout << "H_h_k_full.cols() : " << H_h_k_full.cols() << endl;
    // cout << dynamics_ << endl;
    // cout << H_h_k << endl;
    // cout << std::scientific << H_h_k_full << endl;
    // cout << std::scientific << hessian_f_k << endl;
    // cout << std::scientific << hessian_f_k.inverse() << endl;
  }

  if (Project(xi, hessian_f_k, g_f_k, g_h_k_full, h_k) < 0.) {
    cout << "Error in projection" << endl;
    return false;
  }
  return true;
}

Eigen::MatrixXd TrajOptGoalSet::GetJacobian(const Eigen::VectorXd& x_error,
                                            const Eigen::VectorXd& q_end,
                                            bool check_joint_limits) const {
  bool with_rotation = x_error.size() == 6;
  bool with_height = x_error.size() == 3;

  Eigen::MatrixXd J =
      robot_->getJacobian(active_joints_, eef_, with_rotation, with_height);

  if (!check_joint_limits) {
    return J;
  }

  std::vector<int> badjointinds;
  Eigen::VectorXd q_s = q_end;
  Eigen::VectorXd dq;
  bool violate_limit = false;

  int i=0;
  int iter_max=100;
  do {
    Eigen::VectorXd q_s_old = q_s;

    // eliminate bad joint columns from the Jacobian
    for (size_t d = 0; d < badjointinds.size(); d++) {
      for (int k = 0; k < x_error.size(); k++) {
        J(k, badjointinds[d]) = 0;
      }
    }

    i++;

    Eigen::MatrixXd Jplus = move3d_pinv(alpha_ * J, 1e-3);

    dq = Jplus * alpha_ * x_error;

    // add step
    q_s = q_s_old - dq;

    // get all ids where the joint limits are vialoted
    violate_limit = CheckViolateJointLimits(q_s, badjointinds);

    // move back to previous point if any joint limits
    if (violate_limit) q_s = q_s_old;

  } while (violate_limit && i<iter_max);

  return J;
}

bool TrajOptGoalSet::CheckViolateJointLimits(
    const Eigen::VectorXd& q, std::vector<int>& badjointinds) const {
  bool violate_limit = false;

  // TODO that through the planning group
  int dof_id = -1;
  for (size_t i = 0; i < active_joints_.size(); i++) {
    for (size_t j = 0; j < active_joints_[i]->getNumberOfDof(); j++) {
      // Increment the dof id before passing the circular dofs
      dof_id++;

      double lowerLimit, upperLimit;
      active_joints_[i]->getDofRandBounds(j, lowerLimit, upperLimit);

      // Do not check joint limits if outside of DoF limits
      if (active_joints_[i]->isJointDofCircular(j) &&
          std::fabs(lowerLimit - M_PI) < 1e-3 &&
          std::fabs(upperLimit + M_PI) < 1e-3) {
        continue;
      }

      if (q[dof_id] < lowerLimit || q[dof_id] > upperLimit) {
        badjointinds.push_back(
            dof_id);  // note this will never add the same joint
                      // twice, even if bClearBadJoints = false
        violate_limit = true;
      }
    }
  }

  return violate_limit;
}

bool TrajOptGoalSet::ProjectGoalSetProblem(
    const Eigen::VectorXd& x_goal_pos,
    std::vector<Eigen::VectorXd>& parameters) {
  if (robot_ == NULL) {
    return false;
  }

  // cout << "x_goal_pos : " << x_goal_pos.transpose() << endl;
  // cout << "parameters[0].size() : " << parameters[0].size() << endl;

  // Set trajectory from parameters
  for (size_t d = 0; d < parameters.size(); d++) {
    // cout << "parameters[" << d << "] : " << parameters[d].transpose() <<
    // endl;
    for (int i = 0; i < parameters[d].size(); i++) {
      trajectory_(i, d) = parameters[d][i];
    }
  }

  // Initial values
  Move3D::VectorTrajectory traj_xi = trajectory_;  // update
  Eigen::VectorXd& xi = traj_xi.trajectory_;
  lambda_ = 1.;
  traj_cur_ = trajectory_;  // current solution

  int num_var = trajectory_.getNumFreePoints();
  // int num_dofs = trajectory_.getNumDofs();

  // Eigen::MatrixXd& A = dynamics_;

  bool succeed = false;
  for (int i = 0; i < 200; i++) {
    // Get jacobian of manipulator
    Move3D::confPtr_t q = robot_->getCurrentPos();
    Eigen::VectorXd q_end_cur = traj_cur_.getTrajectoryPoint(num_var - 1);
    q->setFromEigenVector(q_end_cur, active_dofs_);
    robot_->setAndUpdate(*q);
    Eigen::VectorXd x_cur_pos = eef_->getVectorPos();
    Eigen::VectorXd x_error = x_cur_pos - x_goal_pos;
    if (print_convergence_) {
      cout << "i : " << i << " , dist : " << x_error.norm()
           << ", x_error : " << x_error.transpose() << endl;
    }
    if (x_error.norm() < .01) {
      succeed = true;
      break;
    }

    Eigen::MatrixXd J = GetJacobian(x_error, q_end_cur, true);

    if (x_error.size() != J.rows()) {
      cout << "Error: jacobian not right dimensions (1)" << endl;
      break;
    }
    if (int(active_dofs_.size()) != J.cols()) {
      cout << "Error: jacobian not right dimensions (2)" << endl;
      break;
    }

    const bool print_debug = false;
    if (print_debug) {
      cout << "J : " << endl
           << J << endl;

      for (size_t i = 0; i < active_dofs_.size(); i++) {
        cout << "active_dofs[" << i << "] : " << active_dofs_[i] << endl;
      }
    }

    const bool use_qp_ = false;
    if (!use_qp_) {
      // Perform simple gradient descent
      // Get full gradient of the goal set constraint
      Eigen::VectorXd x_d_error = alpha_ * x_error;
      Eigen::VectorXd h_k_s = x_d_error;
      // Get gradient/jacobian of the goal set constraint
      Eigen::MatrixXd g_h_k_s = J;

      // C is a row matrix (only values on the collumns)
      // m : x_d_error.size()
      // n : num_var*num_dofs
      // Eigen::MatrixXd C =
      //    Eigen::MatrixXd::Zero(g_h_k_s.rows(), num_var * num_dofs);
      // for (int j = 0; j < x_d_error.size(); j++) {
      //  for (int d = 0; d < num_dofs; d++) {
      //    C(j, C.cols() - num_dofs + d) = g_h_k_s(j, d);
      //  }
      // }

      Eigen::MatrixXd Reg(
          Eigen::MatrixXd::Zero(x_d_error.size(), x_d_error.size()));
      Reg.diagonal() = 0.0001 * Eigen::VectorXd::Ones(Reg.diagonal().size());

      // Eigen::MatrixXd mat1 = Ainv_ * C.transpose();
      Eigen::MatrixXd mat1 = Ainv_col_block_ * g_h_k_s.transpose();

      // Eigen::MatrixXd mat2 = (C * Ainv_ * C.transpose() + Reg).inverse();
      Eigen::MatrixXd mat2;

      if (has_free_flyer_) {
        Eigen::MatrixXd g_h_without_free_flyer =
            g_h_k_s.block(0, 6, x_d_error.size(), g_h_k_s.cols() - 6);

        mat2 = (g_h_without_free_flyer * Ainv_last_block_without_free_flyer_ *
                    g_h_without_free_flyer.transpose() +
                Reg).inverse();
      } else {
        mat2 =
            (g_h_k_s * Ainv_last_block_ * g_h_k_s.transpose() + Reg).inverse();
      }

      // projection to the constraint does not work ...
      // Eigen::MatrixXd mat3 = (C * traj_cur_.trajectory_ - h_k_s);
      Eigen::MatrixXd mat3 = (-h_k_s);

      xi = mat1 * mat2 * mat3;

    } else {
      // TODO see if the sign is correct this is for a potential of the form
      // h(xi) = || x(q_goal) - x_0 ||^2

      // 1) Constraint evaluation
      Eigen::VectorXd h_k = x_error.squaredNorm() * Eigen::VectorXd::Ones(1);
      // 2) Get gradient/jacobian of the goal set constraint
      Eigen::VectorXd g_h_k = 2. * x_error.transpose() * J;

      QPProject(h_k, g_h_k, xi);
    }

    const double eta = 1.;
    traj_cur_.trajectory_ += (eta * xi);
  }

  if (succeed && is_finite(traj_cur_.trajectory_)) {
    // Set trajectory
    // cout << " n 1 : " << parameters.size() * parameters[0].size() << endl;
    // cout << " n 2 : " << traj_cur.trajectory_.size() << endl;
    for (size_t d = 0; d < parameters.size(); d++) {
      for (int i = 0; i < parameters[d].size(); i++) {
        traj_cur_(0, d) = trajectory_(0, d);  // set the initial config
        parameters[d][i] = traj_cur_(i, d);
      }
    }
  }

  return succeed;
}

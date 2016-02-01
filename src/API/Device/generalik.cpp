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

#include "generalik.hpp"
#include "utils/misc_functions.hpp"

#include <libmove3d/include/P3d-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

GeneralIK::GeneralIK(Move3D::Robot* robot) : robot_(robot) {}

bool GeneralIK::initialize(const std::vector<Move3D::Joint*>& joints,
                           Move3D::Joint* eef) {
  eef_ = eef;
  active_joints_.clear();
  active_dofs_.clear();

  // Remove Freeflyer from active joint set
  for (size_t i = 0; i < joints.size(); i++) {
    if (joints[i]->getP3dJointStruct()->type == P3D_FREEFLYER) continue;
    active_joints_.push_back(joints[i]);
  }

  for (size_t i = 0; i < active_joints_.size(); i++) {
    for (size_t j = 0; j < active_joints_[i]->getNumberOfDof(); j++) {
      active_dofs_.push_back(active_joints_[i]->getIndexOfFirstDof() + j);
      // cout << "active_dof[" << active_dofs_.size() - 1
      //      << "] : " << active_dofs_.back() << endl;
    }
  }
  magnitude_ = 0.1;
  nb_steps_ = 100;
  check_joint_limits_ = true;
  max_distance_to_target_ = 0.01;  // was 0.001 (i.e., 1 mm)
  return true;
}

bool GeneralIK::solve(const Eigen::VectorXd& xdes) const {
  // Check the joint limits before ik
  Move3D::confPtr_t q_cur = robot_->getCurrentPos();
  std::vector<int> badjointids;
  if (check_joint_limits_ &&
      checkViolateJointLimits(
          q_cur->getEigenVector(active_dofs_), badjointids, true)) {
    cout << __PRETTY_FUNCTION__ << " : q_cur violates joint limits" << endl;
    return false;
  }

  double dist = .0;
  bool has_succeeded = false;
  Eigen::VectorXd q_new;
  bool with_rotations = (xdes.size() == 6);

  for (int i = 0; i < nb_steps_; i++)  // IK LOOP
  {
    Eigen::VectorXd q = q_cur->getEigenVector(active_dofs_);

    if (check_joint_limits_)
      q_new = q + single_step_joint_limits(xdes);
    else
      q_new = q + single_step(xdes);

    q_cur->setFromEigenVector(q_new, active_dofs_);
    robot_->setAndUpdate(*q_cur);
    dist = (xdes - (with_rotations ? eef_->getXYZPose()
                                   : Eigen::VectorXd(eef_->getVectorPos().head(
                                         xdes.size())))).norm();
    // cout << "diff = " << dist << endl;

    if (dist < max_distance_to_target_) {
      // cout << "success (" << i << "), diff = " << dist << endl;
      has_succeeded = true;
      break;
    }
  }

  if (!has_succeeded) {
    cout << __PRETTY_FUNCTION__ << "  dist = " << dist << endl;
  }

  return has_succeeded;
}

// TODO shift sign of the xdes to get it
// as the traditional gradient descent
Eigen::VectorXd GeneralIK::single_step(const Eigen::VectorXd& xdes) const {
  bool with_rotation = xdes.size() == 6;
  bool with_height = xdes.size() == 3;

  Eigen::VectorXd x_error =
      xdes -
      (with_rotation ? eef_->getXYZPose()
                     : Eigen::VectorXd(eef_->getVectorPos())).head(xdes.size());

  Eigen::MatrixXd J =
      robot_->getJacobian(active_joints_, eef_, with_rotation, with_height);
  Eigen::MatrixXd Jplus = move3d_pinv(J, 1e-3);
  // cout << "Jt : " << endl << Jt << endl;
  Eigen::VectorXd dq = Jplus * magnitude_ * x_error;
  return dq;
}

Eigen::VectorXd GeneralIK::single_step_joint_limits(
    const Eigen::VectorXd& xdes) const {
  bool with_rotation = xdes.size() == 6;
  bool with_height = xdes.size() == 3;

  Eigen::VectorXd x_error =
      xdes -
      (with_rotation ? eef_->getXYZPose()
                     : Eigen::VectorXd(eef_->getVectorPos())).head(xdes.size());

  std::vector<int> badjointinds;
  bool violate_limit = false;
  Eigen::VectorXd q_s = robot_->getCurrentPos()->getEigenVector(active_dofs_);
  Eigen::MatrixXd J =
      robot_->getJacobian(active_joints_, eef_, with_rotation, with_height);
  Eigen::VectorXd dq;

  // cout << "J : " << endl << J << endl;

  do {
    Eigen::VectorXd q_s_old = q_s;

    // eliminate bad joint columns from the Jacobian
    for (size_t j = 0; j < badjointinds.size(); j++)
      for (int k = 0; k < xdes.size(); k++) J(k, badjointinds[j]) = 0;

    /*
    // Damped Least-Squares (DLS)
    // Jplus = Jt * [(Jt * J ) + lambda * diag(I)]^{-1}
    Eigen::MatrixXd Reg(Eigen::MatrixXd::Zero(J.rows(), J.rows()));
    Reg.diagonal() = 0.0001 * Eigen::VectorXd::Ones(Reg.diagonal().size());
    Eigen::MatrixXd M = (J*J.transpose())+Reg;
    Eigen::MatrixXd Jplus = J.transpose()*M.inverse();
    */

    Eigen::MatrixXd Jplus = move3d_pinv(J, 1e-3);

    dq = Jplus * magnitude_ * x_error;

    // add step
    q_s = q_s_old + dq;

    // construct joint limits
    violate_limit = checkViolateJointLimits(q_s, badjointinds);

    // move back to previous point if any joint limits
    if (violate_limit) q_s = q_s_old;

  } while (violate_limit);

  return dq;
}

bool GeneralIK::checkViolateJointLimits(const Eigen::VectorXd& q,
                                        std::vector<int>& badjointinds,
                                        bool print) const {
  bool violate_limit = false;

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

        if (print) {
          cout << "does not respect joint limits : "
               << active_joints_[i]->getName();
          cout << " , lowerLimit : " << lowerLimit;
          cout << " , upperLimit : " << upperLimit;
          cout << " , q_j : " << q[dof_id];
          cout << " , id : " << active_joints_[i]->getIndexOfFirstDof() + j
               << endl;
        }
      }
    }
  }

  return violate_limit;
}

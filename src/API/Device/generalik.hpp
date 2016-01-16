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

#ifndef GENERALIK_HPP
#define GENERALIK_HPP

#include "robot.hpp"

namespace Move3D {

class GeneralIK {
 public:
  GeneralIK(Move3D::Robot* robot);

  //! Initialize with a set of joints and Joint for the end effector
  bool initialize(const std::vector<Move3D::Joint*>& joints,
                  Move3D::Joint* eef);

  //! Solve for a given task (6Dof)
  bool solve(const Eigen::VectorXd& xdes) const;

  //! single step, Jacobian transpose (6Dof)
  Eigen::VectorXd single_step(const Eigen::VectorXd& xdes) const;

  //! single step with joint limits (6Dof)
  Eigen::VectorXd single_step_joint_limits(const Eigen::VectorXd& xdes) const;

  //! Returns the active dofs
  const std::vector<int>& active_dofs() { return active_dofs_; }

  //! Check the joint limits and adds to the badjointinds structure
  bool checkViolateJointLimits(const Eigen::VectorXd& q,
                               std::vector<int>& badjointinds,
                               bool print = false) const;

  //! Magnitude applied to the gradient (set to 1. to get Jacobian Pseudo
  // inverse X task des)
  double magnitude_;

 protected:
  Move3D::Robot* robot_;
  Move3D::Joint* eef_;
  std::vector<Move3D::Joint*> active_joints_;
  std::vector<int> active_dofs_;

 private:
  bool limits_;
  bool check_joint_limits_;
  int nb_steps_;
  double max_distance_to_target_;
};
}

#endif  // GENERALIK_HPP

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
#include "HRICS_ioc.hpp"

#include "HRICS_parameters.hpp"
#include "HRICS_human_cost_space.hpp"
#include "HRICS_human_simulator.hpp"
#include "HRICS_dynamic_time_warping.hpp"

#include "API/project.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Graphic/drawCost.hpp"
#include "API/Graphic/drawModule.hpp"
#include "API/Device/generalik.hpp"

#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

#include "feature_space/spheres.hpp"
#include "feature_space/squares.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#ifdef OWLQN_LIB
#include <owlqn/OWLQN.h>
#include <owlqn/leastSquares.h>
#include <owlqn/logreg.h>
#endif

#include <iomanip>
#include <sstream>
#include <fstream>

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;
using std::cin;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

IocTrajectory::IocTrajectory(int nb_joints, int nb_var, double dt) {
  //    cout << "nb_joints : " << nb_joints << endl;
  //    cout << "nb_var : " << nb_var << endl;

  dt_ = dt;

  nominal_parameters_.clear();
  parameters_.clear();
  noise_.clear();
  control_costs_.clear();
  total_costs_.clear();

  for (int d = 0; d < nb_joints; ++d) {
    nominal_parameters_.push_back(Eigen::VectorXd::Zero(nb_var));
    parameters_.push_back(Eigen::VectorXd::Zero(nb_var));
    noise_.push_back(Eigen::VectorXd::Zero(nb_var));
    control_costs_.push_back(Eigen::VectorXd::Zero(nb_var));
    total_costs_.push_back(0.2 * Eigen::VectorXd::Ones(nb_var));
  }
  state_costs_ = Eigen::VectorXd::Zero(nb_var);
  out_of_bounds_ = false;
}

//! Returns the move3d configuration at the waypoint instant
Move3D::confPtr_t IocTrajectory::getMove3DConfig(
    int waypoint,
    Move3D::Robot* rob,
    const std::vector<Move3D::ChompDof>& dofs) const {
  confPtr_t q = rob->getCurrentPos();
  for (size_t i = 0; i < dofs.size(); ++i) {
    (*q)[dofs[i].move3d_dof_index_] = parameters_[i][waypoint];
  }
  return q;
}

Move3D::Trajectory IocTrajectory::getMove3DTrajectory(
    const ChompPlanningGroup* planning_group) const {
  Move3D::Robot* rob = planning_group->robot_;

  if (parameters_.empty()) {
    cout << "empty parameters" << endl;
    return Move3D::Trajectory(rob);
  }

  // Create move3d trajectory
  Move3D::Trajectory T(rob);
  if (dt_ != 0.0) {
    T.setUseTimeParameter(true);
    T.setUseConstantTime(true);
    T.setDeltaTime(dt_);
  }

  const std::vector<ChompDof>& dofs = planning_group->chomp_dofs_;

  for (int i = 0; i < parameters_[0].size(); i++) {
    Move3D::confPtr_t q = getMove3DConfig(i, rob, dofs);
    T.push_back(q->copy());
  }

  return T;
}

//! Interpolates linearly two configurations
//! u = 0 -> a
//! u = 1 -> b
Eigen::VectorXd IocTrajectory::interpolate(const Eigen::VectorXd& a,
                                           const Eigen::VectorXd& b,
                                           double u) const {
  Eigen::VectorXd out;
  if (a.size() != b.size()) {
    cout << "Error in interpolate" << endl;
    return out;
  }

  out = a;
  for (int i = 0; i < int(out.size()); i++) {
    out[i] += u * (b[i] - a[i]);
  }
  return out;
}

void IocTrajectory::setSraightLineTrajectory() {
  // Set size
  straight_line_ = parameters_;

  // Get number of points
  int nb_points = straight_line_[0].size();
  int dimension = straight_line_.size();

  if (dimension < 1) {
    cout << "Error : the trajectory only has one dimension" << endl;
    return;
  }

  if (nb_points <= 2) {
    cout << "Warning : the trajectory is "
         << "less or equal than 2 waypoints" << endl;
    return;
  }

  Eigen::VectorXd a(dimension);  // init value
  Eigen::VectorXd b(dimension);  // goal value
  Eigen::VectorXd c(dimension);

  for (int j = 0; j < dimension; j++) {
    a[j] = straight_line_[j][0];
    b[j] = straight_line_[j][nb_points - 1];
  }

  double delta = 1. / double(nb_points - 1);  // nb of localpaths
  double s = delta;                           // start at first localpaths

  // Only fill the inside points of the trajectory
  for (int i = 1; i < nb_points - 1; i++) {
    c = interpolate(a, b, s);
    s += delta;
    for (int j = 0; j < dimension; j++) {
      straight_line_[j][i] = c[j];
    }
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

IocSampler::IocSampler() : planning_group_(NULL) {}

IocSampler::IocSampler(int num_var_free,
                       int num_dofs,
                       const Move3D::ChompPlanningGroup* planning_group)
    : num_vars_free_(num_var_free),
      num_dofs_(num_dofs),
      planning_group_(planning_group) {}

// TODO set this in the constructor (it has to call the derivate class)
bool IocSampler::Initialize() {
  if (num_vars_free_ == 0 || planning_group_ == NULL) {
    return false;
  }

  initPolicy();

  policy_.getControlCosts(control_costs_);
  policy_.getCovariances(convariances_);

  cout << "num_joints_ : " << num_dofs_ << endl;
  cout << "num_vars_free_ : " << num_vars_free_ << endl;
  cout << "control_costs_.size() : " << control_costs_.size() << endl;
  cout << "convariances_.size() : " << convariances_.size() << endl;

  cout << "save to file :" << endl;
  move3d_save_matrix_to_csv_file(control_costs_[0].inverse(),
                                 "control_matrices/ioc_control_cost.txt");
  move3d_save_matrix_to_csv_file(convariances_[0],
                                 "control_matrices/ioc_covariance.txt");

  cout << "allocate gaussian sampler :" << endl;
  preAllocateMultivariateGaussianSampler();

  tmp_noise_ = Eigen::VectorXd::Zero(num_vars_free_);

  // Initialize joint limit projector
  dofs_bounds_.resize(planning_group_->num_dofs_);
  for (size_t i = 0; i < dofs_bounds_.size(); i++) {
    dofs_bounds_[i].circular_ = planning_group_->chomp_dofs_[i].is_circular_;
    dofs_bounds_[i].max_ = planning_group_->chomp_dofs_[i].joint_limit_max_;
    dofs_bounds_[i].min_ = planning_group_->chomp_dofs_[i].joint_limit_min_;
    dofs_bounds_[i].range_ = planning_group_->chomp_dofs_[i].range_;
  }
  InitializeJointLimitProjector();
  return true;
}

void IocSampler::initPolicy() {
  policy_.setPrintDebug(false);

  std::vector<double> derivative_costs(3);
  derivative_costs[0] = 0.0;  // velocity
  derivative_costs[1] = 0.0;  // acceleration
  derivative_costs[2] = 1.0;  // jerk

  // initializes the policy
  policy_.initialize(
      num_vars_free_, num_dofs_, 1.0, 0.0, derivative_costs, false);
}

void IocSampler::InitializeJointLimitProjector() {
  int num_joints = planning_group_->chomp_dofs_.size();
  int num_vars = num_vars_free();

  cout << "num_vars : " << num_vars << endl;
  cout << "num_joints : " << num_joints << endl;

  joint_limits_computers_.resize(num_joints);
  for (int joint = 0; joint < num_joints; joint++) {
    double max_limit =
        planning_group_->chomp_dofs_[joint].joint_limit_max_ - 1e-5;
    double min_limit =
        planning_group_->chomp_dofs_[joint].joint_limit_min_ + 1e-5;

    joint_limits_computers_[joint].dynamics_ = control_costs_[joint];
    joint_limits_computers_[joint].upper_ =
        max_limit * Eigen::VectorXd::Ones(num_vars);
    joint_limits_computers_[joint].lower_ =
        min_limit * Eigen::VectorXd::Ones(num_vars);

    if (!joint_limits_computers_[joint].initialize())

      cout << "ERROR could not initialize joint limits in "
           << __PRETTY_FUNCTION__ << endl;
  }
}

void IocSampler::ProjectToJointLimitsQuadProg(HRICS::IocTrajectory& traj) {
  int num_joints = planning_group_->chomp_dofs_.size();

  for (int joint = 0; joint < num_joints; joint++) {
    if (!planning_group_->chomp_dofs_[joint].has_joint_limits_) {
      continue;
    }

    double cost =
        joint_limits_computers_[joint].project(traj.parameters_[joint]);
    if (cost == std::numeric_limits<double>::infinity()) {
      cout << "joint limits did not converge" << endl;
    }
  }
}

bool IocSampler::preAllocateMultivariateGaussianSampler() {
  // invert the control costs, initialize noise generators:
  noise_generators_.clear();

  for (int j = 0; j < num_dofs_; ++j) {
    // move3d_save_matrix_to_file( inv_control_costs_[0],
    // "../matlab/invcost_matrix.txt" );

    // Uncomment to print the precision Matrix
    // cout << endl << control_costs_[j] << endl;

    // TODO see of the noise generator needs to be
    // var free or var all
    MultivariateGaussian mvg(Eigen::VectorXd::Zero(num_vars_free_),
                             convariances_[j]);
    noise_generators_.push_back(mvg);
  }

  return true;
}

// The rows of the noise matrix is the number of dofs
Eigen::MatrixXd IocSampler::sample(double std_dev) {
  Eigen::MatrixXd traj(num_dofs_, num_vars_free_);

  for (int d = 0; d < num_dofs_; d++) {
    noise_generators_[d].sample(tmp_noise_);
    traj.row(d) = std_dev * (dofs_bounds_[d].range_ / 3.) * tmp_noise_;
  }

  return traj;
}

IocIk IocSampler::sample_ik(double std_dev) {
  IocIk q(num_dofs_);
  for (int i = 0; i < q.noise_.size(); ++i) {
    q.noise_[i] = p3d_gaussian_random2(0.0, std_dev * std_dev);
  }
  return q;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

IocSamplerGoalSet::IocSamplerGoalSet(
    int num_var_free,
    int num_joints,
    const Move3D::ChompPlanningGroup* planning_group)
    : IocSampler(num_var_free, num_joints, planning_group) {}

void IocSamplerGoalSet::initPolicy() {
  policy_.setPrintDebug(false);

  std::vector<double> derivative_costs(3);
  derivative_costs[0] = 0.0;  // velocity
  derivative_costs[1] = 0.0;  // acceleration
  derivative_costs[2] = 1.0;  // jerk

  // initializes the policy
  policy_.initialize( // WAS 2.7 for TRO paper
      num_vars_free_, num_dofs_, 1.0, 0.0, derivative_costs, 1.0);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

IocIk::IocIk(int nb_joints) {
  nominal_parameters_ = Eigen::VectorXd::Zero(nb_joints);
  parameters_ = Eigen::VectorXd::Zero(nb_joints);
  noise_ = Eigen::VectorXd::Zero(nb_joints);
  out_of_bounds_ = false;
}

Move3D::confPtr_t IocIk::getMove3DConfig(
    const Move3D::ChompPlanningGroup* p_g) const {
  confPtr_t q = p_g->robot_->getCurrentPos();
  const std::vector<ChompDof>& joints = p_g->chomp_dofs_;

  for (int i = 0; i < parameters_.size(); ++i) {
    (*q)[joints[i].move3d_dof_index_] = parameters_[i];
  }

  return q;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// TODO not used any more
IocSamplerIk::IocSamplerIk(int num_joints) : IocSampler(0, num_joints, NULL) {
  //    cout << "IocIkSampler -> nb_joints_ : " << nb_joints_ << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

Ioc::Ioc(int num_vars,
         const ChompPlanningGroup* planning_group,
         const GoalsetData_t& goalset_data)
    : planning_group_(planning_group),
      goalset_data_(goalset_data),
      num_vars_(num_vars),
      num_joints_(planning_group->num_dofs_) {
  demonstrations_.clear();
  noise_stddev_ = HriEnv->getDouble(HricsParam::ioc_sample_std_dev);
  noise_stddev_ik_ = HriEnv->getDouble(HricsParam::ioc_sample_std_dev_ik);

  cout << "sampler goal set : " << goalset_data_.sample_goal_set_ << endl;
  if (goalset_data_.sample_goal_set_) {
    sampler_ = MOVE3D_PTR_NAMESPACE::shared_ptr<IocSamplerGoalSet>(
        new IocSamplerGoalSet(num_vars_, num_joints_, planning_group));
  } else {
    sampler_ = MOVE3D_PTR_NAMESPACE::shared_ptr<IocSampler>(
        new IocSampler(num_vars_, num_joints_, planning_group));
  }
  sampler_->Initialize();
  cout << "Ioc -> nb_joints_ : " << num_joints_ << endl;
}

bool Ioc::addDemonstration(const Eigen::MatrixXd& demo, double discretization) {
  if (num_joints_ != demo.rows() || num_vars_ != demo.cols()) {
    cout << "Error in add demonstration ";
    cout << "(num_joints : " << num_joints_ << " , " << demo.rows() << " ) ";
    cout << "(num_vars_ : " << num_vars_ << " , " << demo.cols() << " )";
    cout << endl;
    return false;
  }

  IocTrajectory t(num_joints_, num_vars_, discretization);

  for (int i = 0; i < int(t.parameters_.size()); i++) {
    t.parameters_[i] = demo.row(i);
  }

  demonstrations_.push_back(t);

  // Set straight line for sampling
  demonstrations_.back().setSraightLineTrajectory();

  // Resize sample vector to the number of demos
  samples_.resize(demonstrations_.size());

  return true;
}

bool Ioc::addSample(int d, const Eigen::MatrixXd& sample) {
  if (num_joints_ != sample.rows() || num_vars_ != sample.cols()) {
    cout << "Error in add sample : "
         << "trajectory dimension not appropriate" << endl;
    return false;
  }
  if (d >= int(samples_.size())) {
    cout << "Error in add sample : "
         << "nb of demonstrations too small" << endl;
    return false;
  }

  IocTrajectory* ptrT =
      new IocTrajectory(num_joints_, num_vars_, demonstrations_[d].dt_);

  for (int i = 0; i < int(ptrT->parameters_.size()); i++) {
    ptrT->parameters_[i] = sample.row(i);
  }

  samples_[d].push_back(ptrT);

  return true;
}

//! Set mean
bool Ioc::setNominalSampleValue(int d,
                                int i,
                                const Eigen::MatrixXd& nominal_parameters) {
  if (num_joints_ != nominal_parameters.rows() ||
      num_vars_ != nominal_parameters.cols()) {
    cout << "Error in add nominal sample values : "
         << "trajectory dimension not appropriate" << endl;
    return false;
  }
  if (d >= int(samples_.size())) {
    cout << "Error in add nominal sample values : "
         << "nb of demonstrations too small" << endl;
    return false;
  }
  for (int k = 0; k < int(samples_[d][i]->nominal_parameters_.size()); k++) {
    samples_[d][i]->nominal_parameters_[k] = nominal_parameters.row(k);
  }
  return true;
}

//! Set mean
bool Ioc::setTotalCostsSampleValue(int d,
                                   int i,
                                   const Eigen::VectorXd& total_costs) {
  if (num_vars_ != total_costs.rows()) {
    cout << "Error in add total_costs : "
         << "trajectory dimension not appropriate";

    cout << " : (" << num_vars_ << ") : (" << total_costs.cols() << ", "
         << total_costs.rows() << ")" << endl;

    return false;
  }
  if (d >= int(samples_.size())) {
    cout << "Error in add total_costs : "
         << "nb of demonstrations too small" << endl;
    return false;
  }
  for (int k = 0; k < int(samples_[d][i]->total_costs_.size()); k++) {
    samples_[d][i]->total_costs_[k] = total_costs;
  }
  return true;
}

//! returns true if in joint limits
bool Ioc::jointLimits(IocIk& q) const {
  bool is_in_joint_limits = true;

  for (int j = 0; j < num_joints_; j++) {
    double coeff = 1.0;

    double j_max = planning_group_->chomp_dofs_[j].joint_limit_max_;
    double j_min = planning_group_->chomp_dofs_[j].joint_limit_min_;

    int nb_attempt = 0;

    // TODO SEE PROBLEM WITH smoothness
    if (planning_group_->chomp_dofs_[j].is_circular_ && j_max == M_PI &&
        j_min == -M_PI) {
      q.parameters_[j] = angle_limit_PI(q.parameters_[j]);
    }

    while ((q.parameters_[j] < j_min || q.parameters_[j] > j_max) &&
           (nb_attempt < 10)) {
      // cout << "not in limits, name : "
      // << planning_group_->chomp_dofs_[j].joint_name_ << endl;
      // cout << j << " : upper : " << j_max << ", lower : "
      // << j_min << ", value : " << q.parameters_[j] << endl;
      coeff *= 0.90;  // 90 percent (10 * 0.9 = 0.3)
      q.noise_[j] *= coeff;
      q.parameters_[j] = q.nominal_parameters_[j] + q.noise_[j];
      nb_attempt++;
    }

    if (nb_attempt >= 10) {
      is_in_joint_limits = false;
    }

    nb_attempt = 0;

    if (!is_in_joint_limits)  // If not in joint limits make sure to cap
    {
      // cout << "capping joint : "
      // << planning_group_->chomp_dofs_[j].joint_name_ << endl;

      if (q.parameters_[j] < j_min || q.parameters_[j] > j_max) {
        // cout << "not in limits, name : "
        // << planning_group_->chomp_dofs_[j].joint_name_ << endl;
        // cout << j << " : upper : " << j_max << ", lower : "
        // << j_min << ", value : " << traj.parameters_[j][i] << endl;

        if (q.parameters_[j] < j_min) q.parameters_[j] = j_min + 1e-6;
        if (q.parameters_[j] > j_max) q.parameters_[j] = j_max - 1e-6;
      }
    }

    is_in_joint_limits = true;

    if (q.parameters_[j] < j_min || q.parameters_[j] > j_max)
      is_in_joint_limits = false;

    if (!is_in_joint_limits) {
      cout << "joint " << planning_group_->chomp_dofs_[j].joint_name_
           << " not in limit" << endl;
    }
  }

  //    cout << "is_in_joint_limits : " << is_in_joint_limits << endl;

  return is_in_joint_limits;
}

bool Ioc::checkJointLimits(const IocTrajectory& traj) const {
  bool succes_joint_limits = true;

  for (int joint = 0; joint < num_joints_; joint++) {
    if (!planning_group_->chomp_dofs_[joint].has_joint_limits_) continue;

    double joint_max = planning_group_->chomp_dofs_[joint].joint_limit_max_;
    double joint_min = planning_group_->chomp_dofs_[joint].joint_limit_min_;

    double max_abs_violation = 1e-6;
    double violate_max_with = 0;
    double violate_min_with = 0;
    double absolute_amount = 0.0;

    for (int i = 0; i < num_vars_; i++) {
      if (traj(joint, i) > joint_max) {
        // cout << "value is : " << traj(joint, i) << " , max is : " <<
        // joint_max << endl;
        absolute_amount = std::fabs(joint_max - traj(joint, i));

        if (absolute_amount > violate_max_with) {
          violate_max_with = absolute_amount;
        }
      } else if (traj(joint, i) < joint_min) {
        // cout << "value is : " << traj(joint, i) << " , min is : " <<
        // joint_min << endl;
        absolute_amount = std::fabs(joint_min - traj(joint, i));

        if (absolute_amount > violate_min_with) {
          violate_min_with = absolute_amount;
        }
      }

      if (absolute_amount > max_abs_violation) {
        //        cout << "joint is out of limits : "
        //             << planning_group_->chomp_dofs_[joint].joint_name_ <<
        //             endl;
        //        cout << "value is : " << traj(joint, i);
        //        cout << " , min is : " << joint_min;
        //        cout << " , max is : " << joint_max << endl;
        //        cout << "absolute_amount : " << absolute_amount;
        //        cout << " , max_abs_violation : " << max_abs_violation;
        //        cout << endl;

        max_abs_violation = absolute_amount;
        succes_joint_limits = false;

        break;
      }
    }

    if (!succes_joint_limits) return false;
  }

  return true;
}

bool Ioc::jointLimitsQuadProg(IocTrajectory& traj) const {
  sampler_->ProjectToJointLimitsQuadProg(traj);
  if (checkJointLimits(traj)) {
    return true;
  }

  return false;
}

//! returns true if in joint limits
bool Ioc::jointLimits(IocTrajectory& traj) const {
  bool is_in_joint_limits = true;

  //    cout << "num_vars_ : " << num_joints_ << endl;
  //    cout << "num_vars_ : " << num_vars_ << endl;

  for (int j = 0; j < num_joints_; j++) {
    double coeff = 1.0;

    double j_max = planning_group_->chomp_dofs_[j].joint_limit_max_;
    double j_min = planning_group_->chomp_dofs_[j].joint_limit_min_;

    int nb_attempt = 0;

    for (int i = 0; i < num_vars_; i++) {
      // TODO SEE PROBLEM WITH smoothness
      if (planning_group_->chomp_dofs_[j].is_circular_ && j_max == M_PI &&
          j_min == -M_PI) {
        traj.parameters_[j][i] = angle_limit_PI(traj.parameters_[j][i]);
      }

      while (
          (traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max) &&
          (nb_attempt < 10)) {
        // cout << "not in limits, name : "
        // << planning_group_->chomp_dofs_[j].joint_name_ << endl;
        // cout << j << " : upper : " << j_max
        // << ", lower : " << j_min << ", value : "
        // << traj.parameters_[j][i] << endl;

        coeff *= 0.90;  // 90 percent (10 * 0.9 = 0.3)
        traj.noise_[j] *= coeff;
        traj.parameters_[j] = traj.nominal_parameters_[j] + traj.noise_[j];
        nb_attempt++;
      }

      if (nb_attempt >= 10) {
        is_in_joint_limits = false;
      }

      nb_attempt = 0;
    }

    if (!is_in_joint_limits)  // If not in joint limits make sure to cap
    {
      // cout << "capping joint : "
      // << planning_group_->chomp_dofs_[j].joint_name_ << endl;

      for (int i = 0; i < num_vars_; i++) {
        if (traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max) {
          cout << "not in limits, name : "
               << planning_group_->chomp_dofs_[j].joint_name_ << endl;
          cout << j << " : upper : " << j_max << ", lower : " << j_min
               << ", value : " << traj.parameters_[j][i] << endl;

          if (traj.parameters_[j][i] < j_min)
            traj.parameters_[j][i] = j_min + 1e-6;
          if (traj.parameters_[j][i] > j_max)
            traj.parameters_[j][i] = j_max - 1e-6;
        }
      }
    }

    is_in_joint_limits = true;

    for (int i = 0; i < num_vars_; i++)
      if (traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max)
        is_in_joint_limits = false;

    if (!is_in_joint_limits) {
      cout << "joint " << planning_group_->chomp_dofs_[j].joint_name_
           << " not in limit" << endl;
    }
  }

  //    cout << __PRETTY_FUNCTION__ << endl;
  //    cout << "is_in_joint_limits : " << is_in_joint_limits << endl;

  return is_in_joint_limits;
}

bool Ioc::isTrajectoryValid(const IocTrajectory& traj,
                            bool relax_collision_check) {
  Move3D::Trajectory path = traj.getMove3DTrajectory(planning_group_);
  Move3D::Robot* robot = path.getRobot();
  Move3D::Scene* sce = global_Project->getActiveScene();

  std::vector<Move3D::Robot*> others;
  for (size_t i = 0; i < sce->getNumberOfRobots(); i++) {
    std::string robot_name = sce->getRobot(i)->getName();
    if (robot_name != robot->getName() &&
        (relax_collision_check ?
                               // When relaxed no collision check with human
             robot_name.find("HUMAN") == std::string::npos
                               : true)) {
      others.push_back(sce->getRobot(i));
      // cout << "Add robot " << robot_name << " to others" << endl;
    }
  }

  for (int i = 0; i < path.getNbOfViaPoints(); i++) {
    robot->setAndUpdate(*path[i]);
    if (robot->isInCollisionWithOthers(others)) {
      if (path[i]->isOutOfBounds(true)) {
        cout << "configuration invalid : out of bounds !!!" << endl;
      }
      // cout << "collision with other robot" << endl;
      return false;
    }
  }

  return true;
}

bool Ioc::isIkValid(const IocIk& ik) {
  Move3D::confPtr_t q = ik.getMove3DConfig(planning_group_);
  Move3D::Robot* robot = q->getRobot();
  Move3D::Scene* sce = global_Project->getActiveScene();

  std::vector<Move3D::Robot*> others;
  for (size_t i = 0; i < sce->getNumberOfRobots(); i++) {
    std::string robot_name = sce->getRobot(i)->getName();
    if (robot_name != robot->getName() &&
        robot_name.find("HUMAN") == std::string::npos) {
      others.push_back(sce->getRobot(i));
      //            cout << "Add robot " << robot_name << " to others" << endl;
    }
  }

  robot->setAndUpdate(*q);
  if (robot->isInCollisionWithOthers(others)) {
    return false;
  }

  return true;
}

IocIk Ioc::getLastConfigOfDemo(int d) const {
  IocIk q(num_joints_);
  for (size_t i = 0; i < demonstrations_[d].parameters_.size(); i++)
    q.parameters_[i] = demonstrations_[d].parameters_[i].tail(1)[0];
  return q;
}

void Ioc::deleteSampleForDemo(int d) {
  if (int(samples_.size()) < d) {
    for (size_t ns = 0; ns < samples_.size(); ns++) {
      delete samples_[d][ns];
      samples_[d][ns] = NULL;
    }
  }
}

void Ioc::resetAllSamples() {
  for (size_t d = 0; d < samples_.size(); d++) {
    for (size_t ns = 0; ns < samples_[d].size(); ns++) {
      samples_[d][ns] = NULL;
    }
  }
}

bool Ioc::projectConfiguration(IocIk& q, int d) {
  Move3D::Robot* robot = planning_group_->robot_;

  Move3D::confPtr_t q_demo =
      getLastConfigOfDemo(d).getMove3DConfig(planning_group_);
  Move3D::confPtr_t q_proj = q.getMove3DConfig(planning_group_);

  // Get active joints
  std::vector<Move3D::Joint*> active_joints =
      planning_group_->getActiveJoints();
  active_joints.erase(active_joints.begin());  // Remove Free Flyer

  // Get task pose
  robot->setAndUpdate(*q_demo);
  Eigen::VectorXd xdes = goalset_data_.eef_->getXYZPose();

  // Perform General IK
  robot->setAndUpdate(*q_proj);
  Move3D::GeneralIK ik(robot);
  ik.initialize(active_joints, goalset_data_.eef_);
  bool succeed = ik.solve(xdes);

  // Retreive configuration
  q.parameters_ =
      robot->getCurrentPos()->getEigenVector(planning_group_->getActiveDofs());
  return succeed;
}

bool Ioc::projectToGolset(IocTrajectory& traj) const {
  // cout << "x_task_goal_.size() : " << x_task_goal_.size() << endl;
  Move3D::confPtr_t q_end = traj.getMove3DConfig(traj.last_waypoint(),
                                                 planning_group_->robot_,
                                                 planning_group_->chomp_dofs_);

  planning_group_->robot_->setAndUpdate(*q_end);

  // Solve IK from q_end
  Move3D::GeneralIK ik(planning_group_->robot_);
  ik.initialize(planning_group_->getActiveJoints(), goalset_data_.eef_);

  //  cout << "ik.active_dofs().size() : " << ik.active_dofs().size() << endl;

  bool solved = ik.solve(goalset_data_.x_task_goal_);
  if (solved) {
    Move3D::confPtr_t q_cur = planning_group_->robot_->getCurrentPos();
    // The active dofs from the ik might differ from the dofs
    // of the planning group because IK does not handle P3D_FREEFLYERs
    std::vector<int> active_dofs = planning_group_->getActiveDofs();
    Eigen::VectorXd q_1 = q_end->getEigenVector(active_dofs);
    Eigen::VectorXd q_2 = q_cur->getEigenVector(active_dofs);
    Eigen::VectorXd dq = q_2 - q_1;

    // cout << "dq : " << dq.transpose() << endl;

    int start = double(1. - goalset_data_.ratio_projected_) * num_vars_;
    double alpha = 0.0;
    double delta = 1.0 / double(num_vars_ - start);

    for (int i = start; i < num_vars_; i++) {
      for (int dof = 0; dof < num_joints_; dof++) {
        traj(dof, i) += std::pow(alpha, goalset_data_.strength_) * dq[dof];
      }

      alpha += delta;
    }
  }

  return solved;
}

int Ioc::generateIKSamples(int nb_samples,
                           bool check_in_collision,
                           context_t context) {
  int nb_demos = getNbOfDemonstrations();

  samples_ik_.resize(nb_demos);

  int nb_of_invalid_samples = 0;

  for (int d = 0; d < nb_demos; ++d) {
    cout << "Generating samples for demonstration : " << d << endl;

    samples_ik_[d].resize(nb_samples);

    for (int i = 0; i < int(context.size()); i++) {
      Move3D::Robot* entity = context[i][d]->getRobot();
      entity->setAndUpdate(*context[i][d]);

      // g3d_draw_allwin_active();
      // cout << "wait for key" << endl;
      // std::cin.ignore();
    }

    for (int ns = 0; ns < int(samples_ik_[d].size()); ++ns) {
      // cout.flush();
      // cout << "generating for demo " << d << " sample : " << int(ns+1) <<
      // "\r";

      samples_ik_[d][ns] = IocIk(num_joints_);

      bool is_valid = true;
      int nb_failed = 0;
      do {
        // Sample noisy ik
        samples_ik_[d][ns] = sampler_->sample_ik(noise_stddev_ik_);

        // Change to generate samples around demonstration
        if (HriEnv->getBool(HricsParam::ioc_sample_around_demo))
          samples_ik_[d][ns].nominal_parameters_ =
              getLastConfigOfDemo(d).parameters_;

        samples_ik_[d][ns].parameters_ =
            samples_ik_[d][ns].nominal_parameters_ + samples_ik_[d][ns].noise_;

        is_valid = jointLimits(samples_ik_[d][ns]);
        is_valid = projectConfiguration(samples_ik_[d][ns], d);

        if (check_in_collision) {
          is_valid = isIkValid(samples_ik_[d][ns]);
          // is_valid = samples_[d][ns].getMove3DTrajectory(
          // planning_group_ ).isValid();
        }
      }
      // Commented for humans
      while ((!is_valid) && (nb_failed++ < 10));

      // The sample trajectory is not valid
      if (!is_valid) nb_of_invalid_samples++;
      // cout << "WARNING: generating invalid sample, in collision : ";
      // cout << check_in_collision << endl;
    }
  }

  cout << endl;

  return nb_of_invalid_samples;  // TODO return false when out of bounds
}

int Ioc::generateDemoSamples(int demo_id,
                             int nb_samples,
                             bool check_in_collision,
                             context_t context) {
  cout << "Generating samples for demonstration : " << demo_id << endl;

  int nb_of_invalid_samples = 0;
  samples_[demo_id].resize(nb_samples);

  for (int i = 0; i < int(context.size()); i++) {
    Move3D::Robot* entity = context[i][demo_id]->getRobot();
    entity->setAndUpdate(*context[i][demo_id]);

    // g3d_draw_allwin_active();
    // cout << "wait for key" << endl;
    // std::cin.ignore();
  }

  bool relax_collision_check = false;

  if (!isTrajectoryValid(demonstrations_[demo_id], relax_collision_check)) {
    cout << "DEMO is NOT VALID" << endl;
    g3d_draw_allwin_active();
  }

  for (int ns = 0; ns < int(samples_[demo_id].size()); ++ns) {
    // cout.flush();
    // cout << "generating for demo "
    // << d << " sample : " << int(ns+1) << "\r";

    samples_[demo_id][ns] =
        new IocTrajectory(num_joints_, num_vars_, demonstrations_[demo_id].dt_);

    bool is_valid = true;
    int nb_failed = 0;
    relax_collision_check = false;

    do {
      // Sample noisy trajectory
      Eigen::MatrixXd noisy_traj = sampler_->sample(noise_stddev_);

      for (int j = 0; j < num_joints_; ++j) {
        // Change to generate samples around demonstration
        if (HriEnv->getBool(HricsParam::ioc_sample_around_demo))
          samples_[demo_id][ns]->nominal_parameters_[j] =
              demonstrations_[demo_id].parameters_[j];
        else
          samples_[demo_id][ns]->nominal_parameters_[j] =
              demonstrations_[demo_id].straight_line_[j];
        // TODO why commented

        // cout << std::scientific << "noisy traj : " << noisy_traj.row(j) <<
        // endl;

        samples_[demo_id][ns]->noise_[j] = noisy_traj.row(j);

        samples_[demo_id][ns]->parameters_[j] =
            samples_[demo_id][ns]->nominal_parameters_[j] +
            samples_[demo_id][ns]->noise_[j];
        //.cwiseProduct(samples_[d][ns].total_costs_[j]);
      }

      // Check the joint limits and modify the noise to fit
      // TODO change by a SQP joint limit projection
      // is_valid = jointLimits(*samples_[demo_id][ns]);
      is_valid = jointLimitsQuadProg(*samples_[demo_id][ns]);

      // Project the configuration to the goal set region defined by the
      // task space position of the end-effector
      if (goalset_data_.sample_goal_set_ && is_valid) {
        Move3D::confPtr_t q_end = demonstrations_[demo_id].getMove3DConfig(
            demonstrations_[demo_id].last_waypoint(),
            planning_group_->robot_,
            planning_group_->chomp_dofs_);
        planning_group_->robot_->setAndUpdate(*q_end);
        goalset_data_.x_task_goal_ = goalset_data_.eef_->getVectorPos().head(3);
        // cout << goalset_data_ << endl;
        // is_valid = projectToGolset(*samples_[demo_id][ns]);
      }

      // Set the start and end values constant
      for (int j = 0; j < num_joints_; ++j) {
        // WARNING USED TO BE START AND END
        samples_[demo_id][ns]->parameters_[j].head(1) =
            samples_[demo_id][ns]->nominal_parameters_[j].head(1);
        // traj.parameters_[j].tail(1) = traj.nominal_parameters_[j].tail(1);
      }

      if (check_in_collision) {
        is_valid =
            isTrajectoryValid(*samples_[demo_id][ns], relax_collision_check);
        // s_valid = samples_[d][ns].getMove3DTrajectory(
        // planning_group_ ).isValid();
      }
      if (nb_failed > 10) {
        relax_collision_check = true;
      }
    }
    // Commented for humans
    while ((!is_valid) && (nb_failed++ < 20));

    // The sample trajectory is not valid
    if (!is_valid) nb_of_invalid_samples++;
    // cout << "WARNING: generating invalid sample, in collision : ";
    // cout << check_in_collision << endl;
  }

  return nb_of_invalid_samples;
}

int Ioc::generateSamples(int nb_samples,
                         bool check_in_collision,
                         context_t context) {
  int nb_of_invalid_samples = 0;
  int nb_demos = getNbOfDemonstrations();
  samples_.resize(nb_demos);

  for (int d = 0; d < nb_demos; ++d) {
    nb_of_invalid_samples +=
        generateDemoSamples(d, nb_samples, check_in_collision, context);
  }

  return nb_of_invalid_samples;  // TODO return false when out of bounds
}

std::vector<std::vector<Move3D::confPtr_t> > Ioc::getSamplesIk() const {
  std::vector<std::vector<Move3D::confPtr_t> > samples(demonstrations_.size());

  for (size_t d = 0; d < demonstrations_.size(); ++d)
    for (size_t k = 0; k < samples_ik_[d].size(); ++k)
      samples[d].push_back(samples_ik_[d][k].getMove3DConfig(planning_group_));

  return samples;
}

std::vector<std::vector<Move3D::Trajectory> > Ioc::getSamples() const {
  std::vector<std::vector<Move3D::Trajectory> > samples(demonstrations_.size());

  for (size_t d = 0; d < demonstrations_.size(); ++d)
    for (size_t k = 0; k < samples_[d].size(); ++k)
      samples[d].push_back(
          samples_[d][k]->getMove3DTrajectory(planning_group_));

  return samples;
}

std::vector<Move3D::Trajectory> Ioc::getDemoSamples(int d) const {
  if (d > int(samples_.size())) return std::vector<Move3D::Trajectory>();

  std::vector<Move3D::Trajectory> samples;

  for (size_t k = 0; k < samples_[d].size(); ++k)
    samples.push_back(samples_[d][k]->getMove3DTrajectory(planning_group_));

  return samples;
}

std::vector<Move3D::Trajectory> Ioc::getDemonstrations() const {
  std::vector<Move3D::Trajectory> demos(demonstrations_.size());

  for (size_t d = 0; d < demonstrations_.size(); ++d)
    demos[d] = demonstrations_[d].getMove3DTrajectory(planning_group_);

  return demos;
}

Move3D::confPtr_t Ioc::addTrajectoryToDraw(const IocTrajectory& t, int color) {
  Move3D::Trajectory T = t.getMove3DTrajectory(planning_group_);
  T.setColor(color);
  // global_trajToDraw.push_back( T );
  // Eigen::Vector3d color_traj;

  double color_vect[4];
  g3d_get_color_vect(color, color_vect);

  Eigen::Vector3d color_traj;
  color_traj[0] = color_vect[0];
  color_traj[1] = color_vect[1];
  color_traj[2] = color_vect[2];

  if (planning_group_->robot_->getNumberOfJoints() > 44) {
    Move3D::Joint* draw_joint = planning_group_->robot_->getJoint(45);

    std::pair<Eigen::Vector3d, Eigen::MatrixXd> draw_line =
        std::make_pair(color_traj, T.getJointPoseTrajectory(draw_joint));

    global_linesToDraw.push_back(draw_line);
  } else
    global_trajToDraw.push_back(T);

  return T.getEnd();
}

Move3D::confPtr_t Ioc::addAllToDraw(
    const std::vector<int>& demos_ids,
    const std::vector<std::string>& demo_names) {
  global_trajToDraw.clear();
  global_linesToDraw.clear();

  Move3D::confPtr_t q = planning_group_->robot_->getCurrentPos();

  if (global_DrawModule) {
    global_DrawModule->addDrawFunction("Draw3DTrajs",
                                       boost::bind(&g3d_draw_3d_lines));
    global_DrawModule->enableDrawFunction("Draw3DTrajs");
  }

  std::string split = HriEnv->getString(HricsParam::ioc_traj_split_name);

  cout << "ioc_sample_one_demo : "
       << HriEnv->getBool(HricsParam::ioc_draw_one_demo) << endl;

  if (HriEnv->getBool(HricsParam::ioc_draw_demonstrations)) {
    for (size_t d = 0; d < demonstrations_.size(); ++d) {
      if ((!HriEnv->getBool(HricsParam::ioc_draw_one_demo)) ||
          demo_names.empty() ||
          demo_names[demos_ids[d]].substr(0, 11) == split) {
        cout << "adding to draw demo [" << d << "] : " << demo_names[d] << endl;
        q = addTrajectoryToDraw(demonstrations_[d], 0);
      }
    }
  }

  if (HriEnv->getBool(HricsParam::ioc_draw_samples)) {
    for (size_t d = 0; d < demonstrations_.size(); ++d) {
      if ((!HriEnv->getBool(HricsParam::ioc_draw_one_demo)) ||
          demo_names.empty() ||
          demo_names[demos_ids[d]].substr(0, 11) == split) {
        for (size_t k = 0; k < samples_[d].size(); ++k) {
          if (samples_[d][k] != NULL) {
            cout << "adding to draw sample [" << d << "][" << k << "]" << endl;
           //  Move3D::confPtr_t q =
                addTrajectoryToDraw(*samples_[d][k], k % 7 + 1);
            // global_configToDraw.push_back(q);
          }
        }
      }
    }
  }

  return q;
}

//////////////////////////////////////////////////////
// IOC objective
//////////////////////////////////////////////////////
#ifdef OWLQN_LIB
struct IocObjective : public DifferentiableFunction {
  IocObjective() {}

  //! Main virtual function
  double Eval(const DblVec& w, DblVec& dw);

  Eigen::VectorXd getEigenVector(const DblVec& w);
  Eigen::VectorXd numericalGradient(double loss, const Eigen::VectorXd& w);
  double value(const Eigen::VectorXd& w);

  //! demonstrations features
  std::vector<Eigen::VectorXd> phi_demo_;

  //! sample features
  std::vector<std::vector<Eigen::VectorXd> > phi_k_;
};

Eigen::VectorXd IocObjective::getEigenVector(const DblVec& w) {
  Eigen::VectorXd w_(w.size());

  for (size_t i = 0; i < w.size(); i++) {
    w_[i] = w[i];
  }
  return w_;
}

double IocObjective::value(const Eigen::VectorXd& w) {
  double loss = 0.0;

  for (size_t d = 0; d < phi_demo_.size(); d++) {
    double denominator = 1e-9;
    for (size_t k = 0; k < phi_k_.size(); k++)
      denominator += std::exp(-1.0 * w.transpose() * phi_k_[d][k]);

    loss +=
        std::log(std::exp(-1.0 * w.transpose() * phi_demo_[d]) / denominator);
  }

  loss = -loss;

  return loss;
}

Eigen::VectorXd IocObjective::numericalGradient(double loss,
                                                const Eigen::VectorXd& w) {
  double eps = 1e-6;

  Eigen::VectorXd g(w.size());

  for (int i = 0; i < g.size(); i++) {
    Eigen::VectorXd w_tmp(w);
    w_tmp[i] = w[i] + eps;
    g[i] = (value(w_tmp) - loss) / eps;
  }

  return g;
}

double IocObjective::Eval(const DblVec& w, DblVec& dw) {
  double loss = 1.0;

  if (w.size() != dw.size()) {
    cout << "error in size : " << __PRETTY_FUNCTION__ << endl;
    return loss;
  }

  Eigen::VectorXd w_(getEigenVector(w));
  Eigen::VectorXd dw_(Eigen::VectorXd::Zero(dw.size()));

  // cout << endl;
  // cout << "w : " << w_.transpose() << endl;

  // Loss function
  loss = value(w_);

  // Gradient
  Eigen::VectorXd dw_n = numericalGradient(loss, w_);

  for (int d = 0; d < int(phi_demo_.size()); d++) {
    double denominator = 0.0;
    for (size_t k = 0; k < phi_k_.size(); k++)
      denominator += std::exp(-1.0 * w_.transpose() * phi_k_[d][k]);

    for (int i = 0; i < int(w.size()); i++) {
      double numerator = 0.0;
      for (size_t k = 0; k < phi_k_.size(); k++)
        numerator +=
            std::exp(-1.0 * w_.transpose() * phi_k_[d][k]) * phi_k_[d][k][i];

      dw_[i] += phi_demo_[d][i] - (numerator / denominator);
    }
  }

  for (int i = 0; i < dw_.size(); i++) dw[i] = dw_[i];

  if (std::isnan(loss)) {
    cout << endl;
    cout << "w : " << w_.transpose() << endl;
    cout << "loss : " << loss << endl;
    cout << "----------------" << endl;
    cout << "dw_s : " << dw_.transpose() << endl;
    cout << "dw_n : " << dw_n.transpose() << endl;
    cout << "error : " << (dw_n - dw_).norm() << endl;
  }

  return loss;
}
#endif

Eigen::VectorXd Ioc::solve(
    const std::vector<Eigen::VectorXd>& phi_demo,
    const std::vector<std::vector<Eigen::VectorXd> >& phi_k) {
  if (phi_demo.size() < 1) {
    cout << "no demo passed in Ioc solver" << endl;
    return Eigen::VectorXd();
  }

#ifdef OWLQN_LIB
  size_t size = phi_demo[0].size();

  IocObjective obj;
  obj.phi_demo_ = phi_demo;
  obj.phi_k_ = phi_k;

  Eigen::VectorXd w0 = Eigen::VectorXd::Zero(size);
  Eigen::VectorXd w1 = Eigen::VectorXd::Zero(size);
  for (int i = 0; i < int(size); i++) w1[i] = 1;

  cout << "zeros value : " << obj.value(w0) << endl;
  cout << "ones value : " << obj.value(w1) << endl;

  DblVec ans(size);
  /*
  // Initial value ones
  DblVec init(size,1);

  for( int i=0;i<int(init.size());i++)
  {
//        init[i] = p3d_random(1,2);
      init[i] = 1;
  }

  bool quiet=false;
  int m = 50;
  double regweight=1;
  double tol = 1e-6;

  OWLQN opt(quiet);
  opt.Minimize( obj, init, ans, regweight, tol, m );
  */

  return obj.getEigenVector(ans);
#else

  return Eigen::VectorXd();
#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

IocEvaluation::IocEvaluation(Robot* rob,
                             int nb_demos,
                             int nb_samples,
                             int nb_way_points,
                             MultiplePlanners& planners,
                             StackedFeatures* features,
                             std::vector<int> active_joints,
                             std::string folder,
                             std::string traj_folder,
                             std::string tmp_data_folder)
    : robot_(rob), planners_(planners) {
  // Dimention of the problem space
  nb_demos_ = nb_demos;
  nb_samples_ = nb_samples;
  nb_way_points_ = nb_way_points;
  nb_planning_test_ = 1;

  // Folders
  folder_ = folder;  // move3d_demo_folder; // static
  traj_folder_ = traj_folder;
  tmp_data_folder_ = tmp_data_folder;
  process_dir_ = "";

  load_sample_from_file_ =
      HriEnv->getBool(HricsParam::ioc_load_samples_from_file);
  remove_samples_in_collision_ = true;

  // Active joints
  active_joints_ = active_joints;
  for (size_t i = 0; i < active_joints.size(); i++) {
    cout << "active joint[" << i << "] : " << active_joints[i] << endl;
  }

  planning_group_ = new ChompPlanningGroup(robot_, active_joints_);

  feature_fct_ = features;

  nb_weights_ = feature_fct_->getNumberOfFeatures();
  original_vect_ = feature_fct_->getWeights();

  feature_type_ = "";

  // Active dofs are set when the planning group is created
  smoothness_fct_ = new TrajectorySmoothness;
  smoothness_fct_->setActiveDoFs(planning_group_->getActiveDofs());

  // sample goal set
  goalset_data_.sample_goal_set_ = false;

  // Set the feature type
  if (dynamic_cast<Spheres*>(feature_fct_->getFeatureFunction(0)) != NULL)
    feature_type_ = "spheres";

  if (dynamic_cast<Squares*>(feature_fct_->getFeatureFunction(0)) != NULL)
    feature_type_ = "squares";

  if (dynamic_cast<HumanTrajFeatures*>(feature_fct_) != NULL)
    feature_type_ = "human_trajs";

  // Set active dofs
  if (feature_type_ == "spheres" || feature_type_ == "squares ") {
    global_PlanarCostFct->setActiveDoFs(planning_group_->getActiveDofs());
  }

  use_context_ = false;
  use_simulator_ = false;
}

void IocEvaluation::initializeGoalSetData() {
  goalset_data_.sample_goal_set_ = true;
  goalset_data_.ratio_projected_ = 1.0;  // 1.0 = all along the trajectory
  goalset_data_.strength_ =
      PlanEnv->getDouble(PlanParam::trajStompConstStrength);
  goalset_data_.eef_ = planning_group_->robot_->getJoint(
      PlanEnv->getString(PlanParam::end_effector_joint));  // plannar J4

  if (goalset_data_.eef_ != NULL) {
    goalset_data_.x_task_goal_ =
        goalset_data_.eef_->getVectorPos().head(3);  // head(2) only (x,y)
  } else {
    goalset_data_.sample_goal_set_ = false;
    goalset_data_.x_task_goal_ = Eigen::VectorXd::Zero(0);
  }
}

void IocEvaluation::delete_all_samples() {
  //    for(size_t d=0; d<samples_.size(); d++ )
  //    {
  //        for (size_t ns=0; ns<int(samples_[d].size()); ++ns )
  //        {
  //            delete samples_[d][ns];
  //            samples_[d][ns] = NULL;
  //        }
  //    }
}

// Types are : astar, rrt, stomp
Move3D::Trajectory IocEvaluation::planMotion(planner_t type) {
  if (use_simulator_) {
    cout << "RUN SIMULATOR FOR DEMO : " << demo_id_ << endl;
    global_human_traj_simulator->setDemonstrationId(demo_id_);
    global_human_traj_simulator->run();
    Move3D::Trajectory traj = global_human_traj_simulator->getExecutedPath();
    return traj;
  } else {
    planners_.clearTrajs();
    planners_.setPlannerType(type);

    if (planners_.run()) {
      return planners_.getBestTrajs()[0];
    } else {
      Move3D::Trajectory traj(robot_);
      return traj;
    }
  }
}

void IocEvaluation::runPlannerMultipleFeature(int nb_runs) {
  planners_.clearTrajs();
  planners_.setPlannerType(
      planner_t(HriEnv->getInt(HricsParam::ioc_planner_type)));

  for (int i = 0; i < nb_runs; i++) {
    std::vector<int> active_feature;
    for (int j = 0; j < feature_fct_->getNumberOfFeatures(); j++) {
      cout << "--------------------------------------------" << endl;
      cout << "RUN : " << i << " , FEATURE : " << j << endl;
      active_feature.clear();
      active_feature.push_back(j);
      feature_fct_->setActiveFeatures(active_feature);
      // planners_.multipleRun(1);
      planners_.run();
      g3d_draw_allwin_active();
    }
  }

  planners_.saveTrajsToFile(traj_folder_);
}

void IocEvaluation::activateAllFeatures() {
  std::vector<int> active_feature;
  for (int i = 0; i < feature_fct_->getNumberOfFeatures(); i++)
    active_feature.push_back(i);

  feature_fct_->setActiveFeatures(active_feature);
}

WeightVect IocEvaluation::computeOptimalWeights() {
  WeightVect w(WeightVect::Ones(feature_fct_->getNumberOfFeatures()));

  const Move3D::Trajectory& traj = planners_.getBestTrajs().back();

  // Set all feature active
  activateAllFeatures();

  // Feature count
  FeatureVect phi = feature_fct_->getFeatureCount(traj);
  // phi = phi.array() + EPS6;

  // Jacobian Norm Per DoF
  FeatureJacobian J = feature_fct_->getFeatureJacobian(traj);
  FeatureVect phi_gradient_norm(
      FeatureVect::Zero(feature_fct_->getNumberOfFeatures()));
  for (int i = 0; i < w.size(); i++)  // For each feature
    phi_gradient_norm(i) += J.col(i).norm();

  // COMPUTATION OF THE NEXT
  stored_features_[0] += phi;
  stored_features_[1] += phi_gradient_norm;

  static_cast<Squares*>(global_PlanarCostFct)->phi_cumul_ = stored_features_[0];
  static_cast<Squares*>(global_PlanarCostFct)->phi_jac_cumul_ =
      stored_features_[1];

  //    phi = stored_features_[0].normalized();
  //    phi_gradient_norm = stored_features_[1].normalized();

  w = stored_features_[0];
  w /= w.maxCoeff();

  //    for( int i=0;i<w.size();i++) // For each feature
  //    {
  //        w( = phi_cumul_ ;
  //    }

  //    for( int i=0;i<w.size();i++) // For each feature
  //    {
  //        w(i) = std::abs( phi(i) - phi_demos_[0](i) );
  //    }

  //    w.array() + EPS3;
  //    w.normalize();

  return w;
}

Move3D::Trajectory IocEvaluation::selectBestSample(
    double detla_mean, const std::vector<Move3D::Trajectory>& trajs) {
  PlanarFeature* planar_fct =
      dynamic_cast<PlanarFeature*>(feature_fct_->getFeatureFunction(0));

  double delta_min = std::numeric_limits<double>::max();
  int k = 0;
  for (size_t i = 0; i < trajs.size(); i++) {
    double delta = planar_fct->getDeltaSum(planners_.getAllTrajs()[k]);
    if (delta < delta_min) {
      delta_min = delta;
      k = i;
    }
  }
  return planners_.getAllTrajs()[k];
}

void IocEvaluation::runPlannerWeightedFeature(int nb_runs) {
  cout << __PRETTY_FUNCTION__ << endl;

  planners_.clearTrajs();
  planners_.setPlannerType(
      planner_t(HriEnv->getInt(HricsParam::ioc_planner_type)));

  activateAllFeatures();

  HRICS::Ioc ioc(nb_way_points_, planning_group_, goalset_data_);

  // Get weight vector
  WeightVect w(WeightVect::Ones(feature_fct_->getNumberOfFeatures()));
  feature_fct_->setWeights(w);

  // Get demos features
  phi_demos_ = addDemonstrations(ioc);

  // Jac sum of demos
  std::vector<std::vector<Move3D::Trajectory> > trajs_tmp;
  trajs_tmp.push_back(demos_);

  std::vector<std::vector<FeatureVect> > jac_sum_demos =
      getFeatureJacobianSum(trajs_tmp);
  phi_jac_demos_ = jac_sum_demos.back();

  // Store features
  stored_features_.resize(
      2, FeatureVect::Zero(feature_fct_->getNumberOfFeatures()));

  PlanarFeature* planar_fct =
      dynamic_cast<PlanarFeature*>(feature_fct_->getFeatureFunction(0));
  if (planar_fct != NULL) {
    //        planar_fct->balance_cumul_jac_ = 1.0; // un comment to only track
    //        the feature count
    planar_fct->phi_demos_ = phi_demos_[0];  // = stored_features_[0];
    planar_fct->phi_cumul_ =
        FeatureVect::Ones(feature_fct_->getNumberOfFeatures());
    planar_fct->phi_jac_cumul_ =
        FeatureVect::Zero(feature_fct_->getNumberOfFeatures());
    planar_fct->demos_ = demos_;
  }

  cout << "phi_demos_ : " << planar_fct->phi_demos_.transpose() << endl;

  // Nb of run per_feature
  const int per_feature_run = 10;

  planar_fct->iter_ = 0;
  planar_fct->increment_delta_ = 2.0;

  double sum_delta = 0.0;
  double mean_delta = 0.0;
  double sum_variance = 0.0;
  double mean_variance = 0.0;

  std::vector<Move3D::Trajectory> best_trajs;

  planners_.setStompInit(demos_[0]);

  // Nb of total run
  for (int i = 0; i < nb_runs; i++) {
    for (int j = 0; j < feature_fct_->getNumberOfFeatures(); j++) {
      cout << "--------------------------------------------" << endl;
      cout << "RUN : " << i << " , FEATURE : " << j << endl;
      cout << "BLANCE : " << planar_fct->balance_cumul_jac_ << endl;

      //            cout << "distance to feature count : " << (
      //            planar_fct->phi_demos_ - ( planar_fct->phi_cumul_/
      //            planar_fct->phi_cumul_.maxCoeff() ) ).norm() << endl;
      //            cout << "distance to feature count : " << (
      //            planar_fct->phi_demos_ - ( planar_fct->phi_cumul_/
      //            planar_fct->phi_cumul_.maxCoeff() ) ).transpose() << endl;

      //            std::vector<int> active_features;
      //            active_features.clear();
      //            active_features.push_back( j );
      //            feature_fct_->setActiveFeatures( active_features );

      //            planar_fct->produceDerivativeFeatureCostMap(
      //            i*planar_fct->getNumberOfFeatures() + j );

      planners_.run();

      //            best_trajs.push_back( selectBestSample( mean_delta,
      //            planners_.getAllTrajs() ) );
      best_trajs.push_back(planners_.getBestTrajs().back());
      best_trajs.back().replaceP3dTraj();

      double variance = planar_fct->getVariance(best_trajs.back());
      double delta = planar_fct->getDeltaSum(best_trajs.back());

      planar_fct->iter_++;

      sum_variance += variance;
      mean_variance = sum_variance / double(planar_fct->iter_);

      sum_delta += delta;
      mean_delta = sum_delta / double(planar_fct->iter_);

      if (mean_delta < 5)
        planar_fct->increment_delta_ /= 1.10;
      else {
        if (delta > 0) planar_fct->increment_delta_ *= 1.5;
      }

      cout << "--------------------------------------------" << endl;
      cout << " VARIANCE : " << variance << endl;
      cout << " VARIANCE MEAN : " << mean_variance << endl;
      cout << " DELTA : " << delta << endl;
      cout << " DELTA MEAN : " << mean_delta << endl;
      cout << " DELTA INCREMENT : " << planar_fct->increment_delta_ << endl;
      cout << " ITER : " << planar_fct->iter_ << endl;

      //            global_trajToDraw.clear();

      //            for( size_t k=0;k<planners_.getAllTrajs().size();k++)
      //            {
      //                cout << "var : " << planar_fct->getVariance(
      //                planners_.getAllTrajs()[k] );
      //                cout << " , delta : "  << planar_fct->getDeltaSum(
      //                planners_.getAllTrajs()[k] ) << endl;
      //                global_trajToDraw.push_back( planners_.getAllTrajs()[k]
      //                );
      //            }

      g3d_draw_allwin_active();  // TODO??? Keep this before optimal weight
                                 // computation

      w = computeOptimalWeights();
      //            feature_fct_->setWeights( w );
      //            feature_fct_->printWeights();

      if (planar_fct != NULL) {
        planar_fct->cur_min_diff_ = planar_fct->min_diff_;
        planar_fct->min_diff_ = std::numeric_limits<double>::max();
      }

      if (PlanEnv->getBool(PlanParam::stopPlanner)) break;
    }

    if (PlanEnv->getBool(PlanParam::stopPlanner)) break;

    if (planar_fct != NULL)
      planar_fct->balance_cumul_jac_ += (1.0 / double(per_feature_run));

    // WARNING RESET EVERY 10
    if ((i + 1) % (per_feature_run) == 0) {
      w = Eigen::VectorXd::Ones(feature_fct_->getNumberOfFeatures());
      stored_features_[0] =
          Eigen::VectorXd::Zero(feature_fct_->getNumberOfFeatures());

      sum_delta = 0.0;
      mean_delta = 0.0;
      sum_variance = 0.0;
      mean_variance = 0.0;

      if (planar_fct != NULL) {
        planar_fct->balance_cumul_jac_ = 0.0;
        planar_fct->phi_cumul_ =
            FeatureVect::Zero(feature_fct_->getNumberOfFeatures());
        planar_fct->phi_jac_cumul_ =
            FeatureVect::Zero(feature_fct_->getNumberOfFeatures());
        planar_fct->iter_ = 0;
        planar_fct->increment_delta_ = 2.0;
      }
    }
  }

  saveTrajectories(best_trajs);

  //    planners_.saveTrajsToFile( move3d_traj_folder );
}

void IocEvaluation::generateDemonstrations(int nb_demos) {
  std::vector<Move3D::Trajectory> demos;

  nb_demos_ = nb_demos;

  cout << "nb_demos : " << nb_demos_ << endl;
  cout << "nb_planning_test_ : " << nb_planning_test_ << endl;

  feature_fct_->printInfo();

  std::vector<int> demos_id;

  for (int i = 0; i < nb_demos_; i++) {
    std::vector<std::pair<double, Move3D::Trajectory> > demos_tmp(
        nb_planning_test_);

    cout << "___________________________________________________" << endl;
    cout << " demo nb : " << i << endl;

    if (i >= 1)  // Generate multiple start and goals
    {
      Move3D::confPtr_t q_init = robot_->shoot();
      Move3D::confPtr_t q_goal = robot_->shoot();
      robot_->setInitPos(*q_init);
      robot_->setGoalPos(*q_goal);
    }

    for (int j = 0; j < nb_planning_test_; j++) {
      demos_tmp[j].second = planMotion(planner_type_);
      demos_tmp[j].first = feature_fct_->costTraj(demos_tmp[j].second);
    }

    for (int j = 0; j < nb_planning_test_; j++)
      cout << "cost[" << j << "] : " << demos_tmp[j].first << endl;

    cout << "min cost : "
         << std::min_element(demos_tmp.begin(), demos_tmp.end())->first << endl;

    cout << "nb of via points : "
         << std::min_element(demos_tmp.begin(), demos_tmp.end())
                ->second.getNbOfViaPoints() << endl;

    demos.push_back(
        std::min_element(demos_tmp.begin(), demos_tmp.end())->second);
    demos_id.push_back(i);

    g3d_draw_allwin_active();
  }

  cout << "nb_demos_ : " << nb_demos_ << endl;

  saveDemoToFile(demos);

  cout << "exit generate demo" << endl;
}

void IocEvaluation::saveDemoToFile(const std::vector<Move3D::Trajectory>& demos,
                                   std::vector<Move3D::confPtr_t> context) {
  // int id = 0;

  // Create a buffer to keep track of demo ids being saved
  // each demo is incremented when saved
  std::vector<int> saved_id(
      *std::max_element(demo_ids_.begin(), demo_ids_.end()) + 1, 0);

  global_trajToDraw.clear();

  std::string folder = process_dir_ == "" ? folder_ : process_dir_;

  for (size_t i = 0; i < demos.size(); i++) {
    // if( demos[i].replaceP3dTraj() )
    // {
    //  cout << "has replaced p3d traj" << endl;
    //  cout << robot_->getP3dRobotStruct()->tcur << endl;
    //  cout << robot_->getName() << endl;
    // }

    global_trajToDraw.push_back(demos[i]);

    // Ids
    std::stringstream ss_id;
    ss_id << "_" << std::setw(3) << std::setfill('0') << demo_ids_[i] << "_"
          << std::setw(3) << std::setfill('0') << saved_id[demo_ids_[i]]++;

    // File name
    std::stringstream ss;
    ss << folder << "trajectory_" << feature_type_ << ss_id.str() << ".traj";

    cout << "save demo " << i << " : " << ss.str() << endl;
    // cout << "nb of via points  : "
    // << demos[i].getNbOfViaPoints() << endl;
    // p3d_save_traj( ss.str().c_str(), robot_->getP3dRobotStruct()->tcur );
    demos[i].saveToFile(ss.str());

    cout << "number of way points : " << demos[i].getNbOfViaPoints() << endl;

    // saveTrajToMatlab( demos[i], i );
    // cout << "save traj to matlab format!!!!" << endl;
  }

  if (!context.empty()) {
    // Ids
    std::stringstream ss_id;
    ss_id << "_" << std::setw(3) << std::setfill('0') << int(0) << "_"
          << std::setw(3) << std::setfill('0') << int(0);

    // File name
    std::stringstream ss;
    ss.str("");
    ss << folder << "context_" << feature_type_ << ss_id.str() << ".configs";

    cout << "save context " << ss.str() << endl;
    move3d_save_context_to_csv_file(context, ss.str());
  }

  if (!demo_ids_.empty()) {
    Eigen::VectorXi demo_ids(demo_ids_.size());
    for (int i = 0; i < demo_ids.size(); i++) demo_ids[i] = demo_ids_[i];

    // File name
    std::stringstream ss;
    ss.str("");
    ss << folder << "demo_ids_" << feature_type_ << ".ids";

    // Save to file
    std::ofstream file(ss.str().c_str());
    if (file.is_open()) file << std::scientific << demo_ids << '\n';
    file.close();
  }

  cout << "DEMO SIZE : " << demos.size() << endl;
}

void IocEvaluation::saveSamplesToFile(
    const std::vector<std::vector<Move3D::Trajectory> >& samples) const {
  for (size_t d = 0; d < samples.size(); d++) {
    for (size_t i = 0; i < samples[d].size(); i++) {
      std::stringstream ss;
      ss << folder_ << "samples/"
         << "trajectory_sample_" << feature_type_ << "_" << std::setw(3)
         << std::setfill('0') << d << "_" << std::setw(3) << std::setfill('0')
         << i << ".traj";

      cout << "save sample : " << i << " : " << ss.str() << endl;

      samples[d][i].saveToFile(ss.str());
    }
  }
}

std::vector<std::vector<Move3D::Trajectory> >
IocEvaluation::loadSamplesFromFile(int nb_demos, int nb_samples) const {
  std::vector<std::vector<Move3D::Trajectory> > samples(nb_demos);

  for (int d = 0; d < nb_demos; d++) {
    cout << "Load samples of demo : " << d << endl;

    for (int i = 0; i < nb_samples; i++) {
      std::stringstream ss;
      ss << folder_ << "samples/"
         << "trajectory_sample_" << feature_type_ << "_" << std::setw(3)
         << std::setfill('0') << d << "_" << std::setw(3) << std::setfill('0')
         << i << ".traj";

      // cout << "load sample : " << i << " : " << ss.str() << endl;

      samples[d].push_back(Move3D::Trajectory(robot_));
      samples[d][i].loadFromFile(ss.str());
    }
  }

  return samples;
}

bool file_exists_test(const std::string& name) {
  std::ifstream f(name.c_str());
  if (f.good()) {
    f.close();
    return true;
  } else {
    f.close();
    return false;
  }
}

bool IocEvaluation::loadDemonstrations(const std::vector<int>& demos_ids) {
  if (nb_way_points_ <= 0) {
    cout << "Error in : " << __PRETTY_FUNCTION__
         << " can not load demonstration because"
         << " number of way point not set " << endl;
    return false;
  }

  demos_.clear();
  context_.clear();
  global_trajToDraw.clear();
  std::stringstream ss;

  std::string folder = use_simulator_ ? original_demo_folder_ : folder_;

  int demo_id = 0;
  int instance_id = 0;

  if (use_context_) {
    std::stringstream ss_id;
    ss_id << "_" << std::setw(3) << std::setfill('0') << demo_id << "_"
          << std::setw(3) << std::setfill('0') << instance_id;
    ss.str("");

    ss << folder << "context_" << feature_type_ << ss_id.str() << ".configs";

    context_.push_back(move3d_load_context_from_csv_file(ss.str()));

    // Set number of demos from context
    nb_demos_ = context_[0].size();
  }

  std::vector<int> demo_ids_tmp = demos_ids;
  if (demo_ids_tmp.empty()) {
    for (int i = 0; i < nb_demos_; i++) demo_ids_tmp.push_back(i);
  }

  if (!demo_names_.empty()) {
    cout << "nb_demos_ : " << nb_demos_ << endl;
    cout << "demo_names.size() : " << demo_names_.size() << endl;
    cout << "demo_ids_tmp.size() : " << demo_ids_tmp.size() << endl;
    for (int i = 0; i < int(demo_ids_tmp.size()); i++) {
      cout << "demo ids [" << i << "] : " << demo_ids_tmp[i] << " "
           << demo_names_[demo_ids_tmp[i]] << endl;
    }
  }

  demo_id = 0;
  instance_id = 0;

  int discarded_demo = -1;
  std::vector<int> demos_to_remove;

  // Create a buffer to keep track of demo ids being saved
  // each demo is incremented when saved
  std::vector<int> saved_id(
      *std::max_element(demo_ids_tmp.begin(), demo_ids_tmp.end()) + 1, 0);

  cout << "save_id.size() : " << saved_id.size() << endl;

  for (int d = 0; d < nb_demos_; d++) {
    bool file_exists = false;

    do {
      std::stringstream ss_id;
      // ss_id << "_"  << std::setw(3) << std::setfill( '0' )
      // << demo_id << "_"
      // << std::setw(3) << std::setfill( '0' ) << instance_id ;

      ss_id << "_" << std::setw(3) << std::setfill('0') << demo_ids_tmp[d]
            << "_" << std::setw(3) << std::setfill('0')
            << saved_id[demo_ids_tmp[d]]++;

      ss.str("");
      ss << folder << "trajectory_" << feature_type_ << ss_id.str() << ".traj";

      file_exists = file_exists_test(ss.str());

      if (!file_exists) {
        cout << "warning: file " << ss.str() << " does not exist" << endl;
        demo_id++;
        instance_id = 0;
      }
    } while (!file_exists);

    if (discarded_demo == demo_id) demos_to_remove.push_back(d);

    cout << "Loading demonstration (" << d << ") from : " << ss.str() << endl;

    Move3D::Trajectory T(robot_);

    if (!T.loadFromFile(ss.str()))  //!p3d_read_traj( ss.str().c_str() ) )
    {
      cout << "could not load trajectory" << endl;
      return false;
    }

    // Move3D::Trajectory T = robot_->getCurrentTraj();
    // T.computeSubPortionIntergralCost( T.getCourbe() );

    if (T.getNbOfViaPoints() != nb_way_points_)  // Only cut if needed
    {
      // cout << "ring in " << nb_way_points_ << " way points" << endl;
      T.cutTrajInSmallLP(nb_way_points_ - 1);
      // T.computeSubPortionIntergralCost( T.getCourbe() );
    }

    // T.replaceP3dTraj();

    T.setColor(d % 8);  // cout << "color : " << d%8 << endl;
    global_trajToDraw.push_back(T);

    demos_.push_back(T);

    instance_id++;
  }

  if (!context_.empty()) {
    context_t context(1);
    for (int i = 0; i < int(context_[0].size()); i++) {
      if (std::find(demos_to_remove.begin(), demos_to_remove.end(), i) ==
          demos_to_remove.end()) {
        context[0].push_back(context_[0][i]);
      }
    }
    context_ = context;
  }

  cout << "End loading demonstrations" << endl;

  return true;
}

void IocEvaluation::loadPlannerTrajectories(int nb_trajs,
                                            int offset,
                                            int random) {
  cout << "planners_.getBestTrajs().size() : "
       << planners_.getBestTrajs().size() << endl;

  if (nb_trajs < 0) {
    nb_trajs = planners_.getBestTrajs().size();
  }
  if (offset < 0) {
    offset = 0;
  }
  if ((random == 0) &&
      ((offset + nb_trajs) > int(planners_.getBestTrajs().size()))) {
    samples_.clear();
    cout << "out of bounds in : " << __PRETTY_FUNCTION__ << endl;
    return;
  }

  int sequence_size = 16;

  if (random == 0)  // Continuous sequence
  {
    std::vector<Move3D::Trajectory>::const_iterator first =
        planners_.getBestTrajs().begin() + offset;
    std::vector<Move3D::Trajectory>::const_iterator last =
        planners_.getBestTrajs().begin() + offset + nb_trajs;
    samples_ = std::vector<Move3D::Trajectory>(first, last);
  } else if (random == 1)  // Random pick in each sequence
  {
    // int nb_sequences =
    // int(planners_.getBestTrajs().size()) / sequence_size;

    int nb_big_sequence = int(planners_.getBestTrajs().size()) / nb_trajs;

    samples_.clear();
    // cout << "nb of sequences : " << nb_sequences << endl;
    for (int i = 0; i < nb_trajs; i++) {
      int id = i + nb_trajs * p3d_random_integer(0, nb_big_sequence - 1);
      // cout << "Add id : " << id << endl;
      samples_.push_back(planners_.getBestTrajs()[id]);
    }
  }

  global_trajToDraw.clear();
  for (size_t i = 0; i < samples_.size(); i++) {
    double color = (double(i % sequence_size) / sequence_size);
    // cout << "i : " << i << ", color : " << color << endl;
    samples_[i].cutTrajInSmallLP(nb_way_points_ - 1);
    samples_[i].setUseContinuousColors(true);
    samples_[i].setColor(color);

    global_trajToDraw.push_back(samples_[i]);
  }

  g3d_draw_allwin_active();
}

void IocEvaluation::loadWeightVector(std::string filename) {
  cout << "Load Weight Vector" << endl;

  learned_vect_ = Eigen::VectorXd::Zero(nb_weights_);

  // Load vector from file
  std::stringstream ss;

  if (filename == "")
    ss << tmp_data_folder_ << "spheres_weights_" << std::setw(3)
       << std::setfill('0') << nb_samples_ << ".txt";
  else
    ss << tmp_data_folder_ << filename;

  cout << "LOADING LEARNED WEIGHTS : " << ss.str() << endl;

  Eigen::MatrixXd mat = move3d_load_matrix_from_csv_file(ss.str());

  if (mat.rows() > 1) {
    cout << __PRETTY_FUNCTION__ << " : weight vector is a matrix" << endl;
    return;
  }

  if (mat.cols() != int(learned_vect_.size())) {
    cout << __PRETTY_FUNCTION__ << " : weight vector is resized" << endl;
    cout << "learned_vect_.size() : " << learned_vect_.size() << endl;
    cout << "mat.cols() : " << mat.cols() << endl;
    nb_weights_ = mat.cols();
  }

  learned_vect_ = mat.row(0);

  //    std::ifstream file( ss.str().c_str() );
  //    std::string line, cell;

  //    int i=0;

  //    if( file.good() )
  //    {
  //        std::getline( file, line );
  //        std::stringstream lineStream( line );

  //        while( std::getline( lineStream, cell, ',' ) )
  //        {
  //            std::istringstream iss( cell );
  //            iss >> learned_vect_[i++];
  //        }
  //    }
  //    else {
  //        cout << "ERROR could not load weights" << endl;
  //    }
  //    file.close();

  cout << " LEARNED weight : " << learned_vect_.transpose() << endl;
}

void IocEvaluation::setContext(int d) {
  for (int i = 0; i < int(context_.size()); i++) {
    Move3D::Robot* entity = context_[i][d]->getRobot();
    entity->setAndUpdate(*context_[i][d]);
  }
}

void IocEvaluation::setBuffer(int d) {
  bool fill_buffer = false;
  if ((d > 0) && (demo_ids_[d] != d)) {
    if (!demo_ids_.empty() && demos_[demo_ids_[d]].getUseTimeParameter()) {
      double time_along_original_demo =
          demos_[demo_ids_[d]].getDuration() - demos_[d].getDuration();

      // cout << "demos_[ demo_ids_[d] ].getDuration() : "
      // << demos_[ demo_ids_[d] ].getDuration() << endl;
      // cout << "demos_[" << d << "].getDuration() : "
      // << demos_[d].getDuration() << endl;
      // cout << "time_along_original_demo : "
      // << time_along_original_demo << endl;

      double dt = demos_[d].getDeltaTime();

      std::vector<Eigen::VectorXd> buffer;
      int nb_config = 7;
      for (int i = 0; i < nb_config; i++) {
        double t = time_along_original_demo - double(nb_config - i) * dt;
        Move3D::confPtr_t q = demos_[demo_ids_[d]].configAtTime(t);
        buffer.push_back(q->getEigenVector(planning_group_->getActiveDofs()));
        // cout << "buffer " << d << " [" << i << "] "
        // << t << " : " << buffer[i].transpose() << endl;
      }
      static_cast<SmoothnessFeature*>(
          feature_fct_->getFeatureFunction("SmoothnessAll"))->setBuffer(buffer);
      fill_buffer = true;
    }
  }

  if (!fill_buffer) {
    cout << "Set empty buffer" << endl;
    std::vector<Eigen::VectorXd> buffer;  // empty buffer
    SmoothnessFeature* smoothness = static_cast<SmoothnessFeature*>(
        feature_fct_->getFeatureFunction("SmoothnessAll"));
    cout << "smoothness : " << smoothness << endl;
    smoothness->setBuffer(buffer);
  }
}

std::vector<FeatureVect> IocEvaluation::addDemonstrationsIk(HRICS::Ioc& ioc) {
  // Get features of demos
  std::vector<FeatureVect> phi_demo(demos_.size());

  for (size_t d = 0; d < demos_.size(); d++) {
    if (use_context_) setContext(d);

    ioc.addDemonstration(
        demos_[d].getEigenMatrix(planning_group_->getActiveDofs()),
        demos_[d].getDeltaTime());

    FeatureVect phi = feature_fct_->getFeatures(
        *ioc.getLastConfigOfDemo(d).getMove3DConfig(planning_group_));
    cout << std::scientific << "Feature Demo : " << phi.transpose() << endl;

    phi_demo[d] = phi;
  }

  return phi_demo;
}

std::vector<FeatureVect> IocEvaluation::addDemonstrations(HRICS::Ioc& ioc) {
  // Get features of demos
  std::vector<FeatureVect> phi_demo(demos_.size());

  for (size_t d = 0; d < demos_.size(); d++) {
    if (demos_[d].getNbOfViaPoints() != nb_way_points_)
      // Only cut of needed
      demos_[d].cutTrajInSmallLP(nb_way_points_ - 1);

    if (use_context_) setContext(d);

    if (feature_fct_->getFeatureFunction("SmoothnessAll")) setBuffer(d);

    FeatureVect phi = feature_fct_->getFeatureCount(demos_[d]);

    cout << std::scientific << "Feature Demo : " << phi.transpose() << endl;

    // ioc.addDemonstration( demos_[i].getEigenMatrix(6,7) );
    ioc.addDemonstration(
        demos_[d].getEigenMatrix(planning_group_->getActiveDofs()),
        demos_[d].getDeltaTime());
    phi_demo[d] = phi;
  }

  //    demos_[0].replaceP3dTraj();

  return phi_demo;
}

std::vector<std::vector<FeatureVect> > IocEvaluation::addSamples(
    HRICS::Ioc& ioc) {
  // Get features of demos
  for (size_t i = 0; i < samples_.size(); i++) {
    //        if( samples_[i].getNbOfViaPoints() != nb_way_points_ )
    // Only cut of needed
    samples_[i].cutTrajInSmallLP(nb_way_points_ - 1);
    //        ioc.addSample( 0, samples_[i].getEigenMatrix(6,7) );
    ioc.addSample(0,
                  samples_[i].getEigenMatrix(planning_group_->getActiveDofs()));
    // ioc.setNominalSampleValue( 0, i, samples_[i].getEigenMatrix(6,7)  );
    ioc.setNominalSampleValue(
        0, i, samples_[i].getEigenMatrix(planning_group_->getActiveDofs()));

    // FeatureProfile p = feature_fct_->getFeatureJacobianProfile(
    // samples_[i] );
    // ioc.setTotalCostsSampleValue( 0, i, p );
    // p = p.array().pow( 4 );
    // cout << "p(" << i << ") : " << p.transpose() << endl;
  }

  // ioc.generateSamples( nb_samples_ );

  // Compute the sum of gradient
  // double gradient_sum = 0.0;

  std::vector<std::vector<Move3D::Trajectory> > samples = ioc.getSamples();
  std::vector<std::vector<FeatureVect> > phi_k(samples.size());
  for (int d = 0; d < int(samples.size()); d++) {
    if (use_context_) {
      for (int i = 0; i < int(context_.size()); i++) {
        Move3D::Robot* entity = context_[i][d]->getRobot();
        entity->setAndUpdate(*context_[i][d]);
      }
    }

    for (int i = 0; i < int(samples[d].size()); i++) {
      FeatureVect phi = feature_fct_->getFeatureCount(samples[d][i]);
      // cout << "Feature Sample : " << phi.transpose() << endl;
      // gradient_sum += feature_fct_->getJacobianSum( samples[d][i] );
      phi_k[d].push_back(phi);
    }
  }

  // cout << "gradient sum = " << gradient_sum << endl;

  return phi_k;
}

bool IocEvaluation::isSampleDominated(const FeatureVect& demo,
                                      const FeatureVect& sample) const {
  for (int i = 0; i < sample.size(); i++) {
    if (sample[i] < demo[i]) {
      // cout << "Sample is NOT dominated!!!!" << endl;
      return false;
    }
  }
  cout << "Sample is dominated!!!!" << endl;
  return true;
}

void IocEvaluation::removeDominatedSamplesAndResample(
    HRICS::Ioc& ioc, std::vector<std::vector<FeatureVect> >& phi_k) {
  for (int d = 0; d < int(phi_k.size()); d++) {
    if (use_context_) {  // TODO VERIFY THIS FUNCTION FOR CONTEXT

      for (int i = 0; i < int(context_.size()); i++) {
        Move3D::Robot* entity = context_[i][d]->getRobot();
        entity->setAndUpdate(*context_[i][d]);
      }
    }

    for (int i = 0; i < int(phi_k[d].size()); i++) {
      while (isSampleDominated(phi_demos_[d], phi_k[d][i])) {
        ioc.generateSamples(1, remove_samples_in_collision_);
        std::vector<std::vector<Move3D::Trajectory> > samples =
            ioc.getSamples();

        for (int dd = 0; dd < int(samples.size()); dd++) {
          for (int ii = 0; ii < int(samples[dd].size()); ii++) {
            phi_k[d][i] = feature_fct_->getFeatureCount(samples[dd][ii]);
          }
        }
      }
    }
  }
}

bool IocEvaluation::isTrajectoryValid(Move3D::Trajectory& path) {
  Move3D::Robot* robot = path.getRobot();
  Move3D::Scene* sce = global_Project->getActiveScene();

  std::vector<Move3D::Robot*> others;
  for (size_t i = 0; i < sce->getNumberOfRobots(); i++) {
    std::string robot_name = sce->getRobot(i)->getName();
    if (robot_name != robot->getName() &&
        robot_name.find("HUMAN") == std::string::npos)
      others.push_back(sce->getRobot(i));
  }

  for (int i = 0; i < path.getNbOfViaPoints(); i++) {
    robot->setAndUpdate(*path[i]);
    if (robot->isInCollisionWithOthers(others)) {
      return false;
    }
  }

  return true;
}

//! Sample ik around the demonstrations
std::vector<std::vector<Move3D::confPtr_t> > IocEvaluation::runIKSampling() {
  cout << __PRETTY_FUNCTION__ << endl;

  global_configToDraw.clear();

  for (size_t i = 0; i < demo_ids_.size(); i++) {
    cout << "demo_ids_[" << i << "] : " << demo_ids_[i] << endl;
  }

  feature_fct_->printInfo();

  cout << "nb_demos : " << demos_.size() << endl;
  cout << "nb_samples : " << nb_samples_ << endl;

  std::vector<std::vector<Move3D::confPtr_t> > samples;

  cout << "Create Ioc" << endl;
  HRICS::Ioc ioc(nb_way_points_, planning_group_, goalset_data_);

  // Get demos features
  cout << "Add demonstrations" << endl;
  phi_demos_ = addDemonstrationsIk(ioc);

  // cout << "wait for key" << endl;
  // std::cin.ignore();

  bool generate = true;

  if (generate) {
    // Generate samples by random sampling
    cout << "Generate samples (" << nb_samples_ << ")" << endl;

    remove_samples_in_collision_ = true;

    int nb_invalid_samples = ioc.generateIKSamples(
        nb_samples_, remove_samples_in_collision_, context_);

    cout << "percentage of invalid samples : "
         << (100 * double(nb_invalid_samples) / double(nb_samples_)) << " \%"
         << endl;
    samples = ioc.getSamplesIk();

    robot_->getP3dRobotStruct()->tcur = NULL;
    //        ioc.addAllToDraw();
    //        saveSamplesToFile( samples );
  }

  double demo_cost = 0.0;

  FeatureVect phi = feature_fct_->getFeatures(
      *ioc.getLastConfigOfDemo(0).getMove3DConfig(planning_group_));

  cout << "weight : " << feature_fct_->getWeights().transpose() << endl;
  cout << "cost : " << feature_fct_->getWeights().transpose() * phi << " , ";
  cout << "Feature Sample : " << int(-1) << " , " << phi.transpose() << endl;

  int nb_lower_cost = 0;
  int nb_lower_feature = 0;
  // int nb_shorter = 0;
  int nb_in_collision = 0;

  // Get samples features
  std::vector<std::vector<FeatureVect> > phi_k(samples.size());

  // For each demonstration
  for (int d = 0; d < int(samples.size()); d++) {
    if (use_context_) setContext(d);

    //        dtw_compare_performance( planning_group_, demos_[d], samples[d] );

    for (int i = 0; i < int(samples[d].size()); i++) {
      phi = feature_fct_->getFeatures(*samples[d][i]);

      phi_k[d].push_back(phi);

      double cost = feature_fct_->getWeights().transpose() * phi;

      // Compute stats ...
      if (cost < demo_cost) nb_lower_cost++;
      if (/*!samples[d][i].isValid()*/ !samples[d][i]->isInCollision())
        nb_in_collision++;

      global_configToDraw.push_back(samples[d][i]);
      //            global_configToDraw.back()->print();

      // cout << "cost : " << cost << " , ";
      //            cout.precision(4);
      cout << "Feature Sample : " << i << " , " << phi.transpose() << endl;

      for (int j = 0; j < phi_demos_[d].size(); j++) {
        if (phi[j] - phi_demos_[d][j] < 0) nb_lower_feature++;
      }
    }
  }

  cout << "nb_lower_cost : " << nb_lower_cost << endl;
  cout << "nb_lower_feature : " << nb_lower_feature << endl;
  cout << "nb_in_collision : " << nb_in_collision << endl;

  //    removeDominatedSamplesAndResample( ioc, phi_k );

  // checkStartAndGoal( samples );

  saveToMatrixFile(
      phi_demos_, phi_k, "spheres_features");  // TODO change name to motion...

  g3d_draw_allwin_active();

  //    samples_ = samples[0];
  //    saveTrajectories( samples_ );

  //    std::vector< std::vector<FeatureVect> > jac_sum_samples =
  //    getFeatureJacobianSum( ioc.getSamples() );
  //    saveToMatrixFile( phi_jac_demos_, jac_sum_samples, "spheres_jac_sum" );

  return samples;
}

std::vector<std::vector<Move3D::Trajectory> > IocEvaluation::runSampling() {
  cout << __PRETTY_FUNCTION__ << endl;

  global_trajToDraw.clear();

  for (size_t i = 0; i < demo_ids_.size(); i++) {
    cout << "demo_ids_[" << i << "] : " << demo_ids_[i] << endl;
  }

  for (size_t i = 0; i < demo_names_.size(); i++) {
    cout << "demo_names : " << demo_names_[i] << endl;
  }

  // For human WHY was that here???
  // feature_fct_->setAllFeaturesActive();

  // Compute the sum of gradient
  // double gradient_sum = 0.0;

  feature_fct_->printInfo();

  cout << "nb_demos : " << demos_.size() << endl;
  cout << "nb_samples : " << nb_samples_ << endl;

  cout << "Create Ioc" << endl;
  HRICS::Ioc ioc(nb_way_points_, planning_group_, goalset_data_);

  // Get demos features
  cout << "Add demonstrations" << endl;
  phi_demos_ = addDemonstrations(ioc);

  double demo_cost = 0.0;
  int nb_lower_cost = 0;
  int nb_lower_feature = 0;
  int nb_shorter = 0;
  int nb_in_collision = 0;

  // Get samples features
  std::vector<std::vector<Move3D::Trajectory> > samples(phi_demos_.size());
  std::vector<std::vector<FeatureVect> > phi_k(phi_demos_.size());

  // Reset All samples
  ioc.resetAllSamples();

  remove_samples_in_collision_ = true;

  // Remove current robot trajectory
  robot_->getP3dRobotStruct()->tcur = NULL;

  // Split to draw
  std::string draw_split = HriEnv->getString(HricsParam::ioc_traj_split_name);

  cout << "phi_demos_.size() : " << phi_demos_.size() << endl;

  // Variables to save the splits in collision
  std::vector<std::string> demos_in_collision;
  std::string current_traj_name;

  // For each demonstration
  Move3D::confPtr_t q = robot_->getCurrentPos();
  for (size_t d = 0; d < phi_demos_.size(); d++) {
    // Generate samples by random sampling
    cout << "Generate samples (" << nb_samples_ << ")" << endl;

    int nb_invalid_samples = ioc.generateDemoSamples(
        d, nb_samples_, remove_samples_in_collision_, context_);

    if (demo_ids_[d] < int(demo_names_.size())) {
      current_traj_name = demo_names_[demo_ids_[d]].substr(0, 11);
      cout << "demo : " << current_traj_name << endl;
    }

    if (nb_invalid_samples == nb_samples_) {
      demos_in_collision.push_back(current_traj_name);
    }

    cout << "percentage of invalid samples : "
         << (100 * double(nb_invalid_samples) / double(nb_samples_)) << " \%"
         << endl;

    // Convert samples to Move3D trajectories
    samples[d] = ioc.getDemoSamples(d);
    cout << "nb of samples : " << samples[d].size() << endl;

    // DRAW samples

    if (!ENV.getBool(Env::drawDisabled)) {
      if (HriEnv->getBool(HricsParam::ioc_draw_demonstrations) ||
          HriEnv->getBool(HricsParam::ioc_draw_samples)) {
        if ((!HriEnv->getBool(HricsParam::ioc_draw_one_demo)) ||
            demo_names_.empty() ||
            demo_names_[demo_ids_[d]].substr(0, 11) == draw_split) {
          q = ioc.addAllToDraw(demo_ids_, demo_names_);
        }
      }
    }

    // DELETE SAMPLES
    cout << "delete samples" << endl;
    ioc.deleteSampleForDemo(d);

    if (use_context_) setContext(d);

    if (feature_fct_->getFeatureFunction("SmoothnessAll")) setBuffer(d);

    cout << "start feature computation" << endl;

    phi_k[d].resize(samples[d].size());

    for (size_t i = 0; i < samples[d].size(); i++) {
      // Compute the sample features
      phi_k[d][i] = feature_fct_->getFeatureCount(samples[d][i]);

      // Get sample cost
      double cost = feature_fct_->getWeights().transpose() * phi_k[d][i];

      // Compute stats ...
      if (cost < demo_cost) nb_lower_cost++;
      if (samples[d][i].getParamMax() < demos_[d].getParamMax()) nb_shorter++;
      /*!samples[d][i].isValid()*/
      if (!isTrajectoryValid(samples[d][i])) nb_in_collision++;

      // cout << "cost : " << cost << " , ";
      // cout.precision(4);
      // cout << std::scientific << "Feature Sample : " << i << " , "
      //     << phi_k[d][i].transpose() << endl;
      // cout << "dist wrist " << phi[0] << endl; //24
      // cout << "length : " << samples[d][i].getParamMax() << endl;

      for (int j = 0; j < phi_demos_[d].size(); j++) {
        if (phi_k[d][i][j] - phi_demos_[d][j] < 0) nb_lower_feature++;
      }

      // gradient_sum += feature_fct_->getJacobianSum( samples[d][i] );
      // cout << "Sample(" << d << "," <<  i << ") : "
      // << phi_k[d][i].transpose() << endl;
      // cout << "Smoothness(" << d
      // << "," <<  i << ") : " << phi_k[d][i][0] << endl;
    }
  }

  cout << "save invalid to : " << endl;
  save_strings_to_file(demos_in_collision,
                       process_dir_ + "demo_in_collision.txt");

  cout << "nb_lower_cost : " << nb_lower_cost << endl;
  cout << "nb_lower_feature : " << nb_lower_feature << endl;
  cout << "nb_shorter : " << nb_shorter << endl;
  cout << "nb_in_collision : " << nb_in_collision << endl;

  // removeDominatedSamplesAndResample( ioc, phi_k );

  // checkStartAndGoal( samples );

  saveToMatrixFile(phi_demos_, phi_k, "spheres_features");
  // TODO change name to motion...

  g3d_draw_allwin_active();

  //    samples_ = samples[0];
  //    saveTrajectories( samples_ );

  //    std::vector< std::vector<FeatureVect> > jac_sum_samples =
  // getFeatureJacobianSum( ioc.getSamples() );
  //    saveToMatrixFile( phi_jac_demos_, jac_sum_samples, "spheres_jac_sum" );

  robot_->setAndUpdate(*q);
  return samples;
}

void IocEvaluation::runFromFileSampling(int offset) {
  HRICS::Ioc ioc(nb_way_points_, planning_group_, goalset_data_);

  // Get demos features
  phi_demos_ = addDemonstrations(ioc);

  // Jac sum of demos
  std::vector<std::vector<Move3D::Trajectory> > trajs_tmp;
  trajs_tmp.push_back(demos_);
  std::vector<std::vector<FeatureVect> > jac_sum_demos =
      getFeatureJacobianSum(trajs_tmp);
  phi_jac_demos_ = jac_sum_demos.back();

  // Load samples from file
  // samples_ = planners_.getBestTrajs();

  // Random removal in the vector
  bool random_removal = false;
  if (random_removal) {
    while (int(samples_.size()) != nb_samples_) {
      int pos = p3d_random_integer(0, samples_.size() - 1);
      std::vector<Move3D::Trajectory>::iterator it = samples_.begin();
      std::advance(it, pos);
      samples_.erase(it);
    }
  } else {
    if (offset == -1)
      loadPlannerTrajectories(nb_samples_, -1, 1);
    else
      loadPlannerTrajectories(nb_samples_, offset, 0);
  }

  global_trajToDraw.clear();
  std::vector<std::vector<FeatureVect> > phi_k(1);
  for (int i = 0; i < int(samples_.size()); i++) {
    global_trajToDraw.push_back(samples_[i]);
    FeatureVect phi = feature_fct_->getFeatureCount(samples_[i]);
    phi_k[0].push_back(phi);
  }

  // Get samples features
  // std::vector< std::vector<FeatureVect> > phi_k = addSamples( ioc );

  //    std::vector< std::vector<Move3D::Trajectory> > samples;
  //    samples.push_back( samples_ );
  //    checkStartAndGoal( samples );

  //    ioc.addAllToDraw();
  saveToMatrixFile(phi_demos_, phi_k, "spheres_features");

  std::vector<std::vector<Move3D::Trajectory> > samples_trajs_tmp;
  samples_trajs_tmp.push_back(samples_);

  //    std::vector< std::vector<FeatureVect> > jac_sum_samples =
  //    getFeatureJacobianSum( samples_trajs_tmp );
  //    saveToMatrixFile( phi_jac_demos_, jac_sum_samples, "spheres_jac_sum" );
}

//! Generate a distribution that maximizes entropy
void IocEvaluation::monteCarloSampling(double factor, int nb_tries) {
  cout << __PRETTY_FUNCTION__ << endl;

  HRICS::Ioc ioc(nb_way_points_, planning_group_, goalset_data_);

  // Get demos features
  phi_demos_ = addDemonstrations(ioc);

  std::vector<std::vector<FeatureVect> > phi_k(phi_demos_.size());

  bool sampling_done = false;

  global_trajToDraw.clear();

  setOriginalWeights();

  demos_[0].computeSubPortionIntergralCost(demos_[0].getCourbe());

  while (sampling_done == false) {
    ioc.generateSamples(nb_samples_, remove_samples_in_collision_);

    std::vector<std::vector<Move3D::Trajectory> > samples = ioc.getSamples();

    // cout << "d : " << samples.size() << endl;

    for (int d = 0; (d < int(samples.size())) && (sampling_done == false);
         d++) {
      for (int i = 0; (i < int(samples[d].size())) && (sampling_done == false);
           i++) {
        if (*demos_[d].getBegin() != *samples[d][i].getBegin())
          cout << "Error in begin config" << endl;

        if (*demos_[d].getEnd() != *samples[d][i].getEnd())
          cout << "Error in end config" << endl;

        FeatureVect phi = feature_fct_->getFeatureCount(samples[d][i]);

        double cost = original_vect_.transpose() * (phi - phi_demos_[d]);

        if ((std::exp(-cost) / factor) > p3d_random(0, 1)) {
          phi_k[d].push_back(phi);

          cout << "samples ( " << phi_k[d].size() << " ) : " << cost << endl;

          samples[d][i].setColor(phi_k[d].size() / double(nb_samples_));
          global_trajToDraw.push_back(samples[d][i]);
        }

        if (cost < 0.0) {
          cout << "cost demo : "
               << original_vect_.transpose() *
                      feature_fct_->getFeatureCount(demos_[d]) << endl;

          cout << "cost sample : " << original_vect_.transpose() * phi << endl;
          // samples[d][i].setColor( samples[d].size() / double(nb_samples_) );
          // global_trajToDraw.push_back( samples[d][i] );
        }

        if (int(phi_k[d].size()) == nb_samples_) {
          sampling_done = true;
          break;
        }

        if (PlanEnv->getBool(PlanParam::stopPlanner)) {
          sampling_done = true;
          break;
        }
      }
    }

    g3d_draw_allwin_active();
  }

  saveToMatrixFile(phi_demos_, phi_k, "spheres_features");
}

bool IocEvaluation::checkStartAndGoal(
    const std::vector<std::vector<Move3D::Trajectory> >& samples) const {
  for (int d = 0; d < int(samples.size()); d++) {
    for (int i = 0; i < int(samples[d].size()); i++) {
      cout << "sample(" << d << " , " << i << ") : " << endl;
      samples[d][i].getBegin()->print();
      samples[d][i].getEnd()->print();
    }
  }

  return true;
}

bool IocEvaluation::checkDegeneration(
    const std::vector<FeatureVect>& phi_demos,
    const std::vector<std::vector<FeatureVect> >& phi_k) const {
  if (phi_demos.size() != phi_k.size()) {
    cout << "Error" << endl;
    return false;
  }
  for (size_t d = 0; d < phi_k.size(); d++) {
    for (size_t k = 0; k < phi_k[d].size(); k++) {
      for (int i = 0; i < phi_k[d][k].size(); i++) {
        // TODO check that no domination exists
      }
    }
  }

  return true;
}

void IocEvaluation::runLearning() {
  //    for( int i=0;i<1;i++)
  //    {
  //        Eigen::VectorXd w = ioc.solve( phi_demo, phi_k );
  //        cout << "w : " << w.transpose() << endl;
  //    }
}

Eigen::VectorXd IocEvaluation::getCostsOfDemonstrations() const {
  Eigen::VectorXd costs(demos_.size());

  for (int d = 0; d < int(demos_.size()); d++) {
    costs[d] = feature_fct_->costTraj(demos_[d]);
  }

  return costs;
}

void IocEvaluation::setLearnedWeights() {
  cout << "Set Learned Weight Vector" << endl;
  feature_fct_->setWeights(learned_vect_);
  feature_fct_->printWeights();
}

void IocEvaluation::setOriginalWeights() {
  cout << "Set Original Weight Vector" << endl;
  feature_fct_->setWeights(original_vect_);
  feature_fct_->printWeights();
}

Eigen::VectorXd IocEvaluation::compareDemosAndPlanned() {
  loadDemonstrations();

  setOriginalWeights();
  Eigen::VectorXd costs_demo = getCostsOfDemonstrations();
  Eigen::VectorXd costs_learned(costs_demo.size());
  learned_.resize(costs_demo.size());

  cout << (costs_demo).transpose() << endl;
  cout << "NB OF DEMOS : " << costs_demo.size() << endl;

  int nb_tests = planner_type_ == astar ? 1 : nb_planning_test_;

  loadWeightVector();  // Load learned from file

  for (int i = 0; i < costs_demo.size(); i++) {
    cout << "--------------------" << endl;

    costs_learned[i] = 0.0;

    global_trajToDraw.clear();

    std::vector<std::pair<double, Move3D::Trajectory> > traj_tmp(nb_tests);

    setLearnedWeights();  // Plan under the learned weights

    for (int j = 0; j < nb_tests; j++) {
      demo_id_ = j;
      traj_tmp[j].second = planMotion(planner_type_);
      traj_tmp[j].first = feature_fct_->costTraj(traj_tmp[j].second);
    }

    for (int j = 0; j < nb_tests; j++)
      cout << "cost[" << j << "] : " << traj_tmp[j].first << endl;

    // Min over the number of planning sequence (for radomized planners)
    learned_[i] = std::min_element(traj_tmp.begin(), traj_tmp.end())->second;
    global_trajToDraw.push_back(learned_[i]);
    g3d_draw_allwin_active();

    setOriginalWeights();  // Evaluate under the true weights
    learned_[i].resetCostComputed();
    costs_learned[i] = feature_fct_->costTraj(learned_[i]);
  }

  double weight_dist = (learned_vect_ - original_vect_).norm();

  double mean_demo = costs_demo.mean();
  double sq_sum_demo = costs_demo.transpose() * costs_demo;
  double stdev_demo = std::sqrt(sq_sum_demo / double(costs_demo.size()) -
                                (mean_demo * mean_demo));

  double mean_learned = costs_learned.mean();
  double sq_sum_learned = costs_learned.transpose() * costs_learned;
  double stdev_learned =
      std::sqrt((sq_sum_learned / double(costs_learned.size())) -
                (mean_learned * mean_learned));

  cout << "--------------------------------------" << endl;

  cout << "weight dist : " << weight_dist << endl;
  cout << "cost of demos : " << costs_demo.transpose() << endl;
  cout << "cost of learned : " << costs_learned.transpose() << endl;
  cout << "mean_demo : " << mean_demo << " , stdev_demo : " << stdev_demo
       << endl;
  cout << "mean_learned : " << mean_learned
       << " , stdev_learned : " << stdev_learned << endl;
  cout << "mean diff : " << mean_learned - mean_demo
       << " , stdev diff : " << stdev_learned - stdev_demo << endl;
  cout << "min diff : " << (costs_learned - costs_demo).minCoeff() << endl;
  cout << "max diff : " << (costs_learned - costs_demo).maxCoeff() << endl;

  Eigen::VectorXd result(6);
  result[0] = mean_demo;
  result[1] = mean_learned;
  result[2] = mean_learned - mean_demo;
  result[3] = weight_dist;
  result[4] = (costs_learned - costs_demo).minCoeff();
  result[5] = (costs_learned - costs_demo).maxCoeff();
  return result;
}

std::vector<std::vector<FeatureVect> > IocEvaluation::getFeatureJacobianSum(
    const std::vector<std::vector<Move3D::Trajectory> >& all_trajs) {
  std::vector<std::vector<FeatureVect> > feature_jac_sum(all_trajs.size());

  for (size_t k = 0; k < all_trajs.size(); k++)  // For each demo
  {
    for (size_t l = 0; l < all_trajs[k].size(); l++)  // For each sample
    {
      feature_jac_sum[k].push_back(
          FeatureVect::Zero(feature_fct_->getNumberOfFeatures()));

      FeatureJacobian p = feature_fct_->getFeatureJacobian(all_trajs[k][l]);

      for (int i = 0; i < p.rows(); i++)  // For each via points
      {
        for (int j = 0; j < p.cols(); j++)  // For each feature
        {
          feature_jac_sum[k][l](j) += p(i, j);
        }
      }
    }
  }

  // cout << "return feature_jac_sum" << endl;
  return feature_jac_sum;
}

std::vector<std::vector<FeatureVect> > IocEvaluation::getFeatureCount(
    const std::vector<std::vector<Move3D::Trajectory> >& all_trajs) {
  std::vector<std::vector<FeatureVect> > feature_count(all_trajs.size());

  for (size_t k = 0; k < all_trajs.size(); k++)  // For each demo
  {
    for (size_t l = 0; l < all_trajs[k].size(); l++)  // For each sample
    {
      feature_count[k][l] = feature_fct_->getFeatureCount(all_trajs[k][l]);
    }
  }

  // cout << "return feature_jac_sum" << endl;
  return feature_count;
}

void IocEvaluation::saveDemoToMatlab() { saveTrajToMatlab(demos_[0], 0); }

void IocEvaluation::saveContextToFile(
    const std::vector<Move3D::confPtr_t>& context) const {}

void IocEvaluation::saveNbDemoAndNbFeatures() {
  Eigen::MatrixXd mat(Eigen::MatrixXd::Zero(2, 1));
  mat(0, 0) = nb_demos_;
  mat(1, 0) = feature_fct_->getNumberOfFeatures();

  move3d_save_matrix_to_file(mat, std::string("matlab/problem.txt"));
}

void IocEvaluation::saveTrajToMatlab(const Move3D::Trajectory& t,
                                     int id) const {
  Move3D::Trajectory saved_traj(t);
  int nb_way_points = 100;

  saved_traj.cutTrajInSmallLP(nb_way_points - 1);

  //    Eigen::MatrixXd mat = saved_traj.getEigenMatrix(6,7);
  Eigen::MatrixXd mat =
      saved_traj.getEigenMatrix(planning_group_->getActiveDofs());

  // Save traj to file
  move3d_save_matrix_to_file(
      mat,
      process_dir_ +
          std::string("matlab/traj_" + num_to_string<int>(id) + ".txt"));
}

void IocEvaluation::saveToMatrixFile(
    const std::vector<FeatureVect>& demos,
    const std::vector<std::vector<FeatureVect> >& samples,
    std::string name) {
  if (demos.empty()) {
    return;
  }

  int nb_samples = 0;
  for (int d = 0; d < int(samples.size()); d++)
    for (int i = 0; i < int(samples[d].size()); i++) nb_samples++;

  Eigen::MatrixXd mat(demos.size() + nb_samples, demos[0].size());

  // FIRST ADD DEMOS
  for (int d = 0; d < int(demos.size()); d++) {
    mat.row(d) = demos[d];
  }

  // THEN ADD EACH SET OF SAMPLES AS A ROW, STARTING WITH DEMO 1..N
  int k = 0;
  for (int d = 0; d < int(samples.size()); d++) {
    for (int i = 0; i < int(samples[d].size()); i++) {
      mat.row(demos.size() + k++) = samples[d][i];
    }
  }

  std::string folder = process_dir_ == "" ? tmp_data_folder_ : process_dir_;

  // File name
  std::stringstream ss;
  ss << folder << name << "_" << std::setw(3) << std::setfill('0')
     << nb_samples_ << ".txt";

  cout << "mat.rows() : " << mat.rows() << " , mat.cols() : " << mat.cols()
       << endl;

  // cout << "save samples to : " << feature_matrix_name_ << endl;
  move3d_save_matrix_to_file(mat, ss.str());

  // SAVE DEMO IDS
  if (!demo_ids_.empty()) {
    Eigen::VectorXi demo_ids(demo_ids_.size());
    for (int i = 0; i < demo_ids.size(); i++) demo_ids[i] = demo_ids_[i];

    // File name
    ss.str("");
    ss << folder << name << "_demo_ids_" << std::setw(3) << std::setfill('0')
       << nb_samples_ << ".txt";

    cout << "save matrix to : " << ss.str() << endl;

    // Save to file
    std::ofstream file(ss.str().c_str());
    if (file.is_open()) file << std::scientific << demo_ids << '\n';
    file.close();
  }
}

void IocEvaluation::saveTrajectories(
    const std::vector<Move3D::Trajectory>& trajectories) {
  std::stringstream ss;

  for (size_t i = 0; i < trajectories.size(); i++) {
    ss.str("");
    ss << "trajectory" << std::setw(3) << std::setfill('0') << i << ".traj";

    trajectories[i].replaceP3dTraj();
    p3d_save_traj((traj_folder_ + "/" + ss.str()).c_str(),
                  robot_->getP3dRobotStruct()->tcur);

    cout << "save planner result " << i << " : " << ss.str() << endl;
  }
}

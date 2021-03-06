/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \file    policy_improvement.cpp

 \author  Ludovic Righetti, Peter Pastor, Mrinal Kalakrishnan
 \date    May 26, 2010

 **********************************************************************/

// system includes
#include <time.h>
#include <cfloat>
#include <stdio.h>
//#include <ros/assert.h>

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/LU>
#include <Eigen/Dense>

// local includes
//#include <ros/ros.h>
#include "policy_improvement.hpp"
//#include <stomp_motion_planner/assert.h>
#include <algorithm>
#include <iostream>

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "planner/planEnvironment.hpp"
#include "feature_space/smoothness.hpp"
#include "utils/NumsAndStrings.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/P3d-pkg.h>

#include "stompOptimizer.hpp"

std::vector<double> global_noiseTrajectory1;
std::vector<double> global_noiseTrajectory2;

using std::cout;
using std::endl;
using namespace Move3D;

MOVE3D_USING_BOOST_NAMESPACE

using namespace Eigen;

namespace stomp_motion_planner {
//! Computes the cost the rollout
//! the function sums the control costs of all dimensions
double Rollout::getCost() {
  // Get the total cost
  double cost = state_costs_.sum();

  // This is set to true when all costs
  // are combined in the per state cost by avearaging
  if (use_total_smoothness_cost_) {
    cost += total_smoothness_cost_;
  } else {
    int num_dim = control_costs_.size();
    for (int d = 0; d < num_dim; ++d) cost += control_costs_[d].sum();
  }

  return cost;
}

void Rollout::printCost(double weight) {
  double control_cost = 0.0;

  for (int d = 0; d < int(control_costs_.size()); ++d)
    control_cost += (control_costs_[d].sum());

  cout.precision(6);
  cout << "control cost : " << control_cost
       << " , state cost : " << state_costs_.sum() << endl;
}

void Rollout::printProbabilities() {
  cout << "state_costs_ : " << endl;
  cout << state_costs_.transpose() << endl;

  cout << "control_costs_ : " << endl;
  for (int d = 0; d < int(control_costs_.size()); ++d) {
    cout << control_costs_[d].transpose() << endl;
  }

  cout << "cumulative_cost_ : " << endl;
  for (int d = 0; d < int(probabilities_.size()); ++d) {
    cout << cumulative_costs_[d].transpose() << endl;
  }

  cout << "probabilities_ : " << endl;
  for (int d = 0; d < int(probabilities_.size()); ++d) {
    cout << probabilities_[d].transpose() << endl;
  }
  cout << endl;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------

PolicyImprovement::PolicyImprovement() : initialized_(false) {}

PolicyImprovement::~PolicyImprovement() {}

//! Initialize policy improvment
//! 1 - Allocates a multivariate gaussian sampler
//! 2 - Computes the projection matrix
bool PolicyImprovement::initialize(const int num_rollouts,
                                   const int num_time_steps,
                                   const int num_reused_rollouts,
                                   const int num_extra_rollouts,
                                   MOVE3D_BOOST_PTR_NAMESPACE<Policy> policy,
                                   MOVE3D_BOOST_PTR_NAMESPACE<Task> task,
                                   double discretization,
                                   bool use_cumulative_costs) {
  num_time_steps_ = num_time_steps;
  use_cumulative_costs_ = use_cumulative_costs;
  use_multiplication_by_m_ = PlanEnv->getBool(PlanParam::trajStompMultiplyM);
  policy_ = policy;
  task_ = task;
  discretization_ = discretization;

  cout << "PolicyImprovement::initialize : discretization_ : "
       << discretization_ << endl;

  policy_->setNumTimeSteps(num_time_steps_);
  policy_->getControlCosts(control_costs_);
  policy_->getCovariances(inv_control_costs_);
  policy_->getNumDimensions(num_dimensions_);
  policy_->getNumParameters(num_parameters_);
  policy_->getBasisFunctions(basis_functions_);
  policy_->getParameters(parameters_);

  // assert(preAllocateMultivariateGaussianSampler());
  // assert(setNumRollouts(num_rollouts, num_reused_rollouts,
  // num_extra_rollouts));
  // assert(preAllocateTempVariables());
  // assert(preComputeProjectionMatrices());

  //        cout << "num_reused_rollouts : " << num_reused_rollouts << endl;

  preAllocateMultivariateGaussianSampler();
  setNumRollouts(num_rollouts, num_reused_rollouts, num_extra_rollouts);
  preAllocateTempVariables();
  preComputeProjectionMatrices();

  // Always use the std dev initial tunning
  // at the first iteration
  use_new_std_dev_ = false;
  stdev_intieria_ = 0.2;
  stdev_scaling_ = Eigen::VectorXd::Ones(parameters_.size());
  stdev_min_ = 0.001 * Eigen::VectorXd::Ones(parameters_.size());

  //        project_last_config_ = true;

  //        if( project_last_config_ && ( planning_group_ != NULL ) )
  //        {
  //            Move3D::Robot* robot = planning_group_->robot_;
  //            Move3D::confPtr_t q_goal = robot->getGoalPos();
  //            robot->setAndUpdate(*q_goal);
  //            task_goal_ = robot->getJoint( "J3" )->getVectorPos();
  //        }

  multiple_smoothness_ = true;

  // TODO move somewhere else
  if (multiple_smoothness_) {
    Move3D::StackedFeatures* fct =
        dynamic_cast<Move3D::StackedFeatures*>(global_activeFeatureFunction);

    if (fct != NULL && fct->getFeatureFunction("SmoothnessAll") != NULL &&
        fct->getFeatureFunction("SmoothnessAll")->is_active_) {
      control_cost_weights_ =
          fct->getFeatureFunction("SmoothnessAll")->getWeights();
      //                cout << "control_cost_weights_ : " <<
      //                control_cost_weights_.transpose() << endl;
    } else {
      multiple_smoothness_ = false;
    }
  }

  return (initialized_ = true);
}

//! Allocates the sampler
//! also initializes the control cost inverse structure
bool PolicyImprovement::preAllocateMultivariateGaussianSampler() {
  // invert the control costs, initialize noise generators:
  //        inv_control_costs_.clear();
  noise_generators_.clear();

  for (int d = 0; d < num_dimensions_; ++d) {
    // cout << "control_costs_[" << d << "] = " << endl << control_costs_[d] <<
    // endl;
    // cout << "inv_control_costs_[" << d << "] = " << endl <<
    // control_costs_[d].inverse() << endl;
    //            inv_control_costs_.push_back(control_costs_[d].inverse());

    //            move3d_save_matrix_to_file( inv_control_costs_[d],
    //            "../matlab/invcost_matrix.txt" );

    if (true || !PlanEnv->getBool(PlanParam::trajStompMatrixAdaptation)) {
      // cout << "inv_control_costs_[" << d << "] : " << inv_control_costs_[d]
      // << endl;
      move3d_save_matrix_to_csv_file(inv_control_costs_[d],
                                     "control_matrices/inv_control_costs_" +
                                         num_to_string<int>(d) + ".csv");

      MultivariateGaussian mvg(VectorXd::Zero(num_parameters_[d]),
                               inv_control_costs_[d]);
      noise_generators_.push_back(mvg);
    } else {
      cout << "Error preAllocateMultivariateGaussianSampler" << endl;
      MultivariateGaussian mvg(
          VectorXd::Zero(num_parameters_[d]),
          MatrixXd::Identity(num_time_steps_, num_time_steps_));
      noise_generators_.push_back(mvg);
      return false;
    }
  }

  return true;
}

//! Allocates the rollout structures
//! @param num_rollouts the total number of rollouts
//! @param num_reused_rollouts the reused number of rollouts
//! @param num_extra_rollouts the number of extra rollouts
bool PolicyImprovement::setNumRollouts(const int num_rollouts,
                                       const int num_reused_rollouts,
                                       const int num_extra_rollouts) {
  num_rollouts_ = num_rollouts;
  num_rollouts_reused_ = num_reused_rollouts;
  num_rollouts_extra_ = num_extra_rollouts;
  num_rollouts_gen_ = 0;
  if (num_rollouts_reused_ >= num_rollouts) {
    printf(
        "Number of reused rollouts must be strictly less than number of "
        "rollouts.");
    return false;
  }

  // preallocate memory for a single rollout:
  Rollout rollout;

  rollout.parameters_.clear();
  rollout.noise_.clear();
  rollout.noise_projected_.clear();
  rollout.parameters_noise_projected_.clear();
  rollout.control_costs_.clear();
  rollout.total_costs_.clear();
  rollout.cumulative_costs_.clear();
  rollout.probabilities_.clear();

  // cout << "num_time_steps_  = " << num_time_steps_ << endl;

  for (int d = 0; d < num_dimensions_; ++d) {
    rollout.parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
    rollout.noise_.push_back(VectorXd::Zero(num_parameters_[d]));
    rollout.noise_projected_.push_back(VectorXd::Zero(num_parameters_[d]));

    rollout.control_costs_.push_back(VectorXd::Zero(num_time_steps_));
    rollout.total_costs_.push_back(VectorXd::Zero(num_time_steps_));
    rollout.cumulative_costs_.push_back(VectorXd::Zero(num_time_steps_));
    rollout.probabilities_.push_back(VectorXd::Zero(num_time_steps_));
    rollout.nominal_parameters_.push_back(VectorXd::Zero(num_time_steps_));
  }
  rollout.state_costs_ = VectorXd::Zero(num_time_steps_);
  rollout.out_of_bounds_ = false;

  // duplicate this rollout:
  for (int r = 0; r < num_rollouts; ++r) rollouts_.push_back(rollout);

  for (int r = 0; r < num_reused_rollouts; ++r)
    reused_rollouts_.push_back(rollout);

  for (int r = 0; r < num_extra_rollouts; ++r)
    extra_rollouts_.push_back(rollout);

  rollouts_reused_ = false;
  rollouts_reused_next_ = false;
  //    rollouts_reused_ = true;
  //    rollouts_reused_next_ = true;
  extra_rollouts_added_ = false;

  rollout_costs_total_.resize(num_rollouts);
  rollout_cost_sorter_.reserve(num_rollouts_);

  return true;
}

//! Allocates Temporary Variables
bool PolicyImprovement::preAllocateTempVariables() {
  tmp_noise_.clear();
  tmp_parameters_.clear();
  parameter_updates_.clear();
  for (int d = 0; d < num_dimensions_; ++d) {
    tmp_noise_.push_back(VectorXd::Zero(num_parameters_[d]));
    tmp_parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
    parameter_updates_.push_back(
        MatrixXd::Zero(num_time_steps_, num_parameters_[d]));
  }
  tmp_max_cost_ = VectorXd::Zero(num_time_steps_);
  tmp_min_cost_ = VectorXd::Zero(num_time_steps_);
  tmp_sum_rollout_probabilities_ = VectorXd::Zero(num_time_steps_);

  return true;
}

//! Precomputes the projection matrices M
//! using the inverse of the control cost matrix
//! Each column of the matrix is scaled
bool PolicyImprovement::preComputeProjectionMatrices() {
  //  ROS_INFO("Precomputing projection matrices..");
  projection_matrix_.resize(num_dimensions_);
  for (int d = 0; d < num_dimensions_; ++d) {
    projection_matrix_[d] = inv_control_costs_[d];

    // continue;
    // cout << "Inv control Matrix = " << endl << inv_control_costs_[d] << endl;

    for (int p = 0; p < num_parameters_[d]; ++p) {
      double column_max = inv_control_costs_[d](0, p);

      for (int p2 = 1; p2 < num_parameters_[d]; ++p2) {
        if (inv_control_costs_[d](p2, p) > column_max)
          column_max = inv_control_costs_[d](p2, p);
      }
      projection_matrix_[d].col(p) *= (1.0 / (num_parameters_[d] * column_max));
    }

    // move3d_save_matrix_to_file(projection_matrix_[0],"../matlab/m_mat.txt");
    // cout << "Projection Matrix = " << endl << projection_matrix_[d] << endl;
  }

  //  ROS_INFO("Done precomputing projection matrices.");
  return true;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
bool PolicyImprovement::copyParametersFromPolicy() {
  if (!policy_->getParameters(parameters_)) {
    //        ROS_ERROR("Failed to get policy parametertes.");
    return false;
  }

  //    // draw traj
  //    const std::vector<ChompDof>& joints =
  //    optimizer->getPlanningGroup()->chomp_dofs_;
  //    Robot* robot = optimizer->getPlanningGroup()->robot_;
  //    Move3D::Trajectory traj(robot);
  //
  //    for ( int j=0; j<num_time_steps_; ++j)
  //    {
  //      confPtr_t q = robot->getCurrentPos();
  //
  //      for ( int i=0; i<optimizer->getPlanningGroup()->num_joints_; ++i)
  //      {
  //        (*q)[joints[i].move3d_dof_index_] = parameters_[i][j];
  //      }
  //      traj.push_back(q);
  //    }
  //    traj.replaceP3dTraj();
  //    //    traj.print();
  //    g3d_draw_allwin_active();
  //    cout << "num_time_steps_ : " << num_time_steps_ << endl;

  return true;
}

bool PolicyImprovement::computeProjectedNoise(Rollout& rollout) {
  // for (int d=0; d<num_dimensions_; ++d)
  // {
  //            rollout.noise_projected_[d] = projection_matrix_[d] *
  //            rollout.noise_[d];
  // rollout.parameters_noise_projected_[d] = rollout.parameters_[d] +
  // rollout.noise_projected_[d];
  // cout << "rollout.noise_projected_[" << d << "] = " <<
  // rollout.noise_projected_[d].transpose() << endl;
  //        }

  return true;
}

bool PolicyImprovement::computeProjectedNoise() {
  // for (int r=0; r<num_rollouts_; ++r)
  // {
  //  computeProjectedNoise(rollouts_[r]);
  // }
  return true;
}

bool PolicyImprovement::resetReusedRollouts() {
  rollouts_reused_next_ = false;
  return true;
}

void PolicyImprovement::addStraightLines(std::vector<int> points,
                                         Rollout& rollout) {
  shared_ptr<StompOptimizer> optimizer =
      static_pointer_cast<StompOptimizer>(task_);
  shared_ptr<CovariantTrajectoryPolicy> policy =
      static_pointer_cast<CovariantTrajectoryPolicy>(policy_);

  const std::vector<ChompDof>& joints =
      optimizer->getPlanningGroup()->chomp_dofs_;

  Eigen::MatrixXd parameters(num_dimensions_, num_time_steps_);

  for (int i = 0; i < num_dimensions_; ++i) {
    parameters.row(i) = rollout.parameters_[i].transpose();
  }

  for (int k = 0; k < int(points.size() - 1); k++) {
    int init = points[k];
    int end = points[k + 1];
    if (init == end) continue;

    std::vector<confPtr_t> confs(2);
    confs[0] = optimizer->getPlanningGroup()->robot_->getCurrentPos();
    confs[1] = optimizer->getPlanningGroup()->robot_->getCurrentPos();

    for (int i = 0; i < optimizer->getPlanningGroup()->num_dofs_; ++i)
      (*confs[0])[joints[i].move3d_dof_index_] = parameters(i, init);

    for (int i = 0; i < optimizer->getPlanningGroup()->num_dofs_; ++i)
      (*confs[1])[joints[i].move3d_dof_index_] = parameters(i, end);

    LocalPath path(confs[0], confs[1]);

    double step = path.getParamMax() / (end - init);
    double param = step;
    for (int j = init; j <= end; ++j) {
      confPtr_t q = path.configAtParam(param);

      for (int i = 0; i < optimizer->getPlanningGroup()->num_dofs_; ++i)
        parameters(i, j) = (*q)[joints[i].move3d_dof_index_];

      param += step;
    }
  }

  parameters_.resize(num_dimensions_);

  for (int i = 0; i < num_dimensions_; ++i) {
    rollout.parameters_[i] = parameters.row(i);
  }

  for (int d = 0; d < num_dimensions_; ++d) {
    rollout.noise_[d] = rollout.parameters_[d] - parameters_[d];
  }
}

bool PolicyImprovement::simpleJointLimits(Rollout& traj) const {
  for (int j = 0; j < num_dimensions_; j++) {
    double coeff = 1.0;

    double j_max = static_pointer_cast<StompOptimizer>(task_)
                       ->getPlanningGroup()
                       ->chomp_dofs_[j]
                       .joint_limit_max_;
    double j_min = static_pointer_cast<StompOptimizer>(task_)
                       ->getPlanningGroup()
                       ->chomp_dofs_[j]
                       .joint_limit_min_;

    for (int i = 0; i < num_time_steps_; i++) {
      int k = 0;
      while (traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max) {
        coeff *= 0.90;  // 90 percent
        traj.noise_[j] *= coeff;
        traj.parameters_[j] = traj.nominal_parameters_[j] + traj.noise_[j];
        if (k++ > 3) {  // this was at 100
          break;
        }
      }

      // cout << "Joint limit coefficient : " << coeff << endl;
    }

    // Set the start and end values constant
    traj.parameters_[j].head(1) = traj.parameters_[j].head(1);
    traj.parameters_[j].tail(1) = traj.parameters_[j].tail(1);
  }

  return true;
}

bool PolicyImprovement::generateRollouts(
    const std::vector<double>& noise_stddev) {
  assert(initialized_);
  assert(static_cast<int>(noise_stddev.size()) == num_dimensions_);

  // save the latest policy parameters:
  copyParametersFromPolicy();

  // we assume here that rollout_parameters_ and rollout_noise_ have already
  // been allocated
  num_rollouts_gen_ = num_rollouts_ - num_rollouts_reused_;

  // The rollouts of the run are not used
  if (!rollouts_reused_next_) {
    num_rollouts_gen_ = num_rollouts_;
    if (num_rollouts_reused_ > 0) {
      rollouts_reused_next_ = true;
    }

    rollouts_reused_ = false;
  } else {
    // figure out which rollouts to reuse
    rollout_cost_sorter_.clear();
    for (int r = 0; r < num_rollouts_; ++r) {
      double cost = rollout_costs_total_[r];
      //  cout << "rollouts_(" << r << ") cost : " << rollouts_[r].getCost() <<
      //  " , stored cost : "  << rollout_costs_total_[r] << endl;

      // discard out of bounds rollouts
      if (rollouts_[r].out_of_bounds_) {
        cost = std::numeric_limits<double>::max();
      }
      // cout << "rollout_cost_sorter_(" << r << ") cost : " << cost << endl;
      rollout_cost_sorter_.push_back(std::make_pair(cost, r));
    }
    if (/*extra_rollouts_added_*/ false) {
      for (int r = 0; r < num_rollouts_extra_; ++r) {
        double cost;
        if (r == 0)
          cost = 0.0;
        else
          cost = extra_rollouts_[r].getCost();

        rollout_cost_sorter_.push_back(std::make_pair(cost, -r - 1));
        // cout << "extra cost["<< r <<"]: " << cost << endl;
        // index is -ve if rollout is taken from extra_rollouts
      }
      extra_rollouts_added_ = false;
    }
    std::sort(rollout_cost_sorter_.begin(), rollout_cost_sorter_.end());

    //            for (int r=0; r<num_rollouts_reused_; r++)
    //                cout << "reused_rollouts_(" << r << ") cost : " <<
    //                reused_rollouts_[r].getCost() << endl;

    // use the best ones: (copy them into reused_rollouts)
    for (int r = 0; r < num_rollouts_reused_; ++r) {
      double reuse_index = rollout_cost_sorter_[r].second;
      // double reuse_cost = rollout_cost_sorter_[r].first;

      //                cout <<  "Reuse "<< r << ", cost = " <<
      //                rollout_cost_sorter_[r].first << endl;

      if (reuse_index >= 0)
        reused_rollouts_[r] = rollouts_[reuse_index];
      else {
        // ROS_INFO("Reused noise-less rollout of cost %f", reuse_cost);
        reused_rollouts_[r] = extra_rollouts_[-reuse_index - 1];
      }
    }

    // copy them back from reused_rollouts_ into rollouts_
    for (int r = 0; r < num_rollouts_reused_; ++r) {
      rollouts_[num_rollouts_gen_ + r] = reused_rollouts_[r];

      // update the noise based on the new parameters:
      for (int d = 0; d < num_dimensions_; ++d) {
        rollouts_[num_rollouts_gen_ + r].noise_[d] =
            rollouts_[num_rollouts_gen_ + r].parameters_[d] - parameters_[d];
      }
    }
    rollouts_reused_ = true;
  }

  // Generate new rollouts
  for (int d = 0; d < num_dimensions_; ++d) {
    for (int r = 0; r < num_rollouts_gen_; ++r) {
      noise_generators_[d].sample(tmp_noise_[d]);

      // cout << "noise(" << r << "," << d << ") : " <<
      // tmp_noise_[d].transpose() << endl;

      rollouts_[r].noise_[d] = noise_stddev[d] * tmp_noise_[d];
      rollouts_[r].parameters_[d] = parameters_[d] + rollouts_[r].noise_[d];
      rollouts_[r].nominal_parameters_[d] = parameters_[d];
    }
  }

  if (false)
    for (int r = 0; r < num_rollouts_gen_; ++r) {
      // WARNING ADDED FOR IOC
      simpleJointLimits(rollouts_[r]);
    }

  return true;
}

bool PolicyImprovement::setRollouts(
    const std::vector<std::vector<Eigen::VectorXd> >& rollouts) {
  for (size_t r = 0; r < rollouts.size(); ++r) {
    rollouts_[r].parameters_ = rollouts[r];
    computeNoise(rollouts_[r]);
  }

  return true;
}

bool PolicyImprovement::getRollouts(
    std::vector<std::vector<Eigen::VectorXd> >& generated_rollouts,
    const std::vector<double>& noise_std,
    bool get_reused,
    std::vector<std::vector<Eigen::VectorXd> >& reused_rollouts) {
  // The standard deviation is \sigma^2 * R^-1
  // See Mrinal's thesis TODO see if we just multiply the samples by sigma
  // or sigma square
  std::vector<double> stdev = noise_std;
  if (use_new_std_dev_) {
    for (size_t d = 0; d < stdev.size(); d++) {
      stdev[d] = stdev_[d];
      // cout << "stdev[" <<d << "] : " << stdev[d] << endl;
    }
  } else {
    // for (int d = 0; d < stdev.size(); d++) {
    //   cout << "stdev[" <<d << "] : " << stdev[d] << endl;
    // }
    stdev_.resize(stdev.size());
    stdev_min_.resize(stdev.size());
    for (size_t d = 0; d < stdev.size(); d++) {
      stdev_[d] = stdev[d];
      // stdev_min_[d] = stdev[d] / (stdev_scaling_[d] * 10);
    }
  }

  if (!generateRollouts(stdev)) {
    cout << "Failed to generate rollouts." << endl;
    return false;
  }

  generated_rollouts.clear();
  for (int r = 0; r < num_rollouts_gen_; ++r) {
    generated_rollouts.push_back(rollouts_[r].parameters_);
  }

  if (get_reused) {
    reused_rollouts.clear();

    if (rollouts_reused_) {
      for (int r = 0; r < num_rollouts_reused_; ++r) {
        reused_rollouts.push_back(rollouts_[num_rollouts_gen_ + r].parameters_);
      }
    }
  }
  // computeProjectedNoise();
  return true;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
//----------------------------------------------------------------------
void PolicyImprovement::setRolloutOutOfBounds(int r, bool out_of_bounds) {
  assert(initialized_);

  if (r < 0 || r >= num_rollouts_gen_) return;

  // cout << "rollout[" << r << "] out of bounds : " << out_of_bounds << endl;
  rollouts_[r].out_of_bounds_ = out_of_bounds;
}

bool PolicyImprovement::setRolloutCosts(
    const Eigen::MatrixXd& costs,
    const std::vector<Eigen::MatrixXd>& control_costs,
    const double control_cost_weight,
    const std::vector<std::pair<bool, double> >& total_smoothness_cost,
    std::vector<double>& rollout_costs_total) {
  assert(initialized_);

  control_cost_weight_ = control_cost_weight;

  // Warning some control costs may be added to the state costs
  for (int r = 0; r < num_rollouts_gen_; ++r) {
    if (total_smoothness_cost[r].first)  // Set total cost
    {
      rollouts_[r].use_total_smoothness_cost_ = total_smoothness_cost[r].first;
      rollouts_[r].total_smoothness_cost_ = total_smoothness_cost[r].second;
    } else {
      rollouts_[r].use_total_smoothness_cost_ = false;

      // TODO add control costs only when
      // not using total control cost
      for (int d = 0; d < num_dimensions_; ++d) {
        rollouts_[r].control_costs_[d] = control_costs[r].row(d).transpose();
      }
    }

    rollouts_[r].state_costs_ = costs.row(r).transpose();
  }

  // set control costs
  // computeRolloutControlCosts();

  // set the total costs
  rollout_costs_total.resize(num_rollouts_);
  for (int r = 0; r < num_rollouts_; ++r) {
    rollout_costs_total[r] = rollout_costs_total_[r] = rollouts_[r].getCost();
    // rollouts_[r].printCost();
  }
  return true;
}

bool PolicyImprovement::computeRolloutCumulativeCosts() {
  // compute cumulative costs at each timestep
  for (int r = 0; r < num_rollouts_; ++r) {
    for (int d = 0; d < num_dimensions_; ++d) {
      // Set the total cost which is used
      // to compute the probabilities in the function bellow
      rollouts_[r].total_costs_[d] = rollouts_[r].state_costs_;

      if (!rollouts_[r].use_total_smoothness_cost_) {
        rollouts_[r].total_costs_[d] += rollouts_[r].control_costs_[d];
        // cout << "set control cost" << endl;
      }
      rollouts_[r].cumulative_costs_[d] = rollouts_[r].total_costs_[d];

      if (use_cumulative_costs_) {
        for (int t = num_time_steps_ - 2; t >= 0; --t) {
          rollouts_[r].cumulative_costs_[d](t) +=
              rollouts_[r].cumulative_costs_[d](t + 1);
        }
      }
    }
  }
  return true;
}

bool PolicyImprovement::computeRolloutProbabilities() {

  for (int d = 0; d < num_dimensions_; ++d) {
    for (int t = 0; t < num_time_steps_; t++) {
      // find min and max cost over all rollouts:
      double min_cost = std::numeric_limits<double>::max();
      double max_cost = std::numeric_limits<double>::min(); // WARNING this is 0
      for (int r = 0; r < num_rollouts_; ++r) {
        if (!rollouts_[r].out_of_bounds_) {  // Only sum over valid samples
          double c = rollouts_[r].cumulative_costs_[d](t);
          if (c < min_cost) min_cost = c;
          if (c > max_cost) max_cost = c;
        }
      }

      double denom = max_cost - min_cost;

      // prevent divide by zero:
      if (denom < 1e-8) denom = 1e-8;

      double p_sum = 0.0;
      for (int r = 0; r < num_rollouts_; ++r) {
        if (!rollouts_[r].out_of_bounds_) {  // Only sum over valid samples
          // the -10.0 here is taken from the paper
          rollouts_[r].probabilities_[d](t) = exp(
              -10.0 * (rollouts_[r].cumulative_costs_[d](t)-min_cost) / denom);

          p_sum += rollouts_[r].probabilities_[d](t);
        }
      }
      for (int r = 0; r < num_rollouts_; ++r) {
        rollouts_[r].probabilities_[d](t) /= p_sum;
      }
    }
  }
  return true;
}

void PolicyImprovement::resampleUpdates() {
  shared_ptr<StompOptimizer> optimizer =
      static_pointer_cast<StompOptimizer>(task_);

  const std::vector<ChompDof>& joints =
      optimizer->getPlanningGroup()->chomp_dofs_;

  // New trajectory and parameters trajectory
  Move3D::Trajectory traj(
      static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup()->robot_);
  Eigen::MatrixXd parameters(num_dimensions_, num_time_steps_);

  for (int d = 0; d < num_dimensions_; ++d) {
    parameters.row(d) = parameter_updates_[d].row(0).transpose();
  }

  traj.clear();
  for (int j = 0; j < num_time_steps_; ++j) {
    confPtr_t q = optimizer->getPlanningGroup()->robot_->getCurrentPos();

    for (int i = 0; i < optimizer->getPlanningGroup()->num_dofs_; ++i) {
      (*q)[joints[i].move3d_dof_index_] = parameters(i, j);
    }
    traj.push_back(q);
  }

  double step = traj.getParamMax() / num_time_steps_;
  double param = step;
  for (int j = 1; j < num_time_steps_ - 1; ++j) {
    confPtr_t q = traj.configAtParam(param);

    for (int i = 0; i < optimizer->getPlanningGroup()->num_dofs_; ++i) {
      parameters(i, j) = (*q)[joints[i].move3d_dof_index_];
    }
    param += step;
  }

  for (int d = 0; d < num_dimensions_; ++d) {
    parameter_updates_[d].row(0).transpose() = parameters.row(d);
  }
}

bool PolicyImprovement::computeParameterUpdates() {
  // MatrixXd noise_update = MatrixXd(num_time_steps_, num_dimensions_);

  std::vector<Eigen::VectorXd> parameters;

  const bool count_out_of_joint_limits = false;
  const bool draw_update = false;
  int nb_samples_in_joint_limits = 0;

  for (int d = 0; d < num_dimensions_; ++d) {
    parameter_updates_[d] = MatrixXd::Zero(num_time_steps_, num_parameters_[d]);
  }

  for (int r = 0; r < num_rollouts_; ++r) {
    if (!rollouts_[r].out_of_bounds_) {
      for (int d = 0; d < num_dimensions_; ++d) {
        if (!is_finite(rollouts_[r].probabilities_[d])) {
          cout << "rollout update is not finite [" << r << "][" << d << "]"
               << endl;
        }
        parameter_updates_[d].row(0).transpose() +=
            rollouts_[r].noise_[d].cwiseProduct(rollouts_[r].probabilities_[d]);
      }
    } else {
      nb_samples_in_joint_limits++;
      cout << "joint limits out of bounds for sample : " << r << endl;
    }
  }
  if (count_out_of_joint_limits) {
    cout << "nb of samples in joint limits : " << nb_samples_in_joint_limits
         << endl;
  }

  for (int d = 0; d < num_dimensions_; ++d) {
    if (draw_update) {
      parameters.push_back(parameter_updates_[d].row(0).transpose() +
                           parameters_[d]);
    }

    // cout << "parameter_updates_[" << d << "].row(0).transpose() = " << endl
    // << parameter_updates_[d].row(0).transpose() << endl;
    // This is the multiplication by M
    // const double eta = 0.000002;
    const double eta = 1.;
    if (use_multiplication_by_m_) {
      parameter_updates_[d].row(0).transpose() =
          eta * projection_matrix_[d] *
          parameter_updates_[d].row(0).transpose();
    }
  }

  if (draw_update) addParmametersToDraw(parameters, 33);  // 33 -> Orange

  // resampleUpdates();
  return true;
}

bool PolicyImprovement::improvePolicy(
    std::vector<Eigen::MatrixXd>& parameter_updates) {
  assert(initialized_);

  computeRolloutCumulativeCosts();
  computeRolloutProbabilities();
  computeParameterUpdates();

  parameter_updates = parameter_updates_;

  if (PlanEnv->getBool(PlanParam::trajStompMatrixAdaptation)) {
    covarianceMatrixAdaptaion();
  }

  // for (int d=0; d<num_dimensions_; d++)
  // {
  //   cout << "parameter_updates[" << d << "] = " << endl <<
  //   parameter_updates_[d] << endl;
  // }
  return true;
}

/// TODO implement it
void PolicyImprovement::addParmametersToDraw(
    const std::vector<Eigen::VectorXd>& parameters, int color) {
  const Move3D::ChompPlanningGroup* planning_group =
      static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup();
  const std::vector<Move3D::ChompDof>& dofs = planning_group->chomp_dofs_;

  Move3D::Trajectory traj(planning_group->robot_);

  for (int j = 0; j < num_time_steps_; ++j) {
    confPtr_t q = traj.getRobot()->getCurrentPos();

    for (int i = 0; i < planning_group->num_dofs_; ++i) {
      (*q)[dofs[i].move3d_dof_index_] = parameters[i][j];
    }
    traj.push_back(q);
  }

  // T.print();
  traj.setColor(color);
  traj.setUseContinuousColors(false);
  // global_trajToDraw.clear();
  global_trajToDraw.push_back(traj);
}

bool PolicyImprovement::covarianceMatrixAdaptaion() {
  // find min and max cost over all rollouts:
  double max_cost = rollout_costs_total_.maxCoeff();
  double min_cost = rollout_costs_total_.minCoeff();
  double denom = max_cost - min_cost;

  // prevent divide by zero
  if (denom < 1e-8) denom = 1e-8;

  // Compute the
  Eigen::VectorXd rollout_probabilities(num_rollouts_);

  for (int r = 0; r < num_rollouts_; ++r) {
    // the -10.0 here is taken from the paper
    rollout_probabilities[r] =
        exp(-10.0 * (rollout_costs_total_[r] - min_cost) / denom);
  }
  rollout_probabilities /= rollout_probabilities.sum();

  // TODO here Switch if only one sigma is needed
  std::vector<MatrixXd> covariance(num_dimensions_);

  int num_vars = rollouts_[0].noise_[0].size();

  for (uint d = 0; d < covariance.size(); ++d) {
    covariance[d] = MatrixXd::Zero(num_vars, num_vars);
    for (int r = 0; r < num_rollouts_; ++r) {
      covariance[d] +=
          rollout_probabilities[r] *
          (1 / (stdev_scaling_[d] * stdev_scaling_[d])) *
          (rollouts_[r].noise_[d] * rollouts_[r].noise_[d].transpose());
    }
  }

  double numer_1 = 0.;
  double denom_1 = 0.;
  for (uint d = 0; d < covariance.size(); ++d) {
    for (int i = 0; i < num_vars; i++) {
      for (int j = 0; j < num_vars; j++) {
        numer_1 += covariance[d](i, j) * inv_control_costs_[d](i, j);
        denom_1 += inv_control_costs_[d](i, j) * inv_control_costs_[d](i, j);
      }
    }
  }

  double stdev = 0.;
  stdev = std::sqrt(numer_1 / denom_1);

  // cout << "stdev : " << stdev << endl;

  for (int d = 0; d < stdev_.size(); ++d) {
    double stdev_prev = stdev_[d] / stdev_scaling_[d];
    stdev_[d] =
        stdev_scaling_[d] *
        std::max(stdev_intieria_ * stdev + (1. - stdev_intieria_) * stdev_prev,
                 stdev_min_[d]);
  }

  use_new_std_dev_ = true;
  // cout << "-- sigmas : " << stdev_.transpose() << endl;
  // cout << "-- stdev_scaling_ : " << stdev_scaling_.transpose() << endl;

  return true;
}

bool PolicyImprovement::addExtraRollouts(
    std::vector<std::vector<Eigen::VectorXd> >& rollouts,
    std::vector<Eigen::VectorXd>& rollout_costs) {
  assert(int(rollouts.size()) == num_rollouts_extra_);

  // update our parameter values, so that the computed noise is correct:
  copyParametersFromPolicy();

  for (int r = 0; r < num_rollouts_extra_; ++r) {
    extra_rollouts_[r].parameters_ = rollouts[r];
    extra_rollouts_[r].state_costs_ = rollout_costs[r];
    computeNoise(extra_rollouts_[r]);
    // computeProjectedNoise(extra_rollouts_[r]);
    // computeRolloutControlCosts(extra_rollouts_[r]);

    // TODO
    // extra_rollouts_[r].rollout.control_costs_ = rollout_control_costs[r];
  }

  extra_rollouts_added_ = true;
  return true;
}

bool PolicyImprovement::computeNoise(Rollout& rollout) {
  for (int d = 0; d < num_dimensions_; ++d) {
    rollout.noise_[d] = rollout.parameters_[d] - parameters_[d];
  }
  return true;
}

void PolicyImprovement::testNoiseGenerators() {
  for (int d = 0; d < num_dimensions_; ++d) {
    // For all dimensions
    const unsigned int N = 10000;

    Eigen::VectorXd sum(VectorXd::Zero(num_parameters_[d]));
    Eigen::MatrixXd samples(N, num_parameters_[d]);

    for (unsigned int i = 0; i < N; ++i) {
      Eigen::VectorXd tmp(VectorXd::Zero(num_parameters_[d]));

      noise_generators_[d].sample(tmp);

      // cout << "sampler = " << tmp.transpose() << endl;
      samples.row(i) = tmp.transpose();
      sum += tmp;
    }

    Eigen::VectorXd mean((1 / ((double)N)) * sum);
    Eigen::MatrixXd covariance(
        MatrixXd::Zero(num_parameters_[d], num_parameters_[d]));

    for (int k = 0; k < num_parameters_[d]; ++k) {
      for (int j = 0; j < num_parameters_[d]; ++j) {
        for (int i = 0; i < int(N); ++i) {
          covariance(j, k) +=
              (samples(i, j) - mean(j)) * (samples(i, k) - mean(k));
        }
      }
    }
    covariance /= N;

    cout << "CoV dimension [" << d << "] = " << endl
         << covariance << endl;
    cout << "Sum dimension [" << d << "] = " << endl
         << sum.transpose() << endl;
    cout << "Mean dimension [" << d << "] = " << endl
         << mean.transpose() << endl;
  }
}
};

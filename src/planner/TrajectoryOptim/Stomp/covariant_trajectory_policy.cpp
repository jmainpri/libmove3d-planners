/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include "covariant_trajectory_policy.hpp"
//#include "param_server.hpp"
////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/LU>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Util-pkg.h"

#include "planner/planEnvironment.hpp"
#include "utils/misc_functions.hpp"
#include "feature_space/smoothness.hpp"

#include <sstream>
#include <stdio.h>
#include <iomanip>
#include <fstream>

using namespace std;
using namespace Move3D;

extern double smoothness_phi_coeff_0;
extern double smoothness_phi_coeff_1;
extern double smoothness_phi_coeff_2;
extern double smoothness_phi_coeff_3;
extern double smoothness_phi_coeff_4;
extern double smoothness_phi_coeff_5;
extern double smoothness_phi_coeff_6;
extern double smoothness_phi_coeff_7;

namespace stomp_motion_planner {

CovariantTrajectoryPolicy::CovariantTrajectoryPolicy() : print_debug_(true) {}

CovariantTrajectoryPolicy::~CovariantTrajectoryPolicy() {}

bool CovariantTrajectoryPolicy::initialize(/*ros::NodeHandle& node_handle*/) {
  // node_handle_ = node_handle;

  assert(readParameters());
  assert(initializeVariables());
  assert(initializeCosts());
  assert(initializeBasisFunctions());

  return true;
}

bool CovariantTrajectoryPolicy::initialize(/*ros::NodeHandle& node_handle,*/
                                           const int num_time_steps,
                                           const int num_dimensions,
                                           const double movement_duration,
                                           const double cost_ridge_factor,
                                           const std::vector<double>&
                                               derivative_costs,
                                           const bool free_end_config,
                                           const ChompPlanningGroup*
                                               planning_group) {
  type_ = acc;

  // node_handle_ = node_handle;
  // print_debug_ = true;

  num_time_steps_ = num_time_steps;
  num_dimensions_ = num_dimensions;

  // Computes a control matrices that allow motion of the end part
  // Should be set to 2 for the normal case
  free_offset_ = free_end_config ? int(num_time_steps_/3) : 0;

  cout << "free_offset_ : " << free_offset_ << endl;

  use_buffer_ = false;

  if (PlanEnv->getBool(PlanParam::useSelectedDuration)) {
    movement_duration_ = PlanEnv->getDouble(PlanParam::trajDuration);
  } else {
    movement_duration_ = movement_duration;
  }

  // movement_duration_ = 5.0;
  // movement_duration_ = movement_duration*7.716;
  // movement_duration_ = movement_duration*1.5432;

  cost_ridge_factor_ = cost_ridge_factor;
  derivative_costs_ = derivative_costs;

  planning_group_ = planning_group;

  initializeVariables();
  initializeCosts();
  initializeBasisFunctions();

  if (print_debug_) {
    cout << "--------------------------------- " << endl;
    cout << "Trajectory duration : " << movement_duration << endl;
    cout << "--------------------------------- " << endl;

    cout << "movement_duration_ : " << movement_duration_ << endl;
    cout << "num_time_steps_  : " << num_time_steps_ << endl;
    cout << "num_dimensions_  : " << num_dimensions_ << endl;
  }

  return true;
}

bool CovariantTrajectoryPolicy::readParameters() {
  //    node_handle_.param("num_time_steps", num_time_steps_, 100);
  //    node_handle_.param("num_dimensions", num_dimensions_, 1);
  //    node_handle_.param("movement_duration", movement_duration_, 1.0);
  //    node_handle_.param("cost_ridge_factor", cost_ridge_factor_, 0.00001);
  num_time_steps_ = 100;
  num_dimensions_ = 1;
  movement_duration_ = 1.0;
  cost_ridge_factor_ = 0.00001;

  if (print_debug_) {
    cout << "Movement duration : " << movement_duration_ << endl;
  }

  // assert(stomp_motion_planner::readDoubleArray(node_handle_,
  // "derivative_costs", derivative_costs_));
  return true;
}

bool CovariantTrajectoryPolicy::fillBufferStartAndGoal() {
  //    if( use_buffer_ ){
  //        cout << "buffer_[i].size() : " << buffer_[0].size() << endl;
  //        cout << "num_dimensions_ : " << num_dimensions_ << endl;
  //        cout << "wait for key" << endl;
  //        cin.ignore();
  //    }

  for (int d = 0; d < num_dimensions_; ++d) {
    // set the start and end of the trajectory
    for (int i = 0; i < DIFF_RULE_LENGTH - 1; ++i) {
      if (use_buffer_) {
        parameters_all_[d](i) = buffer_[i](d);
      } else
        parameters_all_[d](i) = start_(d);

      parameters_all_[d](num_vars_all_ - 1 - i) = goal_(d);
    }
  }

  return true;
}

//! compute the minmal control costs
//! given a start and goal state
bool CovariantTrajectoryPolicy::setToMinControlCost(Eigen::VectorXd& start,
                                                    Eigen::VectorXd& goal) {
  // Store start and goal
  start_ = start;
  goal_ = goal;

  fillBufferStartAndGoal();

  // cerr << "1 : " << endl;
  // printParameters();
  computeLinearControlCosts();

  // cerr << "2 : " << endl;
  // printParameters();
  computeMinControlCostParameters();

  // cerr << "3 : " << endl;
  // printParameters();
  return true;
}

bool CovariantTrajectoryPolicy::computeLinearControlCosts() {
  linear_control_costs_.clear();
  linear_control_costs_.resize(num_dimensions_,
                               Eigen::VectorXd::Zero(num_vars_free_));

  for (int d = 0; d < num_dimensions_; ++d) {
    linear_control_costs_[d].transpose() =

        parameters_all_[d].segment(0, DIFF_RULE_LENGTH - 1).transpose() *

        control_costs_all_[d].block(
            0, free_vars_start_index_, DIFF_RULE_LENGTH - 1, num_vars_free_);

    linear_control_costs_[d].transpose() +=
        parameters_all_[d]
            .segment(

                 free_vars_end_index_ + 1, DIFF_RULE_LENGTH - 1)
            .transpose() *

        control_costs_all_[d].block(free_vars_end_index_ + 1,
                                    free_vars_start_index_,
                                    DIFF_RULE_LENGTH - 1,
                                    num_vars_free_);

    linear_control_costs_[d] *= 2.0;
  }

  return true;
}

bool CovariantTrajectoryPolicy::computeMinControlCostParameters() {
  for (int d = 0; d < num_dimensions_; ++d) {
    parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) =
        -0.5 * inv_control_costs_[d] * linear_control_costs_[d];
  }

  return true;
}

bool CovariantTrajectoryPolicy::initializeVariables() {
  movement_dt_ = movement_duration_ / (num_time_steps_ + 1);

  num_vars_free_ = num_time_steps_;
  num_vars_all_ = num_vars_free_ + 2 * (DIFF_RULE_LENGTH - 1);
  free_vars_start_index_ = DIFF_RULE_LENGTH - 1;
  free_vars_end_index_ = free_vars_start_index_ + num_vars_free_ - 1;

  num_parameters_.clear();
  for (int d = 0; d < num_dimensions_; ++d)
    num_parameters_.push_back(num_time_steps_);

  parameters_all_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_vars_all_));

  return true;
}

void CovariantTrajectoryPolicy::createDifferentiationMatrices() {
  double multiplier = 1.0;
  differentiation_matrices_.clear();
  differentiation_matrices_.resize(
      NUM_DIFF_RULES,
      Eigen::MatrixXd::Zero(num_vars_all_ + free_offset_,
                            num_vars_all_ + free_offset_));

  for (int d = 0; d < NUM_DIFF_RULES; ++d) {
    if (print_debug_) {
      cout << "Movement duration : " << movement_duration_ << endl;
      cout << "Movement dt : " << movement_dt_ << endl;
    }
    // multiplier /= movement_dt_;
    // multiplier /= 0.03815;

    for (int i = 0; i < num_vars_all_ + free_offset_; i++) {
      for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++) {
        int index = i + j;
        if (index < 0) continue;
        if (index >= (num_vars_all_ + free_offset_)) continue;
        differentiation_matrices_[d](i, index) =
            multiplier * DIFF_RULES[d][j + DIFF_RULE_LENGTH / 2];
      }
    }
  }

  //    move3d_save_matrix_to_file( differentiation_matrices_[0],
  //    "matlab/vel_diff_matrix.txt" );
}

bool CovariantTrajectoryPolicy::initializeCosts() {
  createDifferentiationMatrices();

  control_costs_all_.clear();
  control_costs_.clear();
  inv_control_costs_.clear();
  covariances_.clear();

  for (int d = 0; d < num_dimensions_; ++d) {
    // Construct the quadratic cost matrices (for all variables)
    Eigen::MatrixXd cost_all =
        cost_ridge_factor_ *
        Eigen::MatrixXd::Identity(num_vars_all_ + free_offset_,
                                  num_vars_all_ + free_offset_);

    for (int i = 0; i < NUM_DIFF_RULES; ++i)  // NUM_DI...
    {
      // cout << "derivative cost : " << i << " , " << derivative_costs_[i] <<
      // endl;
      cost_all +=
          derivative_costs_[i] * (differentiation_matrices_[i].transpose() *
                                  differentiation_matrices_[i]);
    }
    control_costs_all_.push_back(cost_all);

    // Extract the quadratic cost just for the free variables
    Eigen::MatrixXd cost_free = cost_all.block(DIFF_RULE_LENGTH - 1,
                                               DIFF_RULE_LENGTH - 1,
                                               num_vars_free_,
                                               num_vars_free_);

    Eigen::MatrixXd cost_free2 = cost_all.block(DIFF_RULE_LENGTH - 1,
                                                DIFF_RULE_LENGTH - 1,
                                                num_vars_free_ + free_offset_,
                                                num_vars_free_ + free_offset_);

    //        Eigen::MatrixXd cost_free   = cost_all.block(
    //        2*(DIFF_RULE_LENGTH-1), 2*(DIFF_RULE_LENGTH-1), num_vars_free_,
    //        num_vars_free_ );
    //        Eigen::MatrixXd cost_free2  = cost_all.block(
    //        2*(DIFF_RULE_LENGTH-1), 2*(DIFF_RULE_LENGTH-1), num_vars_free_,
    //        num_vars_free_ );

    // cout << "cost_free("<<d<<") = " << endl << cost_free << endl;

    control_costs_.push_back(cost_free);
    inv_control_costs_.push_back(cost_free.inverse());
    //        covariances_.push_back( cost_free.inverse() );
    covariances_.push_back(
        cost_free2.inverse().block(0, 0, num_vars_free_, num_vars_free_));

    //        move3d_save_matrix_to_file( cost_all, "../matlab/cost_all.txt" );

    //        cout << "control_costs["<< d <<"]  = " << endl <<
    //        control_costs_[d] << endl;
  }

  //    exit(0);

  //    cout << "diff_mat : "  << endl << differentiation_matrices_[1] << endl;
  //    cout << "diff_mat : "  << endl << control_costs_all_[0] << endl;

  //    cout << "diff_mat : "       << differentiation_matrices_[1].rows() << "
  //    , " << differentiation_matrices_[1].cols() << endl;
  //    cout << "cost_all : "       << control_costs_all_[0].rows() << " , " <<
  //    control_costs_all_[0].cols() << endl;
  //    cout << "cost_free : "      << control_costs_[0].rows() << " , " <<
  //    control_costs_[0].cols() << endl;
  //    cout << "cost_free2 : "     << covariances_[0].rows() << " , " <<
  //    covariances_[0].cols() << endl;

  //    move3d_save_matrix_to_file( differentiation_matrices_[1],
  //    "../matlab/diff_mat.txt" );
  //    move3d_save_matrix_to_file( control_costs_all_[0],
  //    "../matlab/control_costs_all.txt" );
  //    move3d_save_matrix_to_file( control_costs_[0], "../matlab/cost_free.txt"
  //    );
  //    move3d_save_matrix_to_file( covariances_[0],
  //    "../matlab/invcost_matrix.txt" );
  return true;
}

bool CovariantTrajectoryPolicy::initializeBasisFunctions() {
  basis_functions_.clear();
  for (int d = 0; d < num_dimensions_; ++d) {
    basis_functions_.push_back(
        Eigen::MatrixXd::Identity(num_vars_free_, num_vars_free_));
  }
  return true;
}

/**
bool CovariantTrajectoryPolicy::computeControlCosts(const
std::vector<Eigen::MatrixXd>& control_cost_matrices,
                                                    const
std::vector<Eigen::VectorXd>& parameters,
                                                    const
std::vector<Eigen::VectorXd>& noise,
                                                    const double weight,
std::vector<Eigen::VectorXd>& control_costs)
{
    // this measures the accelerations and squares them
    for (int d=0; d<num_dimensions_; ++d)
    {
        Eigen::VectorXd params_all = parameters_all_[d];
        Eigen::VectorXd costs_all  = Eigen::VectorXd::Zero(num_vars_all_);
        Eigen::VectorXd acc_all    = Eigen::VectorXd::Zero(num_vars_all_);

        params_all.segment( free_vars_start_index_, num_vars_free_) =
parameters[d] + noise[d];


        for (int i=0; i<NUM_DIFF_RULES; ++i)
        {
            acc_all = differentiation_matrices_[i]*params_all;
            costs_all += weight * derivative_costs_[i] *
(acc_all.cwise()*acc_all);
        }

        control_costs[d] = costs_all.segment( free_vars_start_index_,
num_vars_free_ );

//        cout << control_costs[d].transpose() << endl;

        // TODO Why this???
        // commented by jim 06/03/2011
        // The control costs are not computed the same way in
        // policy improvement and on stomp optimizer
//        for (int i=0; i<free_vars_start_index_; ++i)
//        {
//            control_costs[d](0) += costs_all(i);
//            control_costs[d](num_vars_free_-1) +=
costs_all(num_vars_all_-(i+1));
//        }
    }

    return true;
}
**/

bool CovariantTrajectoryPolicy::computeControlCosts(
    const std::vector<Eigen::MatrixXd>& control_cost_matrices,
    const std::vector<Eigen::VectorXd>& parameters,
    const std::vector<Eigen::VectorXd>& noise,
    const double weight,
    std::vector<Eigen::VectorXd>& control_costs,
    double dt,
    bool save_to_file) {
  bool with_time(dt != 0.0);

  fillBufferStartAndGoal();

  // this measures the accelerations and squares them
  for (int d = 0; d < num_dimensions_; ++d) {
    Eigen::VectorXd params_all = parameters_all_[d];
    Eigen::VectorXd costs_all = Eigen::VectorXd::Zero(num_vars_all_);
    Eigen::VectorXd acc_all = Eigen::VectorXd::Zero(num_vars_all_);

    params_all.segment(free_vars_start_index_, num_vars_free_) =
        parameters[d] + noise[d];

    // cout << "params_all : " << params_all.transpose() << endl;

    // Smooth the curve before computing the control cost
    bool is_circular_joint = planning_group_->chomp_dofs_[d].is_circular_;
    if (is_circular_joint) {
      move3d_smooth_circular_parameters(params_all);
    }

    double factor = 0;
    int offset = 2;  // TRO=2, TODO what does that do when setting buffer...

    switch (type_) {
      case dist:
        factor = 1.;
        break;
      case vel:
        factor = 2.;
        break;
      case acc:
        factor = 3.;
        break;
      case jerk:
        factor = 6.;
        break;
      case mix:
        factor = 1.;
        break;
    }

    for (int i = 0; i < num_vars_all_; i++) {
      if (type_ == dist) {
        if ((i >= free_vars_start_index_) && (i <= free_vars_end_index_)) {
          acc_all[i] = std::pow(params_all[i] - params_all[i + 1], 2.);
        }
      } else {
        for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++) {
          int index = i + j;

          if (index < 0) continue;
          if (index >= num_vars_all_) continue;

          if (is_circular_joint && (i < num_vars_all_ - 1)) {
            double diff = params_all[i] - params_all[i + 1];

            if (std::fabs(diff_angle(params_all[i + 1], params_all[i]) - diff) >
                1e-6) {
              cout << "control cost breaks for : "
                   << planning_group_->chomp_dofs_[d].joint_name_ << endl;
            }
          }

          if (type_ == mix) {
            acc_all[i] += std::sqrt(derivative_costs_[0]) *
                          ((params_all[index] *
                            DIFF_RULES[(int)vel][j + DIFF_RULE_LENGTH / 2]) /
                           (with_time ? std::pow(dt, double(1)) : 1.0));
            acc_all[i] += std::sqrt(derivative_costs_[1]) *
                          ((params_all[index] *
                            DIFF_RULES[(int)acc][j + DIFF_RULE_LENGTH / 2]) /
                           (with_time ? std::pow(dt, double(2)) : 1.0));
            acc_all[i] += std::sqrt(derivative_costs_[2]) *
                          ((params_all[index] *
                            DIFF_RULES[(int)jerk][j + DIFF_RULE_LENGTH / 2]) /
                           (with_time ? std::pow(dt, double(3)) : 1.0));
          } else {
            acc_all[i] += (params_all[index] *
                           DIFF_RULES[(int)type_][j + DIFF_RULE_LENGTH / 2]);
            // cout << "type : " << type_ << endl;
          }
        }

        if (type_ != mix && with_time) {
          // Devide by time step
          // acc_all[i] /= std::pow( dt, double(type_+1) );
          double beta = (factor * std::pow(dt, double(int(type_) + 1)));
          acc_all[i] /= beta;
        }
      }
    }

    //        cout << "TYPE : " << type_ << " , dt : " << dt << endl;
    //        cout << "dt : " << dt << endl;
    //        cout << "acc_all : " << acc_all.transpose() << endl;

    if (type_ == dist)
      costs_all = acc_all;
    else
      costs_all = (acc_all.cwiseProduct(acc_all));

    costs_all *= weight;

    control_costs[d] = costs_all.segment(free_vars_start_index_ + offset,
                                         num_vars_free_ - 2 * offset);

    if (save_to_file) {
      std::stringstream ss;
      ss.str("");
      ss << "control_costs_";
      ss << std::setw(3) << std::setfill('0') << d << ".csv";

      move3d_save_matrix_to_csv_file(control_costs[d], ss.str());
    }

    //        if( type_ == acc )
    //            for (int d=0; d<num_dimensions_; ++d) {
    //                control_costs[d] *= 1e-3;
    //            }

    //        if( !PlanEnv->getBool(PlanParam::trajStompNoPrint) )
    //            if( type_ == vel )
    //            {
    //                cout << "control param for joint name : " <<
    //                planning_group_->chomp_joints_[d].joint_name_ << endl;
    //                cout << "control_costs[d].size() : " <<
    //                control_costs[d].size() << endl;
    //                cout << "free_vars_start_index_ : " <<
    //                free_vars_start_index_ << endl;
    //                cout << "free_vars_end_index_ : " << free_vars_end_index_
    //                << endl;
    //                cout << "params_all : " << params_all.transpose() << endl;
    //                cout << "acc_all : "  << acc_all.transpose() << endl;
    //                cout << "costs_all : "  << costs_all.transpose() << endl;
    //            }

    //        control_costs[d][0] = 0.0;
    //        control_costs[d].tail(1) = Eigen::VectorXd::Zero( 1 );

    //        cout.precision(2);
    //        cout << "params_all.transpose() [" << d <<  "] =" <<
    //        params_all.transpose() << endl;
    //        cout << "acc_all.transpose() [" << d <<  "] =" <<
    //        acc_all.transpose() << endl;
    //        cout << "control_costs.transpose() [" << d <<  "] =" <<
    //        control_costs[d].transpose() << endl;
  }

  //    if( type_ == dist ) // TODO FINISH
  //    {
  //        for (int i=free_vars_start_index_; i<= free_vars_end_index_; i++)
  //        {
  //            double tmp = 0.0;

  //            for ( int d=0; d<control_costs.size(); ++d )
  //                tmp += control_costs[d][i];

  //            for ( int d=0; d<control_costs.size(); ++d ) // Distances are
  //            the same for each state
  //                control_costs[d][i] = tmp; // Sum of squarred lengths
  //        }
  //    }
  return true;
}

bool CovariantTrajectoryPolicy::computeControlCosts(
    const std::vector<Eigen::MatrixXd>& control_cost_matrices,
    const std::vector<std::vector<Eigen::VectorXd> >& parameters,
    const double weight,
    std::vector<Eigen::VectorXd>& control_costs) {
  throw string("error");
  return true;
}

/**
bool CovariantTrajectoryPolicy::computeControlCosts(const
std::vector<Eigen::MatrixXd>& control_cost_matrices,
                                                    const
std::vector<std::vector<Eigen::VectorXd> >& parameters,
                                                    const double weight,
std::vector<Eigen::VectorXd>& control_costs)
{
    //Policy::computeControlCosts(control_cost_matrices, parameters, weight,
control_costs);

    // we use the locally stored control costs

    // this uses the already squared control cost matrix
//    for (int d=0; d<num_dimensions_; ++d)
//     {
//     control_costs[d] = VectorXd::Zero(num_time_steps_);
//     VectorXd params_all = parameters_all_[d];
//     for (int t=0; t<num_time_steps_; ++t)
//     {
//     params_all.segment(free_vars_start_index_, num_vars_free_) =
parameters[d][t];
//     VectorXd r_times_u = control_costs_all_[d] * params_all;
//     control_costs[d] += weight * (r_times_u.segment(free_vars_start_index_,
num_vars_free_).cwise() * parameters[d][t]);
//     }
//     }


    // this measures the accelerations and squares them
    for (int d=0; d<num_dimensions_; ++d)
    {
        Eigen::VectorXd params_all = parameters_all_[d];
        Eigen::VectorXd costs_all = Eigen::VectorXd::Zero(num_vars_all_);
        for (int t=0; t<num_time_steps_; ++t)
        {
            params_all.segment(free_vars_start_index_, num_vars_free_) =
parameters[d][t];
            Eigen::VectorXd acc_all = Eigen::VectorXd::Zero(num_vars_all_);
            for (int i=0; i<NUM_DIFF_RULES; ++i)
            {
                acc_all = differentiation_matrices_[i]*params_all;
                costs_all += weight * derivative_costs_[i] *
(acc_all.cwise()*acc_all);
            }
        }

        control_costs[d] = costs_all.segment(free_vars_start_index_,
num_vars_free_);

        for (int i=0; i<free_vars_start_index_; ++i)
        {
            control_costs[d](0) += costs_all(i);
            control_costs[d](num_vars_free_-1) +=
costs_all(num_vars_all_-(i+1));
        }
    }

    return true;
}
**/

bool CovariantTrajectoryPolicy::updateParameters(
    const std::vector<Eigen::MatrixXd>& updates) {
  // ROS_ASSERT(int(updates.size()) == num_dimensions_);

  // this takes only the diagonal elements
  /*for (int d=0; d<num_dimensions_; ++d)
   {
   parameters_all_[d].segment(free_vars_start_index_, num_vars_free_) +=
   updates[d].diagonal();
   }*/

  // this averages all the updates
  // double divisor = 1.0 / num_vars_free_;
  double divisor = 1.0;
  for (int d = 0; d < num_dimensions_; ++d) {
    parameters_all_[d]
        .segment(free_vars_start_index_, num_vars_free_)
        .transpose() += divisor * updates[d].row(0);

    // cout << "parameters_all_[" << d << "] = " << endl << parameters_all_[d]
    // << endl;
  }

  // this weights updates by number of time-steps remaining:
  //    for (int d=0; d<num_dimensions_; ++d)
  //    {
  //        double weight=0.0;
  //        double weight_sum=0.0;
  //        Eigen::VectorXd update = Eigen::VectorXd::Zero(num_vars_free_);
  //        for (int t=0; t<num_time_steps_; ++t)
  //        {
  //            weight = double(num_time_steps_ - t);
  //            weight_sum += weight;
  //            update.transpose() += updates[d].row(t) * weight;
  //            //ROS_INFO_STREAM("Update at time " << t << " = " <<
  //            updates[d].row(t));
  //        }
  //        parameters_all_[d].segment(free_vars_start_index_, num_vars_free_)
  //        += (1.0/weight_sum)*update;
  //    }

  return true;
}

Eigen::VectorXd CovariantTrajectoryPolicy::getAllCosts(
    const std::vector<Eigen::VectorXd>& parameters,
    std::vector<std::vector<Eigen::VectorXd> >& control_costs,
    double dt) {
  const int nb_costs = 8;

  std::vector<Eigen::MatrixXd> control_cost_matrices;
  std::vector<Eigen::VectorXd> noise(parameters.size());
  std::vector<Eigen::VectorXd> control_costs_tmp(parameters.size());
  Eigen::VectorXd costs(Eigen::VectorXd::Zero(nb_costs));

  control_costs.resize(nb_costs);
  for (size_t i = 0; i < control_costs.size(); ++i) {
    control_costs[i].resize(parameters.size());
    for (size_t d = 0; d < control_costs[i].size(); ++d)
      control_costs[i][d] = Eigen::VectorXd::Zero(parameters[d].size());
  }

  for (size_t d = 0; d < parameters.size(); ++d) {
    noise[d] = Eigen::VectorXd::Zero(parameters[d].size());
    //        control_costs[0][d] = Eigen::VectorXd::Zero( parameters[d].size()
    //        );
    //        control_costs[1][d] = Eigen::VectorXd::Zero( parameters[d].size()
    //        );
    //        control_costs[2][d] = Eigen::VectorXd::Zero( parameters[d].size()
    //        );
  }

  const double factor_dist = smoothness_phi_coeff_0;
  const double factor_vel = smoothness_phi_coeff_1;
  const double factor_acc = smoothness_phi_coeff_2;
  const double factor_jerk = smoothness_phi_coeff_3;

  Move3D::StackedFeatures* fct =
      dynamic_cast<StackedFeatures*>(global_activeFeatureFunction);

  if (fct != NULL && fct->getFeatureFunction("SmoothnessAll") != NULL) {
    Move3D::Trajectory traj(planning_group_->robot_);
    setGroupTrajectoryToMove3DTraj(traj, parameters, dt);

    const double factor_task_dist = smoothness_phi_coeff_4;
    const double factor_task_vel = smoothness_phi_coeff_5;
    const double factor_task_acc = smoothness_phi_coeff_6;
    const double factor_task_jerk = smoothness_phi_coeff_7;

    SmoothnessFeature* smoothness = static_cast<SmoothnessFeature*>(
        fct->getFeatureFunction("SmoothnessAll"));

    LengthFeature& length = smoothness->length_;
    VelocitySmoothness& velocity = smoothness->velocity_;
    AccelerationSmoothness& accel = smoothness->acceleration_;
    JerkSmoothness& jerk = smoothness->jerk_;

    int left_padding = velocity.get_left_padding() + 1;
    int right_padding = velocity.get_right_padding() + 1;

    int size_of_traj = control_costs[0][0].size();

    Eigen::MatrixXd traj_smooth = length.getSmoothedTrajectory(traj);

    double dist = length.getControlCosts(traj_smooth, control_costs_tmp);

    for (size_t d = 0; d < parameters.size(); ++d) {
      control_costs[0][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_dist * control_costs_tmp[d];
      //            cout << "control_costs[0][d].segment( left_padding,
      //            size_of_traj - left_padding - right_padding ).size() : "
      //                 <<  control_costs[0][d].segment( left_padding,
      //                 size_of_traj - left_padding - right_padding ).size()
      //                 << endl;
      //            cout << "control_costs_tmp[d].size() : " <<
      //            control_costs_tmp[d].size() << endl;
    }
    costs[0] = dist * factor_dist;

    double vel = velocity.getControlCosts(traj_smooth, control_costs_tmp, dt);

    for (size_t d = 0; d < parameters.size(); ++d) {
      control_costs[1][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_vel * control_costs_tmp[d];
    }
    costs[1] = vel * factor_vel;

    double acc = accel.getControlCosts(traj_smooth, control_costs_tmp, dt);

    for (size_t d = 0; d < parameters.size(); ++d)
      control_costs[2][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_acc * control_costs_tmp[d];
    costs[2] = acc * factor_acc;

    double jer = jerk.getControlCosts(traj_smooth, control_costs_tmp, dt);

    for (size_t d = 0; d < parameters.size(); ++d)
      control_costs[3][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_jerk * control_costs_tmp[d];
    costs[3] = jer * factor_jerk;

    //-------------------------------------------------------------------------

    TaskSmoothnessFeature& task = smoothness->task_features_;

    double cost_t;
    Eigen::VectorXd control_costs_t;

    Eigen::MatrixXd traj_task = task.getTaskTrajectory(traj);

    // DIST
    cost_t = task.getDist(traj, control_costs_t);
    costs[4] = factor_task_dist * cost_t;

    // The task space distance is computed over the entire trajectory (no
    // buffer)
    //.segment( left_padding, size_of_traj - left_padding - right_padding )
    for (size_t d = 0; d < control_costs[4].size(); ++d)
      control_costs[4][d] = factor_task_dist * control_costs_t;

    // VEL
    cost_t = task.getVelocity(traj_task, control_costs_t, dt);
    costs[5] = factor_task_vel * cost_t;

    for (size_t d = 0; d < control_costs[5].size(); ++d) {
      control_costs[5][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_task_vel * control_costs_t;
    }

    // ACC
    cost_t = task.getAcceleration(traj_task, control_costs_t, dt);
    costs[6] = factor_task_acc * cost_t;

    for (size_t d = 0; d < control_costs[6].size(); ++d)
      control_costs[6][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_task_acc * control_costs_t;

    // JERK
    cost_t = task.getJerk(traj_task, control_costs_t, dt);
    costs[7] = factor_task_jerk * cost_t;

    for (size_t d = 0; d < control_costs[7].size(); ++d)
      control_costs[7][d].segment(left_padding,
                                  size_of_traj - left_padding - right_padding) =
          factor_task_jerk * control_costs_t;

    //        cout << "GET ALL CONTROL COSTS" << endl;
  } else {
    cost_type type_tmp = type_;

    //    Move3D::Trajectory traj( planning_group_->robot_ );
    //    setGroupTrajectoryToMove3DTraj( traj, parameters, dt );

    type_ = dist;
    computeControlCosts(
        control_cost_matrices, parameters, noise, 1.0, control_costs_tmp, dt);
    for (size_t d = 0; d < parameters.size(); ++d) {
      costs[0] += control_costs_tmp[d].sum();
      control_costs[0][d] = control_costs_tmp[d];
    }
    costs[0] *= factor_dist;

    type_ = vel;
    computeControlCosts(
        control_cost_matrices, parameters, noise, 1.0, control_costs_tmp, dt);
    for (size_t d = 0; d < parameters.size(); ++d) {
      costs[1] += control_costs_tmp[d].sum();
      control_costs[1][d] = (factor_vel * control_costs_tmp[d]);
    }
    costs[1] *= factor_vel;

    type_ = acc;
    computeControlCosts(
        control_cost_matrices, parameters, noise, 1.0, control_costs_tmp, dt);
    for (size_t d = 0; d < parameters.size(); ++d) {
      costs[2] += control_costs_tmp[d].sum();
      control_costs[2][d] = (factor_acc * control_costs_tmp[d]);
    }
    costs[2] *= factor_acc;

    type_ = jerk;
    computeControlCosts(
        control_cost_matrices, parameters, noise, 1.0, control_costs_tmp, dt);
    for (size_t d = 0; d < parameters.size(); ++d) {
      costs[3] += control_costs_tmp[d].sum();
      control_costs[3][d] = (factor_jerk * control_costs_tmp[d]);
    }
    costs[3] *= factor_jerk;

    type_ = type_tmp;
  }

  return costs;
}

void CovariantTrajectoryPolicy::saveProfiles(
    const std::vector<Eigen::VectorXd>& parameters,
    std::string foldername,
    double dt) {
  std::vector<Eigen::MatrixXd> control_cost_matrices;
  std::vector<Eigen::VectorXd> noise(parameters.size());
  std::vector<Eigen::VectorXd> control_costs(parameters.size());

  for (size_t d = 0; d < parameters.size(); ++d) {
    noise[d] = Eigen::VectorXd::Zero(parameters[d].size());
  }

  cost_type type_tmp = type_;

  foldername += "/";

  type_ = vel;
  computeControlCosts(
      control_cost_matrices, parameters, noise, 1.0, control_costs, dt);
  for (size_t d = 0; d < parameters.size(); ++d) {
    std::stringstream ss;
    ss << "stomp_vel_" << std::setw(3) << std::setfill('0') << d << ".csv";
    move3d_save_matrix_to_file(control_costs[d], foldername + ss.str());
  }

  type_ = acc;
  computeControlCosts(
      control_cost_matrices, parameters, noise, 1.0, control_costs, dt);
  for (size_t d = 0; d < parameters.size(); ++d) {
    std::stringstream ss;
    ss << "stomp_acc_" << std::setw(3) << std::setfill('0') << d << ".csv";
    move3d_save_matrix_to_file(control_costs[d], foldername + ss.str());
  }

  type_ = jerk;
  computeControlCosts(
      control_cost_matrices, parameters, noise, 1.0, control_costs, dt);
  for (size_t d = 0; d < parameters.size(); ++d) {
    std::stringstream ss;
    ss << "stomp_jerk_" << std::setw(3) << std::setfill('0') << d << ".csv";
    move3d_save_matrix_to_file(control_costs[d], foldername + ss.str());
  }

  type_ = type_tmp;
}

void CovariantTrajectoryPolicy::setGroupTrajectoryToMove3DTraj(
    Move3D::Trajectory& traj,
    const std::vector<Eigen::VectorXd>& parameters,
    double dt) {
  if (parameters.empty()) {
    cout << "Error in parameter empty " << endl;
    //        exit(0);
  }

  //    int start = free_vars_start_;
  //    int end = free_vars_end_;
  //    if (iteration_==0) {
  //        start = 0;
  //        end = num_vars_all_-1;
  //    }

  traj.clear();

  //    std::vector< Eigen::VectorXd> params_all(num_dimensions_);

  //    for( int d=0; d<num_dimensions_; ++d )
  //    {
  //        params_all[d] = parameters_all_[d];
  //        params_all[d].segment( free_vars_start_index_, num_vars_free_ ) =
  //        parameters[d];
  //    }

  // Get the map from move3d index to group trajectory
  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  confPtr_t q(new Configuration(*planning_group_->robot_->getCurrentPos()));

  if (dt != 0.0) {
    traj.setUseTimeParameter(true);
    traj.setUseConstantTime(true);
    traj.setDeltaTime(dt);  // WARNING here use detla time
  }

  //        cout << "parameters[d].transpose() : " << parameters[d].transpose()
  //        << endl;

  for (int i = 0; i < parameters[0].size();
       ++i)  // Because we add source and target we start later
  {
    for (int d = 0; d < num_dimensions_; ++d) {
      (*q)[joints[d].move3d_dof_index_] = parameters[d][i];
    }

    traj.push_back(confPtr_t(new Configuration(*q)));
  }

  //    cout << "traj size : " << traj.size() << endl;
}

bool CovariantTrajectoryPolicy::readFromDisc(const std::string directory_name,
                                             const int item_id,
                                             const int trial_id) {
  // TODO: implement this
  return true;
}

bool CovariantTrajectoryPolicy::writeToDisc(const int trial_id) {
  writeToDisc(getFileName(trial_id));
  return true;
}

bool CovariantTrajectoryPolicy::readFromDisc(const std::string abs_file_name) {
  // TODO: implement this
  return true;
}

bool CovariantTrajectoryPolicy::writeToDisc(const std::string abs_file_name) {
  FILE* f;
  f = fopen(abs_file_name.c_str(), "w");
  if (!f) return false;

  for (int i = free_vars_start_index_ - 1; i <= free_vars_end_index_ + 1; ++i) {
    for (int d = 0; d < num_dimensions_; ++d) {
      fprintf(f, "%f\t", parameters_all_[d](i));
    }
    fprintf(f, "\n");
  }

  fclose(f);
  return true;
}

std::string CovariantTrajectoryPolicy::getFileName(const int trial_id) {
  std::ostringstream ss;
  ss << file_name_base_ << trial_id << ".txt";
  return ss.str();
}
}

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

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/LU>
#include <Eigen/Array>

// local includes
//#include <ros/ros.h>
#include "policy_improvement.hpp"
//#include <stomp_motion_planner/assert.h>
#include <algorithm>
#include <iostream>

#include "API/Trajectory/trajectory.hpp"
#include "stompOptimizer.hpp"

std::vector<double> global_noiseTrajectory1;
std::vector<double> global_noiseTrajectory2;

using namespace std;
using namespace boost;

USING_PART_OF_NAMESPACE_EIGEN

namespace stomp_motion_planner
{
  //! Computes the cost the rollout
  //! the function sums the control costs of all dumensions
  double Rollout::getCost()
  {
    double cost = state_costs_.sum();
    int num_dim = control_costs_.size();
    for (int d=0; d<num_dim; ++d)
      cost += control_costs_[d].sum();
    return cost;
  }
  
  void Rollout::printCost()
  {
    cout << "state_costs_ : ";
    for (int i=0; i<state_costs_.size(); i++) {
      cout << state_costs_[i] << " ";
    }
    cout << endl;
    
//    cout << "control_costs_ : ";
//    for (int d=0; d<int(control_costs_.size()); ++d) 
//    {
//      cout << "( "; 
//      for (int i=0; i<int(control_costs_[d].size()); i++) 
//      {
//        cout <<  control_costs_[d][i] ;
//        
//        if(d < int(control_costs_.size()-1))
//          cout << " , ";
//      }
//      cout << ")";
//    }
//    cout << endl;
  }
  
  void Rollout::printProbabilities()
  {
    cout << "state_costs_ : " << endl;
    cout <<  state_costs_.transpose() << endl;
    
    cout << "control_costs_ : " << endl;
    for (int d=0; d<int(control_costs_.size()); ++d) 
    {
      cout <<  control_costs_[d].transpose() << endl;
    }
    
    cout << "cumulative_cost_ : " << endl;
    for (int d=0; d<int(probabilities_.size()); ++d) 
    {
      cout <<  cumulative_costs_[d].transpose() << endl;
    }
    
    cout << "probabilities_ : " << endl;
    for (int d=0; d<int(probabilities_.size()); ++d) 
    {
      cout <<  probabilities_[d].transpose() << endl;
    }
    cout << endl;
  }
  
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  
  PolicyImprovement::PolicyImprovement():
  initialized_(false)
  {
    
  }
  
  PolicyImprovement::~PolicyImprovement()
  {
    
  }
  
  //! Initialize policy improvment
  //! 1 - Allocates a multivariate gaussian sampler
  //! 2 - Computes the projection matrix
  bool PolicyImprovement::initialize(const int num_rollouts, const int num_time_steps, const int num_reused_rollouts,
                                     const int num_extra_rollouts, 
                                     boost::shared_ptr<Policy> policy,
                                     boost::shared_ptr<Task>   task,
                                     bool use_cumulative_costs)
  {
    num_time_steps_ = num_time_steps;
    use_cumulative_costs_ = use_cumulative_costs;
    use_multiplication_by_m_ = false;
    policy_ = policy;
    task_ = task;
    
    policy_->setNumTimeSteps(num_time_steps_);
    policy_->getControlCosts(control_costs_);
    policy_->getNumDimensions(num_dimensions_);
    policy_->getNumParameters(num_parameters_);
    policy_->getBasisFunctions(basis_functions_);
    policy_->getParameters(parameters_);
    
    assert(preAllocateMultivariateGaussianSampler());
    assert(setNumRollouts(num_rollouts, num_reused_rollouts, num_extra_rollouts));
    assert(preAllocateTempVariables());
    assert(preComputeProjectionMatrices());
    
    return (initialized_ = true);
  }
  
  //! Allocates the sampler
  //! also initializes the control cost inverse structure
  bool PolicyImprovement::preAllocateMultivariateGaussianSampler()
  {
    // invert the control costs, initialize noise generators:
    inv_control_costs_.clear();
    noise_generators_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
      //cout << "control_costs_[" << d << "] = " << endl << control_costs_[d] << endl;
      //cout << "inv_control_costs_[" << d << "] = " << endl << control_costs_[d].inverse() << endl;
      inv_control_costs_.push_back(control_costs_[d].inverse());
      MultivariateGaussian mvg(VectorXd::Zero(num_parameters_[d]), inv_control_costs_[d]);
      noise_generators_.push_back(mvg);
    }
    
    return true;
  }
  
  //! Allocates the rollout structures
  //! @param num_rollouts the total number of rollouts
  //! @param num_reused_rollouts the reused number of rollouts
  //! @param num_extra_rollouts the number of extra rollouts
  bool PolicyImprovement::setNumRollouts(const int num_rollouts, const int num_reused_rollouts, const int num_extra_rollouts)
  {
    num_rollouts_ = num_rollouts;
    num_rollouts_reused_ = num_reused_rollouts;
    num_rollouts_extra_ = num_extra_rollouts;
    num_rollouts_gen_ = 0;
    if (num_rollouts_reused_ >= num_rollouts)
    {
      printf("Number of reused rollouts must be strictly less than number of rollouts.");
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
    
    //cout << "num_time_steps_  = " << num_time_steps_ << endl; 
    
    for (int d=0; d<num_dimensions_; ++d)
    {
      //cout << "num_parameters_[" << d << "] = " << num_parameters_[d] << endl;
      rollout.parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.noise_.push_back(VectorXd::Zero(num_parameters_[d]));
      rollout.noise_projected_.push_back(VectorXd::Zero(num_parameters_[d]));
      
      rollout.control_costs_.push_back(VectorXd::Zero(num_time_steps_));
      rollout.total_costs_.push_back(VectorXd::Zero(num_time_steps_));
      rollout.cumulative_costs_.push_back(VectorXd::Zero(num_time_steps_));
      rollout.probabilities_.push_back(VectorXd::Zero(num_time_steps_));
    }
    rollout.state_costs_ = VectorXd::Zero(num_time_steps_);
    rollout.out_of_bounds_ = false;
    
    // duplicate this rollout:
    for (int r=0; r<num_rollouts; ++r)
      rollouts_.push_back(rollout);
    
    for (int r=0; r<num_reused_rollouts; ++r)
      reused_rollouts_.push_back(rollout);
    
    for (int r=0; r<num_extra_rollouts; ++r)
      extra_rollouts_.push_back(rollout);
    
    rollouts_reused_ = false;
    rollouts_reused_next_ = false;
    extra_rollouts_added_ = false;
    rollout_cost_sorter_.reserve(num_rollouts_);
    
    return true;
  }
  
  //! Allocates Temporary Variables
  bool PolicyImprovement::preAllocateTempVariables()
  {
    tmp_noise_.clear();
    tmp_parameters_.clear();
    parameter_updates_.clear();
    for (int d=0; d<num_dimensions_; ++d)
    {
      tmp_noise_.push_back(VectorXd::Zero(num_parameters_[d]));
      tmp_parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
      parameter_updates_.push_back(MatrixXd::Zero(num_time_steps_, num_parameters_[d]));
    }
    tmp_max_cost_ = VectorXd::Zero(num_time_steps_);
    tmp_min_cost_ = VectorXd::Zero(num_time_steps_);
    tmp_sum_rollout_probabilities_ = VectorXd::Zero(num_time_steps_);
    
    return true;
  }
  
  //! Precomputes the projection matrices M 
  //! using the inverse of the control cost matrix
  //! Each column of the 
  bool PolicyImprovement::preComputeProjectionMatrices()
  {
    //  ROS_INFO("Precomputing projection matrices..");
    projection_matrix_.resize(num_dimensions_);
    for (int d=0; d<num_dimensions_; ++d)
    {
      projection_matrix_[d] = inv_control_costs_[d];
      
      for (int p=0; p<num_parameters_[d]; ++p)
      {
        double column_max = inv_control_costs_[d](0,p);
        for (int p2 = 1; p2 < num_parameters_[d]; ++p2)
        {
          if (inv_control_costs_[d](p2,p) > column_max)
            column_max = inv_control_costs_[d](p2,p);
        }
        projection_matrix_[d].col(p) *= (1.0/(num_parameters_[d]*column_max));
      }
      
      //cout << "Projection Matrix = " << endl << projection_matrix_[d] << endl;
    }
    
    //  ROS_INFO("Done precomputing projection matrices.");
    return true;
  }
  
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  bool PolicyImprovement::copyParametersFromPolicy()
  {
    if (!policy_->getParameters(parameters_))
    {
      //        ROS_ERROR("Failed to get policy parametertes.");
      return false;
    }
    return true;
  }
  
  bool PolicyImprovement::computeProjectedNoise(Rollout& rollout)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      rollout.noise_projected_[d] = projection_matrix_[d] * rollout.noise_[d];
      //rollout.parameters_noise_projected_[d] = rollout.parameters_[d] + rollout.noise_projected_[d];
      //cout << "rollout.noise_projected_[" << d << "] = " << rollout.noise_projected_[d].transpose() << endl;
    }
    
    return true;
  }
  
  bool PolicyImprovement::computeProjectedNoise()
  {
    for (int r=0; r<num_rollouts_; ++r)
    {
      computeProjectedNoise(rollouts_[r]);
    }
    return true;
  }
  
  bool PolicyImprovement::generateRollouts(const std::vector<double>& noise_stddev)
  {
    assert(initialized_);
    assert(static_cast<int>(noise_stddev.size()) == num_dimensions_);
    
    // save the latest policy parameters:
    assert(copyParametersFromPolicy());
    
    // we assume here that rollout_parameters_ and rollout_noise_ have already been allocated
    num_rollouts_gen_ = num_rollouts_ - num_rollouts_reused_;
    if (!rollouts_reused_next_)
    {
      num_rollouts_gen_ = num_rollouts_;
      if (num_rollouts_reused_ > 0)
      {
        rollouts_reused_next_ = true;
      }
    }
    else
    {
      // figure out which rollouts to reuse
      rollout_cost_sorter_.clear();
      for (int r=0; r<num_rollouts_; ++r)
      {
        double cost = rollouts_[r].getCost();
        
        // discard out of bounds rollouts
        if (rollouts_[r].out_of_bounds_) {
          cost = std::numeric_limits<double>::max();
        }
        rollout_cost_sorter_.push_back(std::make_pair(cost,r));
      }
      if (extra_rollouts_added_)
      {
        for (int r=0; r<num_rollouts_extra_; ++r)
        {
          double cost = extra_rollouts_[r].getCost();
          rollout_cost_sorter_.push_back(std::make_pair(cost,-r-1));
          // index is -ve if rollout is taken from extra_rollouts
        }
        extra_rollouts_added_ = false;
      }
      std::sort(rollout_cost_sorter_.begin(), rollout_cost_sorter_.end());
      
      // use the best ones: (copy them into reused_rollouts)
      for (int r=0; r<num_rollouts_reused_; ++r)
      {
        double reuse_index = rollout_cost_sorter_[r].second;
        //double reuse_cost = rollout_cost_sorter_[r].first;
        
        //ROS_INFO("Reuse %d, cost = %lf", r, reuse_cost);
        
        if (reuse_index >=0)
          reused_rollouts_[r] = rollouts_[reuse_index];
        else
        {
          //ROS_INFO("Reused noise-less rollout of cost %f", reuse_cost);
          reused_rollouts_[r] = extra_rollouts_[-reuse_index-1];
        }
      }
      // copy them back from reused_rollouts_ into rollouts_
      for (int r=0; r<num_rollouts_reused_; ++r)
      {
        rollouts_[num_rollouts_gen_+r] = reused_rollouts_[r];
        
        // update the noise based on the new parameters:
        for (int d=0; d<num_dimensions_; ++d)
        {
          rollouts_[num_rollouts_gen_+r].noise_[d] = rollouts_[num_rollouts_gen_+r].parameters_[d] - parameters_[d];
        }
      }
      rollouts_reused_ = true;
    }
    
    // generate new rollouts
    for (int d=0; d<num_dimensions_; ++d)
    {
      for (int r=0; r<num_rollouts_gen_; ++r)
      {
        noise_generators_[d].sample(tmp_noise_[d]);
        rollouts_[r].noise_[d] = noise_stddev[d]*tmp_noise_[d];
        rollouts_[r].parameters_[d] = parameters_[d] + rollouts_[r].noise_[d];
        
        // Saves the first rollout for first dimenstion
        if (r == 0 && d == 0) 
        {          
          global_noiseTrajectory1.clear();
          
          for (int i=0; i<rollouts_[r].parameters_[d].size(); i++) 
          {
            global_noiseTrajectory1.push_back(rollouts_[r].noise_[d][i]);
            //global_noiseTrajectory1.push_back(rollouts_[r].parameters_[d][i]);
          }
        }
        // Saves the first rollout for second dimenstion
        if (r == 0 && d == 1) 
        {          
          global_noiseTrajectory2.clear();
          
          for (int i=0; i<rollouts_[r].parameters_[d].size(); i++) 
          {
            global_noiseTrajectory2.push_back(rollouts_[r].noise_[d][i]);
            //global_noiseTrajectory2.push_back(rollouts_[r].parameters_[d][i]);
          }
        }
        //cout << "rollouts_[" << r << "].noise_[" << d << "] = "<< rollouts_[r].noise_[d].transpose() << endl;
        //cout << "rollouts_[" << r << "].parameters_[" << d << "] = "<< rollouts_[r].parameters_[d].transpose() << endl;
      }
    }
    
    return true;
  }
  
  bool PolicyImprovement::getRollouts(std::vector<std::vector<Eigen::VectorXd> >& generated_rollouts, const std::vector<double>& noise_variance, bool get_reused, std::vector<std::vector<Eigen::VectorXd> >& reused_rollouts)
  {
    if (!generateRollouts(noise_variance))
    {
      cout << "Failed to generate rollouts." << endl;
      return false;
    }
    
    generated_rollouts.clear();
    for (int r=0; r<num_rollouts_gen_; ++r)
    {
      generated_rollouts.push_back(rollouts_[r].parameters_);
      
      //  for (int d=0; d<num_dimensions_; d++) 
      //  {
      //     cout << "rollouts_[" << d << "].parameters_ = " << rollouts_[r].noise_[d].transpose() << endl;
      //  }
    }
    
    if( get_reused )
    {
      reused_rollouts.clear();
      
      if (rollouts_reused_)
      {
        for (int r=0; r<num_rollouts_reused_; ++r)
        {
          reused_rollouts.push_back(rollouts_[num_rollouts_gen_+r].parameters_);
//          cout << "resued rollout (" << r << ")" ;
//          cout << " is out of bounds : " << rollouts_[num_rollouts_gen_+r].out_of_bounds_ << endl;
        }
      }
    }
    computeProjectedNoise();
    return true;
  }
  
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  //----------------------------------------------------------------------
  void PolicyImprovement::setRolloutOutOfBounds(int r)
  {
    assert(initialized_);
    
    if( r<0 || r>=num_rollouts_gen_ )
      return;
    
    rollouts_[r].out_of_bounds_ = true;
  }
  
  bool PolicyImprovement::setRolloutCosts(const Eigen::MatrixXd& costs, const double control_cost_weight, std::vector<double>& rollout_costs_total)
  {
    assert(initialized_);
    
    control_cost_weight_ = control_cost_weight;
    // set control costs
    computeRolloutControlCosts();
    
    for (int r=0; r<num_rollouts_gen_; ++r)
    {
      rollouts_[r].state_costs_ = costs.row(r).transpose();
    }
    
    // set the total costs
    rollout_costs_total.resize(num_rollouts_);
    for (int r=0; r<num_rollouts_; ++r)
    {
      rollout_costs_total[r] = rollouts_[r].getCost();
    }
    
    return true;
  }
  
  bool PolicyImprovement::computeRolloutControlCosts(Rollout& rollout)
  {
    policy_->computeControlCosts(control_costs_, rollout.parameters_, rollout.noise_projected_,
                                 0.5*control_cost_weight_, rollout.control_costs_);
    return true;
  }
  
  bool PolicyImprovement::computeRolloutControlCosts()
  {
    for (int r=0; r<num_rollouts_; ++r)
    {
      computeRolloutControlCosts(rollouts_[r]);
    }
    return true;
  }
  
  bool PolicyImprovement::computeRolloutCumulativeCosts()
  {
    // compute cumulative costs at each timestep
    for (int r=0; r<num_rollouts_; ++r)
    {
      for (int d=0; d<num_dimensions_; ++d)
      {
        //cout << "rollouts_[r].total_costs_[d] = " << endl << rollouts_[r].total_costs_[d] << endl;
        //cout << "rollouts_["<<r<<"].state_costs = " << endl << rollouts_[r].state_costs_ << endl;
        //cout << "rollouts_["<<r<<"].control_costs_[d] = " << endl << rollouts_[r].control_costs_[d] << endl;        
        rollouts_[r].total_costs_[d] = rollouts_[r].state_costs_ + rollouts_[r].control_costs_[d];
        rollouts_[r].cumulative_costs_[d] = rollouts_[r].total_costs_[d];
        
        if (use_cumulative_costs_)
        {
          for (int t=num_time_steps_-2; t>=0; --t)
          {
            rollouts_[r].cumulative_costs_[d](t) += rollouts_[r].cumulative_costs_[d](t+1);
          }
        }
      }
    }
    return true;
  }
  
  bool PolicyImprovement::computeRolloutProbabilities()
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      //      cout  << "dimension " << d << ", Cumulative costs " << rollouts_[0].cumulative_costs_[d] << endl;
      //              tmp_min_cost_ = rollout_cumulative_costs_[d].colwise().minCoeff().transpose();
      //              tmp_max_cost_ = rollout_cumulative_costs_[d].colwise().maxCoeff().transpose();
      //              tmp_max_minus_min_cost_ = tmp_max_cost_ - tmp_min_cost_;
      
      for (int t=0; t<num_time_steps_; t++)
      {
        // find min and max cost over all rollouts:
        double min_cost = rollouts_[0].cumulative_costs_[d](t);
        double max_cost = min_cost;
        for (int r=1; r<num_rollouts_; ++r)
        {
          double c = rollouts_[r].cumulative_costs_[d](t);
          if (c < min_cost)
            min_cost = c;
          if (c > max_cost)
            max_cost = c;
        }
        
        double denom = max_cost - min_cost;
        
        // prevent divide by zero:
        if (denom < 1e-8)
          denom = 1e-8;
        
        double p_sum = 0.0;
        for (int r=0; r<num_rollouts_; ++r)
        {
          // the -10.0 here is taken from the paper:
          rollouts_[r].probabilities_[d](t) = exp(-10.0*(rollouts_[r].cumulative_costs_[d](t) - min_cost)/denom);
          p_sum += rollouts_[r].probabilities_[d](t);
        }
        for (int r=0; r<num_rollouts_; ++r)
        {
          rollouts_[r].probabilities_[d](t) /= p_sum;
        }
      }
    }
    
//    for (int r=0; r<num_rollouts_; ++r)
//    {
//      cout << "Rollout nb : " << r << endl;
//      rollouts_[r].printProbabilities();
//    }
    return true;
  }
  
  void PolicyImprovement::resampleUpdates()
  {
    const std::vector<ChompJoint>& joints = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup()->chomp_joints_;
    
    // New trajectory and parameters trajectory
    API::Trajectory traj(optimizer->getPlanningGroup()->robot_);
    Eigen::MatrixXd parameters(num_dimensions_,num_time_steps_);
    
    for ( int d=0; d<num_dimensions_; ++d) {
      parameters.row(d) = parameter_updates_[d].row(0).transpose();
    }
    
    traj.clear();
    for ( int j=0; j<num_time_steps_; ++j)
    {
      confPtr_t q = optimizer->getPlanningGroup()->robot_->getCurrentPos();
      
      for ( int i=0; i<optimizer->getPlanningGroup()->num_joints_; ++i)
      {  
        (*q)[joints[i].move3d_dof_index_] = parameters(i,j);
      }
      traj.push_back(q);
    }
    
    double step = traj.getRangeMax() / num_time_steps_;
    double param = step;
    for ( int j=1; j<num_time_steps_-1; ++j)
    {
      confPtr_t q = traj.configAtParam(param);
      
      for ( int i=0; i<optimizer->getPlanningGroup()->num_joints_; ++i)
      {  
        parameters(i,j) = (*q)[joints[i].move3d_dof_index_];
      }
      param += step;
    }
    
    for ( int d=0; d<num_dimensions_; ++d) {
      parameter_updates_[d].row(0).transpose() = parameters.row(d);
    }
  }
  
  bool PolicyImprovement::computeParameterUpdates()
  {
    //MatrixXd noise_update = MatrixXd(num_time_steps_, num_dimensions_);
    
    for (int d=0; d<num_dimensions_; ++d)
    {
      parameter_updates_[d] = MatrixXd::Zero(num_time_steps_, num_parameters_[d]);
      
      for (int r=0; r<num_rollouts_; ++r)
      {
        //cout << "rollouts_[" << r << "].probabilities_[" << d << "] = " << endl << rollouts_[r].probabilities_[d] << endl;
        //cout << "rollouts_[" << r << "].noise_[" << d << "] = " << endl << rollouts_[r].noise_[d] << endl;
        //cout << "parameter_updates_[" << d << "].row(0).transpose() = ";
        //cout << endl << parameter_updates_[d].row(0).transpose() << endl;
        
        parameter_updates_[d].row(0).transpose() += rollouts_[r].noise_[d].cwise() * rollouts_[r].probabilities_[d];
      }
      // cout << "parameter_updates_[" << d << "].row(0).transpose() = " << endl << parameter_updates_[d].row(0).transpose() << endl;
      // This is the multiplication by M
      if( use_multiplication_by_m_ ) 
      {
        parameter_updates_[d].row(0).transpose() = projection_matrix_[d]*parameter_updates_[d].row(0).transpose();
      }
    }
    
    //resampleUpdates();
    return true;
  }
  
  bool PolicyImprovement::improvePolicy(std::vector<Eigen::MatrixXd>& parameter_updates)
  {
    assert(initialized_);
    
    computeRolloutCumulativeCosts();
    computeRolloutProbabilities();
    computeParameterUpdates();

    parameter_updates = parameter_updates_;
    
    // for (int d=0; d<num_dimensions_; d++) 
    // {
    //   cout << "parameter_updates[" << d << "] = " << endl << parameter_updates_[d] << endl;
    // }
    return true;
  }
  
  bool PolicyImprovement::addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rollout_costs)
  {
    assert(int(rollouts.size()) == num_rollouts_extra_);
    
    // update our parameter values, so that the computed noise is correct:
    assert(copyParametersFromPolicy());
    
    for (int r=0; r<num_rollouts_extra_; ++r)
    {
      extra_rollouts_[r].parameters_ = rollouts[r];
      extra_rollouts_[r].state_costs_ = rollout_costs[r];
      computeNoise(extra_rollouts_[r]);
      computeProjectedNoise(extra_rollouts_[r]);
      // set control cost
      computeRolloutControlCosts(extra_rollouts_[r]);
      //ROS_INFO("Extra rollout cost = %f", extra_rollouts_[r].getCost());
    }
    
    extra_rollouts_added_ = true;
    return true;
  }
  
  bool PolicyImprovement::computeNoise(Rollout& rollout)
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      rollout.noise_[d] =  rollout.parameters_[d] - parameters_[d];
    }
    return true;
  }
  
  void PolicyImprovement::testNoiseGenerators()
  {
    for (int d=0; d<num_dimensions_; ++d)
    {
      // For all dimensions
      const unsigned int N = 10000;
      
      Eigen::VectorXd sum(VectorXd::Zero(num_parameters_[d]));
      Eigen::MatrixXd samples(N,num_parameters_[d]);
      
      for (unsigned int i=0; i<N; ++i) 
      {
        Eigen::VectorXd tmp(VectorXd::Zero(num_parameters_[d]));
        
        noise_generators_[d].sample( tmp );
        
        //cout << "sampler = " << tmp.transpose() << endl;
        samples.row(i) = tmp.transpose();
        sum += tmp;
      }
      
      Eigen::VectorXd mean((1/((double)N))*sum);
      Eigen::MatrixXd covariance(MatrixXd::Zero(num_parameters_[d],num_parameters_[d]));
      
      for ( int k=0; k<num_parameters_[d]; ++k) 
      {
        for ( int j=0; j<num_parameters_[d]; ++j) 
        {
          for ( int i=0; i<int(N); ++i) 
          {
            covariance(j,k) += ( samples(i,j) - mean(j) )*( samples(i,k) - mean(k) ) ;
          }
        }
      }
      covariance /= N;
      
      cout << "CoV dimension [" << d << "] = " << endl << covariance << endl;
      cout << "Sum dimension [" << d << "] = " << endl << sum.transpose() << endl;
      cout << "Mean dimension [" << d << "] = " << endl << mean.transpose() << endl;
    }
  }
};




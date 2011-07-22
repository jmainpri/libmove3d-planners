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

// system includes
#include <cassert>
#include <stdio.h>
// ros includes
//#include <ros/package.h>
#include "policy_improvement_loop.hpp"
//#include <stomp_motion_planner/assert.h>
//#include "param_server.hpp"
//#include <boost/filesystem.hpp>

using namespace std;

namespace stomp_motion_planner
{
  const std::string PI_STATISTICS_TOPIC_NAME = std::string("policy_improvement_statistics");
  
  PolicyImprovementLoop::PolicyImprovementLoop()
  : initialized_(false), policy_iteration_counter_(0)
  {
  }
  
  PolicyImprovementLoop::~PolicyImprovementLoop()
  {
  }
  
  /*
   bool PolicyImprovementLoop::initializeAndRunTaskByName(ros::NodeHandle& node_handle, std::string& task_name)
   {
   // load the task
   //    TaskManager task_manager;
   //    assert(task_manager.initialize());
   
   boost::shared_ptr<Task> task;
   assert(task_manager.getTaskByName(task_name, task));
   
   // initialize the PI loop
   assert(initialize(task));
   
   int first_trial, last_trial;
   
   // first_trial defaults to 1:
   //node_handle.param("first_trial", first_trial, 1);
   //assert(stomp_motion_planner::read(node_handle, std::string("first_trial"), first_trial));
   //    assert(stomp_motion_planner::read(node_handle, std::string("last_trial"), last_trial));
   
   //    for (int i=first_trial; i<=last_trial; ++i)
   //    {
   //        assert(runSingleIteration(i));
   //        ros::spinOnce();
   //    }
   return true;
   }*/
  
  bool PolicyImprovementLoop::initialize(/*ros::NodeHandle& node_handle,*/ boost::shared_ptr<stomp_motion_planner::Task> task)
  {
    //node_handle_ = node_handle;
    task->getPolicy(policy_);
    policy_->getNumDimensions(num_dimensions_);
    policy_->getNumTimeSteps(num_time_steps_);
    
    readParameters();
    
    task_ = task;
    task_->initialize(/*node_handle_,*/ num_time_steps_);
    task_->getControlCostWeight(control_cost_weight_);
    
    assert( num_dimensions_ == static_cast<int>(noise_decay_.size()));
    assert( num_dimensions_ == static_cast<int>(noise_stddev_.size()));
    
    printf("Learning policy with %i dimensions.", num_dimensions_);
    
    policy_improvement_.initialize(num_rollouts_, num_time_steps_, num_reused_rollouts_, 1, policy_, use_cumulative_costs_);
    
    tmp_rollout_cost_ = Eigen::VectorXd::Zero(num_time_steps_);
    rollout_costs_ = Eigen::MatrixXd::Zero(num_rollouts_, num_time_steps_);
    
    policy_iteration_counter_ = 0;
    return (initialized_ = true);
  }
  
  bool PolicyImprovementLoop::readParameters()
  {
    //    assert(stomp_motion_planner::read(node_handle_, std::string("num_rollouts"), num_rollouts_));
    //    assert(stomp_motion_planner::read(node_handle_, std::string("num_reused_rollouts"), num_reused_rollouts_));
    //    assert(stomp_motion_planner::read(node_handle_, std::string("num_time_steps"), num_time_steps_));
    //
    //    assert(stomp_motion_planner::readDoubleArray(node_handle_, "noise_stddev", noise_stddev_));
    //    assert(stomp_motion_planner::readDoubleArray(node_handle_, "noise_decay", noise_decay_));
    //node_handle_.param("write_to_file", write_to_file_, true); // defaults are sometimes good!
    //node_handle_.param("use_cumulative_costs", use_cumulative_costs_, true);
    
    num_rollouts_ = 10;
    num_reused_rollouts_ = 5;
    //num_time_steps_ = 51;
    
    noise_stddev_.clear();
    noise_decay_.clear();
    
    noise_stddev_.resize(num_dimensions_,2.);
    noise_decay_.resize(num_dimensions_,.99);
    
//    noise_stddev
//    noise_decay
    
    write_to_file_ = false; // defaults are sometimes good!
    use_cumulative_costs_ =  true;
    
    return true;
  }
  
  bool PolicyImprovementLoop::readPolicy(const int iteration_number)
  {
    // check whether reading the policy from file is neccessary
    if(iteration_number == (policy_iteration_counter_))
    {
      return true;
    }
    /*    ROS_INFO("Read policy from file %s.", policy_->getFileName(iteration_number).c_str());
     assert(policy_->readFromDisc(policy_->getFileName(iteration_number)));
     assert(task_->setPolicy(policy_));
     */    return true;
  }
  
  bool PolicyImprovementLoop::writePolicy(const int iteration_number, bool is_rollout, int rollout_id)
  {
    return true;
  }
  
  bool PolicyImprovementLoop::runSingleIteration(const int iteration_number)
  {
    assert(initialized_);
    policy_iteration_counter_++;
    
    if (write_to_file_)
    {
      // load new policy if neccessary
      assert(readPolicy(iteration_number));
    }
    
    // compute appropriate noise values
    std::vector<double> noise;
    noise.resize(num_dimensions_);
    for (int i=0; i<num_dimensions_; ++i)
    {
      noise[i] = noise_stddev_[i] * pow(noise_decay_[i], iteration_number-1);
      //cout << "noise_stddev = " << noise[i] << endl;
    }
    
    // get rollouts and execute them
    assert(policy_improvement_.getRollouts(rollouts_, noise));
    
    for (int r=0; r<int(rollouts_.size()); ++r)
    {
      assert(task_->execute(rollouts_[r], tmp_rollout_cost_, iteration_number));
      rollout_costs_.row(r) = tmp_rollout_cost_.transpose();
      //printf("Rollout %d, cost = %lf\n", r+1, tmp_rollout_cost_.sum());
    }
    
    // TODO: fix this std::vector<>
    std::vector<double> all_costs;
    assert(policy_improvement_.setRolloutCosts(rollout_costs_, control_cost_weight_, all_costs));
    
    // improve the policy
    assert(policy_improvement_.improvePolicy(parameter_updates_));
    
//    for (int d=0; d<num_dimensions_; ++d)
//    {
//      parameters_.resize(num_dimensions_, Eigen::VectorXd::Zero(num_time_steps_));
//      parameters_[d] = parameter_updates_[d].row(0).transpose();
//    }
    assert(policy_->updateParameters(parameter_updates_));
    
    // get a noise-less rollout to check the cost
    assert(policy_->getParameters(parameters_));
    
    // Get the trajectory cost
    assert(task_->execute(parameters_, tmp_rollout_cost_, iteration_number));
    //printf("Noiseless cost = %lf\n", stats_msg.noiseless_cost);
    
    // add the noiseless rollout into policy_improvement:
    std::vector<std::vector<Eigen::VectorXd> > extra_rollout;
    std::vector<Eigen::VectorXd> extra_rollout_cost;
    extra_rollout.resize(1);
    extra_rollout_cost.resize(1);
    extra_rollout[0] = parameters_;
    extra_rollout_cost[0] = tmp_rollout_cost_;
    assert(policy_improvement_.addExtraRollouts(extra_rollout, extra_rollout_cost));
    
    //cout << "rollout_cost_ = " << tmp_rollout_cost_.sum() << endl;
    
    if (write_to_file_)
    {
      // store updated policy to disc
      //assert(writePolicy(iteration_number));
      //assert(writePolicyImprovementStatistics(stats_msg));
    }
    
    return true;
  }
  
  /*
   bool PolicyImprovementLoop::writePolicyImprovementStatistics(const policy_improvement_loop::PolicyImprovementStatistics& stats_msg)
   {
   
   std::string directory_name = std::string("/tmp/pi2_statistics/");
   std::string file_name = directory_name;
   file_name.append(std::string("pi2_statistics.bag"));
   
   if (!boost::filesystem::exists(directory_name))
   {
   if(stats_msg.iteration == 1)
   {
   boost::filesystem::remove_all(directory_name);
   }
   ROS_INFO("Creating directory %s...", directory_name.c_str());
   assert(boost::filesystem::create_directories(directory_name));
   }
   
   try
   {
   if(stats_msg.iteration == 1)
   {
   rosbag::Bag bag(file_name, rosbag::bagmode::Write);
   bag.write(PI_STATISTICS_TOPIC_NAME, ros::Time::now(), stats_msg);
   bag.close();
   }
   else
   {
   rosbag::Bag bag(file_name, rosbag::bagmode::Append);
   bag.write(PI_STATISTICS_TOPIC_NAME, ros::Time::now(), stats_msg);
   bag.close();
   }
   }
   catch (rosbag::BagIOException ex)
   {
   ROS_ERROR("Could write to bag file %s: %s", file_name.c_str(), ex.what());
   return false;
   }
   return true;
   }
   */
}
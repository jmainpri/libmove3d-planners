/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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

#include "stompParameters.hpp"
#include "planner/planEnvironment.hpp"

#include "../p3d/env.hpp"
#include "Planner-pkg.h"

namespace stomp_motion_planner
{

StompParameters::StompParameters()
{
}

StompParameters::~StompParameters()
{
}

double StompParameters::getSmoothnessCostWeight() const
{
    return smoothness_factor_ * PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight);
//  return smoothness_cost_weight_;
}

double StompParameters::getObstacleCostWeight() const
{

    return collision_factor_ * PlanEnv->getDouble(PlanParam::trajOptimObstacWeight);
//  return obstacle_cost_weight_;
}

double StompParameters::getGeneralCostWeight() const
{
    return PlanEnv->getDouble(PlanParam::trajOptimGlobalWeight);
//  return obstacle_cost_weight_;
}
  
void StompParameters::init()
{
  max_time_         = PlanEnv->getDouble(PlanParam::trajStompTimeLimit);
  max_iterations_   = PlanEnv->getInt(PlanParam::stompMaxIteration);
  //max_iterations_after_collision_free_ = 100;
  max_iterations_after_collision_free_ = 100;
  max_best_iterations_= 1000;

  // Scaling features for IOC ...
  smoothness_factor_ = PlanEnv->getDouble(PlanParam::trajOptimSmoothFactor); //100.0; // for IOC, scale the features between 0.1
  collision_factor_ = PlanEnv->getDouble(PlanParam::trajOptimObstacFactor);


  stop_when_collision_free_ = PlanEnv->getBool(PlanParam::trajOptimStopWhenCollisionFree);
  
  //smoothness_cost_weight_ = 0.1;
//  smoothness_cost_weight_ = 0.05;
//  obstacle_cost_weight_ = 1.0;

  // Not used anymore
//  smoothness_cost_weight_ = PlanEnv->getDouble(PlanParam::trajOptimSmoothWeight);
//  obstacle_cost_weight_ =   PlanEnv->getDouble(PlanParam::trajOptimObstacWeight);
  
  constraint_cost_weight_ = 0.0;
  torque_cost_weight_ = 0.0;
  
  learning_rate_ = 0.01;
  animate_path_ = false;
  add_randomness_ = true;
  
  smoothness_cost_velocity_     = PlanEnv->getDouble(PlanParam::trajStompSmoothVel);
  smoothness_cost_acceleration_ = PlanEnv->getDouble(PlanParam::trajStompSmoothAcc);
  smoothness_cost_jerk_         = PlanEnv->getDouble(PlanParam::trajStompSmoothJerk);
  
  use_hamiltonian_monte_carlo_ = true;
  hmc_discretization_ = 0.01;
  hmc_stochasticity_ = 0.01;
  hmc_annealing_factor_ = 0.99;
  
  ridge_factor_ = 0.0;
  use_pseudo_inverse_ = false;
  pseudo_inverse_ridge_factor_ = 1e-4;
  
  animate_endeffector_ = true; // ENV.getBool(Env::drawTraj);
  animate_endeffector_segment_ = "r_gripper_tool_frame";
  use_chomp_ = false;
}
  
bool StompParameters::getAnimateEndeffector() const
{
  //  return animate_endeffector_;
  return ENV.getBool(Env::drawTraj);
}
  
void StompParameters::initFromNodeHandle()
{
//  ros::NodeHandle node_handle("~");
//  node_handle.param("planning_time_limit", planning_time_limit_, 1.0);
//  node_handle.param("max_iterations", max_iterations_, 500);
//  node_handle.param("max_iterations_after_collision_free", max_iterations_after_collision_free_, 100);
//  node_handle.param("smoothness_cost_weight", smoothness_cost_weight_, 0.1);
//  node_handle.param("obstacle_cost_weight", obstacle_cost_weight_, 1.0);
//  node_handle.param("constraint_cost_weight", constraint_cost_weight_, 1.0);
//  node_handle.param("torque_cost_weight", torque_cost_weight_, 0.0);
//  node_handle.param("learning_rate", learning_rate_, 0.01);
//  node_handle.param("animate_path", animate_path_, false);
//  node_handle.param("add_randomness", add_randomness_, true);
//  node_handle.param("smoothness_cost_velocity", smoothness_cost_velocity_, 0.0);
//  node_handle.param("smoothness_cost_acceleration", smoothness_cost_acceleration_, 1.0);
//  node_handle.param("smoothness_cost_jerk", smoothness_cost_jerk_, 0.0);
//  node_handle.param("hmc_discretization", hmc_discretization_, 0.01);
//  node_handle.param("hmc_stochasticity", hmc_stochasticity_, 0.01);
//  node_handle.param("hmc_annealing_factor", hmc_annealing_factor_, 0.99);
//  node_handle.param("use_hamiltonian_monte_carlo", use_hamiltonian_monte_carlo_, false);
//  node_handle.param("ridge_factor", ridge_factor_, 0.0);
//  node_handle.param("use_pseudo_inverse", use_pseudo_inverse_, false);
//  node_handle.param("pseudo_inverse_ridge_factor", pseudo_inverse_ridge_factor_, 1e-4);
//  node_handle.param("animate_endeffector", animate_endeffector_, false);
//  node_handle.param("animate_endeffector_segment", animate_endeffector_segment_, std::string("r_gripper_tool_frame"));
//  node_handle.param("use_chomp", use_chomp_, false);
}


} // namespace stomp

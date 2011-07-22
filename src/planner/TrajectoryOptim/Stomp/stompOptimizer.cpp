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

#include "stompOptimizer.hpp"
//#include <ros/ros.h>
//#include <visualization_msgs/MarkerArray.h>
#include "Chomp/chompUtils.hpp"
#include <Eigen/LU>

#include "planner/planEnvironment.hpp"

#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Util-pkg.h"

#include "move3d-headless.h"

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

//--------------------------------------------------------
// External
//--------------------------------------------------------
boost::shared_ptr<stomp_motion_planner::StompOptimizer> optimizer;

namespace stomp_motion_planner
{
  
  StompOptimizer::StompOptimizer(ChompTrajectory *trajectory, 
                                 const StompParameters *parameters, 
                                 const ChompPlanningGroup *planning_group,
                                 const CollisionSpace *collision_space) : 
  full_trajectory_(trajectory),
  planning_group_(planning_group),
  parameters_(parameters),
  collision_space_(collision_space),
  group_trajectory_(*full_trajectory_, DIFF_RULE_LENGTH)
  {
    initialize();
  }

//StompOptimizer::StompOptimizer(StompTrajectory *trajectory, const StompRobotModel *robot_model,
//    const StompRobotModel::StompPlanningGroup *planning_group, const StompParameters *parameters,
//    const ros::Publisher& vis_marker_array_publisher,
//    const ros::Publisher& vis_marker_publisher,
//    const ros::Publisher& stats_publisher,
//    StompCollisionSpace *collision_space,
//    const motion_planning_msgs::Constraints& constraints):
//      full_trajectory_(trajectory),
//      robot_model_(robot_model),
//      planning_group_(planning_group),
//      parameters_(parameters),
//      collision_space_(collision_space),
//      group_trajectory_(*full_trajectory_, planning_group_, DIFF_RULE_LENGTH),
//      kdl_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
//      kdl_vel_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
//      kdl_acc_joint_array_(robot_model_->getKDLTree()->getNrOfJoints()),
//      kdl_group_joint_array_(group_trajectory_.getNumJoints()),
//      kdl_group_vel_joint_array_(group_trajectory_.getNumJoints()),
//      kdl_group_acc_joint_array_(group_trajectory_.getNumJoints()),
//      kdl_group_torque_joint_array_(group_trajectory_.getNumJoints()),
//      vis_marker_array_pub_(vis_marker_array_publisher),
//      vis_marker_pub_(vis_marker_publisher),
//      stats_pub_(stats_publisher),
//      constraints_(constraints)
//{
//  initialize();
//}

void StompOptimizer::initialize()
{
  robot_model_ = group_trajectory_.getRobot();
  
  //clearAnimations();

  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  //ROS_INFO_STREAM("Setting free vars start to " << free_vars_start_ << " end " << free_vars_end_);

  // set up joint index:
  group_joint_to_move3d_joint_index_.resize(num_joints_);
  for (int i=0; i<num_joints_; ++i)
  {
    group_joint_to_move3d_joint_index_[i] = planning_group_->chomp_joints_[i].move3d_joint_index_;
  }

  num_collision_points_ = planning_group_->collision_points_.size();

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;
//  ros::NodeHandle nh("~");
  for (int i=0; i<num_joints_; i++)
  {
    double joint_cost = 1.0;
    std::string joint_name = planning_group_->chomp_joints_[i].joint_name_;
//    nh.param("joint_costs/"+joint_name, joint_cost, 1.0);
    std::vector<double> derivative_costs(3);
    derivative_costs[0] = joint_cost*parameters_->getSmoothnessCostVelocity();
    derivative_costs[1] = joint_cost*parameters_->getSmoothnessCostAcceleration();
    derivative_costs[2] = joint_cost*parameters_->getSmoothnessCostJerk();

    joint_costs_.push_back(ChompCost(group_trajectory_, i, derivative_costs, parameters_->getRidgeFactor()));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale)
      max_cost_scale = cost_scale;
  }

  // scale the smoothness costs
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].scale(max_cost_scale);
  }

  // allocate memory for matrices:
  smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  final_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
  jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);
  jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(num_joints_, 3);
  jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);
  random_state_ = Eigen::VectorXd::Zero(num_joints_);
  joint_state_velocities_ = Eigen::VectorXd::Zero(num_joints_);
  joint_state_accelerations_ = Eigen::VectorXd::Zero(num_joints_);
  
  // SEE IMPORTANT
//  full_joint_state_velocities_ = Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());
//  full_joint_state_accelerations_ = Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());

  group_trajectory_backup_ = group_trajectory_.getTrajectory();
  best_group_trajectory_ = group_trajectory_.getTrajectory();

//  joint_axis_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
//  joint_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(robot_model_->getKDLTree()->getNrOfJoints()));
//  segment_frames_.resize(num_vars_all_, std::vector<KDL::Frame>(robot_model_->getKDLTree()->getNrOfSegments()));
//  collision_point_pos_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
//  collision_point_vel_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));
//  collision_point_acc_.resize(num_vars_all_, std::vector<KDL::Vector>(num_collision_points_));

  collision_point_potential_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
  collision_point_vel_mag_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
//  collision_point_potential_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
//  collision_point_vel_mag_.resize(num_vars_all_, std::vector<double>(num_collision_points_));
  collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));
  
  segment_frames_.resize( num_vars_all_ );
  
  for(int i=0; i<num_vars_all_;i++)
  {
    segment_frames_[i].resize(planning_group_->num_joints_);
  }
  
  joint_axis_eigen_.resize(num_vars_all_);
  joint_pos_eigen_.resize(num_vars_all_);
  collision_point_pos_eigen_.resize(num_vars_all_);
  collision_point_vel_eigen_.resize(num_vars_all_);
  collision_point_acc_eigen_.resize(num_vars_all_);
  
  for(int i=0; i<num_vars_all_;i++)
  {
    joint_axis_eigen_[i].resize(num_collision_points_);
    joint_pos_eigen_[i].resize(num_collision_points_);
    collision_point_pos_eigen_[i].resize(num_collision_points_);
    collision_point_vel_eigen_[i].resize(num_collision_points_);
    collision_point_acc_eigen_[i].resize(num_collision_points_);
  }

  // create the eigen maps:
//  kdlVecVecToEigenVecVec(joint_axis_, joint_axis_eigen_, 3, 1);
//  kdlVecVecToEigenVecVec(joint_pos_, joint_pos_eigen_, 3, 1);
//  kdlVecVecToEigenVecVec(collision_point_pos_, collision_point_pos_eigen_, 3, 1);
//  kdlVecVecToEigenVecVec(collision_point_vel_, collision_point_vel_eigen_, 3, 1);
//  kdlVecVecToEigenVecVec(collision_point_acc_, collision_point_acc_eigen_, 3, 1);

  collision_free_iteration_ = 0;
  is_collision_free_ = false;
  state_is_in_collision_.resize(num_vars_all_);
  point_is_in_collision_.resize(num_vars_all_, std::vector<int>(num_collision_points_));

  last_improvement_iteration_ = -1;

  // HMC initialization:
  momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_joint_momentum_ = Eigen::VectorXd::Zero(num_vars_free_);
  multivariate_gaussian_.clear();
  stochasticity_factor_ = 1.0;
  for (int i=0; i<num_joints_; i++)
  {
    multivariate_gaussian_.push_back(MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_), joint_costs_[i].getQuadraticCostInverse()));
  }

  // animation init:
//  animate_endeffector_segment_number_ = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(parameters_->getAnimateEndeffectorSegment());

  // initialize the policy
  policy_.reset(new CovariantTrajectoryPolicy());

  std::vector<double> derivative_costs = parameters_->getSmoothnessCosts();
  policy_->initialize(/*nh,*/ num_vars_free_, num_joints_, group_trajectory_.getDuration(),
                      parameters_->getRidgeFactor(), derivative_costs);

  // initialize the policy trajectory
  Eigen::VectorXd start = group_trajectory_.getTrajectoryPoint(free_vars_start_-1).transpose();
  Eigen::VectorXd end = group_trajectory_.getTrajectoryPoint(free_vars_end_+1).transpose();
  
  // set paramters
  int free_vars_start_index = DIFF_RULE_LENGTH - 1;
  vector< Eigen::VectorXd > parameters(num_joints_);
  for (int i=0; i<num_joints_; i++) 
  {
    parameters[i] =  group_trajectory_.getJointTrajectory(i).segment(free_vars_start_index, num_vars_free_);
  }
  policy_->setToMinControlCost(start, end);
  policy_->setParameters( parameters );

  // initialize the constraints:
//  for (int i=0; i<int(constraints_.orientation_constraints.size()); ++i)
//  {
//    boost::shared_ptr<OrientationConstraintEvaluator> eval(new OrientationConstraintEvaluator(
//        constraints_.orientation_constraints[i], *robot_model_));
//    constraint_evaluators_.push_back(eval);
//  }
}

StompOptimizer::~StompOptimizer()
{
}

void StompOptimizer::doChompOptimization()
{
  calculateSmoothnessIncrements();
  calculateCollisionIncrements();
  calculateTotalIncrements();

  if (!parameters_->getUseHamiltonianMonteCarlo())
  {
    // non-stochastic version:
    addIncrementsToTrajectory();
  }
  else
  {
    // hamiltonian monte carlo updates:
    getRandomMomentum();
    updateMomentum();
    updatePositionFromMomentum();
    stochasticity_factor_ *= parameters_->getHmcAnnealingFactor();
  }

  handleJointLimits();
  updateFullTrajectory();
  last_trajectory_collision_free_ = performForwardKinematics();
  last_trajectory_constraints_satisfied_ = true;
  
  if (!last_trajectory_collision_free_ && parameters_->getAddRandomness())
  {
    performForwardKinematics();
    double original_cost = getTrajectoryCost();
    group_trajectory_backup_ = group_trajectory_.getTrajectory();
    perturbTrajectory();
    handleJointLimits();
    updateFullTrajectory();
    performForwardKinematics();
    double new_cost = getTrajectoryCost();
    
    if (new_cost < original_cost)
    {
      //printf("Random jump improved cost from %.10f to %.10f!\n", original_cost, new_cost);
    }
    else
    {
      //printf("Random jump worsened cost from %.10f to %.10f!\n", original_cost, new_cost);
      group_trajectory_.getTrajectory() = group_trajectory_backup_;
      updateFullTrajectory();
    }
  }

  double cost = 0.0;
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    double cumulative = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
      //state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
      state_collision_cost += cumulative;
    }
    cost += state_collision_cost * parameters_->getObstacleCostWeight();
  }
  last_trajectory_cost_ = cost;
}

//void StompOptimizer::optimize()
void StompOptimizer::runDeformation( int nbIteration , int idRun )
{
//  ros::WallTime start_time = ros::WallTime::now();

  stomp_statistics_ = boost::shared_ptr<StompStatistics>(new StompStatistics());

  stomp_statistics_->collision_success_iteration = -1;
  stomp_statistics_->success_iteration = -1;
  stomp_statistics_->success = false;
  stomp_statistics_->costs.clear();

  ChronoOn();
  // initialize pi_loop
//  ros::NodeHandle nh("~");
  PolicyImprovementLoop pi_loop;
  pi_loop.initialize(/*nh,*/ this_shared_ptr_);
//  collision_space_->lock();
  
  group_trajectory_.print();

  iteration_ = 0;
  copyPolicyToGroupTrajectory();
  handleJointLimits();
  updateFullTrajectory();
  performForwardKinematics();
  
  group_trajectory_.print();

  if (parameters_->getAnimateEndeffector())
  {
    animateEndeffector();
  }

  if (parameters_->getAnimatePath())
  {
    animatePath();
  }

  // iterate
  for (iteration_=0; 
       iteration_<parameters_->getMaxIterations()&& 
       !PlanEnv->getBool(PlanParam::stopPlanner); 
       iteration_++)
  {
//    if (!ros::ok())
//      break;

    if (!parameters_->getUseChomp())
    {
      // after this, the latest "group trajectory" and "full trajectory" is the one optimized by pi^2
      //ros::WallTime start_time = ros::WallTime::now();
      pi_loop.runSingleIteration(iteration_+1);
      //ROS_INFO("PI loop took %f seconds, ", (ros::WallTime::now() - start_time).toSec());
    }
    else
    {
      doChompOptimization();
    }

    if (last_trajectory_collision_free_ && last_trajectory_constraints_satisfied_)
      collision_free_iteration_++;
    else
      collision_free_iteration_ = 0;

    if (last_trajectory_collision_free_ &&
        stomp_statistics_->collision_success_iteration == -1)
    {
      stomp_statistics_->collision_success_iteration = iteration_;
//      stomp_statistics_->collision_success_duration = (ros::WallTime::now() - start_time).toSec();
    }
    if (last_trajectory_collision_free_ &&
        last_trajectory_constraints_satisfied_ &&
        stomp_statistics_->success_iteration == -1)
    {
      stomp_statistics_->success_iteration = iteration_;
      stomp_statistics_->success = true;

//      stomp_statistics_->success_duration = (ros::WallTime::now() - start_time).toSec();
    }

//    double cost = getTrajectoryCost();
    double cost = last_trajectory_cost_;
    stomp_statistics_->costs.push_back(cost);

    if (iteration_==0)
    {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = cost;
    }
    else
    {
      if (cost < best_group_trajectory_cost_ /*&& last_trajectory_collision_free_ && last_trajectory_constraints_satisfied_*/)
      {
        cout << "New best" << endl;
        best_group_trajectory_ = group_trajectory_.getTrajectory();
        best_group_trajectory_cost_ = cost;
        last_improvement_iteration_ = iteration_;
      }
    }

    //if (iteration_%1==0)
    printf( "%3d Trajectory cost: %f (s=%f, c=%f)\n", iteration_, getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());
    //cout << "We think the path is collision free: " << last_trajectory_collision_free_ << endl;
    
    if (collision_free_iteration_ >= parameters_->getMaxIterationsAfterCollisionFree())
    {
      iteration_++;
      break;
    }

    if (parameters_->getAnimateEndeffector())
    {
      animateEndeffector();
    }

//    if (parameters_->getAnimatePath() && iteration_%1 == 0)
//    {
//      animatePath();
//    }
    
  }
  if (last_improvement_iteration_>-1)
    cout << "We think the path is collision free: " << is_collision_free_ << endl;

//  if (parameters_->getAnimatePath())
//  {
//    animatePath();
//  }

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  updateFullTrajectory();
  performForwardKinematics();
  
  ChronoPrint("");
  ChronoOff();
  
//  if (parameters_->getAnimateEndeffector())
//  {
    animateEndeffector();
//  }
  
  printf("Terminated after %d iterations, using path from iteration %d\n", iteration_, last_improvement_iteration_);
  printf("Best cost = %f\n", best_group_trajectory_cost_);
  //printf("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());
  stomp_statistics_->best_cost = best_group_trajectory_cost_;

//  collision_space_->unlock();
  if (parameters_->getAnimatePath())
    animatePath();

  // calculate the torques for publishing
//  std::vector<KDL::Wrench> wrenches(planning_group_->kdl_chain_.getNrOfSegments());
//  for (unsigned int i=0; i<planning_group_->kdl_chain_.getNrOfSegments(); ++i)
//  {
//    wrenches[i].Zero();
//  }
//  std::vector<double> torques(num_joints_);
//  stomp_statistics_->torques.resize(num_vars_free_);
//  for (int index = free_vars_start_; index <= free_vars_end_; ++index)
//  {
//    getTorques(index, torques, wrenches);
//    stomp_statistics_->torques[index-free_vars_start_] = 0.0;
//    for (int j=0; j<num_joints_; ++j)
//      stomp_statistics_->torques[index-free_vars_start_] += fabs(torques[j]);
//  }

//  stats_pub_.publish(stomp_statistics_);
}

void StompOptimizer::calculateSmoothnessIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i), smoothness_derivative_);
    smoothness_increments_.col(i) = -smoothness_derivative_.segment(
        group_trajectory_.getStartIndex(), num_vars_free_);
  }
  
  //cout <<  "smoothness_increments_  = " << endl << smoothness_increments_ << endl;
}

void StompOptimizer::calculateCollisionIncrements()
{
  double potential;
  double vel_mag_sq;
  double vel_mag;
  Vector3d potential_gradient;
  Vector3d normalized_velocity;
  Matrix3d orthogonal_projector;
  Vector3d curvature_vector;
  Vector3d cartesian_gradient;
  
  //cout <<  "collision_point_potential_  = " << endl << collision_point_potential_ << endl;

  collision_increments_.setZero(num_vars_free_, num_joints_);
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      potential = collision_point_potential_(i,j);
      if (potential <= 1e-10)
        continue;

      potential_gradient = collision_point_potential_gradient_[i][j];
      //cout << collision_point_potential_gradient_[i][j] << endl;

      vel_mag = collision_point_vel_mag_(i,j);
      vel_mag_sq = vel_mag*vel_mag;
      //cout << collision_point_vel_mag_(i,j) << endl;

      // all math from the STOMP paper:
      normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
      orthogonal_projector = Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
      curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
      cartesian_gradient = vel_mag*(orthogonal_projector*potential_gradient - potential*curvature_vector);

      // pass it through the jacobian transpose to get the increments
      planning_group_->collision_points_[j].getJacobian(joint_pos_eigen_[i], 
                                                        joint_axis_eigen_[i],
                                                        collision_point_pos_eigen_[i][j], 
                                                        jacobian_, 
                                                        group_joint_to_move3d_joint_index_);
      
      if (parameters_->getUsePseudoInverse())
      {
        calculatePseudoInverse();
        collision_increments_.row(i-free_vars_start_).transpose() -=
            jacobian_pseudo_inverse_ * cartesian_gradient;
      }
      else
      {
        collision_increments_.row(i-free_vars_start_).transpose() -=
            jacobian_.transpose() * cartesian_gradient;
      }
      if (point_is_in_collision_[i][j])
        break;
    }
  }
  
  //cout <<  "collision_increments_  = " << endl << collision_increments_ << endl;
}

void StompOptimizer::calculatePseudoInverse()
{
  jacobian_jacobian_tranpose_ = jacobian_*jacobian_.transpose() + Eigen::MatrixXd::Identity(3,3)*parameters_->getPseudoInverseRidgeFactor();
  jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void StompOptimizer::calculateTotalIncrements()
{
  for (int i=0; i<num_joints_; i++)
  {
    final_increments_.col(i) = parameters_->getLearningRate() *
        (
            joint_costs_[i].getQuadraticCostInverse() *
            (
                parameters_->getSmoothnessCostWeight() * smoothness_increments_.col(i) +
                parameters_->getObstacleCostWeight() * collision_increments_.col(i)
            )
        );
  }
}

void StompOptimizer::addIncrementsToTrajectory()
{
//  double scale = 1.0;
  for (int i=0; i<num_joints_; i++)
  {
    double scale = 1.0;
    double max = final_increments_.col(i).maxCoeff();
    double min = final_increments_.col(i).minCoeff();
    double max_scale = planning_group_->chomp_joints_[i].joint_update_limit_ / fabs(max);
    double min_scale = planning_group_->chomp_joints_[i].joint_update_limit_ / fabs(min);
    if (max_scale < scale)
      scale = max_scale;
    if (min_scale < scale)
      scale = min_scale;
    group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
  }
  //ROS_DEBUG("Scale: %f",scale);
  //group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
}

void StompOptimizer::updateFullTrajectory()
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void StompOptimizer::debugCost()
{
  double cost = 0.0;
  for (int i=0; i<num_joints_; i++)
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  cout << "Cost = " << cost << endl;
}

double StompOptimizer::getTrajectoryCost()
{
  return getSmoothnessCost() + getCollisionCost();
}

double StompOptimizer::getSmoothnessCost()
{
  double smoothness_cost = 0.0;
  // joint costs:
  for (int i=0; i<num_joints_; i++)
    smoothness_cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));

  return parameters_->getSmoothnessCostWeight() * smoothness_cost;
}

double StompOptimizer::getCollisionCost()
{
  double collision_cost = 0.0;

  double worst_collision_cost = 0.0;
  worst_collision_cost_state_ = -1;

  // collision costs:
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      state_collision_cost += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
    }
    collision_cost += state_collision_cost;
    if (state_collision_cost > worst_collision_cost)
    {
      worst_collision_cost = state_collision_cost;
      worst_collision_cost_state_ = i;
    }
  }

//  cout << "parameters_->getObstacleCostWeight() = " << parameters_->getObstacleCostWeight() << endl;
//  cout << "collision_cost = " << collision_cost << endl;
  
  return parameters_->getObstacleCostWeight() * collision_cost;
}

void StompOptimizer::handleJointLimits()
{
  for (int joint=0; joint<num_joints_; joint++)
  {
    if (!planning_group_->chomp_joints_[joint].has_joint_limits_)
      continue;

    double joint_max = planning_group_->chomp_joints_[joint].joint_limit_max_;
    double joint_min = planning_group_->chomp_joints_[joint].joint_limit_min_;

    int count = 0;

    bool violation = false;
    do
    {
      double max_abs_violation =  1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      for (int i=free_vars_start_; i<=free_vars_end_; i++)
      {
        double amount = 0.0;
        double absolute_amount = 0.0;
        if (group_trajectory_(i, joint) > joint_max)
        {
          amount = joint_max - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        else if (group_trajectory_(i, joint) < joint_min)
        {
          amount = joint_min - group_trajectory_(i, joint);
          absolute_amount = fabs(amount);
        }
        if (absolute_amount > max_abs_violation)
        {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;
          violation = true;
        }
      }

      if (violation)
      {
        int free_var_index = max_violation_index - free_vars_start_;
        double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);
        group_trajectory_.getFreeJointTrajectoryBlock(joint) +=
            multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
      }
      if (++count > 10)
        break;
    }
    while(violation);
  }
}
  
  void StompOptimizer::getFrames(int segment, const Eigen::VectorXd& joint_array)
  {
    //std::vector< Eigen::Vector3d > null_vect;
    //null_vect.push_back( Eigen::Vector3d::Zero() );
    
    Configuration q = *robot_model_->getCurrentPos();
    
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
      int dof = joints[j].move3d_joint_->getIndexOfFirstDof();
      
      if ( !isnan(joint_array[j]) ) 
      {
        q[dof]= joint_array[j];
      }
      else {
        cout << "q[" << dof << "] is nan" << endl;
        q[dof]= 0;
      }
    }
    
    robot_model_->setAndUpdate( q );
    
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
      Eigen::Transform3d t = joints[j].move3d_joint_->getMatrixPos();
      
      std::vector<double> vect;
      eigenTransformToStdVector(t,vect);
      
      segment_frames_[segment][j]  = vect;
      joint_pos_eigen_[segment][j] = t.translation();
      joint_axis_eigen_[segment][j](0) = t(0,2);
      joint_axis_eigen_[segment][j](0) = t(1,2);
      joint_axis_eigen_[segment][j](0) = t(2,2);
    }
  }
  

bool StompOptimizer::performForwardKinematics()
{
  double invTime = 1.0 / group_trajectory_.getDiscretization();
  double invTimeSq = invTime*invTime;

  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_==0)
  {
    start = 0;
    end = num_vars_all_-1;
  }

  is_collision_free_ = true;
  
  Eigen::VectorXd joint_array;

  // for each point in the trajectory
  for (int i=start; i<=end; ++i)
  {
    int full_traj_index = group_trajectory_.getFullTrajectoryIndex(i);
    full_trajectory_->getTrajectoryPointP3d(full_traj_index, joint_array);
    
    getFrames( i, joint_array );
    
    state_is_in_collision_[i] = false;

    // calculate the position of every collision point:
    for (int j=0; j<num_collision_points_; j++)
    {
      planning_group_->collision_points_[j].getTransformedPosition(segment_frames_[i], collision_point_pos_eigen_[i][j]);
      //int segment_number = planning_group_->collision_points_[j].getSegmentNumber();
      //collision_point_pos_[i][j] = segment_frames_[i][segment_number] * planning_group_->collision_points_[j].getPosition();
      
      //cout << "coll_point(" << i << " , "  << j << ") = " << endl << collision_point_pos_eigen_[i][j] << endl;

      bool colliding = collision_space_->getCollisionPointPotentialGradient(planning_group_->collision_points_[j],
          collision_point_pos_eigen_[i][j],
          collision_point_potential_(i,j),
          collision_point_potential_gradient_[i][j]);
      
//      cout << "collision_point_potential_(" << i << " , "  << j << ") = " << endl << collision_point_potential_(i,j) << endl;
//      cout << "collision_point_potential_gradient_(" << i << " , "  << j << ") = " << endl << collision_point_potential_gradient_[i][j] << endl;

      point_is_in_collision_[i][j] = colliding;

      if ( colliding )
      {
        // This is the function that
        // discards joints too close to the base
        if( planning_group_->collision_points_[j].getSegmentNumber() > 8 )
        {
          state_is_in_collision_[i] = true;
        }
      }
    }

    if (state_is_in_collision_[i])
    {
      //cout << "Stat[" << i << "] is colliding" << endl;
      is_collision_free_ = false;
    }
  }

  // now, get the vel and acc for each collision point (using finite differencing)
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    for (int j=0; j<num_collision_points_; j++)
    {
      collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
      collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();
      
      for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
      {
        collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) *
        collision_point_pos_eigen_[i+k][j];
        collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) *
        collision_point_pos_eigen_[i+k][j];
      }
      // get the norm of the velocity:
      collision_point_vel_mag_(i,j) = collision_point_vel_eigen_[i][j].norm();
      
      //      cout << "collision_point_pos_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_pos_eigen_[i][j] << endl;
      //      cout << "collision_point_potential_(" << i << " , "  << j << ") = " << endl << collision_point_potential_[i][j] << endl;
      //      cout << "collision_point_potential_gradient_(" << i << " , "  << j << ") = " << endl << collision_point_potential_gradient_[i][j] << endl;
      //      cout << "collision_point_vel_mag_(" << i << " , "  << j << ") = " << endl << collision_point_vel_mag_[i][j] << endl;
      //      cout << "collision_point_vel_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_vel_eigen_[i][j] << endl;
      //      cout << "collision_point_acc_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_acc_eigen_[i][j] << endl;
    }
  }
  
//  if (is_collision_free_) 
//  {
//    cout << "-----------------------" << endl;
//    cout << "Collision Free Traj" << endl;
//    cout << "-----------------------" << endl;
//  }
//  else {
//    cout << "------------------------------------" << endl;
//    for (int i=start; i<=end; ++i) 
//    {
//      if (state_is_in_collision_[i]) 
//      {
//        cout << "state[" << i << "] in collision" << endl;
//      }
//    }
//    cout << "------------------------------------" << endl;
//  }

  return is_collision_free_;
}

void StompOptimizer::eigenMapTest()
{
  double foo_eigen;
  double foo_kdl;

//  cout << "Eigen location: " << &(joint_axis_eigen_[free_vars_start_][0](0)) <<
//          "  KDL location: " << &(joint_axis_[free_vars_start_][0](0)) << endl;

  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
//  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  //ROS_ASSERT(foo_eigen==foo_kdl);

  joint_axis_eigen_[free_vars_start_][0](0) = 1.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
//  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  //ROS_ASSERT(foo_kdl == foo_eigen);

//  joint_axis_[free_vars_start_][0](0) = 2.0;
  foo_eigen = joint_axis_eigen_[free_vars_start_][0](0);
//  foo_kdl = joint_axis_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  //ROS_ASSERT(foo_eigen == foo_kdl);

  foo_eigen = joint_pos_eigen_[free_vars_start_][0](0);
//  foo_kdl = joint_pos_[free_vars_start_][0](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  //ROS_ASSERT(foo_eigen==foo_kdl);

  foo_eigen = collision_point_pos_eigen_[free_vars_start_][5](0);
//  foo_kdl = collision_point_pos_[free_vars_start_][5](0);
  printf("eigen = %f, kdl = %f\n", foo_eigen, foo_kdl);
  //ROS_ASSERT(foo_eigen==foo_kdl);
}

void StompOptimizer::perturbTrajectory()
{
  //int mid_point = (free_vars_start_ + free_vars_end_) / 2;
  if (worst_collision_cost_state_ < 0)
    return;
  int mid_point = worst_collision_cost_state_;
  planning_group_->getRandomState(random_state_);

  // convert the state into an increment
  random_state_ -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

  // project the increment orthogonal to joint velocities
  group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
  joint_state_velocities_.normalize();
  random_state_ = (Eigen::MatrixXd::Identity(num_joints_, num_joints_) - joint_state_velocities_*joint_state_velocities_.transpose()) * random_state_;

  int mp_free_vars_index = mid_point - free_vars_start_;
  for (int i=0; i<num_joints_; i++)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(i) +=
        joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
  }
}

void StompOptimizer::getRandomMomentum()
{
  if (is_collision_free_)
    random_momentum_.setZero(num_vars_free_, num_joints_);
  else
  for (int i=0; i<num_joints_; ++i)
  {
    multivariate_gaussian_[i].sample(random_joint_momentum_);
    random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
  }
}

void StompOptimizer::updateMomentum()
{
  double eps = parameters_->getHmcDiscretization();

  //  if (iteration_ > 0)
//    momentum_ = (momentum_ + eps*final_increments_);
//  else
//    momentum_ = random_momentum_;

  double alpha = 1.0 - parameters_->getHmcStochasticity();
  //if (iteration_ > 0)
    momentum_ = alpha * (momentum_ + eps*final_increments_) + sqrt(1.0-alpha*alpha)*random_momentum_;
  //else
  //  momentum_ = random_momentum_;
}

void StompOptimizer::updatePositionFromMomentum()
{
  double eps = parameters_->getHmcDiscretization();
  group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
}

void StompOptimizer::animatePath()
{
  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    visualizeState(i);
    //ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
//    ros::WallDuration(0.05).sleep();
  }
}

void StompOptimizer::clearAnimations()
{
//  visualization_msgs::Marker msg;
//  msg.points.resize(0);
//  msg.header.frame_id = robot_model_->getReferenceFrame();
//  msg.header.stamp = ros::Time::now();
//  msg.ns = "stomp_endeffector";
//  msg.id = 0;
//  msg.type = visualization_msgs::Marker::SPHERE_LIST;
//  msg.action = visualization_msgs::Marker::DELETE;
//  vis_marker_pub_.publish(msg);
}

void StompOptimizer::animateEndeffector()
{
  API::Trajectory T(robot_model_);
  
  // calculate the forward kinematics for the fixed states only in the first iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_==0)
  {
    start = 0;
    end = num_vars_all_-1;
  }
  
  Configuration q = *robot_model_->getCurrentPos();
  
  //  cout << "animateEndeffector()" << endl;
  //  cout << "group_trajectory : " << endl;
  //  cout << group_trajectory_.getTrajectory() << endl;
  
  const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
  
  // for each point in the trajectory
  for (int i=start; i<=end; ++i)
  {
    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();
    
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
      q[joints[j].move3d_joint_->getIndexOfFirstDof()] = point[j];
    }
    
    T.push_back( std::tr1::shared_ptr<Configuration>(new Configuration(q)) );
  }
  
  if( T.isValid() )
  {
    cout << "T is valid" << endl;
    //PlanEnv->setBool(PlanParam::stopPlanner, true);
  }
  
  T.replaceP3dTraj();
  ENV.setBool(Env::drawTraj,true);
  
  // Set the robot to the first configuration
  Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(1).transpose();
  
  for(int j=0; j<planning_group_->num_joints_;j++)
    q[joints[j].move3d_joint_->getIndexOfFirstDof()] = point[j];
  
  robot_model_->setAndUpdate( q );

  g3d_draw_allwin_active();
  
//  visualization_msgs::Marker msg;
//  msg.points.resize(num_vars_free_);
// // int joint_index = planning_group_->stomp_joints_[num_joints_-1].kdl_joint_index_;
//  int sn = animate_endeffector_segment_number_;
//  for (int i=0; i<num_vars_free_; ++i)
//  {
//    int j = i+free_vars_start_;
//    msg.points[i].x = segment_frames_[j][sn].p.x();
//    msg.points[i].y = segment_frames_[j][sn].p.y();
//    msg.points[i].z = segment_frames_[j][sn].p.z();
//  }
//  msg.header.frame_id = robot_model_->getReferenceFrame();
//  msg.header.stamp = ros::Time::now();
//  msg.ns = "stomp_endeffector";
//  msg.id = 0;
//  msg.type = visualization_msgs::Marker::SPHERE_LIST;
//  msg.action = visualization_msgs::Marker::ADD;
//  double scale = 0.05;
//  msg.scale.x = scale;
//  msg.scale.y = scale;
//  msg.scale.z = scale;
//  msg.color.a = 1.0;
//  msg.color.r = 0.1;
//  msg.color.g = 0.1;
//  msg.color.b = 1.0;
//  vis_marker_pub_.publish(msg);


//  visualization_msgs::Marker msg2;
//  msg2.points.resize(num_vars_free_*2);
//  for (int i=0; i<num_vars_free_; ++i)
//  {
//    int j = i+free_vars_start_;
//    int start = i*2;
//    int end = start + 1;
//    msg2.points[start].x = segment_frames_[j][sn].p.x();
//    msg2.points[start].y = segment_frames_[j][sn].p.y();
//    msg2.points[start].z = segment_frames_[j][sn].p.z();
//
//    KDL::Vector v(0.0,0.0,0.2);
//    v = segment_frames_[j][sn].M*v;
//
//    msg2.points[end].x = segment_frames_[j][sn].p.x() + v.x();
//    msg2.points[end].y = segment_frames_[j][sn].p.y() + v.y();
//    msg2.points[end].z = segment_frames_[j][sn].p.z() + v.z();
//  }
//  msg2.pose.position.x = 0.0;
//  msg2.pose.position.y = 0.0;
//  msg2.pose.position.z = 0.0;
//  msg2.pose.orientation.w = 1.0;
//  msg2.pose.orientation.x = 0.0;
//  msg2.pose.orientation.y = 0.0;
//  msg2.pose.orientation.z = 0.0;
//  msg2.header.frame_id = robot_model_->getReferenceFrame();
//  msg2.header.stamp = ros::Time::now();
//  msg2.ns = "stomp_endeffector_arrows";
//  msg2.id = 0;
//  msg2.type = visualization_msgs::Marker::LINE_LIST;
//  msg2.action = visualization_msgs::Marker::ADD;
//  double line_width = 0.02;
//  msg2.scale.x = line_width;
//  msg2.color.a = 0.6;
//  msg2.color.r = 0.5;
//  msg2.color.g = 1.0;
//  msg2.color.b = 0.3;
//  vis_marker_pub_.publish(msg2);

//  ros::spinOnce();
//  ros::WallDuration(0.01).sleep();
  //char c;
  //std::cin.get(c);
}

  void StompOptimizer::draw()
  {
    return;
    // Draws two points in the trajectory   
    int middle = (free_vars_start_+free_vars_end_)/2;
    
    int start = middle - 3;
    int end = middle + 3;
  
    
    planning_group_->draw(segment_frames_[start]);
    planning_group_->draw(segment_frames_[end]);
  }
  
void StompOptimizer::visualizeState(int index)
{
//  visualization_msgs::MarkerArray msg;
////  msg.markers.resize(num_collision_points_ + num_joints_);
//  msg.markers.resize(num_collision_points_);
//  int num_arrows = 0;
//  double potential_threshold = 1e-10;
//  for (int i=0; i<num_collision_points_; i++)
//  {
//    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
//    msg.markers[i].header.stamp = ros::Time();
//    msg.markers[i].ns = "stomp_collisions";
//    msg.markers[i].id = i;
//    msg.markers[i].type = visualization_msgs::Marker::SPHERE;
//    msg.markers[i].action = visualization_msgs::Marker::ADD;
//    msg.markers[i].pose.position.x = collision_point_pos_[index][i].x();
//    msg.markers[i].pose.position.y = collision_point_pos_[index][i].y();
//    msg.markers[i].pose.position.z = collision_point_pos_[index][i].z();
//    msg.markers[i].pose.orientation.x = 0.0;
//    msg.markers[i].pose.orientation.y = 0.0;
//    msg.markers[i].pose.orientation.z = 0.0;
//    msg.markers[i].pose.orientation.w = 1.0;
//    double scale = planning_group_->collision_points_[i].getRadius()*2.0;
//    msg.markers[i].scale.x = scale;
//    msg.markers[i].scale.y = scale;
//    msg.markers[i].scale.z = scale;
//    msg.markers[i].color.a = 0.6;
//    msg.markers[i].color.r = 0.5;
//    msg.markers[i].color.g = 1.0;
//    msg.markers[i].color.b = 0.3;
//    if (collision_point_potential_[index][i] > potential_threshold)
//      num_arrows++;
//  }

//  for (int j=0; j<num_joints_; ++j)
//  {
//    int i=num_collision_points_+j;
//    int joint_index = planning_group_->stomp_joints_[j].kdl_joint_index_;
//    msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
//    msg.markers[i].header.stamp = ros::Time();
//    msg.markers[i].ns = "stomp_collisions";
//    msg.markers[i].id = i;
//    msg.markers[i].type = visualization_msgs::Marker::ARROW;
//    msg.markers[i].action = visualization_msgs::Marker::ADD;
//    msg.markers[i].points.resize(2);
//    double scale=0.2;
//    double sx = joint_axis_[index][joint_index].x()*scale;
//    double sy = joint_axis_[index][joint_index].y()*scale;
//    double sz = joint_axis_[index][joint_index].z()*scale;
//    msg.markers[i].points[0].x = joint_pos_[index][joint_index].x() - sx;
//    msg.markers[i].points[0].y = joint_pos_[index][joint_index].y() - sy;
//    msg.markers[i].points[0].z = joint_pos_[index][joint_index].z() - sz;
//    msg.markers[i].points[1].x = joint_pos_[index][joint_index].x() + sx;
//    msg.markers[i].points[1].y = joint_pos_[index][joint_index].y() + sy;
//    msg.markers[i].points[1].z = joint_pos_[index][joint_index].z() + sz;
//    msg.markers[i].scale.x = 0.03;
//    msg.markers[i].scale.y = 0.06;
//    msg.markers[i].color.a = 1.0;
//    msg.markers[i].color.r = 1.0;
//    msg.markers[i].color.g = 0.5;
//    msg.markers[i].color.b = 0.5;
//  }
//  vis_marker_array_pub_.publish(msg);

  // publish arrows for distance field:
//  msg.markers.resize(0);
//  msg.markers.resize(num_collision_points_);
//  for (int i=0; i<num_collision_points_; i++)
//  {
//      msg.markers[i].header.frame_id = robot_model_->getReferenceFrame();
//      msg.markers[i].header.stamp = ros::Time();
//      msg.markers[i].ns = "stomp_arrows";
//      msg.markers[i].id = i;
//      msg.markers[i].type = visualization_msgs::Marker::ARROW;
//      msg.markers[i].action = visualization_msgs::Marker::ADD;
//      msg.markers[i].points.resize(2);
//      msg.markers[i].points[0].x = collision_point_pos_[index][i].x();
//      msg.markers[i].points[0].y = collision_point_pos_[index][i].y();
//      msg.markers[i].points[0].z = collision_point_pos_[index][i].z();
//      msg.markers[i].points[1] = msg.markers[i].points[0];
//      double scale = 0.15;
//      if (collision_point_potential_[index][i] <= potential_threshold)
//        scale = 0.0;
//      msg.markers[i].points[1].x -= scale*collision_point_potential_gradient_[index][i].x();
//      msg.markers[i].points[1].y -= scale*collision_point_potential_gradient_[index][i].y();
//      msg.markers[i].points[1].z -= scale*collision_point_potential_gradient_[index][i].z();
//      msg.markers[i].scale.x = 0.01;
//      msg.markers[i].scale.y = 0.03;
//      msg.markers[i].color.a = 1.0;
//      msg.markers[i].color.r = 0.2;
//      msg.markers[i].color.g = 0.2;
//      msg.markers[i].color.b = 1.0;
//  }
//  vis_marker_array_pub_.publish(msg);
}


bool StompOptimizer::initialize(/*ros::NodeHandle& node_handle,*/ int num_time_steps)
{
//  // we already know these things, so do nothing
  return true;
}

  /*
void StompOptimizer::getTorques(int index, std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches)
{
  int i=index;
  group_trajectory_.getTrajectoryPointKDL(i, kdl_group_joint_array_);
  group_trajectory_.getJointVelocities(i, joint_state_velocities_);
  group_trajectory_.getJointAccelerations(i, joint_state_accelerations_);
  for (int j=0; j<num_joints_; ++j)
  {
    //kdl_group_joint_array_(j) = kdl_joint_array_(full_joint_num);
    kdl_group_vel_joint_array_(j) = joint_state_velocities_(j);
    kdl_group_acc_joint_array_(j) = joint_state_accelerations_(j);
  }
//  ROS_INFO("Index = %d", index);
//  ROS_INFO_STREAM("Input = " << kdl_group_joint_array_.data<< " | "
//                  << kdl_group_vel_joint_array_.data << " | "
//                  << kdl_group_acc_joint_array_.data);
  planning_group_->id_solver_->CartToJnt(kdl_group_joint_array_,
                                         kdl_group_vel_joint_array_,
                                         kdl_group_acc_joint_array_,
                                         wrenches,
                                         kdl_group_torque_joint_array_);
//  ROS_INFO_STREAM("Output = " << kdl_group_torque_joint_array_.data);

  for (int j=0; j<num_joints_; ++j)
  {
    torques[j] = kdl_group_torque_joint_array_(j);
  }

}
*/
  
bool StompOptimizer::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number)
{
//  ros::WallTime start_time = ros::WallTime::now();

  // copy the parameters into group_trajectory_:
  for (int d=0; d<num_joints_; ++d)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
  }

  //cout << "group_trajectory_ = " << endl << group_trajectory_.getTrajectory() << endl;
  
  // respect joint limits:
  handleJointLimits();

  // copy to full traj:
  updateFullTrajectory();

  // do forward kinematics:
  last_trajectory_collision_free_ = performForwardKinematics();

  last_trajectory_constraints_satisfied_ = true;

  double constraint_cost = 0.0;
  double obstacle_cost = 0.0;
  double torque_cost = 0.0;

//  std::vector<KDL::Wrench> wrenches(planning_group_->kdl_chain_.getNrOfSegments());
//  for (unsigned int i=0; i<planning_group_->kdl_chain_.getNrOfSegments(); ++i)
//  {
//    wrenches[i].Zero();
//  }

  std::vector<double> torques(num_joints_);

  for (int i=free_vars_start_; i<=free_vars_end_; i++)
  {
    double state_collision_cost = 0.0;
    double cumulative = 0.0;
    for (int j=0; j<num_collision_points_; j++)
    {
      cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
//      state_collision_cost += collision_point_potential_[i][j] * collision_point_vel_mag_[i][j];
      state_collision_cost += cumulative;
      //cout << "state(" << j << ") = " << collision_point_potential_(i,j) * collision_point_vel_mag_(i,j) << endl;
    }

    // evaluate the constraints:
    double state_constraint_cost = 0.0;
//    for (int j=0; j<int(constraint_evaluators_.size()); ++j)
//    {
//      double cost;
//      if (!constraint_evaluators_[j]->getCost(segment_frames_[i], full_trajectory_->getTrajectoryPoint(i), cost))
//        last_trajectory_constraints_satisfied_ = false;
//      state_constraint_cost += cost;
//    }

    // evaluate inverse dynamics:
    double state_torque_cost = 0.0;

//    if (parameters_->getTorqueCostWeight() > 1e-9)
//    {
//      full_trajectory_->getTrajectoryPointKDL(i, kdl_joint_array_);
//      full_trajectory_->getJointVelocities(i, full_joint_state_velocities_);
//      full_trajectory_->getJointAccelerations(i, full_joint_state_velocities_);
//      for (int j=0; j<num_joints_; ++j)
//      {
//        int full_joint_num = planning_group_->stomp_joints_[j].kdl_joint_index_;
//        kdl_group_joint_array_(j) = kdl_joint_array_(full_joint_num);
//        kdl_group_vel_joint_array_(j) = full_joint_state_velocities_(full_joint_num);
//        kdl_group_acc_joint_array_(j) = full_joint_state_accelerations_(full_joint_num);
//      }
//      planning_group_->id_solver_->CartToJnt(kdl_group_joint_array_,
//                                             kdl_group_vel_joint_array_,
//                                             kdl_group_acc_joint_array_,
//                                             wrenches,
//                                             kdl_group_torque_joint_array_);
//      getTorques(i, torques, wrenches);
//      for (int j=0; j<num_joints_; ++j)
//      {
//        state_torque_cost += fabs(torques[j]);
//      }
//    }

    obstacle_cost += parameters_->getObstacleCostWeight() * state_collision_cost;
    constraint_cost += parameters_->getConstraintCostWeight() * state_constraint_cost;
    torque_cost += parameters_->getTorqueCostWeight() * state_torque_cost;

    costs(i-free_vars_start_) =
        parameters_->getObstacleCostWeight() * state_collision_cost +
        parameters_->getConstraintCostWeight() * state_constraint_cost +
        parameters_->getTorqueCostWeight() * state_torque_cost;
  }

  last_trajectory_cost_ = costs.sum();
  //last_trajectory_constraints_satisfied_ = (constraint_cost < 1e-6);

  //printf("Obstacle cost = %f, constraint cost = %f, torque_cost = %f\n", obstacle_cost, constraint_cost, torque_cost);
  //animateEndeffector();
  //ros::spinOnce();
  //ros::Duration(1.0).sleep();

  //ROS_INFO("Rollout took %f seconds, ", (ros::WallTime::now() - start_time).toSec());
  return true;
}

bool StompOptimizer::getPolicy(boost::shared_ptr<stomp_motion_planner::Policy>& policy)
{
  policy = policy_;
  return true;
}

bool StompOptimizer::setPolicy(const boost::shared_ptr<stomp_motion_planner::Policy> policy)
{
  return true;
}

bool StompOptimizer::getControlCostWeight(double& control_cost_weight)
{
  control_cost_weight = parameters_->getSmoothnessCostWeight();
  return true;
}

void StompOptimizer::copyPolicyToGroupTrajectory()
{
  policy_->getParameters(policy_parameters_);
  for (int d=0; d<num_joints_; ++d)
  {
    group_trajectory_.getFreeJointTrajectoryBlock(d) = policy_parameters_[d];
  }
}

void StompOptimizer::copyGroupTrajectoryToPolicy()
{
  for (int d=0; d<num_joints_; ++d)
  {
    policy_parameters_[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
  }
  policy_->setParameters(policy_parameters_);
}

void StompOptimizer::setSharedPtr(boost::shared_ptr<StompOptimizer>& ptr)
{
  this_shared_ptr_ = ptr;
}

void StompOptimizer::resetSharedPtr()
{
  this_shared_ptr_.reset();
}


} // namespace stomp

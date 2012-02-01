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

#ifndef STOMP_OPTIMIZER_H_
#define STOMP_OPTIMIZER_H_

#include <boost/shared_ptr.hpp>
#include "stompParameters.hpp"
#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/Greedy/CollisionSpace.hpp"
#include "task.hpp"
#include "covariant_trajectory_policy.hpp"
#include "policy_improvement_loop.hpp"
#include "stompStatistics.hpp"

//#include <motion_planning_msgs/Constraints.h>
//#include "constraint_evaluator.hpp"
//#include <stomp_motion_planner/STOMPStatistics.h>
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

#include <vector>
//#include <kdl/frames.hpp>
//#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace stomp_motion_planner
{

class StompOptimizer: public Task
{
public:
  StompOptimizer(ChompTrajectory *trajectory, 
                 const StompParameters *parameters, 
                 const ChompPlanningGroup *planning_group,
                 const CollisionSpace *collision_space);
  
//  StompOptimizer(StompTrajectory *trajectory, const StompRobotModel *robot_model,
//      const StompRobotModel::StompPlanningGroup *planning_group, const StompParameters *parameters,
//      const ros::Publisher& vis_marker_array_publisher,
//      const ros::Publisher& vis_marker_publisher,
//      const ros::Publisher& stats_publisher,
//      StompCollisionSpace *collision_space,
//      const motion_planning_msgs::Constraints& constraints);
  
  virtual ~StompOptimizer();

  /**
   * Main optimizer function
   * @param nbIteration max number of iteration
   * @param idRun
   * @return
   */
  void runDeformation( int nbIteration , int idRun=0 );
  
  /**
   * Generate the noisy trajectories
   * @param num_time_steps
   */
  void generateNoisyTrajectory(const API::Trajectory& traj, std::vector< std::vector<confPtr_t> >& noisy_trajectory);
  
  /**
   * Functions to set the shared pointer
   */
  void setSharedPtr(boost::shared_ptr<StompOptimizer>& ptr);
  void resetSharedPtr();
  
  /**
   * Test the noisy trajectory sampler
   */
  void testMultiVariateGaussianSampler();
  
  /**
   * Get the current trajectory cost profile
   */
  void getTrajectoryCost( std::vector<double>& cost, double step );
  
  /**
   * Get the current joint violation
   */
  int getJointLimitViolations() { return joint_limits_violation_; }
  bool getJointLimitViolationSuccess() { return succeded_joint_limits_; }

  // stuff derived from Task:
  /**
   * Initializes the task for a given number of time steps
   * @param num_time_steps
   * @return
   */
  bool initialize(/*ros::NodeHandle& node_handle,*/ int num_time_steps);

  /**
   * Executes the task for the given policy parameters, and returns the costs per timestep
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
   * @return
   */
  bool execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number);

  /**
   * Get the Policy object of this Task
   * @param policy
   * @return
   */
  bool getPolicy(boost::shared_ptr<stomp_motion_planner::Policy>& policy);

  /**
   * Sets the Policy object of this Task
   * @param policy
   * @return
   */
  bool setPolicy(const boost::shared_ptr<stomp_motion_planner::Policy> policy);

  /**
   * Gets the weight of the control cost
   * @param control_cost_weight
   * @return
   */
  bool getControlCostWeight(double& control_cost_weight);
  
  /**
   * Returns the planning group of the optimizer
   */
  const ChompPlanningGroup* getPlanningGroup() const { return planning_group_; } 
  
  /**
   * Draw function to be called outside
   */
  void draw();


private:
  Robot* robot_model_;
  
  int num_joints_;
  int num_vars_free_;
  int num_vars_all_;
  int num_collision_points_;
  int free_vars_start_;
  int free_vars_end_;
  int iteration_;
  int collision_free_iteration_;
  
  bool succeded_joint_limits_;
  int joint_limits_violation_;
  
  ChompTrajectory *full_trajectory_;
  const ChompPlanningGroup *planning_group_;
  const StompParameters *stomp_parameters_;
  const CollisionSpace *collision_space_;
  ChompTrajectory group_trajectory_;
  std::vector<ChompCost> joint_costs_;

  boost::shared_ptr<stomp_motion_planner::CovariantTrajectoryPolicy> policy_;
  std::vector<Eigen::VectorXd> policy_parameters_;
  boost::shared_ptr<StompOptimizer> this_shared_ptr_;
  
  std::vector<int> planner_p3d_joints_;
  std::vector<int> group_joint_to_move3d_joint_index_;

//  std::vector<std::vector<KDL::Vector> > joint_axis_;
//  std::vector<std::vector<KDL::Vector> > joint_pos_;
//  std::vector<std::vector<KDL::Frame> > segment_frames_;
//  std::vector<std::vector<KDL::Vector> > collision_point_pos_;
//  std::vector<std::vector<KDL::Vector> > collision_point_vel_;
//  std::vector<std::vector<KDL::Vector> > collision_point_acc_;
  
  std::vector<std::vector<std::vector<double> > > segment_frames_;
  std::vector<std::vector<Eigen::Vector3d> >  joint_axis_eigen_;
  std::vector<std::vector<Eigen::Vector3d> >  joint_pos_eigen_;
  std::vector<std::vector<Eigen::Vector3d> >  collision_point_pos_eigen_;
  std::vector<std::vector<Eigen::Vector3d> >  collision_point_vel_eigen_;
  std::vector<std::vector<Eigen::Vector3d> >  collision_point_acc_eigen_;

  Eigen::VectorXd general_cost_potential_;
  
  Eigen::MatrixXd collision_point_potential_;
  Eigen::MatrixXd collision_point_vel_mag_;
  std::vector<std::vector<Eigen::Vector3d> > collision_point_potential_gradient_;
  Eigen::MatrixXd group_trajectory_backup_;
  Eigen::MatrixXd best_group_trajectory_;
  double best_group_trajectory_cost_;
  double last_trajectory_cost_;
  bool last_trajectory_collision_free_;
  bool last_trajectory_constraints_satisfied_;
  int last_improvement_iteration_;

  // HMC stuff:
  Eigen::MatrixXd momentum_;
  Eigen::MatrixXd random_momentum_;
  Eigen::VectorXd random_joint_momentum_; //temporary variable
  std::vector<MultivariateGaussian> multivariate_gaussian_;
  double stochasticity_factor_;

  std::vector<int> state_is_in_collision_;      /**< Array containing a boolean about collision info for each point in the trajectory */
  std::vector<std::vector<int> > point_is_in_collision_;
  bool is_collision_free_;
  double worst_collision_cost_state_;

  Eigen::MatrixXd smoothness_increments_;
  Eigen::MatrixXd collision_increments_;
  Eigen::MatrixXd final_increments_;

  // temporary variables for all functions:
  Eigen::VectorXd smoothness_derivative_;
//  KDL::JntArray kdl_joint_array_;
//  KDL::JntArray kdl_vel_joint_array_;
//  KDL::JntArray kdl_acc_joint_array_;
//
//  KDL::JntArray kdl_group_joint_array_;
//  KDL::JntArray kdl_group_vel_joint_array_;
//  KDL::JntArray kdl_group_acc_joint_array_;
//  KDL::JntArray kdl_group_torque_joint_array_;
  
  //Eigen::VectorXd
  Eigen::MatrixXd jacobian_;
  Eigen::MatrixXd jacobian_pseudo_inverse_;
  Eigen::MatrixXd jacobian_jacobian_tranpose_;
  Eigen::VectorXd random_state_;
  Eigen::VectorXd joint_state_velocities_;
  Eigen::VectorXd joint_state_accelerations_;
  Eigen::VectorXd full_joint_state_velocities_;
  Eigen::VectorXd full_joint_state_accelerations_;

//  ros::Publisher vis_marker_array_pub_;
//  ros::Publisher vis_marker_pub_;
//  ros::Publisher stats_pub_;
  int animate_endeffector_segment_number_;

//  motion_planning_msgs::Constraints constraints_;
//  std::vector<boost::shared_ptr<ConstraintEvaluator> > constraint_evaluators_;
  boost::shared_ptr<StompStatistics>  stomp_statistics_;

  void initialize();
  void initPolicy();
  void calculateSmoothnessIncrements();
  void calculateCollisionIncrements();
  void calculateTotalIncrements();
  void getFrames(int segment, const Eigen::VectorXd& joint_array, Configuration& q);
  bool getConfigObstacleCost(int segment, int dof, Configuration& q);
  bool performForwardKinematics(); /**< Return true if collision free */
  void addIncrementsToTrajectory();
  void updateFullTrajectory();
  void debugCost();
  void eigenMapTest();
  bool handleJointLimits();
  void animatePath();
  void animateEndeffector();
  void setGroupTrajectoryToVectorConfig(std::vector<confPtr_t>& traj);
  void visualizeState(int index);
  double getTrajectoryCost();
  double getSmoothnessCost();
  double getCollisionCost();
  void perturbTrajectory();
  void getRandomMomentum();
  void updateMomentum();
  void updatePositionFromMomentum();
  void calculatePseudoInverse();

  void doChompOptimization();

  void copyPolicyToGroupTrajectory();
  void copyGroupTrajectoryToPolicy();

  void clearAnimations();

//  void getTorques(int index, std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches);
};

}

extern boost::shared_ptr<stomp_motion_planner::StompOptimizer> optimizer;

#endif /* STOMP_OPTIMIZER_H_ */

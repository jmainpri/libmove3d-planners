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
////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/LU>

#include "run_parallel_stomp.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include "feature_space/features.hpp"
#include "feature_space/planar_feature.hpp"
#include "feature_space/smoothness.hpp"

#include "utils/ConfGenerator.h"

#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/HRICS_legibility.hpp"
#include "hri_costspace/HRICS_parameters.hpp"
#include "hri_costspace/gestures/HRICS_human_prediction_cost_space.hpp"

#include <libmove3d/include/Graphic-pkg.h>
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Util-pkg.h>

#include "move3d-headless.h"

#include <iomanip>
#include <boost/shared_ptr.hpp>
#include <sys/time.h>
#include <fstream>
#include <unistd.h>

using namespace std;
using namespace Move3D;

using namespace Eigen;

const double hack_tweek = 1;

//--------------------------------------------------------
// External
//--------------------------------------------------------
MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer>
    global_optimizer;

std::map<Robot*, std::vector<double> > global_MultiStomplinesColors;
std::map<Robot*, std::vector<Eigen::Vector3d> > global_MultiStomplinesToDraw;

//--------------------------------------------------------
// Collision space
//--------------------------------------------------------

static bool use_move3d_functions = true;

void move3d_set_api_functions_collision_space(bool use_move3d_fct) {
  use_move3d_functions = use_move3d_fct;
}

bool move3d_use_api_functions_collision_space() { return use_move3d_functions; }

//--------------------------------------------------------
// External
//--------------------------------------------------------

namespace stomp_motion_planner {
StompOptimizer::StompOptimizer(ChompTrajectory* trajectory,
                               const StompParameters* parameters,
                               const ChompPlanningGroup* planning_group,
                               const CollisionSpace* collision_space,
                               int id)
    :  // The id can keep tack for multi thread cases
      id_(id),
      use_time_limit_(false),
      use_iteration_limit_(false),
      full_trajectory_(trajectory),
      planning_group_(planning_group),
      stomp_parameters_(parameters),
      move3d_collision_space_(collision_space),
      use_external_collision_space_(false),
      group_trajectory_(*full_trajectory_, DIFF_RULE_LENGTH) {
  if (use_move3d_functions) {
    move3d_set_fct_get_nb_collision_points(
        boost::bind(&StompOptimizer::getNumberOfCollisionPoints, this, _1),
        id_);
    move3d_set_fct_get_config_collision_cost(
        boost::bind(
            &StompOptimizer::getConfigObstacleCost, this, _1, _2, _3, _4),
        id_);
    collision_space_id_ = id_;
  } else {
    use_external_collision_space_ = true;
    collision_space_id_ = 0;
  }

  // Use costspace
  use_costspace_ = (move3d_collision_space_ == NULL);
  use_costspace_ |=
      (use_costspace_ && (use_external_collision_space_ == false));

  use_buffer_ = false;
}

void StompOptimizer::initialize() {
  if (!PlanEnv->getBool(PlanParam::trajStompNoPrint)) {
    cout << "-----------------------------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;
  }

  use_collision_free_limit_limit_ =
      stomp_parameters_->getStopWhenCollisionFree();

  Scene* sce = global_Project->getActiveScene();

  //    robot_model_ = group_trajectory_.getRobot();
  robot_model_ = planning_group_->robot_;
  human_model_ = sce->getRobotByNameContaining("HERAKLES");

  // Handover end configuration generator
  use_human_sliders_ = PlanEnv->getBool(PlanParam::trajMoveHuman);
  use_handover_config_generator_ = PlanEnv->getBool(PlanParam::trajUseOtp);
  use_handover_config_list_ = false;
  use_handover_auto_ = true;
  recompute_handover_cell_list_ = true;
  handoverGenerator_ = NULL;

  human_has_moved_ = false;
  last_human_pos_.resize(4);
  last_human_pos_[0] = 0;
  last_human_pos_[1] = 0;
  last_human_pos_[2] = 0;
  last_human_pos_[3] = 0;

  reset_reused_rollouts_ = false;

  if (use_handover_config_generator_) {
    initHandover();
  }

  // TODO move it somewhere else
  if (PlanEnv->getBool(PlanParam::useLegibleCost)) {
    HRICS_activeLegi = new HRICS::Legibility();
    Robot* goal_0 = global_Project->getActiveScene()->getRobotByName("GOAL_0");
    Robot* goal_1 = global_Project->getActiveScene()->getRobotByName("GOAL_1");

    cout << "goal 0 : "
         << goal_0->getCurrentPos()->getEigenVector(6, 7).transpose() << endl;

    cout << "goal 1 : "
         << goal_1->getCurrentPos()->getEigenVector(6, 7).transpose() << endl;

    HRICS_activeLegi->addGoal(goal_0->getCurrentPos()->getEigenVector(6, 7));
    HRICS_activeLegi->addGoal(goal_1->getCurrentPos()->getEigenVector(6, 7));
  }

  // Move the end configuration in the trajectory
  id_fixed_ = 2;
  allow_end_configuration_motion_ =
      PlanEnv->getBool(PlanParam::trajStompMoveEndConfig);
  if (allow_end_configuration_motion_) {
    id_fixed_ = 1;
  }
  full_trajectory_->setFixedId(id_fixed_);
  group_trajectory_.setFixedId(id_fixed_);

  // init some variables:
  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();

  if (!PlanEnv->getBool(PlanParam::trajStompNoPrint)) {
    cout << "num_vars_free_ : " << num_vars_free_ << endl;
    cout << "num_vars_all_ : " << num_vars_all_ << endl;
  }

  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  // ROS_INFO_STREAM("Setting free vars start to " << free_vars_start_ << " end
  // " << free_vars_end_);

  // set up joint index:
  group_joint_to_move3d_joint_index_.resize(num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    group_joint_to_move3d_joint_index_[i] =
        planning_group_->chomp_dofs_[i].move3d_joint_index_;
  }

  // TODO ADD cost computation here !!!
  //    num_collision_points_ =
  //    Move3DGetNumberOfCollisionPoints[collision_space_id_]( robot_model_ );
  //    cout << "num of collision points  : " << num_collision_points_ << endl;

  num_collision_points_ = getNumberOfCollisionPoints(robot_model_);

  // set up the joint costs:
  joint_costs_.reserve(num_joints_);

  double max_cost_scale = 0.0;
  //  ros::NodeHandle nh("~");
  for (int i = 0; i < num_joints_; i++) {
    double joint_cost = 1.0;
    // std::string joint_name = planning_group_->chomp_dofs_[i].joint_name_;
    //    nh.param("joint_costs/"+joint_name, joint_cost, 1.0);
    std::vector<double> derivative_costs(3);
    derivative_costs[0] =
        joint_cost * stomp_parameters_->getSmoothnessCostVelocity();
    derivative_costs[1] =
        joint_cost * stomp_parameters_->getSmoothnessCostAcceleration();
    derivative_costs[2] =
        joint_cost * stomp_parameters_->getSmoothnessCostJerk();
    //        cout << "deriv cost for joint(" << i << ") : " <<
    //        derivative_costs[0] << " " << derivative_costs[1] << " " <<
    //        derivative_costs[2] << endl;

    joint_costs_.push_back(ChompCost(group_trajectory_.getNumPoints(),
                                     i,
                                     derivative_costs,
                                     stomp_parameters_->getRidgeFactor()));
    double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
    if (max_cost_scale < cost_scale) max_cost_scale = cost_scale;
  }

  // scale the smoothness costs
  for (int i = 0; i < num_joints_; i++) {
    joint_costs_[i].scale(max_cost_scale);
  }

  //    compute_fk_main_->getCollisionCostPotential() =
  //    Eigen::VectorXd::Zero(num_vars_all_);
  //    compute_fk_main_->getDts() = Eigen::VectorXd::Zero(num_vars_all_);

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
  //  full_joint_state_velocities_ =
  //  Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());
  //  full_joint_state_accelerations_ =
  //  Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());

  group_trajectory_backup_ = group_trajectory_.getTrajectory();
  best_group_trajectory_ = group_trajectory_.getTrajectory();

  source_ = getSource();
  target_ = getTarget();

  cout << "use_costspace_ : " << use_costspace_ << endl;

  //    compute_fk_main_->getJointAxisEigen().resize(num_vars_all_);
  //    compute_fk_main_->getJointPosEigen().resize(num_vars_all_);
  //    compute_fk_main_->getCollisionPointPosEigen().resize(num_vars_all_);
  //    compute_fk_main_->getCollisionPointVelEigen().resize(num_vars_all_);
  //    compute_fk_main_->getCollisionPointAccEigen().resize(num_vars_all_);

  //    compute_fk_main_->getSegmentFrames().resize( num_vars_all_ );

  //    for(int i=0; i<num_vars_all_;i++)
  //    {
  //        compute_fk_main_->getSegmentFrames()[i].resize(planning_group_->num_dofs_);
  //        compute_fk_main_->getJointPosEigen()[i].resize(planning_group_->num_dofs_);
  //    }

  //    if (num_collision_points_ > 0)
  //    {
  //        compute_fk_main_->getCollisionPointPotential() =
  //        Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
  //        compute_fk_main_->getCollisionVelMag() =
  //        Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
  //        compute_fk_main_->getCollisionPointPotentialGradient().resize(num_vars_all_,
  //        std::vector<Eigen::Vector3d>(num_collision_points_));

  //        for(int i=0; i<num_vars_all_;i++)
  //        {
  //            compute_fk_main_->getJointAxisEigen()[i].resize(num_collision_points_);
  //            compute_fk_main_->getJointPosEigen()[i].resize(num_collision_points_);
  //            compute_fk_main_->getCollisionPointPosEigen()[i].resize(num_collision_points_);
  //            compute_fk_main_->getCollisionPointVelEigen()[i].resize(num_collision_points_);
  //            compute_fk_main_->getCollisionPointAccEigen()[i].resize(num_collision_points_);
  //        }
  //    }

  collision_free_iteration_ = 0;
  is_collision_free_ = false;
  state_is_in_collision_.resize(num_vars_all_);
  point_is_in_collision_.resize(num_vars_all_,
                                std::vector<int>(num_collision_points_));

  last_improvement_iteration_ = -1;

  // HMC initialization:
  momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
  random_joint_momentum_ = Eigen::VectorXd::Zero(num_vars_free_);
  multivariate_gaussian_.clear();
  stochasticity_factor_ = 1.0;
  for (int i = 0; i < num_joints_; i++) {
    multivariate_gaussian_.push_back(
        MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_),
                             joint_costs_[i].getQuadraticCostInverse()));
  }

  // animation init:
  //  animate_endeffector_segment_number_ =
  //  robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(stomp_parameters_->getAnimateEndeffectorSegment());

  // initialize the policy
  initPolicy();

  // initialize the constraints:
  //  for (int i=0; i<int(constraints_.orientation_constraints.size()); ++i)
  //  {
  //    MOVE3D_BOOST_PTR_NAMESPACE<OrientationConstraintEvaluator> eval(new
  //    OrientationConstraintEvaluator(
  //        constraints_.orientation_constraints[i], *robot_model_));
  //    constraint_evaluators_.push_back(eval);
  //  }

  // Construct fk function
  compute_fk_main_ = MOVE3D_BOOST_PTR_NAMESPACE<costComputation>(
      new costComputation(robot_model_,
                          move3d_collision_space_,
                          planning_group_,
                          joint_costs_,
                          group_trajectory_,
                          stomp_parameters_->getObstacleCostWeight(),
                          use_costspace_,
                          source_,
                          target_,
                          use_external_collision_space_,
                          collision_space_id_,
                          stomp_parameters_,
                          policy_));
}

int StompOptimizer::getNumberOfCollisionPoints(Move3D::Robot* R) {
  return planning_group_->collision_points_.size();
}

bool StompOptimizer::initializeFromNewTrajectory(
    const Move3D::Trajectory& traj) {
  cout << "initialize stomp from new trajectory" << endl;

  if (traj.getRobot()->getName() != robot_model_->getName()) {
    cout << "Can not initialize stomp with this trajectory" << endl;
    return false;
  }

  double s(0.0), param_max(traj.getParamMax()),
      step(param_max / num_vars_free_);

  vector<confPtr_t> configs;

  for (int i = 0; i < num_vars_free_; i++) {
    configs.push_back(traj.configAtParam(s));
    s += step;
  }

  Move3D::Trajectory T(configs);

  delete full_trajectory_;
  full_trajectory_ = new ChompTrajectory(
      T,
      DIFF_RULE_LENGTH,
      *planning_group_,
      PlanEnv->getDouble(PlanParam::trajDuration));  // No Duration
  group_trajectory_ = ChompTrajectory(*full_trajectory_, DIFF_RULE_LENGTH);

  // initialize the policy trajectory
  Eigen::VectorXd start =
      group_trajectory_.getTrajectoryPoint(free_vars_start_ - 1).transpose();
  Eigen::VectorXd end =
      group_trajectory_.getTrajectoryPoint(free_vars_end_ + 1).transpose();

  // set paramters
  int free_vars_start_index = DIFF_RULE_LENGTH - 1;
  vector<Eigen::VectorXd> parameters(num_joints_);
  for (int i = 0; i < num_joints_; i++) {
    parameters[i] = group_trajectory_.getJointTrajectory(i)
                        .segment(free_vars_start_index, num_vars_free_);
  }
  policy_->setToMinControlCost(start, end);
  policy_->setParameters(parameters);

  if (use_handover_config_generator_) {
    initHandover();
  }

  return true;
}

void StompOptimizer::initHandover() {
  recompute_handover_once_ = true;
  handover_has_been_recomputed_ = false;

  use_human_sliders_ = PlanEnv->getBool(PlanParam::trajMoveHuman);

  const char* home = getenv("HOME_MOVE3D");
  if (home == NULL) {
    cout << "ERROR home is not defined for config generator in "
         << __PRETTY_FUNCTION__ << endl;
  }

  string dir(home);
  string file("/statFiles/OtpComputing/confHerakles.xml");

  handoverGenerator_ = new Move3D::ConfGenerator(robot_model_, human_model_);
  handoverGenerator_->initialize(dir + file, HRICS_activeNatu);

  if (ENV.getInt(Env::setOfActiveJoints) == 2) {
    use_handover_config_list_ = true;
  }

  HRI_AGENT* robot =
      hri_get_agent_by_name(GLOBAL_AGENTS, robot_model_->getName().c_str());
  hri_activate_coll_robot_and_all_humans_arms(robot, GLOBAL_AGENTS, false);
}

StompOptimizer::~StompOptimizer() {
  delete handoverGenerator_;

  for (int i = 0; i < int(compute_fk_.size()); i++) {
    delete compute_fk_[i];
  }

  cout << "destroy stomp" << endl;
}

void StompOptimizer::doChompOptimization() {
  calculateSmoothnessIncrements();
  calculateCollisionIncrements();
  calculateTotalIncrements();

  if (!stomp_parameters_->getUseHamiltonianMonteCarlo()) {
    // non-stochastic version:
    addIncrementsToTrajectory();
  } else {
    // hamiltonian monte carlo updates:
    getRandomMomentum();
    updateMomentum();
    updatePositionFromMomentum();
    stochasticity_factor_ *= stomp_parameters_->getHmcAnnealingFactor();
  }

  handleJointLimits();
  updateFullTrajectory();
  last_trajectory_collision_free_ = performForwardKinematics(false);
  last_trajectory_constraints_satisfied_ = true;

  if (!last_trajectory_collision_free_ &&
      stomp_parameters_->getAddRandomness()) {
    performForwardKinematics(false);
    double original_cost = getTrajectoryCost();
    group_trajectory_backup_ = group_trajectory_.getTrajectory();
    perturbTrajectory();
    handleJointLimits();
    updateFullTrajectory();
    performForwardKinematics(false);
    double new_cost = getTrajectoryCost();

    if (new_cost > original_cost) {
      // printf("Random jump worsened cost from %.10f to %.10f!\n",
      // original_cost, new_cost);
      group_trajectory_.getTrajectory() = group_trajectory_backup_;
      updateFullTrajectory();
    }
  }

  // Tajectory cost calculation
  Eigen::VectorXd costs =
      Eigen::VectorXd::Zero(free_vars_end_ - free_vars_start_ + 1);

  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    double state_collision_cost = 0.0;
    double cumulative = 0.0;

    if (move3d_collision_space_ || use_external_collision_space_) {
      for (int j = 0; j < num_collision_points_; j++) {
        cumulative += compute_fk_main_->getCollisionPointPotential()(
            i, j) /* compute_fk_main_->getCollisionVelMag()(i,j)*/;
        state_collision_cost += cumulative;
      }
    } else {
      Eigen::VectorXd q_i, q_f;

      int id1 = i;
      int id2 = i + 1;

      if (i == free_vars_end_) {
        id1--;
        id2--;
      }

      int index_i = group_trajectory_.getFullTrajectoryIndex(id1);
      int index_f = group_trajectory_.getFullTrajectoryIndex(id2);

      full_trajectory_->getTrajectoryPointP3d(index_i, q_i);
      full_trajectory_->getTrajectoryPointP3d(index_f, q_f);

      state_collision_cost +=
          (pow(compute_fk_main_->getGeneralCostPotential()(i),
               hack_tweek) /* ( q_f - q_i ).norm() */);
    }
    costs(i - free_vars_start_) =
        stomp_parameters_->getObstacleCostWeight() * state_collision_cost;
  }

  last_trajectory_cost_ = costs.sum();
}

// void StompOptimizer::optimize()
void StompOptimizer::runDeformation(int nbIteration, int idRun) {
  StackedFeatures* fct =
      dynamic_cast<StackedFeatures*>(global_activeFeatureFunction);
  if (fct != NULL && fct->getFeatureFunction("Distance") != NULL)
    fct->printInfo();

  ChronoTimeOfDayOn();

  timeval tim;
  gettimeofday(&tim, NULL);
  double t_init = tim.tv_sec + (tim.tv_usec / 1000000.0);
  time_ = 0.0;

  // Print the active feature space out
  if (!PlanEnv->getBool(PlanParam::trajStompNoPrint))
    if (use_costspace_ && global_activeFeatureFunction != NULL)
      global_activeFeatureFunction->printInfo();

  stomp_statistics_ =
      MOVE3D_BOOST_PTR_NAMESPACE<StompStatistics>(new StompStatistics());
  stomp_statistics_->run_id = idRun;
  stomp_statistics_->collision_success_iteration = -1;
  stomp_statistics_->success_iteration = -1;
  stomp_statistics_->success = false;
  stomp_statistics_->costs.clear();
  stomp_statistics_->convergence_rate.clear();
  stomp_statistics_->convergence_trajs.clear();

  // Clear all trajs
  all_move3d_traj_.clear();

  move3d_traj_ = Move3D::Trajectory(robot_model_);
  traj_convergence_with_time.clear();

  if (!PlanEnv->getBool(PlanParam::trajStompNoPrint))
    cout << "use_time_limit_ : " << use_time_limit_ << endl;

  int ith_save = 1;

  PolicyImprovementLoop pi_loop;
  pi_loop.initialize(/*nh,*/ this_shared_ptr_,
                     false,
                     group_trajectory_.getUseTime()
                         ? group_trajectory_.getDiscretization()
                         : 0.0);

  // group_trajectory_.print();

  // Initialize buffer for control cost computation
  if (use_buffer_) {
    policy_->setBuffer(buffer_);
  }

  iteration_ = 0;
  copyPolicyToGroupTrajectory();
  handleJointLimits();
  updateFullTrajectory();

  if (!PlanEnv->getBool(PlanParam::trajStompNoPrint))
    cout << "compute_fk_main_->getSegmentFrames().size() : "
         << compute_fk_main_->getSegmentFrames().size() << endl;

  if (global_costSpace != NULL &&
      global_costSpace->getSelectedCostName() == "costHumanWorkspaceOccupancy")
    global_humanPredictionCostSpace->computeCurrentOccupancy();

  cout << "before fk" << endl;

  performForwardKinematics(false);

  cout << "after fk" << endl;

  // Print smoothness cost
  getSmoothnessCost();
  getGeneralCost();

  // group_trajectory_.print();

  //    last_move3d_cost_ = computeMove3DCost();
  //    printf("%3d, time: %f, cost: %f\n", 0, 0.0, last_move3d_cost_ );
  //    traj_convergence_with_time.push_back( make_pair( 0.0, last_move3d_cost_
  //    ) );

  // Clear trajs
  global_trajToDraw.clear();

  if ((!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) &&
      stomp_parameters_->getAnimateEndeffector())
    animateEndeffector();

  if (PlanEnv->getBool(PlanParam::trajSaveCost)) saveTrajectoryCostStats();

  //    cout << "wait for key" << endl;
  //    cin.ignore();

  double intitialization_time = 0.0;
  ChronoTimeOfDayTimes(&intitialization_time);
  ChronoTimeOfDayOff();

  if (!PlanEnv->getBool(PlanParam::trajStompNoPrint))
    cout << "Stomp init time : " << intitialization_time << endl;

  // double last_move3d_cost = numeric_limits<double>::max();

  // CHANGE TO initialization
  //    source_ = getSource();
  //    target_ = getTarget();

  best_iteration_ = 0;
  last_improvement_iteration_ = -1;

  best_group_trajectory_cost_ = numeric_limits<double>::max();
  best_group_trajectory_in_collsion_cost_ = numeric_limits<double>::max();

  confPtr_t q_tmp = robot_model_->getCurrentPos();

  cout << "Start stomp loop" << endl;

  // iterate
  for (iteration_ = 0; !PlanEnv->getBool(PlanParam::stopPlanner);
       iteration_++) {
    // Compute wether to draw at a certain iteration
    bool do_draw = false;
    if (PlanEnv->getInt(PlanParam::stompDrawIteration) > 0) {
      do_draw =
          (iteration_ != 0) &&
          (iteration_ % PlanEnv->getInt(PlanParam::stompDrawIteration) == 0);
    }

    if (global_costSpace != NULL &&
        global_costSpace->getSelectedCostName() ==
            "costHumanWorkspaceOccupancy")
      global_humanPredictionCostSpace->computeCurrentOccupancy();

    reset_reused_rollouts_ = false;
    if ((!handover_has_been_recomputed_) && humanHasMoved()) {
      cout << "Human has moved" << endl;
      reset_reused_rollouts_ = true;

      if (reset_reused_rollouts_) {
        pi_loop.resetReusedRollouts();
        reset_reused_rollouts_ = false;
      }

      // Recompute the handover position
      if (use_handover_config_generator_) {
        if (replaceEndWithNewConfiguration()) {
          q_tmp = target_new_;
          copyGroupTrajectoryToPolicy();
          if (recompute_handover_once_) {
            handover_has_been_recomputed_ = true;
          }
          cout << "copyGroupTrajectoryToPolicy" << endl;
        }
      }
    }

    // cout << "run single iteration" << endl;

    // Policy improvement loop
    // pi_loop.runSingleIteration(iteration_+1);
    if (!stomp_parameters_->getUseChomp()) {
      pi_loop.runSingleIteration(iteration_ + 1);
      //            cout << "group_trajectory_.getUseTime() : "
      // << group_trajectory_.getUseTime() << endl;

    } else {
      doChompOptimization();
    }

    // all_move3d_traj_.push_back( Move3D::Trajectory( robot_model_ ) );
    // setGroupTrajectoryToApiTraj( all_move3d_traj_.back() );
    // global_trajToDraw.push_back( all_move3d_traj_.back() );

    double cost = getTrajectoryCost();

    // ( PlanEnv->getBool(PlanParam::trajStompDrawImprovement) &&
    // ( cost < best_group_trajectory_in_collsion_cost_ ))

    if (do_draw && (!ENV.getBool(Env::drawDisabled)) &&
        ENV.getBool(Env::drawTraj) &&
        stomp_parameters_->getAnimateEndeffector()) {
      move3d_draw_clear_handles(robot_model_);
      robot_model_->setAndUpdate(*q_tmp);

      animateEndeffector();

      //            if( !ENV.getBool(Env::drawTrajVector) )
      //                animateEndeffector();
      //            else
      //                g3d_draw_allwin_active();

      // animateTrajectoryPolicy();
    }

    // double cost = last_trajectory_cost_;
    stomp_statistics_->costs.push_back(cost);

    if (reset_reused_rollouts_) {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = cost;
      cout << "Change best to actual current trajectory" << endl;
    }

    if (last_trajectory_collision_free_ &&
        last_trajectory_constraints_satisfied_)
      collision_free_iteration_++;
    else
      collision_free_iteration_ = 0;

    if (last_trajectory_collision_free_ &&
        stomp_statistics_->collision_success_iteration == -1) {
      stomp_statistics_->collision_success_iteration = iteration_;
    }
    if (last_trajectory_collision_free_ &&
        last_trajectory_constraints_satisfied_ &&
        stomp_statistics_->success_iteration == -1) {
      stomp_statistics_->success_iteration = iteration_;
      stomp_statistics_->success = true;
      stomp_statistics_->success_time = time_;
    }

    if (iteration_ == 0) {
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost_ = cost;
    } else {
      //            cout << "cost : " << cost << endl;
      //            cout << "best traj group : " << best_group_trajectory_cost_
      //            << endl;

      if (cost < best_group_trajectory_cost_) {
        if (is_collision_free_) {
          if (!PlanEnv->getBool(PlanParam::trajStompNoPrint))
            cout << "New best" << endl;

          best_group_trajectory_ = group_trajectory_.getTrajectory();
          best_group_trajectory_cost_ = cost;
          last_improvement_iteration_ = iteration_;
          best_iteration_++;
        } else {
          if (cost < best_group_trajectory_in_collsion_cost_) {
            if (!PlanEnv->getBool(PlanParam::trajStompNoPrint))
              cout << "New best in collision" << endl;

            last_improvement_iteration_ = iteration_;
            best_group_trajectory_in_collision_ =
                group_trajectory_.getTrajectory();
            best_group_trajectory_in_collsion_cost_ = cost;
          }
        }
      }
    }

    // if (iteration_%1==0)
    gettimeofday(&tim, NULL);
    time_ = tim.tv_sec + (tim.tv_usec / 1000000.0) - t_init;

    if (PlanEnv->getBool(PlanParam::trajSaveCost) &&
        /**time_>double(ith_save)*/ do_draw) {
      saveTrajectoryCostStats();
      ith_save++;
    }

    //        cout << "move3d_cost : " << computeMove3DCost() << endl;
    // last_move3d_cost_ = computeMove3DCost();

    double move3d_cost = 0.0;

    if (PlanEnv->getBool(PlanParam::drawParallelTraj) &&
        (global_stompRun != NULL)) {
      ENV.setBool(Env::drawTraj, false);
      global_stompRun->lockDraw();
      robot_model_->setAndUpdate(*q_tmp);
      saveEndeffectorTraj();
      global_stompRun->unlockDraw();
    }

    // save the cost and time as pair
    // traj_convergence_with_time.push_back( make_pair( time_, move3d_cost ) );

    // cout << "PID : " << getpid() << endl;

    if (!PlanEnv->getBool(PlanParam::trajStompNoPrint)) {
      if (!ENV.getBool(Env::drawDisabled)) {
        if ((ENV.getBool(Env::drawTraj) ||
             PlanEnv->getBool(PlanParam::drawParallelTraj))) {
          printf(
              "%3d, time: %3f, cost: %f (s=%f, c=%f, g=%f), move3d cost: %f\n",
              iteration_,
              time_,
              cost,
              getSmoothnessCost(),
              getCollisionCost(),
              getGeneralCost(),
              move3d_cost);
        } else {
          printf("%3d, time: %3f, cost: %f\n", iteration_, time_, cost);
          // printf("%3d, time: %3f, cost: %f (s=%f, c=%f, g=%f),
          // move3d cost: %f\n", iteration_, time_, cost,
          // getSmoothnessCost(), getCollisionCost(),
          // getGeneralCost(), move3d_cost );
        }
      }
    }
    // printf( "%3d, time: %f, cost: %f (s=%f, c=%f)\n", iteration_, time,
    // getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());

    // cout << "We think the path is collision free: " <<
    // last_trajectory_collision_free_ << endl;

    if (use_time_limit_)

      if (time_ >= stomp_parameters_->max_time_) {
        cout << "Stopped at time limit (" << time_
             << "), limit : " << stomp_parameters_->max_time_ << endl;
        break;
      }

    if (use_iteration_limit_)

      if (iteration_ >= stomp_parameters_->max_iterations_) {
        cout << "Stopped at iteration (" << iteration_ << "), limit "
             << stomp_parameters_->max_iterations_ << endl;
        break;
      }

    if (use_collision_free_limit_limit_)

      if (is_collision_free_) {
        cout << "Stopped at iteration (" << iteration_
             << "), because found collision free" << endl;
        break;
      }
  }

  cout << " -------------------------------------- " << endl;

  if (best_iteration_ > 0)
    cout << "Found a collision free path!!! " << endl;
  else
    cout << "NO collision free path found!!! " << endl;

  if (last_improvement_iteration_ > -1)
    cout << "We think the last path is collision free: " << is_collision_free_
         << endl;

  // Set the last trajectory
  // Convert to move3d trajectory
  last_traj_ = Move3D::Trajectory(robot_model_);
  setGroupTrajectoryToMove3DTraj(last_traj_);

  // policy_->saveProfiles( policy_parameters_, "./control_cost_profiles" );

  if (best_iteration_ > 0) {
    cout << "set best_group_trajectory_" << endl;
    group_trajectory_.getTrajectory() = best_group_trajectory_;
  } else if (last_improvement_iteration_ > -1) {
    cout << "set best_group_trajectory_in_collision_" << endl;
    group_trajectory_.getTrajectory() = best_group_trajectory_in_collision_;

  } else {
    cout << "set group_trajectory_backup_" << endl;
    group_trajectory_.getTrajectory() = group_trajectory_backup_;
  }

  // convert to move3d trajectory
  best_traj_ = Move3D::Trajectory(robot_model_);
  setGroupTrajectoryToMove3DTraj(best_traj_);
  robot_model_->setCurrentMove3DTraj(best_traj_);
  robot_model_->getCurrentMove3DTraj().replaceP3dTraj();

  cout << "Move3D traj cost : " << best_traj_.costPerPoint() << endl;

  // Set the current move3d traj
  //    robot_model_->getCurrentMove3DTraj().replaceP3dTraj();

  // Best path cost
  performForwardKinematics(false);
  printf("Best trajectory : iter=%3d, cost (s=%f, c=%f, g=%f)\n",
         last_improvement_iteration_,
         getSmoothnessCost(true),
         getCollisionCost(),
         getGeneralCost());

  // Print control costs
  ControlCost cost;
  Eigen::MatrixXd traj = group_trajectory_.getTrajectory().transpose();

  //    cout << "motion matrix 0" << endl;
  //    cout.precision(4);
  //    cout << traj << endl;

  std::vector<Eigen::VectorXd> control_cost = cost.getSquaredQuantities(traj);
  cout.precision(10);
  cout << "size (" << traj.rows() << ", " << traj.cols()
       << ") , control cost : "
       << (stomp_parameters_->getSmoothnessCostWeight()) *
              cost.cost(control_cost) << endl;

  // Becareful replaces group trajectory
  computeTrajectoryFeatures(best_traj_);

  // Set this anywhere
  //    if( PlanEnv->getBool(PlanParam::drawParallelTraj) && ( global_stompRun
  //    != NULL ))
  //    {
  //        ENV.setBool(Env::drawTraj,false);
  //        global_stompRun->lockDraw();
  //        robot_model_->setAndUpdate(*q_tmp);
  //        saveEndeffectorTraj();
  //        global_stompRun->unlockDraw();
  //    }

  // group_trajectory_.print();
  // updateFullTrajectory();
  // performForwardKinematics();

  printf("Collision free success iteration = %d for robot %s (time : %f)\n",
         stomp_statistics_->collision_success_iteration,
         robot_model_->getName().c_str(),
         stomp_statistics_->success_time);
  printf("Terminated after %d iterations, using path from iteration %d\n",
         iteration_,
         last_improvement_iteration_);
  printf("Best cost = %f\n", best_group_trajectory_cost_);
  printf("Stomp has run for : %f sec\n", time_);

  // printf("Optimization core finished in %f sec", (ros::WallTime::now() -
  // start_time).toSec());
  stomp_statistics_->best_cost = best_group_trajectory_cost_;

  traj_optim_convergence.clear();
  traj_optim_convergence.push_back(stomp_statistics_->costs);

  // Save traj cost to file
  if (PlanEnv->getBool(PlanParam::trajSaveCost)) {
    stringstream s;
    s << "StompOptim_" << setfill('0') << setw(4) << idRun;
    saveOptimToFile(s.str());
  }
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------

void StompOptimizer::setRobotPool(const std::vector<Robot*>& robots) {
  compute_fk_.clear();
  int collision_space_id = 0;

  for (int i = 0; i < int(robots.size()); i++) {
    if (!use_external_collision_space_) {
      // TODO FIX THIS (becarful with robot pointer)
      collision_space_id = i + 1;
    } else {
      use_external_collision_space_ = true;
      collision_space_id = 0;
    }

    compute_fk_.push_back(
        new costComputation(robots[i],
                            move3d_collision_space_,
                            new ChompPlanningGroup(*planning_group_, robots[i]),
                            joint_costs_,
                            group_trajectory_,
                            stomp_parameters_->getObstacleCostWeight(),
                            use_costspace_,
                            source_,
                            target_,
                            use_external_collision_space_,
                            collision_space_id,
                            stomp_parameters_,
                            policy_));

    if (!use_external_collision_space_) {
      move3d_set_fct_get_nb_collision_points(
          boost::bind(&costComputation::getNumberOfCollisionPoints,
                      compute_fk_.back(),
                      _1),
          collision_space_id);
      move3d_set_fct_get_config_collision_cost(
          boost::bind(&costComputation::getConfigObstacleCost,
                      compute_fk_.back(),
                      _1,
                      _2,
                      _3,
                      _4),
          collision_space_id);
    }
  }
}

/**
  * get the cost conputer
  */
const vector<costComputation*>& StompOptimizer::getCostComputers() {
  return compute_fk_;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//! Generate noisy trajectories
void StompOptimizer::generateNoisyTrajectory(
    const Move3D::Trajectory& traj,
    vector<vector<confPtr_t> >& noisy_trajectories) {
  // Set the full and group trajectory
  if (full_trajectory_ != NULL) delete full_trajectory_;
  full_trajectory_ =
      new ChompTrajectory(traj, DIFF_RULE_LENGTH, *planning_group_, 0.0);
  group_trajectory_ = *full_trajectory_;

  stomp_statistics_ =
      MOVE3D_BOOST_PTR_NAMESPACE<StompStatistics>(new StompStatistics());
  stomp_statistics_->collision_success_iteration = -1;
  stomp_statistics_->success_iteration = -1;
  stomp_statistics_->success = false;
  stomp_statistics_->costs.clear();

  ChronoTimeOfDayOn();

  // Initialize
  initPolicy();

  PolicyImprovementLoop pi_loop;
  pi_loop.initialize(this_shared_ptr_,
                     true,
                     group_trajectory_.getUseTime()
                         ? group_trajectory_.getDiscretization()
                         : 0.0);

  // group_trajectory_.print();
  iteration_ = 0;
  copyPolicyToGroupTrajectory();
  handleJointLimits();
  updateFullTrajectory();
  performForwardKinematics(false);
  // group_trajectory_.print();

  //    if (stomp_parameters_->getAnimateEndeffector())
  //    {
  //      animateEndeffector();
  //    }

  //    pi_loop.generateSingleNoisyTrajectory(/*iteration_+1*/);

  pi_loop.getRollouts(noisy_trajectories);

  //    if (stomp_parameters_->getAnimateEndeffector())
  //    {
  //      animateEndeffector();
  //    }

  // setGroupTrajectoryToVectorConfig( noisy_trajectory[0] );

  //    updateFullTrajectory();
  //    performForwardKinematics();
  ChronoTimeOfDayPrint("");
  ChronoTimeOfDayOff();
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------

void StompOptimizer::calculateSmoothnessIncrements() {
  for (int i = 0; i < num_joints_; i++) {
    joint_costs_[i].getDerivative(group_trajectory_.getJointTrajectory(i),
                                  smoothness_derivative_);
    smoothness_increments_.col(i) = -smoothness_derivative_.segment(
        group_trajectory_.getStartIndex(), num_vars_free_);
  }

  // cout <<  "smoothness_increments_  = " << endl << smoothness_increments_ <<
  // endl;
}

void StompOptimizer::calculateCollisionIncrements() {
  double potential;
  double vel_mag_sq;
  double vel_mag;
  Vector3d potential_gradient;
  Vector3d normalized_velocity;
  Matrix3d orthogonal_projector;
  Vector3d curvature_vector;
  Vector3d cartesian_gradient;

  // cout <<  "compute_fk_main_->getCollisionPointPotential()  = " << endl <<
  // compute_fk_main_->getCollisionPointPotential() << endl;

  collision_increments_.setZero(num_vars_free_, num_joints_);
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    for (int j = 0; j < num_collision_points_; j++) {
      potential = compute_fk_main_->getCollisionPointPotential()(i, j);
      if (potential <= 1e-10) continue;

      potential_gradient =
          compute_fk_main_->getCollisionPointPotentialGradient()[i][j];
      // cout << compute_fk_main_->getCollisionPointPotentialGradient()[i][j] <<
      // endl;

      vel_mag = compute_fk_main_->getCollisionVelMag()(i, j);
      vel_mag_sq = vel_mag * vel_mag;
      // cout << compute_fk_main_->getCollisionVelMag()(i,j) << endl;

      // all math from the STOMP paper:
      normalized_velocity =
          compute_fk_main_->getCollisionPointVelEigen()[i][j] / vel_mag;
      orthogonal_projector =
          Matrix3d::Identity() -
          (normalized_velocity * normalized_velocity.transpose());
      curvature_vector = (orthogonal_projector *
                          compute_fk_main_->getCollisionPointAccEigen()[i][j]) /
                         vel_mag_sq;
      cartesian_gradient =
          vel_mag * (orthogonal_projector * potential_gradient -
                     potential * curvature_vector);

      // pass it through the jacobian transpose to get the increments
      // TODO FIX FOR CHOMP
      //            planning_group_->collision_points_[j].getJacobian(compute_fk_main_->getJointPosEigen()[i],
      //                                                              compute_fk_main_->getJointAxisEigen()[i],
      //                                                              compute_fk_main_->getCollisionPointPosEigen()[i][j],
      //                                                              jacobian_,
      //                                                              group_joint_to_move3d_joint_index_);

      if (stomp_parameters_->getUsePseudoInverse()) {
        calculatePseudoInverse();
        collision_increments_.row(i - free_vars_start_).transpose() -=
            jacobian_pseudo_inverse_ * cartesian_gradient;
      } else {
        collision_increments_.row(i - free_vars_start_).transpose() -=
            jacobian_.transpose() * cartesian_gradient;
      }
      if (point_is_in_collision_[i][j]) break;
    }
  }

  // cout <<  "collision_increments_  = " << endl << collision_increments_ <<
  // endl;
}

void StompOptimizer::calculatePseudoInverse() {
  jacobian_jacobian_tranpose_ =
      jacobian_ * jacobian_.transpose() +
      Eigen::MatrixXd::Identity(3, 3) *
          stomp_parameters_->getPseudoInverseRidgeFactor();
  jacobian_pseudo_inverse_ =
      jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void StompOptimizer::calculateTotalIncrements() {
  for (int i = 0; i < num_joints_; i++) {
    final_increments_.col(i) = stomp_parameters_->getLearningRate() *
                               (joint_costs_[i].getQuadraticCostInverse() *
                                (stomp_parameters_->getSmoothnessCostWeight() *
                                     smoothness_increments_.col(i) +
                                 stomp_parameters_->getObstacleCostWeight() *
                                     collision_increments_.col(i)));
  }
}

void StompOptimizer::addIncrementsToTrajectory() {
  //  double scale = 1.0;
  for (int i = 0; i < num_joints_; i++) {
    double scale = 1.0;
    double max = final_increments_.col(i).maxCoeff();
    double min = final_increments_.col(i).minCoeff();
    double max_scale =
        planning_group_->chomp_dofs_[i].joint_update_limit_ / fabs(max);
    double min_scale =
        planning_group_->chomp_dofs_[i].joint_update_limit_ / fabs(min);
    if (max_scale < scale) scale = max_scale;
    if (min_scale < scale) scale = min_scale;
    group_trajectory_.getFreeTrajectoryBlock().col(i) +=
        scale * final_increments_.col(i);
  }
  // ROS_DEBUG("Scale: %f",scale);
  // group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
}

void StompOptimizer::updateFullTrajectory() {
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void StompOptimizer::debugCost() {
  double cost = 0.0;
  for (int i = 0; i < num_joints_; i++)
    cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  cout << "Cost = " << cost << endl;
}

double StompOptimizer::getTrajectoryCost() {
  double cost = getSmoothnessCost();
  if (move3d_collision_space_ || use_external_collision_space_)
    cost += getCollisionCost();
  if (use_costspace_) cost += getGeneralCost();
  return cost;
}

double StompOptimizer::getCollisionCost() {
  double collision_cost = 0.0;

  double worst_collision_cost = 0.0;
  worst_collision_cost_state_ = -1;

  double time = 0.;
  int nb_points = 0;

  // collision costs:
  // WARNING, integration is performed over number of points and not paths...
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    double state_collision_cost = 0.0;

    if (move3d_collision_space_ || use_external_collision_space_) {
      // TODO watch out for cumulative case
      double cumulative = 0.0;
      for (int j = 0; j < num_collision_points_; j++) {
        cumulative += compute_fk_main_->getCollisionPointPotential()(i, j) *
                      compute_fk_main_->getCollisionVelMag()(i, j);
        state_collision_cost += cumulative;
        // state_collision_cost +=
        // compute_fk_main_->getCollisionPointPotential()(i,j);
      }
    }

    collision_cost += (state_collision_cost * compute_fk_main_->getDts()[i]);

    nb_points++;
    time += compute_fk_main_->getDts()[i];

    if (state_collision_cost > worst_collision_cost) {
      worst_collision_cost = state_collision_cost;
      worst_collision_cost_state_ = i;
    }
  }

  //    cout << "duration : " << time
  //         << " , collision_cost : " << collision_cost
  //         << ", nb of points : " << nb_points
  //         << endl;

  //    cout << "dt : " << compute_fk_main_->getDts().transpose() << endl;

  //    cout << "collision weight : "
  //         << stomp_parameters_->getObstacleCostWeight()
  //         << endl;

  return stomp_parameters_->getObstacleCostWeight() * collision_cost;
}

double StompOptimizer::getGeneralCost() {
  if (!use_costspace_ && !PlanEnv->getBool(PlanParam::useLegibleCost)) {
    return 0.0;
  }

  double general_cost = 0.0;

  Eigen::VectorXd costs(
      Eigen::VectorXd::Zero(compute_fk_main_->getDts().size()));

  double time = 0.0;

  // WARNING, integration is performed over number of points and not paths...
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    // general_cost += ( pow( compute_fk_main_->getCollisionCostPotential()(i) ,
    // hack_tweek )  * compute_fk_main_->getDts()[i] );
    costs[i] = compute_fk_main_->getGeneralCostPotential()(i) *
               compute_fk_main_->getDts()[i];
    time += compute_fk_main_->getDts()[i];
    general_cost += costs[i];
  }

  // TEST FOR DISTANCE FEATURE
  //    StackedFeatures* fct = dynamic_cast<StackedFeatures*>(
  //    global_activeFeatureFunction );

  //    if( ( fct != NULL ) && ( fct->getFeatureFunction("Distance") != NULL ) )
  //    {
  //        Move3D::Trajectory traj( robot_model_ );
  //        setGroupTrajectoryToMove3DTraj( traj );

  //        Move3D::FeatureVect costs =
  //        fct->getFeatureFunction("Distance")->getFeatureCount( traj );
  //        Move3D::WeightVect w =
  //        fct->getFeatureFunction("Distance")->getWeights();

  //        cout.precision(4);
  //        cout << "distance_phi  : " << std::scientific << costs.transpose()
  //        << endl;
  //        cout << "distance_w    : " << w.transpose() << endl;
  //        cout << "distance_cost : " << std::scientific <<  ( w.cwise() *
  //        costs ).transpose()  << endl;
  //        cout << "distance total cost : " << w.transpose() * costs << endl;
  //        cout << "general cost : " << general_cost << endl;traj_evaluation
  //    }

  //    cout << "time length : " << time << " , general_cost : " << general_cost
  //    << endl;

  // cout << "dt : " << compute_fk_main_->getDts().transpose() << endl;
  // cout << "dt : " << compute_fk_main_->getDts().segment( free_vars_start_,
  // free_vars_end_-free_vars_start_ ).transpose() << endl;
  // cout << "costs : " <<
  // costs.segment(free_vars_start_,free_vars_end_-free_vars_start_).transpose()
  // << endl;

  // cout << "use_costspace : " << use_costspace_ << endl;
  // cout << "move3d_collision_space_ : " << move3d_collision_space_ << endl;
  // cout <<
  // compute_fk_main_->getCollisionCostPotential().segment(free_vars_start_,free_vars_end_-free_vars_start_).transpose()
  // << endl;

  return stomp_parameters_->getGeneralCostWeight() * general_cost;
}

double StompOptimizer::getSmoothnessCost(bool save_to_file) {
  bool use_weight = true;
  double weight =
      use_weight ? stomp_parameters_->getSmoothnessCostWeight() : 1.0;
  double smoothness_cost = 0.0;

  //    double joint_cost = 0.0;
  //    // joint costs:
  //    for ( int i=0; i<num_joints_; i++ )
  //    {
  //        joint_cost =
  // joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
  //        smoothness_cost += joint_cost;
  //    }
  //    return stomp_parameters_->getSmoothnessCostWeight() * smoothness_cost;

  std::vector<Eigen::MatrixXd> control_cost_matrices;
  std::vector<Eigen::VectorXd> noise(num_joints_);
  std::vector<Eigen::VectorXd> control_costs(num_joints_);

  for (int d = 0; d < num_joints_; ++d) {
    // policy_parameters_[d] = group_trajectory_.getFreeTrajectoryBlock();
    policy_parameters_[d].segment(1, policy_parameters_[d].size() - id_fixed_) =
        group_trajectory_.getFreeJointTrajectoryBlock(d);

    noise[d] = VectorXd::Zero(policy_parameters_[d].size());
  }

  double discretization = group_trajectory_.getUseTime()
                              ? group_trajectory_.getDiscretization()
                              : 0.0;

  bool classic = HriEnv->getBool(HricsParam::ioc_use_baseline);

  StackedFeatures* fct =
      dynamic_cast<StackedFeatures*>(global_activeFeatureFunction);

  if ((classic == false) && (fct != NULL) &&
      (fct->getFeatureFunction("SmoothnessAll") != NULL)) {
    std::vector<std::vector<Eigen::VectorXd> > control_costs;

    Eigen::VectorXd costs =
        policy_->getAllCosts(policy_parameters_, control_costs, discretization);

    //        Move3D::Trajectory traj( robot_model_ );
    //        setGroupTrajectoryToMove3DTraj( traj );

    smoothness_cost =
        fct->getFeatureFunction("SmoothnessAll")->getWeights().transpose() *
        costs;

    // cout << "costs (control) : " << costs.transpose() << endl;
    //        cout << "costs (control) : " << costs.array() *
    // fct->getFeatureFunction("SmoothnessAll")->getWeights().array()
    // << endl;

    //         SmoothnessFeature*  smoothness_feature =
    // dynamic_cast<SmoothnessFeature*>(fct->getFeatureFunction("SmoothnessAll"));
    //         if( smoothness_feature == NULL )
    //             exit(0);

    //         cout << "left padding : "
    // << smoothness_feature->task_features_.get_left_padding() << endl;
    //         cout << "right padding : "
    // << smoothness_feature->task_features_.get_right_padding() << endl;

    //        cout.precision(4);
    //        cout << "smoothness_phi  : " << std::scientific <<
    //        costs.transpose() << endl;
    //        cout << "smoothness_w    : " << w.transpose() << endl;
    //        cout << "smoothness_cost : " << std::scientific
    // <<  ( w.cwise() * costs ).transpose()  << endl;
  } else {
    // TODO remove weight multiplication

    // cout << "NORMAL COMPUTATION : " << weight << endl;

    // cout << "discretization : " << discretization << endl;
    policy_->computeControlCosts(control_cost_matrices,
                                 policy_parameters_,
                                 noise,
                                 weight,
                                 control_costs,
                                 discretization);

    if (save_to_file) {
      policy_->saveProfiles(
          policy_parameters_, "./control_cost_profiles", discretization);
    }

    // cout << "policy_parameters_ : " << policy_parameters_[0].transpose() <<
    // endl;
    // cout << std::scientific << control_costs[0].transpose() << endl;

    Eigen::VectorXd costs = Eigen::VectorXd::Zero(control_costs[0].size());
    for (int d = 0; d < int(control_costs.size()); ++d) {
      smoothness_cost += control_costs[d].sum();
      costs += control_costs[d];
    }
  }

  //    cout << "control cost : " << costs.transpose() << endl;

  return smoothness_cost;
}

void StompOptimizer::getCostProfiles(vector<double>& smoothness_cost,
                                     vector<double>& collision_cost,
                                     vector<double>& general_cost) {
  //    cout << "perform forward kinematics" << endl;
  //    group_trajectory_.getTrajectory() = best_group_trajectory_;
  performForwardKinematics(false);

  std::vector<Eigen::MatrixXd> control_cost_matrices;
  std::vector<Eigen::VectorXd> noise(num_joints_);
  std::vector<Eigen::VectorXd> control_costs(num_joints_);

  for (int d = 0; d < num_joints_; ++d) {
    // policy_parameters_[d] = group_trajectory_.getFreeTrajectoryBlock();
    policy_parameters_[d].segment(1, policy_parameters_[d].size() - id_fixed_) =
        group_trajectory_.getFreeJointTrajectoryBlock(d);
    noise[d] = VectorXd::Zero(policy_parameters_[d].size());
  }

  policy_->computeControlCosts(control_cost_matrices,
                               policy_parameters_,
                               noise,
                               stomp_parameters_->getSmoothnessCostWeight(),
                               control_costs,
                               false);

  int state = 0;

  smoothness_cost.resize(free_vars_end_ - free_vars_start_ + 1, 0.0);
  collision_cost.resize(free_vars_end_ - free_vars_start_ + 1, 0.0);
  general_cost.resize(free_vars_end_ - free_vars_start_ + 1, 0.0);

  //    cout << smoothness_cost.size() << endl;
  //    cout << collision_cost.size() << endl;
  //    cout << general_cost.size() << endl;
  //    cout << control_costs[0].size() << endl;
  //    cout << control_costs.size() << endl;
  //    cout << num_joints_ << endl;

  // collision and general costs
  // WARNING, integration is performed over number of points and not paths...
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    double state_smoothness_cost = 0.0;
    double state_collision_cost = 0.0;

    if (move3d_collision_space_ || use_external_collision_space_) {
      // TODO watch out for cumulative case
      double cumulative = 0.0;
      for (int j = 0; j < num_collision_points_; j++) {
        cumulative += compute_fk_main_->getCollisionPointPotential()(
            i, j) /* compute_fk_main_->getCollisionVelMag()(i,j) */;
        state_collision_cost += cumulative;
        // state_collision_cost +=
        // compute_fk_main_->getCollisionPointPotential()(i,j);
      }

      collision_cost[state] = stomp_parameters_->getObstacleCostWeight() *
                              state_collision_cost *
                              compute_fk_main_->getDts()[i];
    }

    general_cost[state] = stomp_parameters_->getGeneralCostWeight() *
                          compute_fk_main_->getGeneralCostPotential()(i) *
                          compute_fk_main_->getDts()[i];
    cout << "compute_fk_main_->getDts()[" << i
         << "] : " << compute_fk_main_->getDts()[i] << endl;

    //        if( state > 0 && state < smoothness_cost.size()-1 )
    for (int d = 0; d < num_joints_; ++d)
      state_smoothness_cost += control_costs[d][state];

    smoothness_cost[state] = state_smoothness_cost;

    state++;
  }

  cout << "cost profiles end" << endl;
}

void StompOptimizer::computeTrajectoryFeatures(const Move3D::Trajectory& traj) {
  cout << "____________________________________________________________________"
          "______________________" << endl;
  cout << __PRETTY_FUNCTION__ << endl;
  cout << " traj uses time : " << traj.getUseTimeParameter() << endl;

  performForwardKinematics(false);

  TrajectorySmoothness smooth;
  smooth.setActiveDoFs(planning_group_->getActiveDofs());
  cout << "smooth.getFeatureCount( best_traj_ ) = "
       << ((stomp_parameters_->getSmoothnessCostWeight() /
            PlanEnv->getDouble(PlanParam::trajOptimSmoothFactor)) *
           smooth.getFeatureCount(traj)) << endl;

  // Get Stacked feature
  StackedFeatures* fct =
      dynamic_cast<StackedFeatures*>(global_activeFeatureFunction);

  // Get the value for the Distance feature
  // compare it with the general cost
  if (fct != NULL && fct->getFeatureFunction("Distance") != NULL) {
    fct->printInfo();

    FeatureVect phi =
        fct->getFeatureFunction("Distance")->getFeatureCount(traj);

    cout << "distance cost : "
         << fct->getFeatureFunction("Distance")->costTraj(traj);
    cout << " , Features " << phi.transpose() << endl;

    // cout << "costs 2 : " << costs.transpose() << endl;
    // cout << "dts 2 : " << dts.transpose() << endl;
    // cout << "general features : " <<  phi.transpose() << endl;
    cout << "general distance cost 1 : " << getGeneralCost() << endl;
    cout << "general distance cost 2 : "
         << fct->getFeatureFunction("Distance")->getWeights().transpose() * phi
         << endl;
  }

  // Get the value for the collision feature
  // and compare it with the collision cost
  if (fct != NULL && fct->getFeatureFunction("Collision") != NULL) {
    fct->printInfo();

    Eigen::VectorXd phi =
        fct->getFeatureFunction("Collision")->getFeatureCount(traj);

    performForwardKinematics(false);
    getCollisionCost();

    cout << "general collision cost 1 : " << getCollisionCost() << endl;
    cout << "general collision cost 2 : "
         << fct->getFeatureFunction("Collision")->getWeights().transpose() * phi
         << endl;
  }

  // Get the value for the smoothness feature
  if (fct != NULL && fct->getFeatureFunction("SmoothnessAll") != NULL) {
    Eigen::VectorXd phi =
        fct->getFeatureFunction("SmoothnessAll")->getFeatureCount(traj);
  }

  cout << "____________________________________________________________________"
          "______________________" << endl;
}

bool StompOptimizer::handleJointLimits() {
  //    compute_fk_main_.
  return compute_fk_main_->handleJointLimitsQuadProg(group_trajectory_);
}

double StompOptimizer::getCollisionSpaceCost(const Move3D::Configuration& q) {
  return compute_fk_main_->getCollisionSpaceCost(q);
}

void StompOptimizer::getFrames(int segment,
                               const Eigen::VectorXd& joint_array,
                               Move3D::Configuration& q) {
  compute_fk_main_->getFrames(segment, joint_array, q);
}

bool StompOptimizer::getCollisionPointObstacleCost(
    int segment,
    int coll_point,
    double& collion_point_potential,
    Eigen::Vector3d& pos) {
  return compute_fk_main_->getCollisionPointObstacleCost(
      segment, coll_point, collion_point_potential, pos);
}

bool StompOptimizer::getConfigObstacleCost(
    Move3D::Robot* robot,
    int i,
    Eigen::MatrixXd& collision_point_potential,
    std::vector<std::vector<Eigen::Vector3d> >& collision_point_pos) {
  return compute_fk_main_->getConfigObstacleCost(
      robot, i, collision_point_potential, collision_point_pos);
}

bool StompOptimizer::performForwardKinematics(bool is_rollout) {
  return compute_fk_main_->performForwardKinematics(group_trajectory_,
                                                    is_rollout);
}

bool StompOptimizer::execute(std::vector<Eigen::VectorXd>& parameters,
                             Eigen::VectorXd& costs,
                             const int iteration_number,
                             bool joint_limits,
                             bool resample,
                             bool is_rollout) {
  is_collision_free_ = compute_fk_main_->getCost(
      parameters, costs, iteration_number, joint_limits, resample, is_rollout);
  group_trajectory_ = compute_fk_main_->getGroupTrajectory();
  updateFullTrajectory();
  return  compute_fk_main_->getJointLimitViolationSuccess();
}

void StompOptimizer::getTrajectoryCost(std::vector<double>& cost, double step) {
  cost.clear();
  cout << "best_group_trajectory_ = " << best_group_trajectory_ << endl;

  group_trajectory_.getTrajectory() = best_group_trajectory_;
  group_trajectory_.print();

  updateFullTrajectory();
  performForwardKinematics(false);

  vector<double> collision_cost;

  // collision costs:
  double collision_cost_sum = 0.0;
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    double state_collision_cost = 0.0;

    if (move3d_collision_space_ || use_external_collision_space_) {
      for (int j = 0; j < num_collision_points_; j++) {
        state_collision_cost += compute_fk_main_->getCollisionPointPotential()(
            i, j) /* compute_fk_main_->getCollisionVelMag()(i,j) */;
      }
    } else {
      state_collision_cost =
          pow(compute_fk_main_->getGeneralCostPotential()(i), hack_tweek);
    }

    collision_cost.push_back(state_collision_cost);
    collision_cost_sum += state_collision_cost;
  }

  cout << "Collision cost : "
       << stomp_parameters_->getObstacleCostWeight() * collision_cost_sum
       << endl;

  double vector_steps = (double(collision_cost.size()) / 100);

  for (double s = 0; s <= double(collision_cost.size()); s += vector_steps) {
    if (int(s) >= int(collision_cost.size())) break;

    cost.push_back(collision_cost[int(s)]);
  }
}

void StompOptimizer::eigenMapTest() {
  //    double foo_eigen;
  //    double foo_kdl;
}

void StompOptimizer::perturbTrajectory() {
  // int mid_point = (free_vars_start_ + free_vars_end_) / 2;
  if (worst_collision_cost_state_ < 0) return;
  int mid_point = worst_collision_cost_state_;
  planning_group_->getRandomState(random_state_);

  // convert the state into an increment
  random_state_ -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();

  // project the increment orthogonal to joint velocities
  group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
  joint_state_velocities_.normalize();
  random_state_ =
      (Eigen::MatrixXd::Identity(num_joints_, num_joints_) -
       joint_state_velocities_ * joint_state_velocities_.transpose()) *
      random_state_;

  int mp_free_vars_index = mid_point - free_vars_start_;
  for (int i = 0; i < num_joints_; i++) {
    group_trajectory_.getFreeJointTrajectoryBlock(i) +=
        joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) *
        random_state_(i);
  }
}

void StompOptimizer::getRandomMomentum() {
  if (is_collision_free_)
    random_momentum_.setZero(num_vars_free_, num_joints_);
  else
    for (int i = 0; i < num_joints_; ++i) {
      multivariate_gaussian_[i].sample(random_joint_momentum_);
      random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
    }
}

void StompOptimizer::updateMomentum() {
  double eps = stomp_parameters_->getHmcDiscretization();

  //  if (iteration_ > 0)
  //    momentum_ = (momentum_ + eps*final_increments_);
  //  else
  //    momentum_ = random_momentum_;

  double alpha = 1.0 - stomp_parameters_->getHmcStochasticity();
  // if (iteration_ > 0)
  momentum_ = alpha * (momentum_ + eps * final_increments_) +
              sqrt(1.0 - alpha * alpha) * random_momentum_;
  // else
  //  momentum_ = random_momentum_;
}

void StompOptimizer::updatePositionFromMomentum() {
  double eps = stomp_parameters_->getHmcDiscretization();
  group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
}

void StompOptimizer::animatePath() {
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    visualizeState(i);
    // ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
    //    ros::WallDuration(0.05).sleep();
  }
}

void StompOptimizer::clearAnimations() {}

void StompOptimizer::setGroupTrajectoryFromVectorConfig(
    const vector<confPtr_t>& traj) {
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_ == 0) {
    start = 0;
    end = num_vars_all_ - 1;
  }

  // Get the map from move3d index to group trajectory
  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  for (int i = start; i <= end; ++i) {
    for (int j = 0; j < planning_group_->num_dofs_; j++) {
      double point = (*traj[i])[joints[j].move3d_dof_index_];
      group_trajectory_.getTrajectoryPoint(i).transpose()[j] = point;
    }
  }
}

void StompOptimizer::setGroupTrajectoryToVectorConfig(vector<confPtr_t>& traj) {
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_ == 0) {
    start = 0;
    end = num_vars_all_ - 1;
  }

  // Get the map fro move3d index to group trajectory
  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  confPtr_t q = robot_model_->getCurrentPos();

  for (int i = start; i <= end; ++i) {
    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

    for (int j = 0; j < planning_group_->num_dofs_; j++) {
      (*q)[joints[j].move3d_dof_index_] = point[j];
    }
    traj.push_back(confPtr_t(new Configuration(*q)));
  }
}

void StompOptimizer::setGroupTrajectoryToMove3DTraj(Move3D::Trajectory& traj) {
  int start = free_vars_start_;
  int end = free_vars_end_;
  //    if (iteration_==0) {
  //        start = 0;
  //        end = num_vars_all_-1;
  //    }

  traj.clear();

  // Get the map fro move3d index to group trajectory
  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  confPtr_t q(new Configuration(*source_));
  // source_->setConstraints();

  // traj.push_back( source_ );

  //    cout << "group_trajectory_.getUseTime() : " <<
  //    group_trajectory_.getUseTime() << endl;

  if (group_trajectory_.getUseTime()) {
    traj.setUseTimeParameter(true);
    traj.setUseConstantTime(true);
    traj.setDeltaTime(
        group_trajectory_.getDiscretization());  // WARNING here use detla time
  }

  for (int i = start; i <= end;
       ++i)  // Because we add source and target we start later
  {
    // Set passive dofs as source
    *q = *source_;

    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

    for (int j = 0; j < planning_group_->num_dofs_; j++)
      (*q)[joints[j].move3d_dof_index_] = point[j];

    // q->setConstraints();
    traj.push_back(confPtr_t(new Configuration(*q)));
  }

  //    traj.push_back( target_ );
}

void StompOptimizer::animateTrajectoryPolicy() {
  //    Move3D::Trajectory T(robot_model_);
  //    const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;
  //    std::vector<Eigen::VectorXd> parameters;
  //
  //    policy_->getParameters(parameters);
  //
  //    // for each point in the trajectory
  //    for (int i=0; i<int(parameters[0].size()); ++i)
  //    {
  //      confPtr_t q = robot_model_->getCurrentPos();
  //
  //      for(int j=0; j<planning_group_->num_joints_;j++)
  //      {
  //        (*q)[joints[j].move3d_dof_index_] = parameters[j][i];
  //      }
  //      T.push_back(q);
  //    }
  //    T.replaceP3dTraj();
  // g3d_draw_allwin_active();
}

void StompOptimizer::saveEndeffectorTraj() {
  // Get draw joint
  Joint* drawnjnt = NULL;
  int indexjnt = p3d_get_user_drawnjnt();
  if (indexjnt != -1 && indexjnt >= 0 &&
      indexjnt <= int(robot_model_->getNumberOfJoints())) {
    drawnjnt = robot_model_->getJoint(indexjnt);
  }
  if (drawnjnt == NULL) {
    return;
  }

  Move3D::Trajectory T(robot_model_);

  // calculate the forward kinematics for the fixed states only in the first
  // iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_ == 0) {
    start = 0;
    end = num_vars_all_ - 1;
  }

  confPtr_t q = robot_model_->getCurrentPos();

  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  // for each point in the trajectory
  for (int i = start; i <= end; ++i) {
    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

    for (int j = 0; j < planning_group_->num_dofs_; j++)
      (*q)[joints[j].move3d_dof_index_] = point[j];

    T.push_back(confPtr_t(new Configuration(*q)));
  }

  // Set the robot to the first configuration
  //    Eigen::VectorXd point =
  //    group_trajectory_.getTrajectoryPoint(1).transpose();
  //    for(int j=0; j<planning_group_->num_joints_;j++)
  //        (*q)[joints[j].move3d_dof_index_] = point[j];

  traj_color_.resize(4, 0);

  global_MultiStomplinesColors[robot_model_] = traj_color_;
  global_MultiStomplinesToDraw[robot_model_].clear();

  for (int i = 0; i < T.getNbOfViaPoints(); ++i) {
    robot_model_->setAndUpdate(*T[i]);
    global_MultiStomplinesToDraw[robot_model_].push_back(
        drawnjnt->getVectorPos());
  }

  robot_model_->setAndUpdate(*source_);

  //    g3d_draw_allwin_active();
}

void StompOptimizer::animateEndeffector(bool print_cost) {
  Move3D::Trajectory T(robot_model_);

  // calculate the forward kinematics for the fixed states only in the first
  // iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;
  if (iteration_ == 0) {
    start = 0;
    end = num_vars_all_ - 1;
  }

  Move3D::confPtr_t q_tmp = robot_model_->getCurrentPos();
  Move3D::confPtr_t q = robot_model_->getCurrentPos();
  // cout << "animateEndeffector()" << endl;
  // cout << "group_trajectory : " << endl;
  // cout << group_trajectory_.getTrajectory() << endl;

  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  Move3D::ChompTrajectory group_trajectory(group_trajectory_, 7);

  if (iteration_ != 0) {
    if (PlanEnv->getInt(PlanParam::stompDrawIteration) != 1) {
      if (best_group_trajectory_in_collsion_cost_ < best_group_trajectory_cost_)

        group_trajectory.getTrajectory() = best_group_trajectory_in_collision_;
      else
        group_trajectory.getTrajectory() = best_group_trajectory_;
    }
  }

  // for each point in the trajectory
  for (int i = start; i <= end; ++i) {
    Eigen::VectorXd point = group_trajectory.getTrajectoryPoint(i).transpose();

    for (int j = 0; j < planning_group_->num_dofs_; j++)
      (*q)[joints[j].move3d_dof_index_] = point[j];

    // q->print();

    T.push_back(confPtr_t(new Configuration(*q)));
  }

  if (print_cost) {
    cout << "Move3D Trajectory, nb via points : " << T.getNbOfViaPoints()
         << ", cost : " << T.cost();
    if (T.isValid())
      cout << " and is valid" << endl;
    else
      cout << " and is NOT valid" << endl;
  }

  T.replaceP3dTraj();

  if (!robot_model_->getUseLibmove3dStruct()) {
    T.draw();
  }

  // Set the robot to the first configuration
  Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(1).transpose();

  for (int j = 0; j < planning_group_->num_dofs_; j++)
    (*q)[joints[j].move3d_dof_index_] = point[j];

  // robot_model_->setAndUpdate( *q_tmp );
  //    robot_model_->setAndUpdate( *source_ );
  robot_model_->setAndUpdate(*T.getEnd());

  if (!ENV.getBool(Env::drawDisabled)) g3d_draw_allwin_active();
}

bool StompOptimizer::getMobileManipHandOver() {
  double best_cost = 0.0;
  pair<confPtr_t, confPtr_t> best_handover_conf;

  // Parse list to find
  // the best feasible hand-over configuration
  bool found_hover = handoverGenerator_->computeHandoverConfigFromList(
      best_handover_conf, best_cost);

  if (found_hover) {
    human_model_->setAndUpdate(*best_handover_conf.first);
    robot_model_->setAndUpdate(*best_handover_conf.second);
    target_new_ = best_handover_conf.second;
    // g3d_draw_allwin_active();
    // cout << "Hand-over found with cost : " << best_cost << endl;
  }

  return found_hover;
}

bool StompOptimizer::getManipulationHandOver() {
  std::vector<Eigen::Vector3d> points;

  if (use_handover_auto_) {
    HRICS_humanCostMaps->getHandoverPointList(
        points, recompute_handover_cell_list_, true);
    recompute_handover_cell_list_ = false;
  } else {
    points.resize(1);
    points[1] = human_model_->getJoint("rPalm")->getVectorPos();
    points[1][2] += 0.10;
  }

  // Compute IK
  bool found_ik = false;
  configPt q = NULL;
  ChronoTimeOfDayOn();
  double time = 0.0;
  for (int i = 0; i < int(points.size()) && time < 1.0 && found_ik == false;
       i++) {
    found_ik = handoverGenerator_->computeRobotIkForGrabing(q, points[i]);
    ChronoTimeOfDayTimes(&time);
  }
  ChronoTimeOfDayOff();

  // Disable cntrts
  p3d_cntrt* ct;
  p3d_rob* rob = robot_model_->getP3dRobotStruct();
  for (int i = 0; i < rob->cntrt_manager->ncntrts; i++) {
    ct = rob->cntrt_manager->cntrts[i];
    p3d_desactivateCntrt(rob, ct);
  }

  ArmManipulationData& armData = (*rob->armManipulationData)[0];

  deactivateCcCntrts(rob, 0);
  setAndActivateTwoJointsFixCntrt(
      rob,
      armData.getManipulationJnt(),
      armData.getCcCntrt()->pasjnts[armData.getCcCntrt()->npasjnts - 1]);

  if (found_ik) {
    target_new_ = confPtr_t(new Configuration(robot_model_, q));
    robot_model_->setAndUpdate(*target_new_);
    // g3d_draw_allwin_active();
  }

  return found_ik;
}

bool StompOptimizer::humanHasMoved() {
  if (human_model_ == NULL) {
    return false;
  }

  human_has_moved_ = false;

  confPtr_t q_hum = human_model_->getCurrentPos();

  (*q_hum)[6] = PlanEnv->getDouble(PlanParam::env_futurX);
  (*q_hum)[7] = PlanEnv->getDouble(PlanParam::env_futurY);
  (*q_hum)[8] = PlanEnv->getDouble(PlanParam::env_futurZ);
  (*q_hum)[11] = PlanEnv->getDouble(PlanParam::env_futurRZ);

  if (last_human_pos_[0] != (*q_hum)[6] || last_human_pos_[1] != (*q_hum)[7] ||
      last_human_pos_[2] != (*q_hum)[8] || last_human_pos_[3] != (*q_hum)[11]) {
    human_has_moved_ = true;
    human_model_->setAndUpdate(*q_hum);
  }

  last_human_pos_[0] = (*q_hum)[6];
  last_human_pos_[1] = (*q_hum)[7];
  last_human_pos_[2] = (*q_hum)[8];
  last_human_pos_[3] = (*q_hum)[11];

  return human_has_moved_;
}

bool StompOptimizer::getNewTargetFromHandOver() {
  if (handoverGenerator_ == NULL) {
    return false;
  }

  reset_reused_rollouts_ = human_has_moved_;

  if (!reset_reused_rollouts_) {
    return true;
  }

  bool success = false;
  if (use_handover_config_list_) {
    success = getMobileManipHandOver();
  } else {
    success = getManipulationHandOver();
  }
  return success;
}

bool StompOptimizer::replaceEndWithNewConfiguration() {
  // Retrieve a new configuration
  if (!getNewTargetFromHandOver()) {
    cout << "Fail to get new target to handover configuration" << endl;
    return false;
  }

  if (!reset_reused_rollouts_) {
    // Do not replace group trajectory
    return false;
  }

  // Generate new trajectory
  Move3D::CostOptimization T(robot_model_);
  setGroupTrajectoryToMove3DTraj(T);
  double step = T.getParamMax() / 20;

  // Try connection
  if (T.connectConfigurationToEnd(target_new_, step)) {
    target_ = target_new_;

    // Calculate the forward kinematics for the fixed states only in the first
    // iteration
    int start = free_vars_start_;
    int end = free_vars_end_;
    if (iteration_ == 0) {
      start = 0;
      end = num_vars_all_ - 1;
    }

    double param = 0.0;
    double step = T.getParamMax() / (num_vars_free_ - 1);

    vector<confPtr_t> vect(num_vars_all_);
    for (int i = start; i <= end; ++i) {
      vect[i] = T.configAtParam(param);
      param += step;
    }
    setGroupTrajectoryFromVectorConfig(vect);
    return true;
  } else {
    cout << "Fail to connect to handover configuration" << endl;
    return false;
  }
}

void StompOptimizer::draw() {
  // Draws two points in the trajectory
  //    int middle = (free_vars_start_+free_vars_end_)/2;

  //    int start = middle - 3;
  //    int end = middle + 3;

  //    planning_group_->draw(compute_fk_main_->getSegmentFrames()[start]);
  //    planning_group_->draw(compute_fk_main_->getSegmentFrames()[end]);

  //    planning_group_->draw();

  if (move3d_collision_space_) {
    // drawCollisionPoints();
  }
}

void StompOptimizer::drawCollisionPoints() {
  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  confPtr_t q = robot_model_->getCurrentPos();

  // Get the configuration dof values in the joint array
  Eigen::VectorXd joint_array(planning_group_->num_dofs_);

  for (int j = 0; j < planning_group_->num_dofs_; j++) {
    joint_array[j] = (*q)[joints[j].move3d_dof_index_];
  }

  getFrames(free_vars_start_, joint_array, *q);

  if (move3d_collision_space_) {
    // calculate the position of every collision point:
    for (int j = 0; j < num_collision_points_; j++) {
      const CollisionPoint& cp = planning_group_->collision_points_[j];
      cp.draw(
          compute_fk_main_
              ->getSegmentFrames()[free_vars_start_][cp.getSegmentNumber()]);
    }
  }
}

void StompOptimizer::visualizeState(int index) {}

void StompOptimizer::testMultiVariateGaussianSampler() {
  PolicyImprovementLoop pi_loop;
  pi_loop.initialize(this_shared_ptr_,
                     false,
                     group_trajectory_.getUseTime()
                         ? group_trajectory_.getDiscretization()
                         : 0.0);
  pi_loop.testSampler();
}

bool StompOptimizer::initialize(
    /*ros::NodeHandle& node_handle,*/ int num_time_steps) {
  //  // we already know these things, so do nothing
  return true;
}

/*
   void StompOptimizer::getTorques(int index, std::vector<double>& torques,
   const std::vector<KDL::Wrench>& wrenches)
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

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

confPtr_t StompOptimizer::getSource() {
  return getConfigurationOnGroupTraj(free_vars_start_);
}

confPtr_t StompOptimizer::getTarget() {
  return getConfigurationOnGroupTraj(free_vars_end_);
}

confPtr_t StompOptimizer::getConfigurationOnGroupTraj(int ith) {
  confPtr_t q = planning_group_->robot_->getCurrentPos();

  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;

  Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(ith).transpose();

  for (int j = 0; j < planning_group_->num_dofs_; j++)
    (*q)[joints[j].move3d_dof_index_] = point[j];

  return q;
}

double StompOptimizer::computeMove3DCost() {
  // calculate the forward kinematics for the fixed states only in the first
  // iteration:
  Move3D::Trajectory T(robot_model_);
  // const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;
  int ith_point = 0;

  // for each point in the trajectory
  for (int i = free_vars_start_; i <= free_vars_end_; ++i) {
    T.push_back(getConfigurationOnGroupTraj(i));
    ith_point++;
  }

  // cout << "T.getParamMax() : " << T.getParamMax() << endl;
  // cout << "T.getNbOfPaths() : " << T.getNbOfPaths() << endl;
  // cout << "T.cost() : " << T.cost() << endl;
  // T.print();
  // T.replaceP3dTraj();
  // return T.cost();
  // return T.costNPoints(ith_point);
  //    return T.costSum();
  return T.costDeltaAlongTraj();
}

void StompOptimizer::initPolicy() {
  policy_.reset(new CovariantTrajectoryPolicy());
  policy_->setPrintDebug(false);

  std::vector<double> derivative_costs =
      stomp_parameters_->getSmoothnessCosts();

  policy_->initialize(/*nh,*/
                      num_vars_free_,
                      num_joints_,
                      group_trajectory_.getDuration(),
                      stomp_parameters_->getRidgeFactor(),
                      derivative_costs,
                      planning_group_);

  // initialize the policy trajectory
  Eigen::VectorXd start =
      group_trajectory_.getTrajectoryPoint(free_vars_start_ - 1).transpose();
  Eigen::VectorXd end =
      group_trajectory_.getTrajectoryPoint(free_vars_end_ + 1).transpose();

  // set paramters
  int free_vars_start_index = DIFF_RULE_LENGTH - 1;
  vector<Eigen::VectorXd> parameters(num_joints_);
  for (int i = 0; i < num_joints_; i++) {
    parameters[i] = group_trajectory_.getJointTrajectory(i)
                        .segment(free_vars_start_index, num_vars_free_);
  }
  policy_->setToMinControlCost(start, end);
  policy_->setParameters(parameters);
}

bool StompOptimizer::getPolicy(
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy>& policy) {
  policy = policy_;
  return true;
}

bool StompOptimizer::setPolicy(
    const MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy> policy) {
  return true;
}

bool StompOptimizer::getControlCostWeight(double& control_cost_weight) {
  control_cost_weight = stomp_parameters_->getSmoothnessCostWeight();
  return true;
}

bool StompOptimizer::getStateCostWeight(double& state_cost_weight) {
  state_cost_weight = stomp_parameters_->getObstacleCostWeight();
  return true;
}

void StompOptimizer::copyPolicyToGroupTrajectory() {
  policy_->getParameters(policy_parameters_);

  for (int d = 0; d < num_joints_; ++d) {
    // cout << "group_trajectory_ : " << endl <<
    // group_trajectory_.getFreeJointTrajectoryBlock(d) << endl;
    // cout << "policy_parameters_ : " << endl << policy_parameters_[d] << endl;
    group_trajectory_.getFreeJointTrajectoryBlock(d) =
        policy_parameters_[d].segment(1, num_vars_free_ - id_fixed_);
  }
}

void StompOptimizer::copyGroupTrajectoryToPolicy() {
  for (int d = 0; d < num_joints_; ++d) {
    // policy_parameters_[d] = group_trajectory_.getFreeTrajectoryBlock();
    policy_parameters_[d].segment(1, num_vars_free_ - id_fixed_) =
        group_trajectory_.getFreeJointTrajectoryBlock(d);
  }

  // draw traj
  //    const std::vector<ChompDof>& joints =
  //    optimizer->getPlanningGroup()->chomp_dofs_;
  //    Robot* robot = optimizer->getPlanningGroup()->robot_;
  //    Move3D::Trajectory traj(robot);
  //
  //    for ( int j=0; j<policy_parameters_[0].size(); ++j)
  //    {
  //      confPtr_t q = robot->getCurrentPos();
  //
  //      for ( int i=0; i<optimizer->getPlanningGroup()->num_joints_; ++i)
  //      {
  //        (*q)[joints[i].move3d_dof_index_] = policy_parameters_[i][j];
  //      }
  //      traj.push_back(q);
  //    }
  //    traj.replaceP3dTraj();
  //    //    traj.print();
  //    g3d_draw_allwin_active();
  //    cout << "num_time_steps_ : " << policy_parameters_[0].size() << endl;

  policy_->setParameters(policy_parameters_);
}

void StompOptimizer::setSharedPtr(
    MOVE3D_BOOST_PTR_NAMESPACE<StompOptimizer>& ptr) {
  this_shared_ptr_ = ptr;
}

void StompOptimizer::resetSharedPtr() { this_shared_ptr_.reset(); }

void StompOptimizer::saveTrajectoryCostStats() {
  //    TrajectoryStatistics stat;
  //    setGroupTrajectoryToApiTraj( move3d_traj_ );
  //    move3d_traj_.costStatistics( stat );
  //    stomp_statistics_->convergence_rate.push_back( make_pair( time_, stat
  //    ));

  vector<confPtr_t> traj;
  setGroupTrajectoryToVectorConfig(traj);
  stomp_statistics_->convergence_trajs.push_back(make_pair(time_, traj));
  //    move3d_traj_.replaceP3dTraj();
  //    g3d_draw_allwin_active();
}

void StompOptimizer::saveCostFromConvergenceTraj() {
  stomp_statistics_->convergence_rate.clear();

  for (int i = 0; i < int(stomp_statistics_->convergence_trajs.size()); i++) {
    double time = stomp_statistics_->convergence_trajs[i].first;
    Move3D::Trajectory traj(stomp_statistics_->convergence_trajs[i].second);

    TrajectoryStatistics stat;
    traj.costStatistics(stat);
    stomp_statistics_->convergence_rate.push_back(make_pair(time, stat));
  }
}

/*!
   * This function saves the
   * trajectory cost to a file along the smoothing process
   */
void StompOptimizer::saveOptimToFile(string fileName) {
  saveCostFromConvergenceTraj();

  std::ostringstream oss;
  std::ofstream s;

  //----------------------------------------------------
  oss << "statFiles/convergence_traj_stomp_" << std::setfill('0')
      << std::setw(4) << stomp_statistics_->run_id << ".csv";

  const char* res = oss.str().c_str();

  s.open(res);
  cout << "Opening save file : " << res << endl;

  s << "TIME"
    << ";";
  s << "LENGTH"
    << ";";
  s << "MAX"
    << ";";
  s << "AVERAGE"
    << ";";
  s << "INTEGRAL"
    << ";";
  s << "MECHA WORK"
    << ";";
  s << "IS_VALID"
    << ";";
  s << "SUM"
    << ";";
  s << endl;

  for (int i = 0; i < int(stomp_statistics_->convergence_rate.size()); i++) {
    s << stomp_statistics_->convergence_rate[i].first << ";";
    s << stomp_statistics_->convergence_rate[i].second.length << ";";
    s << stomp_statistics_->convergence_rate[i].second.max << ";";
    s << stomp_statistics_->convergence_rate[i].second.average << ";";
    s << stomp_statistics_->convergence_rate[i].second.integral << ";";
    s << stomp_statistics_->convergence_rate[i].second.mecha_work << ";";
    s << stomp_statistics_->convergence_rate[i].second.is_valid << ";";
    s << stomp_statistics_->convergence_rate[i].second.sum << ";";
    s << endl;
  }

  s.close();
  cout << "Closing save file" << endl;

  //----------------------------------------------------
  std::ostringstream oss2;
  oss2.clear();
  oss2 << "statFiles/costsum_traj_stomp_" << std::setfill('0') << std::setw(4)
       << stomp_statistics_->run_id << ".csv";

  const char* res2 = oss2.str().c_str();

  s.open(res2);
  cout << "Opening save file : " << res2 << endl;

  s << "Cost";
  s << endl;

  for (int i = 0; i < int(stomp_statistics_->costs.size()); i++) {
    s << stomp_statistics_->costs[i];
    s << endl;
  }

  cout << "Closing save file" << endl;
  s.close();
}

}  // namespace stomp

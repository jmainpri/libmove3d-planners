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
#include "cost_computation.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "hri_costspace/HRICS_costspace.hpp"
#include "feature_space/features.hpp"
#include "feature_space/smoothness.hpp"
#include "API/Device/generalik.hpp"
#include "utils/misc_functions.hpp"
#include "utils/NumsAndStrings.hpp"

using namespace stomp_motion_planner;
using namespace Move3D;

using std::cout;
using std::endl;

const double hack_tweek = 1;

// *******************************************************
// API FUNCTIONS
// *******************************************************
static std::vector<boost::function<int(Move3D::Robot*)> >
    Move3DGetNumberOfCollisionPoints;
static std::vector<
    boost::function<bool(Move3D::Robot*,
                         int i,
                         Eigen::MatrixXd&,
                         std::vector<std::vector<Eigen::Vector3d> >&)> >
    Move3DGetConfigCollisionCost;

// *******************************************************
// SETTERS
// *******************************************************

void move3d_set_fct_get_nb_collision_points(
    boost::function<int(Move3D::Robot*)> fct, int id) {
  if (id == int(Move3DGetNumberOfCollisionPoints.size())) {
    Move3DGetNumberOfCollisionPoints.push_back(fct);
  } else if (id < int(Move3DGetConfigCollisionCost.size())) {
    Move3DGetNumberOfCollisionPoints[id] = fct;
  } else {
    cout << "ERROR SIZE in " << __PRETTY_FUNCTION__ << endl;
  }
}
void move3d_set_fct_get_config_collision_cost(
    boost::function<bool(Move3D::Robot* robot,
                         int i,
                         Eigen::MatrixXd&,
                         std::vector<std::vector<Eigen::Vector3d> >&)> fct,
    int id) {
  if (id == int(Move3DGetConfigCollisionCost.size())) {
    Move3DGetConfigCollisionCost.push_back(fct);
  } else if (id < int(Move3DGetConfigCollisionCost.size())) {
    Move3DGetConfigCollisionCost[id] = fct;
  } else {
    cout << "ERROR SIZE in " << __PRETTY_FUNCTION__ << endl;
  }
}

// *******************************************************
// MAIN
// *******************************************************

costComputation::costComputation(
    Robot* robot,
    const CollisionSpace* collision_space,
    const ChompPlanningGroup* planning_group,
    std::vector<ChompCost> joint_costs,
    ChompTrajectory group_trajectory,
    double obstacle_weight,
    bool use_costspace,
    Move3D::confPtr_t q_source,
    Move3D::confPtr_t q_target,
    bool use_external_collision_space,
    int collision_space_id,
    const stomp_motion_planner::StompParameters* stomp_parameters,
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy> policy,
    bool use_goal_set,
    const Eigen::VectorXd& x_task_goal) {
  robot_model_ = robot;

  cout << "new cost computation with robot : " << robot_model_->getName()
       << " collision id : " << collision_space_id << endl;

  stomp_parameters_ = stomp_parameters;

  source_ = q_source;
  target_ = q_target;

  // Collision space
  move3d_collision_space_ = collision_space;
  use_external_collision_space_ = use_external_collision_space;
  collision_space_id_ = collision_space_id;

  planning_group_ = planning_group;
  joint_costs_ = joint_costs;

  group_trajectory_ = group_trajectory;

  obstacle_weight_ = obstacle_weight;
  use_costspace_ = use_costspace;
  hack_tweek_ = 1.0;

  iteration_ = 0;

  num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();
  num_joints_ = group_trajectory_.getNumJoints();
  free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

  num_collision_points_ = planning_group_->collision_points_.size();

  collision_point_pos_eigen_.resize(num_vars_all_);
  collision_point_vel_eigen_.resize(num_vars_all_);
  collision_point_acc_eigen_.resize(num_vars_all_);

  state_is_in_collision_.resize(num_vars_all_);

  general_cost_potential_ = Eigen::VectorXd::Zero(num_vars_all_);
  // smoothness_cost_potential_ = Eigen::VectorXd::Zero(num_vars_all_);
  use_total_smoothness_cost_ = true;
  total_smoothness_cost_ = 0.;

  dt_ = Eigen::VectorXd::Zero(num_vars_all_);

  if (num_collision_points_ > 0) {
    collision_point_potential_ =
        Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
    collision_point_vel_mag_ =
        Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
    collision_point_potential_gradient_.resize(
        num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

    for (int i = 0; i < num_vars_all_; i++) {
      collision_point_pos_eigen_[i].resize(num_collision_points_);
      collision_point_vel_eigen_[i].resize(num_collision_points_);
      collision_point_acc_eigen_[i].resize(num_collision_points_);
    }
  }

  ///---------------------------------------------------------------------------
  /// Smoothness features
  // Initialize control costs
  multiple_smoothness_ = true;

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

  ///---------------------------------------------------------------------------
  /// Get the policy pointer
  policy_ = boost::static_pointer_cast<
      stomp_motion_planner::CovariantTrajectoryPolicy,
      stomp_motion_planner::Policy>(policy);

  policy_->getControlCosts(control_costs_);
  policy_->getCovariances(inv_control_costs_);
  control_cost_weight_ = stomp_parameters_->getSmoothnessCostWeight();

  for (size_t d = 0; d < control_costs_.size(); d++) {
    move3d_save_matrix_to_csv_file(
        control_costs_[d].inverse(),
        "control_matrices/cost_computation_control_inverse_" +
            num_to_string<int>(d) + ".csv");
  }

  ///---------------------------------------------------------------------------
  /// GOAL SET CONSTRAINT

  // Move the end configuration in the trajectory
  id_fixed_ = 2;
  allow_end_configuration_motion_ = use_goal_set;
  if (allow_end_configuration_motion_) {
    id_fixed_ = 1;
  }
  group_trajectory_.setNbFixedPoints(id_fixed_);
  project_last_config_ = use_goal_set;
  // PlanEnv->getBool(PlanParam::trajStompMoveEndConfig);
  terminal_cost_ = 0.0;  // Terminal cost for the robot

  // se the goal set projector
  use_goal_set_projector_ = true;

  if (project_last_config_ && (planning_group_ != NULL)) {
    ratio_projected_ = 1.0;  // 1.0 = all along the trajectory
    strength_ = PlanEnv->getDouble(PlanParam::trajStompConstStrength);

    Move3D::Robot* robot = planning_group_->robot_;
    eef_ = robot->getJoint(
        PlanEnv->getString(PlanParam::end_effector_joint));  // plannar J4
    if (eef_ != NULL) {
      x_task_goal_ = x_task_goal.head(3);  // head(2) only (x,y)
    } else {
      project_last_config_ = false;
      x_task_goal_ = Eigen::VectorXd::Zero(0);
    }

    if (use_goal_set_projector_) {
      std::vector<Eigen::MatrixXd> A_mat;
      for (size_t d = 0; d < inv_control_costs_.size(); d++) {
        A_mat.push_back(inv_control_costs_[d].inverse());
      }

      goal_set_projector_.Initialize(planning_group_, A_mat);
      goal_set_projector_.set_print_convergence(false);

      for (size_t d = 0; d < control_costs_.size(); d++) {
        // cout << "control_cost[" << d << "] : " << control_costs_[d] << endl;

        move3d_save_matrix_to_csv_file(
            control_costs_[d].inverse(),
            "control_matrices/inv_control_costs_goalset_" +
                num_to_string<int>(d) + ".csv");

        move3d_save_matrix_to_csv_file(inv_control_costs_[d],
                                       "control_matrices/covariance_goalset_" +
                                           num_to_string<int>(d) + ".csv");
      }

      cout << "initialized goal_set_projector_ " << endl;
    }
  }

  ///--------------------------------------------------------------------------
  /// Joint limits

  //  joint_limits_projector_.resize(num_joints_);

  //  for (int joint = 0; joint < num_joints_; joint++) {
  //    double max_limit =
  //        planning_group_->chomp_dofs_[joint].joint_limit_max_ - 1e-5;
  //    double min_limit =
  //        planning_group_->chomp_dofs_[joint].joint_limit_min_ + 1e-5;

  //    joint_limits_projector_[joint].dynamics_ = control_costs_[joint].block(
  //        1, 1, num_vars_free_ - id_fixed_, num_vars_free_ - id_fixed_);
  //    joint_limits_projector_[joint].upper_ =
  //        max_limit * Eigen::VectorXd::Ones(num_vars_free_ - id_fixed_);
  //    joint_limits_projector_[joint].lower_ =
  //        min_limit * Eigen::VectorXd::Ones(num_vars_free_ - id_fixed_);

  //    if (!joint_limits_projector_[joint].initialize())

  //      cout << "ERROR could not initialize joint limits in "
  //           << __PRETTY_FUNCTION__
  //           << " for joint : " <<
  //           planning_group_->chomp_dofs_[joint].joint_name_
  //           << endl;
  //  }

  if (!CreateJointLimitProjectors(joint_limits_projector_,
                                  *planning_group_,
                                  control_costs_,
                                  num_vars_free_,
                                  1,
                                  id_fixed_)) {
    cout << "Error: in cost computation constructor" << endl;
  }
}

costComputation::~costComputation() {
  cout << "destroy cost computation" << endl;
}

bool costComputation::checkJointLimits(const ChompTrajectory& group_traj) {
  bool succes_joint_limits = true;

  for (int joint = 0; joint < num_joints_; joint++) {
    if (!planning_group_->chomp_dofs_[joint].has_joint_limits_) continue;

    double joint_max = planning_group_->chomp_dofs_[joint].joint_limit_max_;
    double joint_min = planning_group_->chomp_dofs_[joint].joint_limit_min_;

    double max_abs_violation = 1e-6;
    double violate_max_with = 0;
    double violate_min_with = 0;
    double absolute_amount = 0.0;

    for (int i = free_vars_start_; i <= free_vars_end_; i++) {
      if (group_traj(i, joint) > joint_max) {
        // cout << "value is : " << group_traj( i, joint ) << " , max is : " <<
        // joint_max << endl;
        absolute_amount = std::fabs(joint_max - group_traj(i, joint));

        if (absolute_amount > violate_max_with) {
          violate_max_with = absolute_amount;
        }
      } else if (group_traj(i, joint) < joint_min) {
        // cout << "value is : " << group_traj( i, joint ) << " , min is : " <<
        // joint_min << endl;
        absolute_amount = std::fabs(joint_min - group_traj(i, joint));

        if (absolute_amount > violate_min_with) {
          violate_min_with = absolute_amount;
        }
      }

      if (absolute_amount > max_abs_violation) {
        cout << "joint is out of limits : "
             << planning_group_->chomp_dofs_[joint].joint_name_ << endl;
        cout << "value is : " << group_traj(i, joint);
        cout << " , min is : " << joint_min;
        cout << " , max is : " << joint_max << endl;
        cout << "absolute_amount : " << absolute_amount;
        cout << " , max_abs_violation : " << max_abs_violation;
        cout << endl;

        max_abs_violation = absolute_amount;
        succes_joint_limits = false;

        break;
      }
    }

    if (!succes_joint_limits) return false;
  }

  return true;
}

bool costComputation::handleJointLimitsQuadProg(ChompTrajectory& group_traj) {
  for (int joint = 0; joint < num_joints_; joint++) {
    if (!planning_group_->chomp_dofs_[joint].has_joint_limits_) continue;

    Eigen::VectorXd joint_traj = group_traj.getFreeJointTrajectoryBlock(joint);

    double cost = joint_limits_projector_[joint].project(joint_traj);

    if (cost == std::numeric_limits<double>::infinity()) {
      cout << "joint limits did not converge" << endl;
    }
    // cout << "cost : " << cost << endl;
    group_traj.getFreeJointTrajectoryBlock(joint) = joint_traj;
  }

  if (checkJointLimits(group_traj)) {
    // cout << "Quad prog ok for joint limits" << endl;
    return true;
  }

  // cout << "joint limits not ok! " << endl;

  return false;
}

bool costComputation::handleJointLimits(ChompTrajectory& group_traj) {
  bool succes_joint_limits = true;
  bool joint_limits_violation = 0;

  for (int joint = 0; joint < num_joints_; joint++) {
    if (!planning_group_->chomp_dofs_[joint].has_joint_limits_) continue;

    // Added by jim for pr2 free flyer
    if (planning_group_->robot_->getName() == "PR2_ROBOT") {
      int index = planning_group_->chomp_dofs_[joint].move3d_dof_index_;

      if (index == 8 || index == 9 || index == 10) {
        group_traj.getFreeJointTrajectoryBlock(joint) =
            Eigen::VectorXd::Zero(num_vars_free_);
        continue;
      }
    }

    double joint_max = planning_group_->chomp_dofs_[joint].joint_limit_max_;
    double joint_min = planning_group_->chomp_dofs_[joint].joint_limit_min_;

    // cout << "Joint max : " << joint_max << endl;
    // cout << "Joint min : " << joint_min << endl;
    int count = 0;
    bool violation = false;
    int count_violation = false;

    do {
      double max_abs_violation = 1e-6;
      double max_violation = 0.0;
      int max_violation_index = 0;
      violation = false;
      count_violation = false;
      // bool violate_max = false;
      // bool violate_min = false;
      double violate_max_with = 0;
      double violate_min_with = 0;

      for (int i = free_vars_start_; i <= free_vars_end_; i++) {
        double amount = 0.0;
        double absolute_amount = 0.0;

        if (group_traj(i, joint) > joint_max) {
          amount = joint_max - group_traj(i, joint);
          absolute_amount = fabs(amount);
          // violate_max = true;
          if (absolute_amount > violate_max_with) {
            violate_max_with = absolute_amount;
          }
        } else if (group_traj(i, joint) < joint_min) {
          amount = joint_min - group_traj(i, joint);
          absolute_amount = fabs(amount);
          // violate_min = true;
          if (absolute_amount > violate_min_with) {
            violate_min_with = absolute_amount;
          }
        }
        if (absolute_amount > max_abs_violation) {
          max_abs_violation = absolute_amount;
          max_violation = amount;
          max_violation_index = i;

          // cout << "Violation (" << absolute_amount ;
          // cout << " , " << max_abs_violation << ")" <<
          // endl;

          if (i != free_vars_start_ && i != free_vars_end_) {
            count_violation = true;
            violation = true;
          }
        }
      }

      if (violation) {
        // Only count joint limits violation for
        // interpolated configurations
        if (count_violation) {
          joint_limits_violation++;
        }

        int free_var_index = max_violation_index - free_vars_start_;
        const Eigen::MatrixXd& matrix_joint =
            joint_costs_[joint].getQuadraticCostInverse();
        double multiplier =
            max_violation / matrix_joint(free_var_index, free_var_index);
        Eigen::VectorXd offset =
            (multiplier * matrix_joint.col(free_var_index));

        group_traj(max_violation_index, joint) =
            group_traj(max_violation_index, joint) + offset(free_var_index);

        // group_traj.getFreeJointTrajectoryBlock(joint) += offset.segment( 1,
        // num_vars_free_-id_fixed_ );

        /**
        cout << " ---- " << endl;
        cout << "joint : " << joint << endl;
        cout << "max_violation_index : " << max_violation_index << endl;
        cout << "joint_max : " << joint_max << endl;
        cout << "joint_min : " << joint_min << endl;


        cout << "group_traj(max_violation_index, joint) : " <<
        group_traj(max_violation_index, joint) << endl;

        cout << "max_violation : " << max_violation << endl;
        cout << "offset : " << offset(free_var_index) << endl;

        group_traj.getFreeJointTrajectoryBlock(joint) += offset.segment( 1,
        num_vars_free_-id_fixed_ );

        cout << "group_traj(max_violation_index, joint) : " <<
        group_traj(max_violation_index, joint) << endl;
        **/
      }
      if (++count > num_vars_free_) {
        succes_joint_limits = false;
        // cout << "group_trajectory_(i, joint) = " << endl <<
        // group_trajectory_.getFreeJointTrajectoryBlock(joint) << endl;
        break;
      }
    } while (violation);

    if (violation || !succes_joint_limits) {
      cout << "Violation of joint limits (joint) = "
           << planning_group_->chomp_dofs_[joint].joint_name_ << endl;
    }
  }

  //    cout << "succes_joint_limits : " << succes_joint_limits << endl;

  return succes_joint_limits;
}

bool costComputation::getControlCosts(const ChompTrajectory& group_traj) {
  std::vector<Eigen::VectorXd> parameters(num_joints_);
  group_traj.getFreeParameters(parameters);  // TODO check the paramters size

  // cout << "num_vars_free_ : " << num_vars_free_ << endl;
  current_control_costs_ = std::vector<Eigen::VectorXd>(
      num_joints_, Eigen::VectorXd::Zero(num_vars_free_));

  if (multiple_smoothness_) {
    // cout << "policy_.get() : " << policy_.get() << endl;

    std::vector<std::vector<Eigen::VectorXd> > control_costs;

    double dt = group_traj.getUseTime() ? group_traj.getDiscretization() : 0.;

    static_cast<CovariantTrajectoryPolicy*>(policy_.get())
        ->getAllCosts(parameters, control_costs, dt);

    if (!use_total_smoothness_cost_) {
      // cout << "current_control_costs_ : " <<
      // control_cost_weights_.transpose()
      //    << endl;

      // joint space smoothness is defined per joint
      for (int c = 0; c < 4; ++c) {
        for (int d = 0; d < num_joints_; ++d) {
          // cout << std::scientific << "control_costs[" << c << "][" << d
          //    << "] : " << control_costs[c][d].transpose() << endl;

          current_control_costs_[d] +=
              (control_cost_weights_[c] * control_costs[c][d]);
        }
      }
      // for the task space smoothness is only defined per state

      for (int c = 4; c < 8; ++c) {
        for (int d = 0; d < num_joints_; ++d) {
          // cout << std::scientific << "control_costs[" << c << "][" << d
          //    << "] : " << control_costs[c][d].transpose() << endl;

          current_control_costs_[d] +=
              (control_cost_weights_[c] * control_costs[c][d] /
               double(num_joints_));
        }
      }
    } else {
      // Get total smoothness cost (see Mrinal's thesis)
      total_smoothness_cost_ = 0.;
      for (int c = 0; c < 4; ++c)
        for (int d = 0; d < num_joints_; ++d)
          total_smoothness_cost_ +=
              control_cost_weights_[c] * control_costs[c][d].sum();
      for (int c = 4; c < 8; ++c)
        total_smoothness_cost_ +=
            control_cost_weights_[c] * control_costs[c][0].sum();
    }
  } else {
    std::vector<Eigen::VectorXd> noise = std::vector<Eigen::VectorXd>(
        num_joints_, Eigen::VectorXd::Zero(num_vars_free_));

    policy_->computeControlCosts(
        control_costs_,
        parameters,
        noise,
        control_cost_weight_,
        current_control_costs_,
        group_traj.getUseTime() ? group_traj.getDiscretization() : 0.);

    if (use_total_smoothness_cost_) {
      // Get total smoothness cost current control costs are already
      // multipied by the cost weight
      total_smoothness_cost_ = 0.;
      int num_dim = current_control_costs_.size();
      for (int d = 0; d < num_dim; ++d)
        total_smoothness_cost_ += current_control_costs_[d].sum();
    }
  }

  return true;
}

double costComputation::getCollisionSpaceCost(const Configuration& q) {
  bool quiet = true;

  Configuration q_tmp = q;
  robot_model_->setAndUpdate(q_tmp);

  double state_collision_cost = 0.0;
  bool colliding = false;

  // Calculate the 3d position of every collision point
  for (int j = 0; j < num_collision_points_; j++) {
    // OLD (free_vars_start_ is the first point, does not matter
    // when called from outside)
    colliding |= getCollisionPointObstacleCost(
        free_vars_start_,
        j,
        collision_point_potential_(free_vars_start_, j),
        collision_point_pos_eigen_[free_vars_start_][j]);

    state_collision_cost += collision_point_potential_(free_vars_start_, j);
    // * collision_point_vel_mag_( free_vars_start_, j );

    if (!quiet)
      cout << "collision_point_potential_( " << free_vars_start_ << " , " << j
           << " ) : " << collision_point_potential_(free_vars_start_, j)
           << endl;
  }

  if (colliding && !quiet) {
    cout << "config is in collision" << endl;
  }

  return state_collision_cost;
}

void costComputation::getMove3DConfiguration(const Eigen::VectorXd& joint_array,
                                             Configuration& q) const {
  const std::vector<ChompDof>& dofs = planning_group_->chomp_dofs_;

  if (int(dofs.size()) != joint_array.size()) {
    cout << "Error: in the costComputation get move3d configuration" << endl;
  }

  // Set the configuration to the joint array value
  for (size_t j = 0; j < dofs.size(); j++) {
    int dof = dofs[j].move3d_dof_index_;
    if (!std::isnan(joint_array[j])) {
      q[dof] = joint_array[j];
    } else {
      cout << "q[" << dof << "] is nan" << endl;
      q[dof] = 0;
    }
  }
}

bool costComputation::getCollisionPointObstacleCost(
    int segment,
    int coll_point,
    double& collion_point_potential,
    Eigen::Vector3d& pos) {
  bool colliding = false;

  if (move3d_collision_space_) {
    int i = segment;
    int j = coll_point;
    double distance;

    const Move3D::CollisionPoint& coll_point =
        planning_group_->collision_points_[j];

    //    cout << "collision point[" << j
    //         << "] : " <<
    //         planning_group_->collision_points_[j].joint()->getName()
    //         << endl;

    // TRO testing
    // does not do the same for PR2 !!!
    // the number of segment frames is not equal to the number of key points
    // TODO store the matrices somewhere
    coll_point.getTransformedPosition(coll_point.joint()->getMatrixPos(), pos);

    // To fix in collision space The joint 1 is allways colliding
    // is this true?
    colliding = move3d_collision_space_->getCollisionPointPotentialGradient(
        coll_point,
        pos,
        distance,
        collion_point_potential,
        collision_point_potential_gradient_[i][j]);
  }

  return colliding;
}

bool costComputation::getConfigObstacleCost(
    Move3D::Robot* robot,
    int i,
    Eigen::MatrixXd& collision_point_potential,
    std::vector<std::vector<Eigen::Vector3d> >& collision_point_pos) {
  //    cout << "get collision cost : " << robot->getName() << endl;
  bool in_collision = false;

  // calculate the position of every collision point
  for (int j = 0; j < num_collision_points_; j++) {
    bool colliding = getCollisionPointObstacleCost(
        i, j, collision_point_potential(i, j), collision_point_pos[i][j]);

    if (colliding) {
      // TRO 2016 Warning there was a functin here to discard some of the
      // collision points, make sure is is still safe without
      // This is the function that discards joints too close to the base
      // planning_group_->collision_points_[j].getSegmentNumber() > 1
      in_collision = true;
    }
  }

  return in_collision;
}

int costComputation::getNumberOfCollisionPoints(Move3D::Robot* R) {
  return planning_group_->collision_points_.size();
}

bool costComputation::performForwardKinematics(
    const ChompTrajectory& group_traj, bool is_rollout) {
  double invTime = 1.0 / group_traj.getDiscretization();
  double invTimeSq = invTime * invTime;

  // calculate the forward kinematics for the fixed states only in the first
  // iteration:
  int start = free_vars_start_;
  int end = free_vars_end_;

  is_collision_free_ = true;

  Eigen::VectorXd joint_array(num_joints_);

  Move3D::Configuration q(*source_);
  Move3D::Configuration q_tmp(*robot_model_->getCurrentPos());
  Move3D::Configuration q_prev(robot_model_);
  Move3D::Trajectory current_traj(robot_model_);

  int k = 0;
  int way_point_ratio = -1;

  // for each point in the trajectory
  for (int i = start; i <= end; ++i) {
    state_is_in_collision_[i] = false;

    group_traj.getTrajectoryPointP3d(group_traj.getFullTrajectoryIndex(i),
                                     joint_array);

    // Get configuration
    getMove3DConfiguration(joint_array, q);
    current_traj.push_back(confPtr_t(new Configuration(q)));

    // Set robot to configuration
    robot_model_->setAndUpdate(q);

    // this->getFrames(i, joint_array, q);  // Perform FK (set and update)

    if (false && is_rollout && (i % way_point_ratio != 0) && (i != start) &&
        (i != end)) {
      cout << "no fk for : " << i << endl;
      general_cost_potential_[i] = general_cost_potential_[i - 1];
      state_is_in_collision_[i] = state_is_in_collision_[i - 1];

      if (move3d_collision_space_ || use_external_collision_space_)
        for (int j = 0; j < num_collision_points_; j++) {
          collision_point_potential_(i, j) =
              collision_point_potential_(i - 1, j);
          collision_point_pos_eigen_[i][j] =
              collision_point_pos_eigen_[i - 1][j];
        }
    } else {
      if (use_costspace_) {
        // no set and update
        general_cost_potential_[i] = global_costSpace->cost(q);
      } else if (PlanEnv->getBool(PlanParam::useLegibleCost)) {
        general_cost_potential_[i] = HRICS_activeLegi->legibilityCost(i);
      }

      if (move3d_collision_space_ || use_external_collision_space_) {
        state_is_in_collision_[i] =
            Move3DGetConfigCollisionCost[collision_space_id_](
                robot_model_,
                i,
                collision_point_potential_,
                collision_point_pos_eigen_);
      } else if ((PlanEnv->getBool(PlanParam::useLegibleCost) ||
                  use_costspace_) &&
                 (move3d_collision_space_ == NULL)) {
        state_is_in_collision_[i] = false;
      }

      if (state_is_in_collision_[i]) {
        is_collision_free_ = false;
      }
    }

    dt_[i] = group_traj.getUseTime() || (i == start)
                 ? group_traj.getDiscretization()
                 : q.dist(q_prev);

    q_prev = q;    // store configuration
    q = *source_;  // Make sure dofs are set to source

    k++;
  }

  // now, get the vel and acc for each collision point
  // using finite differencing)
  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    for (int j = 0; j < num_collision_points_; j++) {
      collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
      collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();

      for (int k = -DIFF_RULE_LENGTH / 2; k <= DIFF_RULE_LENGTH / 2; k++) {
        collision_point_vel_eigen_[i][j] +=
            (invTime * DIFF_RULES[0][k + DIFF_RULE_LENGTH / 2]) *
            collision_point_pos_eigen_[i + k][j];

        collision_point_acc_eigen_[i][j] +=
            (invTimeSq * DIFF_RULES[1][k + DIFF_RULE_LENGTH / 2]) *
            collision_point_pos_eigen_[i + k][j];
      }
      // get the norm of the velocity:
      collision_point_vel_mag_(i, j) = collision_point_vel_eigen_[i][j].norm();
    }
  }

  robot_model_->setAndUpdate(q_tmp);
  return is_collision_free_;
}

double costComputation::resampleParameters(
    std::vector<Eigen::VectorXd>& parameters) {
  const std::vector<ChompDof>& joints = planning_group_->chomp_dofs_;
  Move3D::Smoothing traj(planning_group_->robot_);
  Eigen::MatrixXd parameters_tmp(num_joints_, num_vars_free_);

  for (int i = 0; i < num_joints_; ++i) {
    parameters_tmp.row(i) = parameters[i].transpose();
  }

  traj.clear();
  for (int j = 0; j < num_vars_free_; ++j) {
    // Set the configuration from the stored source and target
    confPtr_t q;

    if (j == 0) {
      q = source_;
    } else if (j == num_vars_free_ - 1) {
      q = target_;
    } else {
      // q = passive_dofs_[j];
      q = planning_group_->robot_->getCurrentPos();

      for (int i = 0; i < planning_group_->num_dofs_; ++i) {
        (*q)[joints[i].move3d_dof_index_] = parameters_tmp(i, j);
      }
    }

    traj.push_back(q);
  }

  PlanEnv->setBool(PlanParam::trajStompComputeColl, false);
  traj.runShortCut(15);
  PlanEnv->setBool(PlanParam::trajStompComputeColl, true);

  // calculate the forward kinematics for the fixed states only in the first
  // iteration:
  double step = traj.getParamMax() / (num_vars_free_ - 1);
  double param = step;
  for (int j = 0; j < num_vars_free_; ++j) {
    confPtr_t q = traj.configAtParam(param);

    for (int i = 0; i < planning_group_->num_dofs_; ++i) {
      parameters_tmp(i, j) = (*q)[joints[i].move3d_dof_index_];
    }
    param += step;
  }

  for (int i = 0; i < num_joints_; ++i) {
    parameters[i].transpose() = parameters_tmp.row(i);
  }

  return traj.cost();
}

double costComputation::terminalCost(const ChompTrajectory& group_traj) const {
  Eigen::VectorXd joint_array(num_joints_);
  Move3D::confPtr_t q_end = robot_model_->getCurrentPos();

  group_traj.getTrajectoryPointP3d(
      group_traj.getFullTrajectoryIndex(free_vars_end_), joint_array);
  getMove3DConfiguration(joint_array, *q_end);

  robot_model_->setAndUpdate(*q_end);

  double squared_norm =
      (x_task_goal_ - Eigen::VectorXd(eef_->getVectorPos().head(
                          x_task_goal_.size()))).squaredNorm();

  const double coeff = PlanEnv->getDouble(PlanParam::trajOptimTermWeight);
  return coeff * squared_norm;
}

bool costComputation::projectToConstraints(ChompTrajectory& group_traj) const {
  Eigen::VectorXd joint_array(num_joints_);
  Move3D::confPtr_t q_end = robot_model_->getCurrentPos();

  group_traj.getTrajectoryPointP3d(
      group_traj.getFullTrajectoryIndex(free_vars_end_), joint_array);
  getMove3DConfiguration(joint_array, *q_end);

  Move3D::GeneralIK ik(robot_model_);
  robot_model_->setAndUpdate(*q_end);

  ik.initialize(planning_group_->getActiveJoints(), eef_);

  // cout << "x_task_goal_.size() : " << x_task_goal_.size() << endl;

  bool solved = ik.solve(x_task_goal_);
  if (solved) {
    Move3D::confPtr_t q_cur = robot_model_->getCurrentPos();
    // The active dofs from the ik might differ from the dofs
    // of the planning group because IK does not handle P3D_FREEFLYERs
    std::vector<int> active_dofs = planning_group_->getActiveDofs();
    Eigen::VectorXd q_1 = q_end->getEigenVector(active_dofs);
    Eigen::VectorXd q_2 = q_cur->getEigenVector(active_dofs);
    Eigen::VectorXd dq = q_2 - q_1;
    // cout << "dq : " << dq.transpose() << endl;

    int start = double(1. - ratio_projected_) * num_vars_free_;
    double alpha = 0.0;
    double delta = 1.0 / double(num_vars_free_ - start);

    for (int i = free_vars_start_ + start; i <= free_vars_end_; i++) {
      for (int dof = 0; dof < num_joints_; dof++) {
        group_traj(i, dof) += std::pow(alpha, strength_) * dq[dof];
      }

      alpha += delta;
    }
  }

  // cout << "ik solved : " << solved << endl;
  return solved;
}

bool costComputation::projectToConstraintWithQP(
    Move3D::ChompTrajectory& group_traj) {
  // Get parameters
  bool succeded = false;
  bool use_jacobian_projector = true;
  if (use_jacobian_projector) {
    std::vector<Eigen::VectorXd> parameters;
    parameters.resize(num_joints_);
    for (int dof = 0; dof < num_joints_; dof++) {
      parameters[dof] = Eigen::VectorXd::Zero(num_vars_free_);
    }
    for (int i = free_vars_start_; i <= free_vars_end_; i++) {
      for (int dof = 0; dof < num_joints_; dof++) {
        parameters[dof][i - free_vars_start_] = group_traj(i, dof);
      }
    }

    if (goal_set_projector_.ProjectGoalSetProblem(x_task_goal_, parameters)) {
      // set the parameters back to the trajectory
      Move3D::ChompTrajectory group_traj_tmp = group_traj;
      // cout << "set new parameter to group traj" << endl;
      for (int i = free_vars_start_; i <= free_vars_end_; i++) {
        for (int dof = 0; dof < num_joints_; dof++) {
          group_traj_tmp(i, dof) = parameters[dof][i - free_vars_start_];
        }
      }

      // Might be off the goal set because of joint limit projected
      // withing joint limits
      // if (checkJointLimits(group_traj_tmp)) {
      //  cout << "Warning: projection ouside joint limts" << endl;
      // }
      if (handleJointLimitsQuadProg(group_traj_tmp)) {
        group_traj = group_traj_tmp;
        succeded = true;
      }
    }
  } else {
    Move3D::ChompTrajectory group_traj_tmp = group_traj;
    if (projectToConstraints(group_traj_tmp)) {
      if (handleJointLimitsQuadProg(group_traj_tmp)) {
        group_traj = group_traj_tmp;
        succeded = true;
      }
    }
  }
  return succeded;
}

bool costComputation::getCost(std::vector<Eigen::VectorXd>& parameters,
                              Eigen::VectorXd& costs,
                              bool is_rollout,
                              bool& within_constraints) {
  // copy the parameters to the group_trajectory_
  for (int d = 0; d < num_joints_; ++d) {
    group_trajectory_.getFreeJointTrajectoryBlock(d) =
        parameters[d].segment(1, parameters[d].size() - id_fixed_);
    // The use of segment prevents the drift
  }
  // cout << " is rollout : " << is_rollout
  //     << " is finite : " << is_finite(group_trajectory_.getTrajectory())
  //     << endl;

  // Project to the joint limits
  succeded_joint_limits_ = handleJointLimitsQuadProg(group_trajectory_);
  //  succeded_joint_limits_ = handleJointLimits(group_trajectory_);
  within_constraints = succeded_joint_limits_;

  terminal_cost_ = 0.0;
  bool ik_solved = false;
  if (project_last_config_ && succeded_joint_limits_) {
    if (use_goal_set_projector_) {
      if (projectToConstraintWithQP(group_trajectory_)) {
        ik_solved = true;
      } else {
        cout << "ik NOT solved" << endl;
      }
    } else {
      ik_solved = true;
      terminal_cost_ = terminalCost(group_trajectory_);
    }

    // Set to ouside joint limits if could not
    // be projected on goal set
    within_constraints = ik_solved;

    // TODO change that for using the new constraint variable ...
    succeded_joint_limits_ = within_constraints;
  }

  // copy the group_trajectory_ if constraints are valid
  if (within_constraints) {
    // cout << "return with joint limits" << endl;
    // cout << "Change rollout" << endl;
    for (int d = 0; d < num_joints_; ++d) {
      parameters[d].segment(1, parameters[d].size() - id_fixed_) =
          group_trajectory_.getFreeJointTrajectoryBlock(d);
    }
  }

  // do forward kinematics
  bool trajectory_collision_free =
      performForwardKinematics(group_trajectory_, is_rollout);
  // bool last_trajectory_constraints_satisfied = true;

  // Special case for not handling joint limits
  if (!within_constraints) {
    is_collision_free_ = false;
    trajectory_collision_free = false;  // collision free but not valid
  }

  getControlCosts(group_trajectory_);

  for (int i = free_vars_start_; i <= free_vars_end_; i++) {
    double state_collision_cost = 0.0;
    double state_general_cost = 0.0;
    double cumulative = 0.0;

    if (move3d_collision_space_ || use_external_collision_space_) {
      for (int j = 0; j < num_collision_points_; j++) {
        cumulative +=
            collision_point_potential_(i, j) *
            collision_point_vel_mag_(
                i, j);  // Becarful ... with velocity breaks if taken off
        state_collision_cost += cumulative;
      }
    }
    if (use_costspace_ || PlanEnv->getBool(PlanParam::useLegibleCost)) {
      state_general_cost = (std::pow(general_cost_potential_(i), hack_tweek));
    }

    double cost = 0.0;

    // First terms compute the integral of cost
    cost += stomp_parameters_->getObstacleCostWeight() * state_collision_cost;
    cost += stomp_parameters_->getGeneralCostWeight() * state_general_cost;
    cost *= dt_[i];

    // Second term added from Mrinal's thesis
    // to account for the smoothness criteria
    cost += (total_smoothness_cost_ / double(num_vars_free_));

    // Add the terminal cost
    cost += (terminal_cost_ / double(num_vars_free_));

    // Add the cost to the pertime variable cost
    costs(i - free_vars_start_) = cost;

    if (std::isnan(state_collision_cost)) {
      cout << "nan state_collision_cost : " << i << endl;
    }
    if (std::isnan(state_general_cost)) {
      cout << "nan state_general_cost : " << i << endl;
    }

    // cout << "state_collision_cost : " << state_collision_cost << endl;
  }

  // Test that the costs can be used for update
  if (!is_finite(costs)) {
    succeded_joint_limits_ = false;
    is_collision_free_ = false;
    trajectory_collision_free = false;
    cout << "set invalid sample" << endl;
    cout << "Error : " << __PRETTY_FUNCTION__ << endl;
  }

  return trajectory_collision_free;
}

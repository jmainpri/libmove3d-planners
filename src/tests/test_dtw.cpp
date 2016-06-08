/*
 * Copyright (c) 2010-2015 LAAS/CNRS, WPI
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
 *                                               Jim Mainprice Sun 5 July 2015
 */

#include <iostream>
#include <iomanip>

#include "planner/TrajectoryOptim/vector_trajectory.hpp"
#include "planner/TrajectoryOptim/goal_set_projection.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"
#include "planner/TrajectoryOptim/jointlimits.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerSequences.hpp"
#include "collision_space/collision_space_factory.hpp"
#include "human_trajectories/HRICS_ioc.hpp"
#include "human_trajectories/HRICS_dynamic_time_warping.hpp"
#include "utils/misc_functions.hpp"
#include "API/project.hpp"

// For the seed ...
#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;

//------------------------------------------------------------------------------
// Sample a vector of trajectories
std::vector<HRICS::IocTrajectory> sample_trajectories(
    HRICS::IocSampler& sampler,
    const HRICS::IocTrajectory& mean_traj,
    int nb_samples,
    double std_dev)
{
  std::vector<HRICS::IocTrajectory> samples(
      nb_samples,
      HRICS::IocTrajectory(mean_traj.parameters_.size(),
                           mean_traj.parameters_[0].size(),
                           mean_traj.dt_));

  for (int i = 0; i < nb_samples; i++) {
    // samples low control cost trajetories (rows = nb_dofs, cols = nb_vars)
    Eigen::MatrixXd noisy_traj = sampler.sample(std_dev);
    for (int d = 0; d < noisy_traj.rows(); ++d) {
      samples[i].noise_[d] = noisy_traj.row(d);
      samples[i].parameters_[d] =
          mean_traj.parameters_[d] + samples[i].noise_[d];
    }
  }

  return samples;
}

//------------------------------------------------------------------------------
// Project samples to the joint limits

bool project_samples_to_joint_limits(
    std::vector<std::vector<HRICS::IocTrajectory> >& samples,
    const Move3D::ChompPlanningGroup& planning_group,
    int nb_dofs,
    int num_time_steps,
    double duration)
{
  if (samples.empty()) {
    cout << "Error: no samples" << endl;
    return false;
  }

  // set acceleration as the derivative cost
  std::vector<double> derivative_costs;
  derivative_costs.push_back(0);
  derivative_costs.push_back(1);
  derivative_costs.push_back(0);

  double ridge_factor = 0;

  // set noise at the end configuration ...
  stomp_motion_planner::CovariantTrajectoryPolicy policy;
  policy.setPrintDebug(false);
  policy.initialize(num_time_steps,
                    nb_dofs,
                    duration,
                    ridge_factor,
                    derivative_costs,
                    0,
                    &planning_group);

  // Projectors to joint limits
  std::vector<Eigen::MatrixXd> control_costs;
  policy.getControlCosts(control_costs);
  std::vector<Move3D::TrajOptJointLimit> joint_limits_projector;
  if (!CreateJointLimitProjectors(joint_limits_projector,
                                  planning_group,
                                  control_costs,
                                  num_time_steps,
                                  0)) {
    cout << "Error: initializing QP for joint limits" << endl;
    return false;
  }

  for (int d = 0; d < nb_dofs; d++) {
    std::stringstream ss;
    ss.str("");
    ss << std::string(getenv("HOME_MOVE3D")) +
              "/../move3d-launch/launch_files/";
    ss << "control_matrices/dtw_control_costs";
    ss << "_" << std::setw(3) << std::setfill('0') << d << ".csv";

    move3d_save_matrix_to_csv_file(control_costs[d].inverse(), ss.str());

    //    cout << "control_costs[" << d << "] :"
    //         << " rows =" << control_costs[d].rows()
    //         << " cols =" << control_costs[d].cols() << endl;
  }

  bool succeeded = true;
  int nb_of_single_dimentional_trajectories = 0;
  for (size_t i = 0; i < samples.size(); i++) {
    for (int k = 0; k < samples[i].size(); k++) {
      for (int d = 0; d < nb_dofs; d++) {
        double cost =
            joint_limits_projector[d].project(samples[i][k].parameters_[d]);
        nb_of_single_dimentional_trajectories++;
        if (cost == std::numeric_limits<double>::infinity()) {
          cout << "joint limits did not converge" << endl;
          succeeded = false;
        }
      }
    }
  }

  cout << "has projected : " << nb_of_single_dimentional_trajectories << endl;
  return succeeded;
}

//------------------------------------------------------------------------------
// Compute mean of vector
bool is_config_in_dof_limits(Move3D::confPtr_t q,
                             const Move3D::ChompPlanningGroup& planning_group)
{
  bool succeed = true;
  for (size_t d = 0; d < planning_group.chomp_dofs_.size(); d++) {
    const Move3D::ChompDof& chomp_dof = planning_group.chomp_dofs_[d];
    double max = chomp_dof.joint_limit_max_;
    double min = chomp_dof.joint_limit_min_;
    double dof = (*q)[chomp_dof.move3d_dof_index_];
    if (dof < min || dof > max) {
      cout << "dof " << chomp_dof.joint_name_
           << " is out of bounds (val : " << dof << ")" << endl;
      succeed = false;
    }
  }
  return succeed;
}

//------------------------------------------------------------------------------
// Compute mean of vector
double avg(const std::vector<double>& vect)
{
  double value = 0.;
  for (size_t i = 0; i < vect.size(); i++) {
    value += vect[i];
  }
  return (value / double(vect.size()));
}

//------------------------------------------------------------------------------

bool cost_avg(const std::vector<double>& values_joint_centers,
              const std::vector<double>& values_task_space,
              double& prev_joint_centers,
              double& prev_task)
{
  bool success = true;
  double avg_joint_centers = avg(values_joint_centers);
  double avg_task = avg(values_task_space);

  cout << "mean joint centers : \t" << avg_joint_centers << endl;
  cout << "mean task space : \t" << avg_task << endl;

  if (avg_joint_centers > 0 && prev_task > 0) {
    if (avg_joint_centers < prev_joint_centers) {
      cout << "Error (joint center)" << endl;
      success = false;
    }
    if (avg_task < prev_task) {
      cout << "Error (joint task)" << endl;
      success = false;
    }
  }
  prev_joint_centers = avg_joint_centers;
  prev_task = avg_task;

  return success;
}

//------------------------------------------------------------------------------
// Visualize, set this function in one of the test buttons
// of Move3D-studio and use one of the launch files using
// for example launch_collaboration_planning_aterm.sh
void test_visualize_dtw()
{
  std::vector<std::string> files;
  int nb_trajs = 10;
  for (int i = 0; i < nb_trajs; i++) {
    std::stringstream ss;
    ss.str("");
    ss << std::string(getenv("HOME_MOVE3D")) +
              "/../move3d-launch/launch_files/";
    ss << "dtw_test_trajectories/dtw_trajectory";
    ss << "_" << std::setw(3) << std::setfill('0') << i;
    ss << "_" << std::setw(3) << std::setfill('0') << 0 << ".csv";
    files.push_back(ss.str());
  }

  Move3D::Robot* robot =
      Move3D::global_Project->getActiveScene()->getActiveRobot();
  Move3D::SequencesPlanners pool(robot);
  pool.loadTrajsFromFile(files);
  pool.playTrajs();
}

//------------------------------------------------------------------------------
// Test
// checks that trajectories sampled with increasing standard deviations
// have increasing DTW and Euclidean costs
int main(int argc, char* argv[])
{
  double std_dev = 0.00003;
  for (int i = 1; i < argc; i++) {
    printf("\narg%d=%s\n", i, argv[i]);
    std_dev = ::atof(argv[i]);
  }

  std::string filename = "/../assets/Collaboration/TwoHumansUserExp.p3d";
  Move3D::Robot* robot = move3d_start_all_p3d_environment(filename);
  if (robot == NULL) {
    cout << "Error: could not find robot" << endl;
    return EXIT_FAILURE;
  }

  // Set seed
  p3d_init_random_seed(2);

  cout << "robot is : " << robot->getName() << endl;

  // Name of the end effector
  std::string end_effector_name = "rPalm";
  Move3D::Joint* end_effector = robot->getJoint(end_effector_name);
  if (end_effector == NULL) {
    cout << "Error: could not find end effector : " << end_effector_name
         << endl;
    return EXIT_FAILURE;
  }

  // Get Move3D active joints
  std::vector<Move3D::Joint*> move3d_joints;
  move3d_joints.push_back(robot->getJoint("Pelvis"));
  move3d_joints.push_back(robot->getJoint("TorsoX"));
  move3d_joints.push_back(robot->getJoint("TorsoZ"));
  move3d_joints.push_back(robot->getJoint("TorsoY"));
  move3d_joints.push_back(robot->getJoint("rShoulderTransX"));
  move3d_joints.push_back(robot->getJoint("rShoulderTransY"));
  move3d_joints.push_back(robot->getJoint("rShoulderTransZ"));
  move3d_joints.push_back(robot->getJoint("rShoulderY1"));
  move3d_joints.push_back(robot->getJoint("rShoulderX"));
  move3d_joints.push_back(robot->getJoint("rShoulderY2"));
  move3d_joints.push_back(robot->getJoint("rArmTrans"));
  move3d_joints.push_back(robot->getJoint("rElbowZ"));
  move3d_joints.push_back(robot->getJoint("rElbowX"));
  move3d_joints.push_back(robot->getJoint("rElbowY"));
  move3d_joints.push_back(robot->getJoint("rForeArmTrans"));
  move3d_joints.push_back(robot->getJoint("rWristZ"));
  move3d_joints.push_back(robot->getJoint("rWristX"));
  move3d_joints.push_back(robot->getJoint("rWristY"));
  std::vector<int> joints_ids;
  for (size_t i = 0; i < move3d_joints.size(); i++) {
    if (move3d_joints[i] == NULL) {
      cout << "Error: could not find proper move3d joints" << endl;
      return EXIT_FAILURE;
    }
    joints_ids.push_back(move3d_joints[i]->getId());
  }

  // Set dof random limits
  if (!traj_optim_hrics_human_trajectory_biomech_dof_bounds(robot)) {
    cout << "Error: could not set dof bounds" << endl;
    return EXIT_FAILURE;
  }

  Move3D::ChompPlanningGroup planning_group(robot, joints_ids);

  int nb_dofs = planning_group.num_dofs_;
  int num_time_steps = 100;

  // Movement duration
  double duration = 1.;

  // Create trajectory sampler
  HRICS::IocSampler sampler(num_time_steps, nb_dofs, &planning_group);
  // TODO set this in constructor ...
  if (!sampler.Initialize()) {
    cout << "Error: could not initialize sampler" << endl;
    return EXIT_FAILURE;
  }

  // Define a trajectory for sampling
  HRICS::IocTrajectory mean_traj(nb_dofs, num_time_steps, duration / 100.);

  // Set straight line mean trajectory
  Move3D::confPtr_t q_init = robot->getInitPos();
  Move3D::confPtr_t q_goal = robot->getGoalPos();

  if (!is_config_in_dof_limits(q_init, planning_group)) {
    cout << "q_init is not in dof limits" << endl;
    return EXIT_FAILURE;
  }
  if (!is_config_in_dof_limits(q_goal, planning_group)) {
    cout << "q_goal is not in dof limits" << endl;
    return EXIT_FAILURE;
  }
  // q_init->print();
  // q_goal->print();

  for (size_t d = 0; d < planning_group.chomp_dofs_.size(); d++) {
    int dof_index = planning_group.chomp_dofs_[d].move3d_dof_index_;
    mean_traj.parameters_[d][0] = (*q_init)[dof_index];
    mean_traj.parameters_[d][num_time_steps - 1] = (*q_goal)[dof_index];
  }
  mean_traj.setSraightLineTrajectory();
  mean_traj.parameters_ = mean_traj.straight_line_;

  // Sample a vector of trajectories

  double delta = 3. * std_dev / 10.;
  int nb_samples = 100;
  int nb_steps = 10;
  std::vector<std::vector<HRICS::IocTrajectory> > trajs(nb_steps);
  for (size_t i = 0; i < trajs.size(); i++) {
    cout << "sample trajectories with std_dev : " << std_dev << endl;
    trajs[i] = sample_trajectories(sampler, mean_traj, nb_samples, std_dev);
    std_dev += delta;
  }

  const bool project_to_joint_limits = true;
  if (project_to_joint_limits) {
    if (!project_samples_to_joint_limits(
            trajs, planning_group, nb_dofs, num_time_steps, duration)) {
      cout << "Error : could not project samples in joint limits" << endl;
      return EXIT_FAILURE;
    }
  }

  cout << "End Effector : " << end_effector->getName() << endl;
  std::vector<Move3D::Joint*> end_effector_vect;
  end_effector_vect.push_back(end_effector);

  std::vector<Move3D::Joint*> active_joints;
  active_joints.push_back(robot->getJoint("Pelvis"));
  active_joints.push_back(robot->getJoint("TorsoX"));
  active_joints.push_back(robot->getJoint("rShoulderX"));
  active_joints.push_back(robot->getJoint("rElbowZ"));
  active_joints.push_back(robot->getJoint("rWristX"));
  // active_joints.push_back(robot->getJoint("rPalm"));

  Move3D::Trajectory mean = mean_traj.getMove3DTrajectory(&planning_group);

  bool succeed = true;
  double dtw_prev_joint_centers = 0;
  double dtw_prev_task = 0;
  double euc_prev_joint_centers = 0;
  double euc_prev_task = 0;

  for (size_t i = 0; i < trajs.size(); i++) {

    std::vector<Move3D::Trajectory> move3d_trajs;
    for (int k = 0; k < nb_samples; k++) {
      move3d_trajs.push_back(trajs[i][k].getMove3DTrajectory(&planning_group));

      if (mean.size() != move3d_trajs.back().size()) {
        cout << "Error: sizes of samples and mean and differ" << endl;
        cout << "   - mean.size() : " << mean.size() << endl;
        cout << "   - move3d_trajs.back().size() : "
             << move3d_trajs.back().size() << endl;
        return EXIT_FAILURE;
      }

      std::stringstream ss;
      ss.str("");
      ss << std::string(getenv("HOME_MOVE3D")) +
                "/../move3d-launch/launch_files/";
      ss << "dtw_test_trajectories/dtw_trajectory";
      ss << "_" << std::setw(3) << std::setfill('0') << i;
      ss << "_" << std::setw(3) << std::setfill('0') << k << ".csv";

      // Save to visualize with move3d-studio
      const Move3D::Trajectory& last_trajectory = move3d_trajs.back();
      last_trajectory.saveToFile(ss.str());
    }

    // DTW
    cout << "Compute dtw for sample set " << i << " in joint limits" << endl;
    std::vector<double> dtw_values_joint_centers = dtw_compare_performance(
        &planning_group, mean, move3d_trajs, active_joints);
    std::vector<double> dtw_values_task_space = dtw_compare_performance(
        &planning_group, mean, move3d_trajs, end_effector_vect);
    if (!cost_avg(dtw_values_joint_centers,
                  dtw_values_task_space,
                  dtw_prev_joint_centers,
                  dtw_prev_task)) {
      cout << "Error for DTW" << endl;
      succeed = false;
    }

    // EUCLIDEAN
    cout << "Compute euclidean for sample set " << i << " in joint limits"
         << endl;
    std::vector<double> euc_values_joint_centers = euc_compare_performance(
        &planning_group, mean, move3d_trajs, active_joints);
    std::vector<double> euc_values_task_space = euc_compare_performance(
        &planning_group, mean, move3d_trajs, end_effector_vect);
    if (!cost_avg(euc_values_joint_centers,
                  euc_values_task_space,
                  euc_prev_joint_centers,
                  euc_prev_task)) {
      cout << "Error for Euclidean" << endl;
      succeed = false;
    }

    if (succeed) {
      cout << "Succeeded !!! " << endl;
    } else {
      cout << "Not succeeded !!! " << endl;
    }
  }

  cout << "End test" << endl;
  return EXIT_SUCCESS;
}

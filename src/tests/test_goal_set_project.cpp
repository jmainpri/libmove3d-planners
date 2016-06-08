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
#include "planner/TrajectoryOptim/vector_trajectory.hpp"
#include "planner/TrajectoryOptim/goal_set_projection.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"
#include "planner/TrajectoryOptim/jointlimits.hpp"
#include "planner/planEnvironment.hpp"
#include "human_trajectories/HRICS_ioc.hpp"
#include "utils/misc_functions.hpp"
#include "API/project.hpp"

using std::cout;
using std::endl;

//------------------------------------------------------------------------------
// Test

int main(int argc, char* argv[])
{
  std::string filename = "/../assets/CostDistanceKCD/3dof/manipulator.p3d";
  Move3D::Robot* robot = move3d_start_all_p3d_environment(filename);
  if (robot == NULL) {
    return EXIT_FAILURE;
  }

  cout << "get active joints for robot : " << robot->getName() << endl;

  std::vector<int> active_joints = robot->getActiveJointsIds();

  stomp_motion_planner::CovariantTrajectoryPolicy policy;
  policy.setPrintDebug(false);

  Move3D::ChompPlanningGroup* planning_group =
      new Move3D::ChompPlanningGroup(robot, active_joints);

  int nb_dofs = planning_group->num_dofs_;
  int num_time_steps = 100;

  // set acceleration as the derivative cost
  std::vector<double> derivative_costs;
  derivative_costs.push_back(0);
  derivative_costs.push_back(1);
  derivative_costs.push_back(0);

  double ridge_factor = 0;

  // set noise at the end configuration ...
  policy.initialize(num_time_steps,
                    nb_dofs,
                    0.0,
                    ridge_factor,
                    derivative_costs,
                    2.7,
                    planning_group);

  std::vector<Eigen::MatrixXd> control_cost;
  policy.getControlCosts(control_cost);

  cout << "active joints size() : " << active_joints.size() << endl;
  cout << "nb_dofs : " << nb_dofs << endl;
  PlanEnv->setString(PlanParam::end_effector_joint, "J4");

  cout << "1) Goal set projector" << endl;
  TrajOptGoalSet projector;
  if (!projector.Initialize(planning_group, control_cost)) {
    cout << "Error : could not initilize projector" << endl;
    return EXIT_FAILURE;
  }

  cout << "2) Initialize sampler" << endl;
  double noise_stddev = .0002;  // .02 for 10 way points // .0002 for 100
  HRICS::IocSamplerGoalSet sampler(num_time_steps, nb_dofs, planning_group);
  sampler.Initialize();

  cout << "3) sampler" << endl;
  Eigen::MatrixXd noise = sampler.sample(noise_stddev);

  cout << "4) noisy traj" << endl;
  HRICS::IocTrajectory traj(nb_dofs, num_time_steps, 0.0);

  // set straight line
  Move3D::confPtr_t q_init = robot->getInitPos();
  Move3D::confPtr_t q_goal = robot->getGoalPos();
  for (size_t j = 0; j < planning_group->chomp_dofs_.size(); j++) {
    int dof_index = planning_group->chomp_dofs_[j].move3d_dof_index_;
    traj.parameters_[j][0] = (*q_init)[dof_index];
    traj.parameters_[j][num_time_steps - 1] = (*q_goal)[dof_index];
  }
  traj.setSraightLineTrajectory();

  // set parameters of the trajectory
  for (int j = 0; j < nb_dofs; ++j) {
    cout << "noise(" << j << ") : " << noise.row(j) << endl;
    cout << "line(" << j << ") : " << traj.straight_line_[j].transpose()
         << endl;
    traj.noise_[j] = noise.row(j);
    traj.parameters_[j] = traj.straight_line_[j] + traj.noise_[j];
  }

  cout << "5) project to goal set" << endl;
  robot->setAndUpdate(*q_goal);
  Eigen::Vector3d x_goal = projector.end_effector()->getVectorPos();
  projector.set_print_convergence(true);
  projector.ProjectGoalSetProblem(x_goal, traj.parameters_);

  cout << "End test" << endl;
  return EXIT_SUCCESS;
}

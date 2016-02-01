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

#include "utils/misc_functions.hpp"
#include "API/project.hpp"

#include "planEnvironment.hpp"
#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"

#include "feature_space/features.hpp"
#include "feature_space/smoothness.hpp"

#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Collision-pkg.h>

using std::cout;
using std::endl;

bool test_feature_stack() {
  Move3D::Robot* robot =
      Move3D::global_Project->getActiveScene()->getActiveRobot();
  cout << "get active joints for robot : " << robot->getName() << endl;

  std::vector<int> active_joints = robot->getActiveJointsIds();
  std::vector<int> active_dofs = robot->getActiveDoFsFromJoints(active_joints);

  for (size_t i = 0; i < active_dofs.size(); i++) {
    cout << "active_dofs[" << i << "] : " << active_dofs[i] << endl;
  }

  Move3D::Joint* eef = robot->getJoint(6);
  cout << "joint name : " << eef->getName() << endl;

  // Create feature functions
  Move3D::TaskSmoothnessFeature task_smoothness(robot, eef);
  Move3D::LengthFeature length;
  Move3D::VelocitySmoothness velocity;
  Move3D::AccelerationSmoothness acceleration;
  Move3D::JerkSmoothness jerk;

  // Set the active dofs
  task_smoothness.setActiveDoFs(active_dofs);
  length.setActiveDoFs(active_dofs);
  velocity.setActiveDoFs(active_dofs);
  acceleration.setActiveDoFs(active_dofs);
  jerk.setActiveDoFs(active_dofs);

  // Add the features to the stack
  Move3D::StackedFeatures feature_stack;
  feature_stack.addFeatureFunction(&length);
  feature_stack.addFeatureFunction(&velocity);
  feature_stack.addFeatureFunction(&acceleration);
  feature_stack.addFeatureFunction(&jerk);

  // Set all features in the stack active
  feature_stack.setAllFeaturesActive();

  cout << "stacked size : " << feature_stack.getWeights().size() << endl;
  if (feature_stack.getWeights().size() != 4) {
    return false;
  }

  Move3D::confPtr_t q_curr = robot->getCurrentPos();
  Move3D::confPtr_t q_init = robot->getInitPos();
  Move3D::confPtr_t q_goal = robot->getGoalPos();

  Move3D::Trajectory T(robot);
  T.push_back(q_init);
  T.push_back(q_goal);
  T.cutTrajInSmallLPSimple(20);  // TODO see why it fails without this line

  cout << "test traj" << endl;

  cout << "traj task smoothness : " << task_smoothness.getFeatureCount(T)
       << endl;
  cout << "traj task smoothness : " << task_smoothness.getWeights().size()
       << endl;
  cout << "traj task smoothness : " << task_smoothness.getFeatureCount(T).size()
       << endl;
  cout << "traj task smoothness : "
       << task_smoothness.getFeatures(*q_curr).size() << endl;

  if (feature_stack.getFeatures(*q_curr).size() !=
      feature_stack.getFeatureCount(T).size()) {
    cout << "error" << endl;
    return false;
  }

  cout << "test dimension" << endl;
  if (feature_stack.getWeights().size() != 4) {
    cout << "error" << endl;
    return false;
  }

  feature_stack.addFeatureFunction(&task_smoothness);

  if (feature_stack.getWeights().size() != 4) {
    cout << "1 : stacked size : " << feature_stack.getWeights().size() << endl;
    cout << "error" << endl;
    return false;
  }

  // Set all features in the stack active
  feature_stack.setAllFeaturesActive();
  feature_stack.printInfo();

  if (feature_stack.getWeights().size() != 8) {
    cout << "4 : stacked size : " << feature_stack.getWeights().size() << endl;
    cout << "error" << endl;
    return false;
  }

  // Can only test trajectory because smoothness does not make sense for
  // configuration cost only
  if (feature_stack.getWeights().size() !=
      feature_stack.getFeatureCount(T).size()) {
    cout << "5 : " << feature_stack.getWeights().size() << endl;
    cout << "5 : " << feature_stack.getFeatureCount(T).size() << endl;
    cout << "error" << endl;
    return false;
  }

  if (feature_stack.getWeights().size() !=
      feature_stack.getFeatures(*q_curr).size()) {
    cout << "6 : " << feature_stack.getWeights().size() << endl;
    cout << "6 : " << feature_stack.getFeatures(*q_curr).size() << endl;
    cout << "error" << endl;
    return false;
  }

  std::vector<std::string> features;
  features.push_back("TaskSmoothness");
  feature_stack.setActiveFeatures(features);
  feature_stack.printInfo();

  if (feature_stack.getWeights().size() != 4) {
    cout << "7 : stacked size : " << feature_stack.getWeights().size() << endl;
    cout << "error" << endl;
    return false;
  }

  if (feature_stack.getWeights().size() !=
      feature_stack.getFeatureCount(T).size()) {
    cout << "8 : " << feature_stack.getWeights().size() << endl;
    cout << "8 : " << feature_stack.getFeatureCount(T).size() << endl;
    cout << "error" << endl;
    return false;
  }

  if (feature_stack.getWeights().size() !=
      feature_stack.getFeatures(*q_curr).size()) {
    cout << "9 : " << feature_stack.getWeights().size() << endl;
    cout << "9 : " << feature_stack.getFeatures(*q_curr).size() << endl;
    cout << "error" << endl;
    return false;
  }

  cout << "end feature stack test function" << endl;
  return true;
}

int main(int argc, char* argv[]) {
  if (!move3d_start_and_load_manipulator()) {
    return EXIT_FAILURE;
  }

  std::string robot_name;
  robot_name = XYZ_ENV->robot[0]->name;
  cout << "Robot name is : " << robot_name << endl;

  // Make c++ project interface
  Move3D::global_Project = new Move3D::Project(new Move3D::Scene(XYZ_ENV));
  Move3D::global_Project->getActiveScene()->setActiveRobot(robot_name);

  // GET QT PARAMETERS (used to be in project)
  initPlannerParameters();
  initGestureParameters();
  initHricsParameters();

  bool succeeded = true;
  if (!test_feature_stack()) {
    cout << "feature stack testing did not succeed" << endl;
    succeeded = false;
  }
  if (!succeeded) {
    return EXIT_FAILURE;
  } else {
    return EXIT_SUCCESS;
  }
}

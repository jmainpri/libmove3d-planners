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
#include "API/project.hpp"
#include "utils/misc_functions.hpp"

#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;

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
  Move3D::Robot* robot =
      Move3D::global_Project->getActiveScene()->getActiveRobot();

  Move3D::Trajectory traj(robot);

  bool with_time = false;
  if (with_time) {
    traj.setUseConstantTime(true);
    traj.setUseTimeParameter(true);
    traj.setDeltaTime(.1);
  }
  traj.push_back(robot->getInitPos());
  traj.push_back(robot->getGoalPos());

  std::string filename = "manip_test.traj";

  traj.cutTrajInSmallLPSimple(10);
  traj.saveToFile(filename);

  Eigen::MatrixXd mat1 = traj.getEigenMatrix();
  cout << "traj1 : " << mat1.rows() << " , " << mat1.cols() << endl;
  cout << mat1 << endl;

  traj.clear();
  traj.loadFromFile(filename);

  Eigen::MatrixXd mat2 = traj.getEigenMatrix();
  cout << "traj2 : " << mat2.rows() << " , " << mat2.cols() << endl;
  cout << mat2 << endl;

  double threshold = 1e-5;
  if( mat1.isApprox( mat2, threshold ) ){
    cout << "Test ok! isApprox(" << threshold << ")" << endl;
  }else {
    cout << "Test NOT ok! isApprox(" << threshold << ")" << endl;
  }

  return EXIT_SUCCESS;
}

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

#include "ik_generator.hpp"
#include "hri_costspace/HRICS_costspace.hpp"
#include "API/project.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

IKGenerator::IKGenerator(Robot* robot) : GeneralIK(robot), draw_(false) {}

Move3D::confPtr_t IKGenerator::sample(Move3D::confPtr_t q,
                                      double variance_factor) {
  double jmin, jmax;

  Move3D::confPtr_t q_tmp = q->copy();

  for (size_t j = 0; j < active_joints_.size(); j++) {
    Move3D::Joint* jntPt = active_joints_[j];

    for (size_t i = 0; i < jntPt->getNumberOfDof(); i++) {
      int k = jntPt->getIndexOfFirstDof() + i;

      if (jntPt->isJointDofUser(i)) {
        jntPt->getDofRandBounds(i, jmin, jmax);

        if (std::fabs(jmax - jmin) > 1e-6) {
          double alpha = 1.0;
          double var = (std::fabs(jmax - jmin) / 2) * variance_factor;
          do {
            (*q_tmp)[k] = p3d_gaussian_random2((*q)[k], alpha * var);
            //                        cout << "(*q_tmp)[" << k << "] = " <<
            //                        (*q_tmp)[k] << " , jmax : "  << jmax << "
            //                        , jmin : " << jmin << endl;
            //                        alpha *= 0.9;
          } while ((jmin >= (*q_tmp)[k]) || ((*q_tmp)[k] >= jmax));
        }
      }
    }
  }

  return q_tmp;
}

bool IKGenerator::generate(const Eigen::VectorXd& xdes) {
  int nb_iterations = 100;
  int nb_samples = 10;
  double temperature = 0.3;
  double alpha = temperature / (nb_iterations + 1);
  std::vector<std::pair<double, Move3D::confPtr_t> > configs;
  std::vector<std::pair<double, Move3D::confPtr_t> > best;
  best.push_back(std::make_pair(std::numeric_limits<double>::max(),
                                robot_->getCurrentPos()));

  for (int i = 0; i < nb_iterations; i++) {
    cout << "temp : " << temperature << endl;

    configs.clear();

    for (int j = 0; j < nb_samples; j++) {
      Move3D::confPtr_t q_tmp =
          sample(best[j < int(best.size()) ? j : 0].second, temperature);

      robot_->setAndUpdate(*q_tmp);

      // q_tmp->print();

      // cout << "SOLVE IK" << endl;
      if (solve(xdes)) {
        q_tmp = robot_->getCurrentPos();
        q_tmp->adaptCircularJointsLimits();
        robot_->setAndUpdate(*q_tmp);
        configs.push_back(std::make_pair(q_tmp->cost(), q_tmp));

        if (draw_) {
          if (HRICS_activeNatu){
            HRICS_activeNatu->setRobotColorFromConfiguration(true);
          }
          g3d_draw_allwin_active();
        }

        cout.precision(3);

        // cout << std::scientific << "cost : " <<
        // configs.back().first << " iter : ( " << i << " , " <<
        //  j << " )" <<  endl;
      } else {
        // cout << std::scientific << "no ik iter : ( " << i << "
        //  , " << j << " )" <<  endl;
      }
    }

    if (!configs.empty()) {
      std::sort(configs.begin(), configs.end());

      // robot_->setAndUpdate(*configs[0].second);
      // HRICS_activeNatu->setRobotColorFromConfiguration( true );
      // g3d_draw_allwin_active();

      if (configs[0].first < best[0].first) {
        cout << "improvement at : " << i << ", cost " << configs[0].first
             << endl;
      }

      best.push_back(configs[0]);
      std::sort(best.begin(), best.end());
      if (best.size() > size_t(nb_samples)) best.resize(nb_samples);
    }
    temperature -= alpha;
  }

  cout << "END" << endl;

  if (best[0].first != std::numeric_limits<double>::max()) {
    robot_->setAndUpdate(*best[0].second);
    HRICS_activeNatu->setRobotColorFromConfiguration(true);
    g3d_draw_allwin_active();
    return true;
  }

  return false;
}

bool test_ik_generator() {
  /// IK
  Move3D::Robot* object =
      global_Project->getActiveScene()->getRobotByNameContaining("VISBALL");
  if (object == NULL) return false;

  ////    HRICS_activeNatu->computeIsReachableAndMove(
  /// object->getJoint(1)->getVectorPos(), false );

  Move3D::Robot* robot =
      global_Project->getActiveScene()->getRobotByName("BIOMECH_HUMAN");
  if (!robot) {
    cout << "no human" << endl;
    robot =
        global_Project->getActiveScene()->getRobotByName("manip_3dofs_ROBOT");
    if (!robot) {
      cout << "no robot" << endl;
      return false;
    }
  }

  Move3D::Joint* eef = NULL;
  if (HRICS_activeNatu != NULL) {
    eef = robot->getJoint("rPalm");
    if (eef == NULL) return false;
  } else {
    eef = robot->getJoint("J4");
    if (eef == NULL) return false;
  }

  std::vector<Move3D::Joint*> active_joints;
  //    active_joints.push_back( robot->getJoint( "Pelvis" ) );
  active_joints.push_back(robot->getJoint("TorsoX"));
  active_joints.push_back(robot->getJoint("TorsoZ"));
  active_joints.push_back(robot->getJoint("TorsoY"));
  active_joints.push_back(robot->getJoint("rShoulderTransX"));
  active_joints.push_back(robot->getJoint("rShoulderTransY"));
  active_joints.push_back(robot->getJoint("rShoulderTransZ"));
  active_joints.push_back(robot->getJoint("rShoulderY1"));
  active_joints.push_back(robot->getJoint("rShoulderX"));
  active_joints.push_back(robot->getJoint("rShoulderY2"));
  active_joints.push_back(robot->getJoint("rArmTrans"));
  active_joints.push_back(robot->getJoint("rElbowZ"));
  active_joints.push_back(robot->getJoint("rElbowX"));
  active_joints.push_back(robot->getJoint("rElbowY"));
  active_joints.push_back(robot->getJoint("lPoint"));
  active_joints.push_back(robot->getJoint("rWristZ"));
  active_joints.push_back(robot->getJoint("rWristX"));
  active_joints.push_back(robot->getJoint("rWristY"));

  p3d_jnt_set_dof_rand_bounds(
      robot->getJoint("rShoulderTransX")->getP3dJointStruct(), 0, .016, .020);
  p3d_jnt_set_dof_rand_bounds(
      robot->getJoint("rShoulderTransY")->getP3dJointStruct(), 0, .32, .34);
  p3d_jnt_set_dof_rand_bounds(
      robot->getJoint("rShoulderTransZ")->getP3dJointStruct(), 0, .24, .26);
  p3d_jnt_set_dof_rand_bounds(
      robot->getJoint("rArmTrans")->getP3dJointStruct(), 0, .38, .40);
  p3d_jnt_set_dof_rand_bounds(
      robot->getJoint("lPoint")->getP3dJointStruct(), 0, .23, .25);

  Eigen::VectorXd xdes = object->getJoint(1)->getXYZPose();
  //    Eigen::VectorXd xdes = object->getJoint(1)->getVectorPos();

  Move3D::confPtr_t q_tmp = robot->getCurrentPos();
  //    robot->setAndUpdate( *HRICS_activeNatu->getComfortPosture() );
  //    q_tmp = HRICS_activeNatu->getComfortPosture()->copy();

  bool succeed = false;

  Move3D::IKGenerator ik(robot);
  ik.set_drawing(false);
  ik.initialize(active_joints, eef);

  succeed = ik.generate(xdes);
  cout << "diff = " << (xdes - eef->getXYZPose()).norm() << endl;

  if (!succeed) {
    robot->setAndUpdate(*q_tmp);
  }

  return true;
}

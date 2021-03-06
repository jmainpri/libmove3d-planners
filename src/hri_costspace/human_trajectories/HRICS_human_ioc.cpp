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
#include "HRICS_human_ioc.hpp"
#include "HRICS_human_simulator.hpp"
#include "HRICS_play_motion.hpp"
#include "HRICS_parameters.hpp"

#include "feature_space/features.hpp"

#include "API/project.hpp"

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

static std::string move3d_root =
    std::string(getenv("HOME_MOVE3D")) + std::string("/../");
static std::string move3d_demo_folder(move3d_root + "assets/IOC/TRAJECTORIES/");
static std::string move3d_traj_folder(
    move3d_root + "move3d-launch/matlab/stomp_trajs_home/per_feature_square/");
static std::string move3d_tmp_data_folder("matlab/move3d_tmp_data_home/");

void HRICS_run_human_ioc_from_recorded_motion() {
  std::string foldername =
      move3d_root +
      "libmove3d/statFiles/collaboration/recorded_motion_01_09_13";

  Scene* sce = global_Project->getActiveScene();
  Robot* human1 = sce->getRobotByName("HERAKLES_HUMAN1");
  Robot* human2 = sce->getRobotByName("HERAKLES_HUMAN2");
  if (human1 == NULL || human2 == NULL) {
    cout << "No humans HERAKLES in the the scene" << endl;
    return;
  }

  global_motionRecorders.push_back(new HRICS::RecordMotion(human1));
  global_motionRecorders.push_back(new HRICS::RecordMotion(human2));

  global_motionRecorders[0]->loadCSVFolder(foldername + "/human0", false, +0.5);
  global_motionRecorders[1]->loadCSVFolder(foldername + "/human1", false, -0.5);

  HRICS::PlayMotion player(global_motionRecorders);
  //    for(int
  //    i=0;i<int(global_motionRecorders[0]->getStoredMotions().size());i++)
  for (int i = 0; i < int(1); i++) {
    player.play(i);
  }

  int nb_demos = 30;
  int nb_samples = 1000;
  int nb_way_points = HriEnv->getInt(HricsParam::ioc_nb_of_way_points);

  MultiplePlanners planners(human2);

  // feature_matrix_name_ = "matlab/features.txt";
  Move3D::StackedFeatures* feature_fct =
      new HRICS::HumanTrajFeatures(human2, human1);

  std::vector<int> active_joints;

  HumanIoc ioc(human2,
               human1,
               nb_demos,
               nb_samples,
               nb_way_points,
               planners,
               feature_fct,
               active_joints,
               move3d_demo_folder,
               move3d_traj_folder,
               move3d_tmp_data_folder);

  ioc.setDemos(global_motionRecorders[0]->getStoredMotions());
  ioc.runLearning();
}

void HRICS_run_human_ioc_evaluation() {
  Move3D::Scene* sce = global_Project->getActiveScene();
  Move3D::Robot* human1 = sce->getRobotByName("HERAKLES_HUMAN1");
  Move3D::Robot* human2 = sce->getRobotByName("HERAKLES_HUMAN2");
  if (human1 == NULL || human2 == NULL) {
    cout << "No humans HERAKLES in the the scene" << endl;
    return;
  }

  int nb_demos = 30;
  int nb_samples = 1000;
  int nb_way_points = HriEnv->getInt(HricsParam::ioc_nb_of_way_points);

  MultiplePlanners planners(human2);

  // feature_matrix_name_ = "matlab/features.txt";
  Move3D::StackedFeatures* feature_fct =
      new HRICS::HumanTrajFeatures(human2, human1);

  std::vector<int> active_joints;

  HumanIoc ioc(human2,
               human1,
               nb_demos,
               nb_samples,
               nb_way_points,
               planners,
               feature_fct,
               active_joints,
               move3d_demo_folder,
               move3d_traj_folder,
               move3d_tmp_data_folder);

  ioc.loadDemonstrations();
  ioc.runLearning();
}

HumanIoc::HumanIoc(Robot* active,
                   Robot* passive,
                   int nb_demos,
                   int nb_samples,
                   int nb_way_points,
                   MultiplePlanners& planners,
                   StackedFeatures* features,
                   std::vector<int> active_joints,
                   std::string folder,
                   std::string traj_folder,
                   std::string tmp_data_folder)
    : IocEvaluation(active,
                    nb_demos,
                    nb_samples,
                    nb_way_points,
                    planners,
                    features,
                    active_joints,
                    folder,
                    traj_folder,
                    tmp_data_folder) {
  //    nb_demos_ = nb_demos;
  //    nb_samples_ = nb_samples; // 1000
  //    nb_way_points_ = 15;
  //    folder_ =
  //    "/home/jmainpri/workspace/move3d/assets/Collaboration/TRAJECTORIES/";

  //    original_vect_ = feature_fct_->getWeights();
  //    nb_weights_ = original_vect_.size();

  // Sets the joint limits
  //    HumanTrajSimulator sim( global_human_traj_features );
  //    sim.init();
  //    sim.getActiveJoints();

  // Set active joints
  //    setPlanningGroup();
}

void HumanIoc::setPlanningGroup() {
  // traj_optim_hrics_human_trajectory_manip_init_joints()
  // Set the planner joints
  //    active_joints_.clear();
  //    active_joints_.push_back( 1 );
  //    active_joints_.push_back( 2 );
  //    active_joints_.push_back( 3 );
  //    active_joints_.push_back( 4 );
  //    active_joints_.push_back( 8 );
  //    active_joints_.push_back( 9 );
  //    active_joints_.push_back( 10 );
  //    active_joints_.push_back( 11 );
  //    active_joints_.push_back( 12 );

  // Set the active joints (links)
  //    active_joints_.clear();
  //    active_joints_.push_back( 1 ); // Pelvis
  //    active_joints_.push_back( 2 ); // TorsoX
  //    active_joints_.push_back( 3 ); // TorsoY
  //    active_joints_.push_back( 4 ); // TorsoZ
  //    active_joints_.push_back( 8 ); // rShoulderX
  //    active_joints_.push_back( 9 ); // rShoulderZ
  //    active_joints_.push_back( 10 ); // rShoulderY
  //    active_joints_.push_back( 11 ); // rArmTrans
  //    active_joints_.push_back( 12 ); // rElbowZ

  //    active_joints_.push_back( 13 );
  //    active_joints_.push_back( 14 );
  //    active_joints_.push_back( 15 );
  //    active_joints_.push_back( 16 );
}

void HumanIoc::setDemos(const std::vector<motion_t>& stored_motions) {
  nb_demos_ = stored_motions.size();
  demos_.clear();
  demos_.resize(nb_demos_);

  for (int i = 0; i < int(demos_.size()); i++) {
    demos_[i] = getTrajectoryFromMotion(stored_motions[i]);
  }
}

Move3D::Trajectory HumanIoc::getTrajectoryFromMotion(const motion_t& m) const {
  Move3D::Trajectory t(robot_);

  for (int i = 0; i < int(m.size()); i++) t.push_back(m[i].second->copy());

  t.cutTrajInSmallLP(nb_way_points_ - 1);

  return t;
}

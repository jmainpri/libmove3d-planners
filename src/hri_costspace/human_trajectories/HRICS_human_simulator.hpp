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

#ifndef HRICS_HUMAN_SIMULATOR_HPP
#define HRICS_HUMAN_SIMULATOR_HPP

//#include "HRICS_human_cost_space.hpp"
#include "HRICS_human_features.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"
#include "feature_space/smoothness.hpp"

#include "API/Device/robot.hpp"

namespace HRICS {

enum test_set {
  icra_paper_sept = 0,
  icra_paper_feb = 1,
  user_study = 2,
  human_robot_experiment = 3,
  gmm_data = 4
};

class HumanTrajFeatures : public Move3D::StackedFeatures {
 public:
  HumanTrajFeatures(Move3D::Robot* active, Move3D::Robot* passive);
  ~HumanTrajFeatures();

  void draw() {}

  //    FeatureVect getFeatureCount(const Move3D::Trajectory& t);
  //    FeatureVect getFeatures(const Configuration& t);

  //! Add a passive trajectory
  void setPassiveTrajectory(const motion_t& traj);

  //! Set the passive configuration
  void setPassiveConfig(const Move3D::Configuration& q);

  Move3D::Robot* getActiveHuman() { return human_active_; }
  Move3D::Robot* getPassiveHuman() { return human_passive_; }

  //! init collision space
  bool initCollisionSpace() { return collision_feat_.init(); }

  Move3D::FeatureVect normalizing_by_sampling();

  void addFeaturesSmoothness();
  void addFeaturesDistance();
  void addFeaturesMusculo();

  void setActiveDoFsAllFeatures();

 private:
  Move3D::Robot* human_active_;
  Move3D::Robot* human_passive_;

  Move3D::Trajectory passive_traj_;
  int nb_way_points_;

  DistanceFeature dist_feat_;
  VisibilityFeature visi_feat_;
  MusculoskeletalFeature musc_feat_;
  ReachabilityFeature reach_feat_;
  LegibilityFeature legib_feat_;
  CollisionFeature collision_feat_;

  Move3D::SmoothnessFeature smoothness_feat_;
  Move3D::LengthFeature length_feat_;
  // Move3D::TrajectorySmoothness smoothness_feat_;

  // Store a pointer to all feature
  // to iterate of all when not active in the base class
  std::vector<Move3D::Feature*> all_features_;

  bool use_bio_models_;
};
}

//! main test function for human planning
bool HRICS_init_human_trajectory_cost();

namespace HRICS {

class HumanTrajSimulator {
 public:
  HumanTrajSimulator(HumanTrajFeatures* features);

  bool init();
  double run();

  //! Returns the active degree of freedom
  Move3D::Robot* getActiveHuman() const { return human_active_; }

  //! Returns the active degree of freedom
  Move3D::Robot* getPassiveHuman() const { return human_passive_; }

  //! Returns active joints
  std::vector<Move3D::Joint*> getActiveJoints() { return active_joints_; }

  //! Returns the active degree of freedom
  std::vector<int> getActiveDofs() const { return active_dofs_; }

  void clearLastSimulationMotions() {
    human_1_simulation_.clear();
    human_2_simulation_.clear();
  }
  std::vector<std::vector<motion_t> > getLastSimulationMotions();
  std::vector<std::vector<motion_t> > getMotions();
  std::vector<Move3D::Trajectory> getDemoTrajectories() const;
  std::vector<Move3D::confPtr_t> getContext() const;

  void setDemonstrations(
      const std::vector<motion_t>& demos);  // { human_2_motions_ = demos; }
  void setDemonstrationId(int demo_id) { id_of_demonstration_ = demo_id; }
  bool setDemonstrationId(std::string split);
  const std::vector<motion_t>& getCurrentMotions() { return human_2_motions_; }
  const std::vector<motion_t>& getDemonstrations() { return human_2_demos_; }
  const std::vector<motion_t>& getDemonstrationsPassive() {
    return human_1_motions_;
  }
  int getNumberOfDemos() { return human_2_motions_.size(); }
  void setPelvisBoundsByUser(bool user_defined) {
    is_pelvis_bound_user_defined_ = user_defined;
  }

  Move3D::Trajectory getCurrentPath() const;
  Move3D::Trajectory getExecutedPath() const;
  motion_t getExecutedTrajectory() const;
  double getCost(const motion_t& traj) const;

  // Human trajectory features
  HumanTrajFeatures* getCostSpace() { return human_traj_features_; }

  void setDrawExecution(bool draw_execution) {
    draw_execute_motion_ = draw_execution;
  }

  const std::vector<std::string>& getMotionsNames() const {
    return motions_demo_names_;
  }
  std::vector<int> getDemoIds() const { return motions_demo_ids_; }

 private:
  // ---------------------------------------------------------------------------

  void loadHumanTrajectory(const motion_t& motion);
  Eigen::Transform3d getHumanPose();
  void setPassiveHumanConfig(Move3D::confPtr_t q);
  void setMatrixCol(Eigen::MatrixXd& matrix, int j, Move3D::confPtr_t q) const;
  bool updatePassiveMotion();
  bool loadActiveHumanGoalConfig();
  void printCosts() const;
  Move3D::Trajectory setTrajectoryToMainRobot(
      const Move3D::Trajectory& traj) const;
  void runStandardStomp(int iter);
  void execute(const Move3D::Trajectory& path, bool to_end = true);
  void draw();

  // ---------------------------------------------------------------------------

  void setTranslationBounds();
  void setAllDofBounds();
  void updateDofBounds(bool& initialized, Move3D::confPtr_t q_tmp);
  void updateAllDofBounds(bool& initialized, Move3D::confPtr_t q_tmp);
  void setReplanningDemonstrations();
  void setInitAndGoalConfig();
  void setHumanColor(Move3D::Robot* human, int color);
  void setActiveJoints();
  void addCutMotions();

  // ---------------------------------------------------------------------------

  double time_step_;

  double global_discretization_;
  double motion_duration_;
  double current_motion_duration_;
  double current_discretization_;
  double current_time_;
  double time_along_current_path_;
  int current_id_on_path_;

  int id_of_demonstration_;
  motion_t human_passive_motion_;
  motion_t human_active_motion_;
  Eigen::MatrixXd current_human_traj_;
  Move3D::confPtr_t q_init_;
  Move3D::confPtr_t q_goal_;
  motion_t executed_trajectory_;
  Move3D::Trajectory path_;

  std::vector<double> cost_;
  std::vector<double> colors_;
  int best_path_id_;
  int current_frame_;
  int human_passive_increment_;
  int human_active_increments_per_exection_;
  double human_active_step_;
  bool is_scenario_init_;
  bool draw_execute_motion_;
  bool draw_trace_;

  // ---------------------------------------------------------------------------

  HumanTrajFeatures* human_traj_features_;

  Move3D::Robot* human_active_;
  Move3D::Robot* human_passive_;

  bool init_scenario_;

  std::string traj_folder_;

  std::vector<Move3D::Joint*> active_joints_;
  std::vector<int> active_dofs_;

  std::vector<HRICS::RecordMotion*> motion_recorders_;

  // Limited to only active dofs
  std::vector<motion_t> human_1_motions_;
  std::vector<motion_t> human_2_motions_;

  // original recorded motions
  std::vector<motion_t> human_1_demos_;
  std::vector<motion_t> human_2_demos_;

  std::vector<std::string> motions_1_names_;
  std::vector<std::string> motions_2_names_;

  // Last simulation motion
  std::vector<motion_t> human_1_simulation_;
  std::vector<motion_t> human_2_simulation_;

  std::vector<std::string> motions_demo_names_;
  std::vector<int> motions_demo_ids_;

  // ---------------------------------------------------------------------------

  bool set_all_dof_bounds_;
  bool is_pelvis_bound_user_defined_;
  Eigen::VectorXd pelvis_max_;
  Eigen::VectorXd pelvis_min_;
  Eigen::VectorXd shoulder_trans_max_;
  Eigen::VectorXd shoulder_trans_min_;
  double arm_min_;
  double arm_max_;
  double forearm_min_;
  double forearm_max_;
  Move3D::confPtr_t q_max_;
  Move3D::confPtr_t q_min_;

  // ---------------------------------------------------------------------------

  bool goal_set_optimization_;
  Eigen::VectorXd x_task_goal_;

  // ---------------------------------------------------------------------------

  int minimal_demo_size_;
  bool trajectories_cut_;
  bool use_one_traj_;
  int cut_step_;

  bool use_bio_models_;

  bool is_active_agent_human_;
  bool is_passive_agent_human_;

  bool end_simulation_;
};
}

extern HRICS::HumanTrajSimulator* global_human_traj_simulator;

#endif  // HRICS_HUMAN_SIMULATOR_HPP

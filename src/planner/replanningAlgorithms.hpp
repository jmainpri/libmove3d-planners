//
//  replanning.h
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef REPLANNING_HPP
#define REPLANNING_HPP

#include "API/Trajectory/trajectory.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Roadmap/graph.hpp"

#include "hri_costspace/HRICS_Navigation.hpp"

#include "utils/ConfGenerator.h"

#include "P3d-pkg.h"
#include "LightPlanner-pkg.h"

//#include "planner/Diffusion/Variants/Star-RRT.hpp"

#include <string>
#include <tr1/memory>

//! Function to cancel all kinematic constraints
void p3d_deactivate_all_cntrts( Robot* r );

//! Base replanner
//!
class Replanner 
{   
public:
  Replanner(Robot* r);
  ~Replanner();
  
  void setHuman(Robot* hum);
  
  bool getPlanSucceeded() const { return m_planningSucceded; }
  
  bool isPlanning() const { return m_isPlanning; }
  
  void setSwitchData( confPtr_t qSwitch, int switch_id, double s_switch, double t_rep, double lp_avera_length, double initial_step );
  
  confPtr_t getQSwitch() { return m_qSwitch; }
  confPtr_t getQGoal() { return m_qGoal; }
  
  API::Trajectory& getCurrentTraj() { return m_CurrentTraj; }
  
  virtual bool init() = 0;
  virtual void run() = 0;
  
protected:
  bool init_mlp();
  p3d_traj* concat_to_current_traj(const std::vector<p3d_traj*>& trajs);
  std::pair<bool,API::Trajectory> concat_to_current_traj(const API::Trajectory& newPortion);
  
  Robot* m_robot;
  Robot* m_human;
  
  confPtr_t m_qSwitch;
  confPtr_t m_qGoal;
  
  API::Trajectory m_CurrentTraj;
  
  bool m_init;
  
  bool m_useMLP;
  int m_BaseMLP;
  int m_BaseSmMLP;
  int m_HeadMLP;
  int m_UpBodyMLP;
  int m_UpBodySmMLP;
  
  bool m_isPlanning;
  bool m_planningSucceded;
  int m_switch_id;
  int m_idRun;
  double m_t_rep; // 5 seconds for SM
  double m_lp_avera_length;
  double m_initial_step;
  double m_s_switch;
};

//! Simple replanner
//!
class SimpleReplanner : public Replanner
{   
public:
  SimpleReplanner(Robot* r);
  ~SimpleReplanner();
  
  bool init();
  void run();
  
protected:
  void set_active_joints(int set);
  void init_active_joints();
  void init_for_navigation();
  void init_for_manipuation();
  void init_for_mobile_manip();
  bool init_create_straightline();
  
  enum ReplanningType 
  { 
    NAVIGATION = 0, 
    MANIPULATION = 1, 
    MOBILE_MANIP = 2 
  } 
  m_ReplanningType; 
};

//! Handover replanner
//!
class HandoverReplanner : public SimpleReplanner
{   
public:
  HandoverReplanner(Robot* r);
  ~HandoverReplanner();
  
  bool init();
  void run();
  
private:
  std::pair<bool,confPtr_t> newGoalFromIK();
  std::pair<bool,confPtr_t> newGoalFromList();
  bool m_first_run;
  bool m_goal_from_list;
  Robot* m_human;
  ConfGenerator* handoverGenerator_;
};

//! Stomp replanner
//!
class StompReplanner : public SimpleReplanner
{   
public:
  StompReplanner(Robot* r);
  ~StompReplanner();
  
  bool init();
  void run();
  
private:
  bool m_first_run;
};

//! AStar replanner
//!
class AStarReplanner : public SimpleReplanner
{   
public:
  AStarReplanner(Robot* r);
  ~AStarReplanner();
  
  bool init();
  void run();
  
private:
  bool m_first_run;
  Robot* m_human;
  HRICS::Navigation* m_navigation;
};

//! T-RRT replanner
//!
class RRTReplanner : public SimpleReplanner
{   
public:
  RRTReplanner(Robot* r);
  ~RRTReplanner();
  
  bool init();
  void run();
};

//! Softmotion replanner
//!
class SoftmotionReplanner : public Replanner
{   
public:
  SoftmotionReplanner(Robot* r);
  ~SoftmotionReplanner();
  
  bool init();
  void run();
  
private:
  //bool init_manipulationPlanner();
  bool compute_softmotion(MANPIPULATION_TRAJECTORY_CONF_STR &confs, p3d_traj* traj, SM_TRAJ& smTraj);
  bool compute_softmotion(p3d_traj* traj);
  bool generate_new_trajectory(const std::vector<p3d_traj*>& trajs);
  
  ManipulationPlanner* m_manipPlanner;
};



#endif

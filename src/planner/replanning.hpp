//
//  replanning.h
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef REPLANNING_HPP
#define REPLANNING_HPP

#include "P3d-pkg.h"
#include "LightPlanner-pkg.h"

#include "API/Trajectory/trajectory.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Roadmap/graph.hpp"

#include <string>
#include <tr1/memory>

//! Base replanner
//!
class Replanner 
{   
public:
  Replanner(Robot* r);
  ~Replanner();
  
  bool getPlanSucceeded() const { return m_planningSucceded; }
  
  bool isPlanning() const { return m_isPlanning; }
  
  void setSwitchData( confPtr_t qSwitch, int switch_id, double t_rep, double lp_avera_length, double initial_step );
  
  confPtr_t getQSwitch() { return m_qSwitch; }
  confPtr_t getQGoal() { return m_qGoal; }
  
  API::Trajectory& getCurrentTraj() { return m_CurrentTraj; }
  
  virtual bool init() = 0;
  virtual void run() = 0;
  
protected:
  bool init_mlp();
  p3d_traj* concat_to_current_traj(const std::vector<p3d_traj*>& trajs);
  
  Robot* m_robot;
  
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
  
  void set_planner_type(int type);
  void init_planner_type();
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

//! Star replanner
//!
class StarReplanner : public SimpleReplanner
{   
public:
  StarReplanner(Robot* r);
  ~StarReplanner();
  
  bool init();
  void run();
  
private: 
  Graph* m_graph;
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

//! Class for implementing a replanning simulator
//!
class ReplanningSimulator 
{   
public:
  ReplanningSimulator();
  ~ReplanningSimulator();
  
  p3d_rob* getRobot();
  
//  bool init_simple_replanning();
  
  double time_since_last_call(bool& is_first_call, double& t_init);
  void set_multithread_graphical(bool enable);
  
  void store_traj_to_vect(API::Trajectory& traj, double step);
  void store_exploration(const API::Trajectory& traj, double lPrev, double lNext, std::tr1::shared_ptr<Configuration> qNew);
  void store_traj_to_draw(const API::Trajectory& traj, double step);
  
  int execute_simple_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) );
  int execute_softmotion_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) );
  
  void draw();
  
private:
  
  bool init_simulator();
  void init(std::string robotName);
  bool init_find_robot_basename( std::string& robotBaseName );
  void init_deactivate_all_cntrts( p3d_rob* rob );
  bool init_rosim_cntrts_and_collisions();
  
  void optimize_current_traj();
  
  void store_traj_to_vect(SM_TRAJ& smTraj, double current, double step);
  void store_shortcut(const API::Trajectory& traj, double lPrev, double lNext);
  
  void set_executed_traj_to_current(API::Trajectory& traj);
  bool time_switch_and_id(double s, double s_rep, int& id_switch, API::Trajectory& traj, double &s_switch);
  int execute_softmotion_simulation_traj( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) );
  
  //----------------------------------------------------
  //! Local variables
  bool m_init;
  
  Replanner* m_replanner;
  
  int m_switch_id;
  
  Robot* m_robot;
  Robot* m_rosim;
  Robot* m_human;
  
  API::Trajectory m_ExecuteTraj;
  
  bool m_isWritingDisplay;
  bool m_isReadingDisplay;
  
  std::vector< std::vector<double> > m_currentLine;
  std::vector< std::vector<double> > m_lastLine;
  std::vector< std::vector<double> > m_deviateLine;
};

extern ReplanningSimulator* global_rePlanningEnv;

#endif
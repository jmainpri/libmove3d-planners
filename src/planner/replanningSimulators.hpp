//
//  replanningSimulator.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef REPLANNING_SIMULATOR_hpp
#define REPLANNING_SIMULATOR_hpp

#include "replanningAlgorithms.hpp"

namespace Move3D
{

//! Class for implementing a replanning simulator
//!
class ReplanningSimulator 
{   
public:
  ReplanningSimulator();
  ~ReplanningSimulator();
  
  p3d_rob* getRobot();
  
  void set_multithread_graphical(bool enable);
  
  void store_traj_to_vect(Move3D::Trajectory& traj, double step);
  void store_exploration(const Move3D::Trajectory& traj, double lPrev, double lNext, confPtr_t qNew);
  void store_traj_to_draw(const Move3D::Trajectory& traj, double step);
  void store_graph_to_draw(const Graph& graph);
  
  int execute_simple_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) );
  int execute_softmotion_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) );
  
  double time_since_last_call(bool& is_first_call, double& t_init);
  
  void setDrawStep(double step) { m_draw_step = step; }
  void resetTrajectoriesToDraw() {   m_currentLine.clear(); m_deviateLine.clear(); m_lastLine.clear(); }
  void draw();
  
private:
  bool init_simulator();
  void init(std::string robotName);
  bool init_find_robot_basename( std::string& robotBaseName );
  void init_deactivate_all_cntrts( p3d_rob* rob );
  bool init_rosim_cntrts_and_collisions();
  
  void optimize_current_traj();
  
  void store_traj_to_vect(SM_TRAJ& smTraj, double current, double step);
  void store_shortcut(const Move3D::Trajectory& traj, double lPrev, double lNext);
  void store_human_pos();
  
  bool set_executed_traj_to_current(Move3D::Trajectory& traj);
  bool time_switch_and_id(double s, double s_rep, int& id_switch, Move3D::Trajectory& traj, double &s_switch);
  
  //----------------------------------------------------
  //! Local variables
  bool m_init;
  
  Replanner* m_replanner;
  
  int m_switch_id;
  
  Robot* m_robot;
  Robot* m_rosim;
  Robot* m_rocyl;
  Robot* m_human;
  
  confPtr_t m_q_end;
  
  Move3D::Trajectory m_ExecuteTraj;
  
  bool m_isWritingDisplay;
  bool m_isReadingDisplay;
  bool m_draw_final_config;
  
  double m_draw_step;
  
  std::vector< std::vector<double> > m_currentLine;
  std::vector< std::vector<double> > m_lastLine;
  std::vector< std::vector<double> > m_deviateLine;
};

extern ReplanningSimulator* global_rePlanningEnv;
extern Graph* global_rePlanningGraph;

}

#endif

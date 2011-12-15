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

#include "API/Trajectory/trajectory.hpp"

#include <string>

/**
 * Get the replanning robot
 */
p3d_rob* replan_getRobot();
void replan_init(std::string robotName);
void replan_init_for_navigation();
bool replan_init_execution();
void replan_create_straightline();
void replan_optimize_current_traj();
void replan_store_traj_to_vect(API::Trajectory& traj, double step);

/**
 * @ingroup NEW_CPP_MODULE
 * Execute simulation replanning trajectory
 */
int replan_execute_simple_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) );
int replan_execute_simulation_traj( int (*fct)(p3d_rob* robot, p3d_localpath* curLp) );

#endif
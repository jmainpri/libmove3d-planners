//
//  trajectoryOptim.h
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/07/11.
//  Copyright 2011 LAAS-CNRS. All rights reserved.
//

#ifndef TRAJECTORY_OPTIM_HPP_
#define TRAJECTORY_OPTIM_HPP_

#include <vector>

#include "API/Trajectory/trajectory.hpp"

#include "collision_space/CollisionSpace.hpp"

bool traj_optim_init_mlp_cntrts_and_fix_joints();
bool traj_optim_switch_cartesian_mode(bool cartesian);

Move3D::Trajectory traj_optim_create_sraight_line_traj();

bool traj_optim_initStomp();
bool traj_optim_runChomp();
bool traj_optim_runStomp(int runId);
bool traj_optim_runStompNoInit(int runId, const Move3D::Trajectory& traj);
bool traj_optim_runStompNoReset(int runId);

void traj_optim_add_human_to_collision_space(bool add);
void traj_optim_set_use_iteration_limit(bool use);
void traj_optim_set_iteration_limit(int max_iter);
void traj_optim_set_use_extern_trajectory( bool use );
void traj_optim_set_extern_trajectory( const Move3D::Trajectory& traj );
void traj_optim_set_discretization( double discretization );
void traj_optim_set_discretize( bool discretize );

bool traj_optim_initScenario();
std::vector<int> traj_optim_get_planner_joints();
const Move3D::CollisionSpace* traj_optim_get_collision_space();
std::vector<Move3D::CollisionPoint> traj_optim_get_collision_points();

void traj_optim_draw_collision_points();
bool traj_optim_generate_softMotion();
bool traj_optim_generate_pointsOnTraj();

extern std::vector< std::vector <double> > traj_optim_to_plot;
extern std::vector< std::vector <double> > traj_optim_convergence;

#endif

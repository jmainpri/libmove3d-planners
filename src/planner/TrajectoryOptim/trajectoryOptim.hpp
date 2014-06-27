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
#include "collision_space/collision_space.hpp"

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
void traj_optim_set_traj_duration( double duration );

bool traj_optim_initScenario();

void traj_optim_draw_collision_points();

extern std::vector< std::vector <double> > traj_optim_to_plot;
extern std::vector< std::vector <double> > traj_optim_convergence;

#endif

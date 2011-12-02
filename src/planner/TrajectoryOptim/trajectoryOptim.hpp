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

bool traj_optim_runChomp();
bool traj_optim_runStomp();

void traj_optim_draw_collision_points();
bool traj_optim_generate_softMotion();

extern std::vector< std::vector <double> > traj_optim_to_plot;

#endif
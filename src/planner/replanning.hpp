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

/**
 * Get the replanning robot
 */
p3d_rob* replann_getRobot();

/**
 * Initializes the replanning environnment
 */
bool replann_initialize();

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion to replan from a certain via point
 */
p3d_traj* replanning_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint);


/**
 * @ingroup NEW_CPP_MODULE
 * Execute simulation replanning trajectory
 */
int replann_execute_simulation_traj( int (*fct)(p3d_rob* robot, p3d_localpath* curLp) );

#endif
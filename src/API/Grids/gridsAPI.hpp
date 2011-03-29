/*
 *  gridsAPI.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/05/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef GRID_API_H
#define GRID_API_H

#include <Eigen/Core>

#include "API/Grids/BaseGrid.hpp"
#include "API/Grids/TwoDGrid.hpp"
#include "API/Grids/ThreeDGrid.hpp"
#include "API/Grids/ThreeDPoints.hpp"

extern API::BaseGrid* API_activeGrid;
extern API::TwoDGrid* API_activeRobotGrid;
#ifdef CXX_PLANNER
extern Eigen::Vector3d global_DrawnSphere;
#endif
#endif

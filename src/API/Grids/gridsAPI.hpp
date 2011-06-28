/*
 *  gridsAPI.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/05/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef GRID_API_H
#define GRID_API_H

#include <Eigen/Core>

#include "API/Grids/BaseGrid.hpp"
#include "API/Grids/TwoDGrid.hpp"
#include "API/Grids/ThreeDGrid.hpp"
#include "API/Grids/PointCloud.hpp"

extern API::BaseGrid* API_activeGrid;
extern API::TwoDGrid* API_activeRobotGrid;

extern Eigen::Vector3d global_DrawnSphere;

/**
 * Global Vector Of Grids
 */

extern void api_store_new_grid( API::BaseGrid* grid );
extern std::vector<API::BaseGrid*> api_get_all_grids();
extern void (*ext_add_grid_to_ui)( API::BaseGrid* grid );

#endif

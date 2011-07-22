/*
 *  drawModule.cpp
 *  OOMove3D
 *
 *  Created by Jim Mainprice on 18/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "drawModule.hpp"
#include "drawCost.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#include "../p3d/env.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Trajectory/trajectory.hpp"

#ifdef HRI_PLANNER
#include <hri/hri.h>
#endif

// These are function that are called
// From within libmove3d 

void g3d_export_cpp_graph()
{
	//std::cout << "API_activeGraph : " << API_activeGraph << std::endl;
	if (ENV.getBool(Env::drawGraph) && (!ENV.getBool(Env::use_p3d_structures)) && API_activeGraph ) 
	{
		try
		{
			if (API_activeGraph->isGraphChanged()) 
			{
				XYZ_GRAPH = API_activeGraph->exportCppToGraphStruct();
			}
		}
		catch(std::string str)
		{
			std::cout << "Exception in exporting the cpp graph" << std::endl;
			std::cout << str << std::endl;
		}
	}
}

void g3d_draw_cost_features()
{
#ifdef HRI_COSTSPACE
	g3d_draw_costspace();
	g3d_draw_hrics();
#endif
	g3d_draw_grids();
}

void Graphic::initDrawFunctions()
{
  ext_g3d_traj_debug = draw_traj_debug;
	ext_g3d_draw_cost_features = (void (*)())(g3d_draw_cost_features);
	ext_g3d_export_cpp_graph = (void (*)())(g3d_export_cpp_graph);
  
#ifdef HRI_PLANNER
  ext_g3d_draw_hri_features = g3d_hri_main;
#endif
  ext_compute_config_cost_along_traj = computeConfigCostOnTraj;
}

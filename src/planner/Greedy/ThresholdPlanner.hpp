/*
 *  ThresholdPlanner.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef THRESHOLD_PLANNER_H_
#define THRESHOLD_PLANNER_H_

#include "API/Trajectory/trajectory.hpp"

#include "planner/Diffusion/TreePlanner.hpp"

namespace Move3D
{

class ThresholdPlanner : public TreePlanner
{
public:
	ThresholdPlanner(Robot* R, Graph* G);
	~ThresholdPlanner();
	
	/**
	 * Initializes the planner
	 */
	unsigned init();
	
	/**
	 * stop Conditions
	 */
	bool checkStopConditions();
	
	/**
	 * Tries to extract a trajectory and erase the old one
	 */ 	
	bool newTrajectoryExtracted(Move3D::Trajectory* trajPt);
	
	/**
	 * Expand One Step
	 */
	int expandOneStep(Node* fromComp, Node* toComp);
	
	/**
	 * Main function of the planner
	 */
	unsigned int run();
	
};

}

#endif

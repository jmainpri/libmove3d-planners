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

#include "TreePlanner.hpp"
#include "API/Trajectory/trajectory.hpp"

class ThresholdPlanner : public TreePlanner
{
public:
	ThresholdPlanner(Robot* R, Graph* G);
	~ThresholdPlanner();
	
	/**
	 * Initializes the planner
	 */
	int init();
	
	/**
	 * stop Conditions
	 */
	bool checkStopConditions();
	
	/**
	 * Tries to extract a trajectory and erase the old one
	 */ 	
	bool newTrajectoryExtracted(API::Trajectory* trajPt);
	
	/**
	 * Expand One Step
	 */
	int expandOneStep(Node* fromComp, Node* toComp);
	
	/**
	 * Main function of the planner
	 */
	unsigned int run();
	
};

#endif
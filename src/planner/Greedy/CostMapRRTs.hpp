/*
 *  CostMapRRTs.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef COSTMAP_PLANNER_HPP_
#define COSTMAP_PLANNER_HPP_

#include "planner/Diffusion/TreePlanner.hpp"

#include "API/Trajectory/trajectory.hpp"

class CostmapPlanner : public TreePlanner
{
public:
	CostmapPlanner(Robot* R, Graph* G);
	~CostmapPlanner();
	
	/**
	 * Initializes the planner
	 */
	int init();
	
	/**
	 * stop Conditions
	 */
	bool checkStopConditions();
	
	/**
	 * Compute the gain of the n last iterations
	 */
	double gainOfLastIterations(unsigned int n);
	
	/**
	 * Tries to extract a trajectory and erase the old one
	 */ 	
	void autumnLeaves();
	
	/**
	 * Prints all edge cost
	 */
	void computeAllEdgeCost();
	
	/**
	 * Delete the high cost part
	 */
	bool deleteHighCostPortion(LocalPath& path, double thresh);
	
	/**
	 * Removes the parts of the trees that are of higher cost
	 */ 
	void deleteHighIntergralOfCost();
	
	/**
	 * Returns the trajectory in the graph and compute cost
	 */
	API::Trajectory* getTrajectoryAndComputeCost();
	
	/**
	 * Expand One Step
	 */
	int expandOneStep(Node* fromComp, Node* toComp);
	
	/**
	 * Main function of the planner
	 */
	unsigned int run();
	
private:
	
	double m_bestPathCost;
	double m_bestSoFar;
	
	bool m_withTimeLimit;
	bool m_withGainLimit;
	
	double m_time;
	
	std::vector<double> m_GainOfIterations;
	
};

#endif

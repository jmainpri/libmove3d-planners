/*
 *  ThresholdPlanner.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "ThresholdPlanner.hpp"
#include "Threshold-RRT.hpp"

#include "API/Roadmap/graph.hpp"
#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

ThresholdPlanner::ThresholdPlanner(Robot* R, Graph* G) : TreePlanner(R,G)
{
	
}

ThresholdPlanner::~ThresholdPlanner()
{
	
}

/**
 * Initializes the planner
 */
unsigned ThresholdPlanner::init()
{
	int added = Planner::init();
	added += Planner::setInit(_Robot->getInitPos());
	added += Planner::setGoal(_Robot->getGoalPos());
	return added;
}

/**
 * Expand One Step
 */
int ThresholdPlanner::expandOneStep(Node* fromComp, Node* toComp)
{
	return 0;
}

/*!
 * Checks out if the plannification
 * problem has reach its goals
 */
bool ThresholdPlanner::checkStopConditions()
{	
	if (!(*_stop_func)())
	{
		p3d_SetStopValue(true);
	}
	
	if (p3d_GetStopValue())
	{
		cout << "Tree expansion cancelled." << endl;
		return (true);
	}
	
	return (false);
}

/**
 * Tries to extract a trajectory and erase the old one
 */ 
bool ThresholdPlanner::newTrajectoryExtracted(Move3D::Trajectory* trajPt)
{
/**	if( trajFound() )
	{
		if (trajPt) 
		{
			delete trajPt;
		}
		
		trajPt = _Graph->extractBestTraj(qi,qf);
		
		if (!traj) 
		{
			cout << "Error: Extracting traj" << endl;
			return false;
		}
		
		traj->replaceP3dTraj(_Robot->getTrajStruct()) ;
		
		if (!ENV.getBool(Env::drawDisabled))
		{
			(*_draw_func)();
		}
	}
	else 
	{
		if (trajPt) 
		{
			traj->replaceP3dTraj(_Robot->getTrajStruct());
			
			if (!ENV.getBool(Env::drawDisabled))
			{
				(*_draw_func)();
			}
		}
		else 
		{
			cout << "No path found" << endl;
		}
		
		return false;
	}
 */
    
    return false;
} 


/**
 * Main function of the planner
 * Sevral RRT are run from with different thresholds
 */
unsigned int ThresholdPlanner::run()
{
	shared_ptr<Configuration> tmp = _Robot->getCurrentPos();
	
	shared_ptr<Configuration> qi = _Start->getConfiguration();
	shared_ptr<Configuration> qf = _Goal->getConfiguration();
	
	//	cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << endl;
	if( !preConditions() )
	{
		cout << "Stoped in Tree planner, pre condition" << endl;
		return 0;
	}
	
	double previousThreshold = ENV.getDouble(Env::costThreshold);
	double Threshold = previousThreshold;
	const double nbIter = 10;
	const double delta = previousThreshold/nbIter;
	
	Move3D::Trajectory* traj = NULL;
	
	while ( (!checkStopConditions()) && (Threshold > 0))
	{
		ThresholdRRT* rrt = new ThresholdRRT(_Robot,_Graph);
		rrt->init();
		rrt->setThreshold(Threshold);
		
		cout << "Threshold = " << rrt->getThreshold() << endl;
		rrt->run();
		
		previousThreshold = Threshold;
		Threshold = previousThreshold - delta;
		
		if ( !newTrajectoryExtracted(traj) ) 
		{
			break;
		}
		
		delete _Graph;
		
		_Graph = new Graph(_Robot);
	}
	if (ENV.getBool(Env::drawExploration))
	{
		(*_draw_func)();
	}
	
	ENV.setBool(Env::isRunning,false);
	_Robot->setAndUpdate(*tmp);
	return _Graph->getNumberOfNodes();
}

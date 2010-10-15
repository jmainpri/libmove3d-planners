/*
 *  ThresholdRRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
/**
 * Run the Threshold Planner
 */
/*
 void qt_runThresholdPlanner()
{
	Robot* rob = global_Project->getActiveScene()->getActiveRobot();
	Graph* graph = new Graph(rob);
	
	ThresholdPlanner* _planner = new ThresholdPlanner(rob,graph);
	
	_planner->init();
	_planner->run();
}
*/

#include "Threshold-RRT.hpp"
#include "Expansion/ThresholdExpansion.hpp"

#include "planEnvironment.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include <iostream>

#include "Planner-pkg.h"

using namespace std;

/** 
 * Constructor from a WorkSpace object
 * @param WS the WorkSpace
 */
ThresholdRRT::ThresholdRRT(Robot* R, Graph* G) : RRT(R,G)
{
	cout << "ThresholdRRT::ThresholdRRT(R,G)" << endl;
}

/** 
 * Destructor
 */
ThresholdRRT::~ThresholdRRT()
{
	
}

/**
 * Initialzation of the plannificator
 * @return the number of node added during the init phase
 */
int ThresholdRRT::init()
{
    int added = TreePlanner::init();
    _expan = new ThresholdExpansion(_Graph);
	dynamic_cast<ThresholdExpansion*>(_expan)->setThreshold(ENV.getDouble(Env::costThreshold));
    setInit(true);
    return added;
}

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the cost space
 *
 * @param: node a node frome the graph
 * @param: compNode is a connected component form the graph
 * @return: true if the node and the componant have been connected.
 */
bool ThresholdRRT::connectNodeToCompco(Node* node, Node* compNode)
{
    int SavedIsMaxDis = FALSE;
    Node* node2(NULL);
	
    SavedIsMaxDis =  PlanEnv->getBool(PlanParam::isMaxDisNeigh);
    //p3d_SetIsMaxDistNeighbor(FALSE);
		PlanEnv->setBool(PlanParam::isMaxDisNeigh,SavedIsMaxDis);
	
    node2 = _Graph->nearestWeightNeighbour(compNode,
                                           node->getConfiguration(),
                                           false,
                                           ENV.getInt(Env::DistConfigChoice));
	
    //p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
		PlanEnv->setBool(PlanParam::isMaxDisNeigh,SavedIsMaxDis);
	
    double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);
	
    LocalPath path(node->getConfiguration(),node2->getConfiguration());
	
    if(!ENV.getBool(Env::costBeforeColl))
    {
        if( path.isValid() )
        {
            if( path.getParamMax() <= _expan->step() )
            {
                int nbCreatedNodes=0;
				
                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "Path Valid Connected" << endl;
                return true;
            }
			
            if( ENV.getBool(Env::costExpandToGoal) &&
			   (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
			   dynamic_cast<ThresholdExpansion*>(_expan)->expandToGoal(
																	   node,
																	   node2->getConfiguration()))
            {
                int nbCreatedNodes=0;
				
                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
                return true;
            }
        }
        return false;
    }
    else
    {
        // Warning no expand to goal
        if( path.getParamMax() <= _expan->step() )
        {
            cout << "path.getMaxParam() <= _expan->step()" << endl;
			
            int nbCreatedNodes=0;
			
            if( path.isValid() )
            {
                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "Path Valid Connected" << endl;
                return true;
            }
            else
            {
                return false;
            }
        }
		
        if( ENV.getBool(Env::costExpandToGoal) &&
		   (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
		   dynamic_cast<ThresholdExpansion*>(_expan)->expandToGoal(
																   node,
																   node2->getConfiguration() ))
        {
            if( path.isValid() )
            {
                int nbCreatedNodes=0;
                _expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
                return true;
            }
            else
            {
                return false;
            }
        }
		return(false);
    }
}

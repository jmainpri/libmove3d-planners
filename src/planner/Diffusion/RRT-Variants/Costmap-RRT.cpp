/*
 *  Costmap-RRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 */

#include "Costmap-RRT.hpp"

#include "Expansion/CostmapExpansion.hpp"

#include "planEnvironment.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include <iostream>

#include "P3d-pkg.h"

using namespace std;

/** 
 * Constructor from a WorkSpace object
 * @param WS the WorkSpace
 */
CostmapRRT::CostmapRRT(Robot* R, Graph* G) : RRT(R,G)
{
	cout << "CostmapRRT::CostmapRRT(R,G)" << endl;
}

/** 
 * Destructor
 */
CostmapRRT::~CostmapRRT()
{
	
}

/**
 * Initialzation of the plannificator
 * @return the number of node added during the init phase
 */
int CostmapRRT::init()
{
	int added = TreePlanner::init();
	_expan = new CostmapExpansion(_Graph);
	setInit(true);
	return added;
}

/**
 * Function called to set the
 * tree variables when connecting to the final node
 */
void CostmapRRT::addFinalNode(Node* node, Node* final)
{
	LocalPath path(node->getConfiguration(),
								 final->getConfiguration());
	
	int nbCreatedNodes = 0;
	
	_expan->addNode(node,path,1.0,final,nbCreatedNodes);
	
	final->sumCost() =	node->sumCost() + path.cost();
	final->parent() =		node;
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
bool CostmapRRT::connectNodeToCompco(Node* node, Node* compNode)
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
	
//	double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);
	
	LocalPath path(node->getConfiguration(),node2->getConfiguration());
	
	if(!ENV.getBool(Env::costBeforeColl))
	{
		if( path.isValid() )
		{
			if( path.getParamMax() <= _expan->step() )
			{
				addFinalNode(node,node2);
				cout << "Path Valid Connected" << endl;
				return true;
			}
			
			if( ENV.getBool(Env::costExpandToGoal) &&
			   //(path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
			   dynamic_cast<CostmapExpansion*>(_expan)->expandToGoal(
																																 node,
																																 node2->getConfiguration()))
			{
				addFinalNode(node,node2);
				
				cout << "attempting connect " << node->getConfiguration()->cost() 
						 << " to " << node2->getConfiguration()->cost() << endl;
				
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
			
			if( path.isValid() )
			{
				addFinalNode(node,node2);
				cout << "Path Valid Connected" << endl;
				
				return true;
			}
			else
			{
				return false;
			}
		}
		
		if( ENV.getBool(Env::costExpandToGoal) &&
		   //(path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
		   dynamic_cast<CostmapExpansion*>(_expan)->expandToGoal(
																														node,
																														node2->getConfiguration() ))
		{
			if( path.isValid() )
			{
				addFinalNode(node,node2);
				
				cout << "attempting connect " << node->getConfiguration()->cost() 
					   << " to " << node2->getConfiguration()->cost() << endl;
				
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

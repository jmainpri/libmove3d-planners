/*
 *  Star-RRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 31/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "Star-RRT.hpp"


#include "Threshold-RRT.hpp"
#include "Expansion/StarExpansion.hpp"

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
StarRRT::StarRRT(Robot* R, Graph* G) : RRT(R,G)
{
	cout << "StarRRT::StarRRT(R,G)" << endl;
}

/** 
 * Destructor
 */
StarRRT::~StarRRT()
{
	
}

/**
 * Initialzation of the plannificator
 * @return the number of node added during the init phase
 */
int StarRRT::init()
{
	int added = TreePlanner::init();
	_expan = new StarExpansion(_Graph);
	setInit(true);
	return added;
}

/**
 *
 */
bool StarRRT::connectNodeToCompco(Node* node, Node* compNode)
{
	bool savedIsMaxDis = PlanEnv->getBool( PlanParam::isMaxDisNeigh );
	PlanEnv->setBool(PlanParam::isMaxDisNeigh,false);
	
	Node* nearestNode = _Graph->nearestWeightNeighbour(compNode,
																				 node->getConfiguration(),
																				 false,
																				 ENV.getInt(Env::DistConfigChoice));
	
	PlanEnv->setBool( PlanParam::isMaxDisNeigh , savedIsMaxDis );
	
	LocalPath path( node->getConfiguration(), nearestNode->getConfiguration() );
	
	if( path.getParamMax() <= _expan->step() )
	{
		//			cout << "path.getParamMax() <= _expan->step()" << endl;
		//			cout << "path.getParamMax() = " << path.getParamMax() << endl;
		//			cout << "_expan->step()     = " << _expan->step() << endl;
		
		if ( path.getParamMax() == 0.0 ) {
			node->print();
			nearestNode->print();
		}
		
		int nbCreatedNodes=0;
		
		if( path.isValid() )
		{
			_expan->addNode(node,path,1.0,nearestNode,nbCreatedNodes);
			cout << "Path Valid Connected" << endl;
			return true;
		}
		else
		{
			return false;
		}
	}
	
	return false;
}


//
// C++ Implementation: rrt
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "RRT.hpp"

#include "API/Grids/PointCloud.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include <iostream>

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

RRT::RRT(Robot* R, Graph* G) :
TreePlanner(R,G)
{
    #ifdef DEBUG_STATUS
        cout << "RRT::RRT(R,G)" << endl;
#endif
}

RRT::~RRT()
{
}

/**
 * Checks out the Stop condition
 */
bool RRT::checkStopConditions()
{
	if(TreePlanner::checkStopConditions())
	{
		return true;
	}
	
	return false;
}

/**
 * Checks out the preconditions
 */
bool RRT::preConditions()
{
	if(TreePlanner::preConditions())
	{
		if (ENV.getBool(Env::expandToGoal))
		{
			if(trajFound())
			{
				cout << "Start And Goal in same component" << endl;
				return true;
			}
			
			if(!ENV.getBool(Env::isCostSpace))
			{
				LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
				if (direct.isValid())
				{
					connectNodeToCompco(_Start,_Goal);
                                        #ifdef DEBUG_STATUS
                                        cout << "Direct connection" << endl;
#endif
					return true;
				}
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Initializes an RRT Planner
 */
int  RRT::init()
{
	int added = TreePlanner::init();
	_expan = new RRTExpansion(_Graph);
	return added;
}

/**
 * Three phases One Step Expansion
 *
 *  - Direction
 *  - Node
 *  - Process
 *
 *  @param fromComp the component which is expanded
 *  @param toComp the goal component
 */
int RRT::expandOneStep(Node* fromComp, Node* toComp)
{
	// Resets the expansion structure
	_expan->setGraph(_Graph);
	_expan->setFromComp(fromComp);
	_expan->setToComp(toComp);
	
	Node* directionNode(NULL);
	Node* expansionNode(NULL);
	confPtr_t directionConfig;
  
	
	// get direction
	directionConfig = _expan->getExpansionDirection(fromComp, 
																									toComp, 
																									false,
																									directionNode);
  
#ifdef LIGHT_PLANNER
	if( ENV.getBool(Env::drawPoints) && PointsToDraw )
	{
		PointsToDraw->push_back(directionConfig->getTaskPos());
	}
#endif
	
	//    cout << "***********************************************************"  << endl;
	//    cout << "directionConfig->print()"  << endl;
	//   directionConfig->print();
	
	// get node for expansion
	expansionNode = _expan->getExpansionNode(fromComp, 
																					 directionConfig,
																					 ENV.getInt(Env::DistConfigChoice));
	
	//    cout << "***********************************************************"  << endl;
	//    cout << "expansionNode->print()"  << endl;
	//    expansionNode->getConfiguration()->print();
	
	// expansion in one direction
	int nbNodeCreated = _expan->expandProcess(expansionNode, 
																						directionConfig, 
																						directionNode,
																						ENV.getExpansionMethod());
	
	if ( nbNodeCreated < 1 ) 
	{
		m_nbFailedExpansion++;
	}
	
	return nbNodeCreated;
	//	}
}


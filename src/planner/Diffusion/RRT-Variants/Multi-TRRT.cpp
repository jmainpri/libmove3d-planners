/*
 *  Multi-TRRT.c
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 09/06/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "Multi-TRRT.hpp"

/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "Expansion/TransitionExpansion.hpp"
#include "Roadmap/node.hpp"
#include "Roadmap/graph.hpp"

#include "planEnvironment.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_Workspace.hpp"
#endif

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

MultiTRRT::MultiTRRT(Robot* R, Graph* G) :
MultiRRT(R,G)
{
    cout << "Multi-Transition-RRT Constructor" << endl;
}

MultiTRRT::~MultiTRRT()
{
	
}

/*!
 * Configuration comparator
 */
class ConfigurationComparator
{	
public:
	
    bool operator()(shared_ptr<Configuration> first, shared_ptr<Configuration> second)
    {
        return ( first->cost() < second->cost() );
    }
	
} ConfigurationComparatorObject;


int MultiTRRT::init()
{
  int added = TreePlanner::init();
	
	m_Roots.push_back( _Start );
	m_Roots.push_back( _Goal );
	
	vector< shared_ptr<Configuration> > lowCostSeeds;
	
	for (int i=0; i<ENV.getInt(Env::nbOfSeeds)*10; i++)
	{
		shared_ptr<Configuration> q = _Robot->shoot();
		
		while (q->isInCollision()) 
		{
			q = _Robot->shoot();
		}
		
		lowCostSeeds.push_back(q);
	}
	
	sort(lowCostSeeds.begin(), lowCostSeeds.end(), ConfigurationComparatorObject);
	
	
	for (int i=0; i<ENV.getInt(Env::nbOfSeeds); i++) 
	{
		addSeed(lowCostSeeds[i]);
	}
	
    _expan = new TransitionExpansion(this->getActivGraph());
	//_expan->setDirectionMethod(NAVIGATION_BEFORE_MANIPULATION);
	
	
	//    p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
	//                           this->getStart()->getNodeStruct(),
	//                           this->getGoal()->getNodeStruct());
	
    this->getInit()->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
    this->getInit()->getNodeStruct()->comp->temperature = ENV.getDouble(Env::initialTemperature);
    this->getInit()->getNodeStruct()->nbFailedTemp = 0;
	
	p3d_SetGlobalNumberOfFail(0);
	
    //  GlobalNbDown = 0;
    //  Ns->NbDown = 0;
    p3d_SetNodeCost(this->getActivGraph()->getGraphStruct(),
					this->getInit()->getNodeStruct(), 
					this->getInit()->getConfiguration()->cost());
	
    p3d_SetCostThreshold(this->getInit()->getNodeStruct()->cost);
	
    p3d_SetInitCostThreshold( 
							 p3d_GetNodeCost(this->getInit()->getNodeStruct()) );
	
	p3d_SetCostThreshold(MAX(
							 p3d_GetNodeCost(this->getInit()->getNodeStruct()), 
							 p3d_GetNodeCost(this->getInit()->getNodeStruct()) ));
	
	for (unsigned int i=0; i<m_Roots.size(); i++) 
	{
		initalizeRoot(m_Roots[i]);
	}
	
    return added;
}

void MultiTRRT::initalizeRoot(Node* rootNode)
{
	rootNode->getNodeStruct()->temp	= ENV.getDouble(Env::initialTemperature);
	rootNode->getNodeStruct()->comp->temperature = ENV.getDouble(Env::initialTemperature);
	rootNode->getNodeStruct()->nbFailedTemp = 0;
	
	p3d_SetNodeCost(_Graph->getGraphStruct(), 
					rootNode->getNodeStruct(), 
					rootNode->getConfiguration()->cost());
	
	//        p3d_SetCostThreshold(MAX(
	//								p3d_GetNodeCost(this->getStart()->getNodeStruct()), 
	//								p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
	
	p3d_SetAverQsQgCost(
						( _Graph->getGraphStruct()->search_start->cost
						 + _Graph->getGraphStruct()->search_goal->cost) / 2.);
}


/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool MultiTRRT::connectNodeToCompco(Node* node, Node* compNode)
{
	if ( node->equalCompco(compNode) ) 
	{
		cout << "Error tries to connect to its own component" << endl;
	}
	
    int SavedIsMaxDis = FALSE;
	
    SavedIsMaxDis =  PlanEnv->getBool(PlanParam::isMaxDisNeigh);
    //p3d_SetIsMaxDistNeighbor(FALSE);
		PlanEnv->setBool(PlanParam::isMaxDisNeigh,false);
	
    Node* node2 = _Graph->nearestWeightNeighbour(compNode,
                                           node->getConfiguration(),
                                           false,
                                           ENV.getInt(Env::DistConfigChoice));
	
    //p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
		PlanEnv->setBool(PlanParam::isMaxDisNeigh,SavedIsMaxDis);
	
    double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);
	
    LocalPath path(node->getConfiguration(),node2->getConfiguration());
	
	if( ENV.getBool(Env::costExpandToGoal) &&
	   (path.getParamMax() <= (minumFinalCostGap*_expan->step())) &&
	   _expan->expandToGoal(
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
	return false;
}

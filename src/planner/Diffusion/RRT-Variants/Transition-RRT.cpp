/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "Transition-RRT.hpp"
#include "Expansion/TransitionExpansion.hpp"
#include "Roadmap/compco.hpp"

#include "planEnvironment.hpp"
#include "cost_space.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_Workspace.hpp"
#endif

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

TransitionRRT::TransitionRRT(Robot* R, Graph* G) :
RRT(R,G)
{
	cout << "TransitionRRT::TransitionRRT" << endl;
}

TransitionRRT::~TransitionRRT()
{
	
}

int TransitionRRT::init()
{
	int added = TreePlanner::init();
	
	_expan = new TransitionExpansion(this->getActivGraph());
	_expan->setDirectionMethod(NAVIGATION_BEFORE_MANIPULATION);
	
	
	//    p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
	//                           this->getStart()->getNodeStruct(),
	//                           this->getGoal()->getNodeStruct());
	
	this->getInit()->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
	this->getInit()->getConnectedComponent()->getCompcoStruct()->temperature = ENV.getDouble(Env::initialTemperature);
	this->getInit()->getNodeStruct()->nbFailedTemp = 0;
	
	p3d_SetGlobalNumberOfFail(0);
	
	//  GlobalNbDown = 0;
	//  Ns->NbDown = 0;
	
	setNodeCost(this->getInit());
	
	//	p3d_SetNodeCost(this->getActivGraph()->getGraphStruct(),
	//					this->getStart()->getNodeStruct(), 
	//					this->getStart()->getConfiguration()->cost());
	
  p3d_SetCostThreshold(this->getInit()->getNodeStruct()->cost);
	
  p3d_SetInitCostThreshold( 
													 p3d_GetNodeCost(this->getInit()->getNodeStruct()) );
	
	if ( ENV.getBool(Env::expandToGoal) && (this->getGoal() != NULL))
	{
		this->getGoal()->getNodeStruct()->temp	= ENV.getDouble(Env::initialTemperature);
		this->getInit()->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
		this->getGoal()->getConnectedComponent()->getCompcoStruct()->temperature = ENV.getDouble(Env::initialTemperature);
		this->getGoal()->getNodeStruct()->nbFailedTemp = 0;
		//    Ng->NbDown = 0;
		
		setNodeCost(this->getGoal());
		
		p3d_SetCostThreshold(MAX(
														 p3d_GetNodeCost(this->getInit()->getNodeStruct()), 
														 p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
		
		//        p3d_SetCostThreshold(MAX(
		//								p3d_GetNodeCost(this->getStart()->getNodeStruct()), 
		//								p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
		
		p3d_SetAverQsQgCost(
												( this->getActivGraph()->getGraphStruct()->search_start->cost
												 + this->getActivGraph()->getGraphStruct()->search_goal->cost) / 2.);
	}
	else
	{
		p3d_SetCostThreshold(this->getInit()->getNodeStruct()->cost);
		p3d_SetInitCostThreshold( this->getInit()->getNodeStruct()->cost );
		p3d_SetAverQsQgCost( this->getActivGraph()->getGraphStruct()->rob->GRAPH->search_start->cost);
	}
	
	return added;
}


void TransitionRRT::setNodeCost(Node* node)
{
	global_costSpace->setNodeCost(node,
																node->parent() );
}

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool TransitionRRT::connectNodeToCompco(Node* node, Node* compNode)
{
	int SavedIsMaxDis = FALSE;
	Node* node2(NULL);
	
	SavedIsMaxDis =  PlanEnv->getBool(PlanParam::isMaxDisNeigh);
//	p3d_SetIsMaxDistNeighbor(FALSE);
	PlanEnv->setBool(PlanParam::isMaxDisNeigh,false);
	
	node2 = _Graph->nearestWeightNeighbour(compNode,
																				 node->getConfiguration(),
																				 false,
																				 ENV.getInt(Env::DistConfigChoice));
	
	PlanEnv->setBool(PlanParam::isMaxDisNeigh,SavedIsMaxDis);
//	p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
	
	double minumFinalCostGap = ENV.getDouble(Env::minimalFinalExpansionGap);
	
	LocalPath path(node->getConfiguration(),
								 node2->getConfiguration());
	
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
				 _expan->expandToGoal(
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
		if( path.getParamMax() <= _expan->step() )
		{
//			cout << "path.getParamMax() <= _expan->step()" << endl;
//			cout << "path.getParamMax() = " << path.getParamMax() << endl;
//			cout << "_expan->step()     = " << _expan->step() << endl;
			
			if ( path.getParamMax() == 0.0 ) {
				node->print();
				node2->print();
			}
			
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
		
		// Warning 
		// The expansion to goal happens if only if the gap is inferior to some
		// multiple of the expansion step
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
		return(false);
	}
}

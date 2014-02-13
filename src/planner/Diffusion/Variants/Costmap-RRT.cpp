/*
 *  Costmap-RRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 */

#include "Costmap-RRT.hpp"

#include "planner/Diffusion/Variants/Costmap-RRT.hpp"

#include "planEnvironment.hpp"

#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include <iostream>

#include "P3d-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

/*!
 * Constructors with default values
 */
CostmapExpansion::CostmapExpansion() : RRTExpansion()
{
	
}

/*!
 * Constructors with default values
 */
CostmapExpansion::CostmapExpansion(Graph* G) : RRTExpansion(G)
{
	
}

/*!
 * Destructor
 */
CostmapExpansion::~CostmapExpansion()
{
	
}

/*!
 * Expand the localpath
 */
bool CostmapExpansion::expandToGoal(Node* expansionNode, shared_ptr<Configuration> directionConfig)
{
	LocalPath extensionLocalpath(expansionNode->getConfiguration(), 
															 directionConfig);
  
	double costOfNode = expansionNode->sumCost() + extensionLocalpath.cost();
	
	if ( costOfNode > m_lastBestPath )
	{
    //		cout << "expansionNode->sumCost = "						<< expansionNode->sumCost() << endl;
    //		cout << "extensionLocalpath.cost() = "				<< extensionLocalpath.cost() << endl;
    //		cout << "Failed because of cost sup to : ( "	<< m_lastBestPath << " ) " << endl;
		return false;
	}
	
	return true;
}

/*!
 * Connect expansion method
 */
int CostmapExpansion::connectExpandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, Node* directionNode)
{
	bool failed(false);
	int nbCreatedNodes(0);
	Node* extensionNode(NULL);
	
	// Perform extension toward directionConfig
	LocalPath directionLocalpath(expansionNode->getConfiguration(), directionConfig);
	
	double delta = this->pathDelta(directionLocalpath);
	
	shared_ptr<Configuration> ptrConf;
	
	double param = directionLocalpath.whenCostIntegralPasses( m_lastBestPath*0.80 );
	
	LocalPath extensionLocalpath(directionLocalpath.getBegin(),
															 directionLocalpath.configAtParam(param));
	
	// Expansion control
	// Discards potential nodes that are to close to the graph
	if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath, *expansionNode))
	{
		//cout << "Failed expandControl test in " << __PRETTY_FUNCTION__ << endl;
		return 0;
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		extensionNode = addNode(expansionNode, 
														extensionLocalpath, 
														delta,directionNode, nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ) )
		{
			// Components were merged
			return 0;
		}
	}
	else
	{
		//cout << "Expansion failed in " << __PRETTY_FUNCTION__ << endl;
		this->expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
}

/*!
 * Extend expansion method
 */
int CostmapExpansion::extendExpandProcess(Node* expansionNode, 
																					MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig,
																					Node* directionNode)
{
	bool failed(false);
	int nbCreatedNodes(0);
	Node* extensionNode(NULL);
	
	// Perform extension toward directionConfig
	LocalPath directionLocalpath(expansionNode->getConfiguration(), directionConfig);
	
	double delta = this->pathDelta(directionLocalpath);
	
	shared_ptr<Configuration> ptrConf;
	
	if( (delta==1)&& directionNode )
	{
		ptrConf = directionNode->getConfiguration();
	}
	else
	{
		ptrConf = directionLocalpath.configAtParam( delta * directionLocalpath.getParamMax() );
	}
  
	
	LocalPath extensionLocalpath(directionLocalpath.getBegin(),ptrConf);
	
	// Expansion control
	// Discards potential nodes that are to close to the graph
	if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,*expansionNode))
	{
		//cout << "Failed expandControl test in " << __PRETTY_FUNCTION__ << endl;
		return 0;
	}
	
	// Transition test and collision check
	if (ENV.getBool(Env::costBeforeColl))
	{
		double costOfNode = expansionNode->sumCost() + extensionLocalpath.cost();
		
		if( costOfNode  > m_lastBestPath )
		{
      //			cout << "expansionNode->sumCost = "						<< expansionNode->sumCost() << endl;
      //			cout << "extensionLocalpath.cost() = "				<< extensionLocalpath.cost() << endl;
      //			cout << "Failed because of cost sup to : ( "	<< m_lastBestPath << " ) " << endl;
			
			failed = true;
		}
		
		if (!failed)
		{
			if (!extensionLocalpath.isValid())
			{
				failed = true;
			}
		}
	}
	else
	{
		if (!extensionLocalpath.isValid())
		{
			failed = true;
		}
		if (!failed)
		{
			double costOfNode = expansionNode->sumCost() + extensionLocalpath.cost();
			
			if( costOfNode > m_lastBestPath )
			{
				failed = true;
			}
		}
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		//cout << "Expansion succeded in " << __PRETTY_FUNCTION__ << endl;
		extensionNode = addNode(expansionNode, 
														extensionLocalpath, 
														delta,directionNode, nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ) )
		{
			// Components were merged
			//cout << "Connected in Transition" << __PRETTY_FUNCTION__ << endl;
			return 0;
		}
	}
	else
	{
		//cout << "Expansion failed in " << __PRETTY_FUNCTION__ << endl;
		this->expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
}

/*!
 * expandProcess
 */
unsigned CostmapExpansion::expandProcess(Node* expansionNode,
                                    shared_ptr<Configuration> directionConfig,
                                    Node* directionNode,
                                    Env::expansionMethod method)
{
	
	switch (method) 
	{
		case Env::Connect:
			
			return connectExpandProcess(expansionNode,
																	directionConfig,
																	directionNode);
			
		case Env::Extend:
			
			return extendExpandProcess(expansionNode,
																 directionConfig,
																 directionNode);
		default:
			cerr << "Error : expand process not implemented" << endl;
			return 0;
	} 
}

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
unsigned CostmapRRT::init()
{
	int added = TreePlanner::init();
	_expan = new CostmapExpansion(_Graph);
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

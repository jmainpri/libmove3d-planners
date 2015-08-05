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
#include "planner/Diffusion/Variants/Threshold-RRT.hpp"

#include "planEnvironment.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/ConfigSpace/localpath.hpp"

#include <iostream>

#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

/*!
 * Constructors with default values
 */
ThresholdExpansion::ThresholdExpansion() : RRTExpansion()
{
	
}

/*!
 * Constructors with default values
 */
ThresholdExpansion::ThresholdExpansion(Graph* G) : RRTExpansion(G)
{
	
}

/*!
 * Destructor
 */
ThresholdExpansion::~ThresholdExpansion()
{
	
}

/*!
 * Expand the localpath
 */
bool ThresholdExpansion::expandToGoal(Node* expansionNode, confPtr_t directionConfig)
{
  LocalPath directionLocalPath(expansionNode->getConfiguration(), directionConfig);
	const double paramMax = directionLocalPath.getParamMax();
	
	double param = 0;
  for ( int i = 1; param < paramMax; i++)
  {
		confPtr_t q;
    param = ((double) i) * step();
		
    if (param > paramMax)
    {
      q = directionConfig;
    }
    else
    {
      q = directionLocalPath.configAtParam(param);
    }
    
		if ( q->cost() > m_threshold )
		{
      return false;
		}
  }
	
  return true;
}

/*!
 * Connect expansion method
 */
int ThresholdExpansion::connectExpandProcess(Node* expansionNode, confPtr_t directionConfig, Node* directionNode)
{
	cout << "ThresholdExpansion::connectExpandProcess Not implemented" << endl;
	return 0;
}

/*!
 * Extend expansion method
 */
int ThresholdExpansion::extendExpandProcess(Node* expansionNode, confPtr_t directionConfig, Node* directionNode)
{
	bool failed(false);
  int nbCreatedNodes(0);
  Node* extensionNode(NULL);
	
	// Perform extension toward directionConfig
  LocalPath directionLocalpath(expansionNode->getConfiguration(), directionConfig);
	
  double pathDelta = directionLocalpath.getParamMax() == 0. ? 1 : MIN(1., step() / directionLocalpath.getParamMax() );
	
  LocalPath extensionLocalpath(directionLocalpath.getBegin(), pathDelta == 1.
                               && directionNode ? directionNode->getConfiguration()
                               : directionLocalpath.configAtParam(pathDelta * directionLocalpath.getParamMax()));
	
  // Expansion control
  // Discards potential nodes that are to close to the graph
  if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,*expansionNode))
  {
		//		cout << "Failed expandControl test in " << __PRETTY_FUNCTION__ << endl;
    return 0;
  }
	
  // Transition test and collision check
  if (ENV.getBool(Env::costBeforeColl))
  {
		if( extensionLocalpath.getEnd()->cost() > m_threshold )
		{
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
			if( extensionLocalpath.getEnd()->cost() > m_threshold )
			{
				failed = true;
			}
		}
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		extensionNode = addNode(expansionNode, 
                            extensionLocalpath, 
                            pathDelta,
                            directionNode, nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ))
		{
			// Components were merged
			cout << "Connected in Transition" << __PRETTY_FUNCTION__ << endl;
			return 0;
		}
	}
	else
	{
		this->expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
	
}

/*!
 * expandProcess
 */
unsigned ThresholdExpansion::expandProcess(Node* expansionNode,
                                      confPtr_t directionConfig,
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
			cerr << "Error : expand process not omplemented" << endl;
			return 0;
	} 
}

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
unsigned ThresholdRRT::init()
{
    int added = TreePlanner::init();
    _expan = new ThresholdExpansion(_Graph);
    dynamic_cast<ThresholdExpansion*>(_expan)->setThreshold(ENV.getDouble(Env::costThreshold));
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

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
#include "planner/Diffusion/Variants/Star-RRT.hpp"

#include "planEnvironment.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include <iostream>

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

const bool print_exploration = false;
const bool print_rewiring = false;

/*!
 * Constructors with default values
 */
StarExpansion::StarExpansion(Graph* G) : RRTExpansion(G)
{
	m_K_Nearest = 10;
  m_RrgRadiusFactor = 1.0;
  m_nb_rewiring = 0;
  m_step = step();
  initCSpace();
}

/*!
 * Destructor
 */
StarExpansion::~StarExpansion()
{
	
}

void StarExpansion::initCSpace()
{
  m_cspace = new CSpaceCostMap2D();
  m_cspace->set_step( m_step );
  m_cspace->set_cost_step( m_step );
  m_cspace->set_robot( m_Graph->getRobot() );
}

double StarExpansion::rrgBallRadius()
{
  if(m_cspace->get_connection_radius_flag())
  {
    double inv_d = 1.0 / m_cspace->dimension();
    double gamma_rrg = 2 * pow(1.0 + inv_d, inv_d) * pow(m_cspace->volume() / m_cspace->unit_sphere(), inv_d);
    double nb_nodes = m_Graph->getNumberOfNodes();
    return(std::min(m_cspace->get_step(), gamma_rrg * pow((log(nb_nodes)/nb_nodes), inv_d)));
  }
  else
  {
    return(m_cspace->get_step());
  }
}

/*!
 * Expand the localpath
 */
bool StarExpansion::expandToGoal(Node* expansionNode, 
																 shared_ptr<Configuration> directionConfig)
{
	return false;
}

/*!
 * Connect expansion method
 */
int StarExpansion::connectExpandProcess(Node* expansionNode, 
																				std::tr1::shared_ptr<Configuration> directionConfig, 
																				Node* directionNode)
{
	cout << "StarExpansion::connectExpandProcess Not implemented" << endl;
	return 0;
}

/*!
 * Extend expansion method
 *
 * The expansion node is the node from the 
 * graph that has been selected by the voronoi bias
 */
int StarExpansion::extendExpandProcess(Node* expansionNode, confPtr_t directionConfig, Node* directionNode)
{
	bool failed(false);
	int nbCreatedNodes(0);
	
	LocalPath directionLocalpath( expansionNode->getConfiguration(), directionConfig );
  
	// Perform extension toward directionConfig
	// Construct a smaller path from direction path
	LocalPath extensionLocalpath = getExtensiontPath (expansionNode->getConfiguration(), directionConfig );
	
	Node* NodeNew = NULL;
	
	if ( extensionLocalpath.isValid() )
	{
		// Create new node and add it to the graph
		NodeNew = new Node(m_Graph,extensionLocalpath.getEnd());
		nbCreatedNodes++;
    
    double radius = rrgBallRadius() * m_RrgRadiusFactor;
    
//    if( print_exploration )
//    {
//      cout << "radius : " << radius << endl;
      cout << "radius : " << radius  << " , number of nodes : " << m_Graph->getNumberOfNodes() << endl;
//    }

		vector<Node*> NodesNear = m_Graph->KNearestWeightNeighbour(NodeNew->getConfiguration(),
																															m_K_Nearest,
																															radius,
																															false,
																															ENV.getInt(Env::DistConfigChoice));
    
    
    if( print_exploration )
    {
      cout << "Number of Neighboors : " << NodesNear.size() << endl;
    }
    
		// The normal RRT procedure
		Node* NodeMin = expansionNode;
    confPtr_t q_new = NodeNew->getConfiguration();
    
		std::vector<Node*> ValidNodes;
    
		// Compute the node that minimizes the Sum of Cost
    double min_sum_of_cost=std::numeric_limits<double>::max();
    
		for (int i=0; i<int(NodesNear.size()); i++) 
		{
			//cout << "NodesNear[i] : " << NodesNear[i] <<  " , d : " << NodesNear[i]->dist(NodeNew) << endl;
			LocalPath path( NodesNear[i]->getConfiguration(), q_new );
			
      if ( path.isValid() )
      {
        ValidNodes.push_back( NodesNear[i] );
        
        double cost_to_node = NodesNear[i]->sumCost()+path.cost();
        if ( cost_to_node < min_sum_of_cost )
        {
          NodeMin = NodesNear[i];
          min_sum_of_cost = cost_to_node;
        }
      }
		}
		
    // Add NodeNew to the graph, and add the edge n_min -> n_new
		m_Graph->addNode(NodeNew);
		m_Graph->addEdges(NodeMin,NodeNew,NULL,true);
    
    // Merge compco with minNode and set as parent
		NodeNew->merge(NodeMin);
		NodeNew->sumCost() = min_sum_of_cost;
		NodeNew->parent() = NodeMin;
    NodeMin->isLeaf() = false;
    
		// Rewire the graph 
		for ( int i=0; i<int(ValidNodes.size()); i++) 
		{
			if ( ValidNodes[i] == NodeMin ) continue;
			
			LocalPath path( ValidNodes[i]->getConfiguration(), q_new );
			
			// Compute the cost to get to the node
			// by each neighbor to the new node and change edge if superior
			if ( ValidNodes[i]->sumCost() > NodeNew->sumCost()+path.cost() )
			{
        //cout << "should rewire" << endl;
        if( ValidNodes[i] && ValidNodes[i]->parent() )
        {
          m_Graph->removeEdges( ValidNodes[i], ValidNodes[i]->parent() );
          m_Graph->addEdges( ValidNodes[i], NodeNew, path.getParamMax() );
          
          ValidNodes[i]->sumCost() = NodeNew->sumCost() + path.cost();
          ValidNodes[i]->parent() = NodeNew;
          NodeNew->isLeaf() = false;
          
          if( print_rewiring )
            cout << "Rewiring : " << ++m_nb_rewiring << endl;
        }
			}
		}
	}
	else 
	{
		failed = true;
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		// Components were merged
		if(( directionNode != NULL )&&( NodeNew == directionNode ))
		{
			cout << "Connected in Transition" << __func__ << endl;
			return 0;
		}
	}
	else
	{
		expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
}

/*!
 * expandProcess
 */
int StarExpansion::expandProcess(Node* expansionNode,
																 shared_ptr<Configuration> directionConfig,
																 Node* directionNode,
																 Env::expansionMethod method)
{
	switch (method) 
	{
		case Env::Connect:
			return connectExpandProcess( expansionNode, directionConfig, directionNode );
			
		case Env::Extend:
			return extendExpandProcess(expansionNode, directionConfig, directionNode);
      
		default:
			cerr << "Error : expand process not implemented" << endl;
			return 0;
	} 
}


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
		if ( path.getParamMax() == 0.0 && print_exploration ) {
      cout << "path.getParamMax() == 0.0 in " << __func__ << endl;
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


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
#include "API/Trajectory/trajectory.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"

#include "TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "TrajectoryOptim/trajectoryOptim.hpp"

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
  m_Iteration = 0;
  m_biasRatio = 5;
  m_ith_on_traj = -1;
  m_ith_trajectory = -1;
  m_start_bias = false;
  
  initCSpace();
  initStomp();
}

/*!
 * Destructor
 */
StarExpansion::~StarExpansion()
{
	
}

void StarExpansion::setInitAndGoal( confPtr_t q_init, confPtr_t q_goal )
{
  m_q_init = q_init;
  m_q_goal = q_goal;
}

void StarExpansion::initCSpace()
{
  m_cspace = new CSpaceCostMap2D();
  m_cspace->set_step( m_step );
  m_cspace->set_cost_step( m_step );
  m_cspace->set_robot( m_Graph->getRobot() );
}

void StarExpansion::initStomp()
{
  traj_optim_initStomp();
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

confPtr_t StarExpansion::getExpansionDirection(Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
  confPtr_t q;
  
  m_Iteration++;
  
//  if( /*!(m_Iteration % m_biasRatio != 0)*/ false )
//  {
    // Selection in the entire CSpace and
    // biased to the Comp of the goal configuration
    if ( (!m_start_bias) && (p3d_random(0., 1.) <= 0.25) )
    {
      // the goal component as direction of expansion
      q = m_q_goal;
    }
    else {
      // the entire CSpace
      q = m_Graph->getRobot()->shoot(samplePassive);
    }
//  }
//  else
//  {
  
    // Generate noisy trajectories 
    if( m_ith_on_traj == -1 && m_ith_trajectory == -1)
    {
      if( m_start_bias == false )
        m_start_bias = m_Graph->searchConf( *m_q_goal );
      
      API::Trajectory* traj = NULL;
      
      if( m_start_bias ) { 
        traj = m_Graph->extractBestTrajSoFar( m_q_init, m_q_goal );
      }
      if( traj != NULL ){
        traj->cutTrajInSmallLPSimple( (PlanEnv->getInt( PlanParam::nb_pointsOnTraj )-1) );
        
        if( traj->size() == PlanEnv->getInt( PlanParam::nb_pointsOnTraj )-1 ) 
        {
          m_biasTrajectory.clear();
          optimizer->generateNoisyTrajectory(*traj,m_biasTrajectory);
          
          m_ith_on_traj = 1;
          m_ith_trajectory = 0;
          
          return m_biasTrajectory[m_ith_trajectory][m_ith_on_traj];
        }
      }
    }
    else 
    {
      m_ith_trajectory++; // Next trajectories
      
      if( m_ith_trajectory < int(m_biasTrajectory.size()) )
      {
        if( m_ith_on_traj < int(m_biasTrajectory[m_ith_trajectory].size()-1) )
        {
          q = m_biasTrajectory[m_ith_trajectory][m_ith_on_traj];
          return q;
        }
        else { 
          m_ith_on_traj = -1; // Terminates
          m_ith_trajectory = -1;
        }
      }
      else {
        m_ith_on_traj += 5; // All config on the noisy trajectory
        m_ith_trajectory = 0;
      }
    }
  //  }
  
  return (q);
}

void StarExpansion::rewireGraph(Node* new_node, Node* min_node, const vector<Node*>& neigh_nodes)
{
  confPtr_t q_new = new_node->getConfiguration();
  
  // Rewire the graph 
  for ( int i=0; i<int(neigh_nodes.size()); i++) 
  {
    // min node is already wired to new node
    if ( neigh_nodes[i] == min_node ) continue;
    
    LocalPath path( neigh_nodes[i]->getConfiguration(), q_new );
    
    // Compute the cost to get to the new node from each neighbor
    // and change edge if superior
    if ( neigh_nodes[i]->sumCost() > new_node->sumCost()+path.cost() )
    {
      //cout << "should rewire" << endl;
      if( neigh_nodes[i] && neigh_nodes[i]->parent() )
      {
        m_Graph->removeEdges( neigh_nodes[i], neigh_nodes[i]->parent() );
        m_Graph->addEdges( neigh_nodes[i], new_node, path.getParamMax() );
        
        neigh_nodes[i]->sumCost() = new_node->sumCost() + path.cost();
        neigh_nodes[i]->parent() = new_node;
        new_node->isLeaf() = false;
        
        if( print_rewiring )
        {
          cout << "Rewiring : " << ++m_nb_rewiring << endl;
        }
      }
    }
  }
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
  LocalPath extensionLocalpath = directionLocalpath;
  
  if( expansionNode->getConfiguration()->equal( *directionConfig ) )
  {
    failed=true;
  }
  else
  {
    // Perform extension toward directionConfig
    // Construct a smaller path from direction path
    extensionLocalpath = getExtensiontPath (expansionNode->getConfiguration(), directionConfig );
	}
  
	Node* node_new = NULL;
  
  bool isValid = true;
  if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
  {
    isValid = extensionLocalpath.isValid();
  }
  
	if ( isValid && !failed )
	{
		// Create new node and add it to the graph
		node_new = new Node(m_Graph,extensionLocalpath.getEnd());
		nbCreatedNodes++;
    
    double radius = rrgBallRadius() * m_RrgRadiusFactor;
    
//    if( print_exploration )
//    {
//      cout << "radius : " << radius << endl;
      cout << "radius : " << radius  << " , number of nodes : " << m_Graph->getNumberOfNodes() << endl;
//    }

		vector<Node*> near_nodes = m_Graph->KNearestWeightNeighbour(node_new->getConfiguration(),
																															m_K_Nearest,
																															radius,
																															false,
																															ENV.getInt(Env::DistConfigChoice));
    
    
    if( print_exploration )
    {
      cout << "Number of Neighboors : " << near_nodes.size() << endl;
    }
    
		// The normal RRT procedure
		Node* node_min = expansionNode;
    confPtr_t q_new = node_new->getConfiguration();
    
		std::vector<Node*> ValidNodes;
    
		// Compute the node that minimizes the Sum of Cost
    double min_sum_of_cost=std::numeric_limits<double>::max();
    
		for (int i=0; i<int(near_nodes.size()); i++) 
		{
			//cout << "near_nodes[i] : " << near_nodes[i] <<  " , d : " << near_nodes[i]->dist(node_new) << endl;
			LocalPath path( near_nodes[i]->getConfiguration(), q_new );
			
      isValid = true;
      if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
      {
        isValid = path.isValid();
      }
      
      if ( isValid  )
      {
        ValidNodes.push_back( near_nodes[i] );
        
        double cost_to_node = near_nodes[i]->sumCost()+path.cost();
        if ( cost_to_node < min_sum_of_cost )
        {
          node_min = near_nodes[i];
          min_sum_of_cost = cost_to_node;
        }
      }
		}
		
    // Add node_new to the graph, and add the edge n_min -> n_new
		m_Graph->addNode( node_new );
		m_Graph->addEdges( node_min, node_new, NULL, true );
    
    // Merge compco with minNode and set as parent
		node_new->merge( node_min );
		node_new->sumCost() = min_sum_of_cost;
		node_new->parent() = node_min;
    node_min->isLeaf() = false;
    
    // Call to rewire function
    rewireGraph( node_new, node_min, near_nodes );
	}
	else 
	{
		failed = true;
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
    //cout << "Success" << endl;
		// Components were merged
		if(( directionNode != NULL )&&( node_new == directionNode ))
		{
			cout << "Connected in Transition" << __func__ << endl;
			return 0;
		}
	}
	else
	{
    //cout << "Failed" << endl;
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
  dynamic_cast<StarExpansion*>(_expan)->setInitAndGoal( _q_start, _q_goal );
	return added;
}

//! prune tree
void StarRRT::pruneTreeFromNode(Node* node)
{
  //double cost  = _Graph->searchConf(*q)->sumCost();
  Node* parent = node->parent();
  
  if( parent == NULL ) {
    return;
  }
  
  Node* parent_parent = parent->parent();
  _Graph->removeNode( parent );
  
  if( parent_parent == NULL ) {
    return;
  }
  else 
  {
    ConnectedComponent* compco = parent_parent->getConnectedComponent();
    if( compco != NULL ) 
    {
      vector<Node*> nodes = compco->getNodes();
      for (int i=0; i<int(nodes.size()); i++) 
      {
        _Graph->removeNode( nodes[i], false );
      }
      _Graph->rebuildCompcoFromBoostGraph();
    }
  }
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


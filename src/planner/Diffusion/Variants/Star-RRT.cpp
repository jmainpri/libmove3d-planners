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
#include <iomanip>
#include <fstream>

#include "Planner-pkg.h"
#include "Graphic-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

const bool print_exploration = false;
const bool print_rewiring = false;
const bool print_lower_connect = false;

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
  m_goal_bias = false;
  m_tube_bias =false;
  m_cspace = NULL;
  
  Robot* rob = m_Graph->getRobot();
  
  if( rob->getRobotStruct()->njoints == 1 &&
      rob->getRobotStruct()->joints[1]->type == P3D_PLAN ) 
  {
    m_cspace = new CSpaceCostMap2D();
    initCSpace();
  }
  
  if( rob->getName() == "PR2_ROBOT" ) 
  {
    m_cspace = new Pr2CSpace();
    initCSpace();
  }
  
  if( m_tube_bias )
  {
    initStomp();
  }
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
  if( m_cspace == NULL ) {
    return 5*step();
  }
  
  if(m_cspace->get_connection_radius_flag() )
  {
    double inv_d = 1.0 / m_cspace->dimension();
    double gamma_rrg = 2 * pow(1.0 + inv_d, inv_d) * pow(m_cspace->volume() / m_cspace->unit_sphere(), inv_d);
    double nb_nodes = m_Graph->getNumberOfNodes();
    double radius = gamma_rrg * pow((log(nb_nodes)/nb_nodes), inv_d);
    //double radius = std::min(m_cspace->get_step(), radius );
    
    if( print_lower_connect || print_rewiring )
      cout << "radius : " << radius << endl;
    
    return radius;
  }
  else
  {
    return( m_cspace->get_step() );
  }
}

/**
 * Compco
 */
void StarExpansion::setInitialCompco( ConnectedComponent* compco )
{
  m_compco = compco;
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
																				MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, 
																				Node* directionNode)
{
	cout << "StarExpansion::connectExpandProcess Not implemented" << endl;
	return 0;
}

confPtr_t StarExpansion::sampleInTube()
{
  confPtr_t q;
  
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
  
  return q;
}

confPtr_t StarExpansion::getExpansionDirection(Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode)
{
  confPtr_t q;
  
  m_Iteration++;
  
//  if( /*!(m_Iteration % m_biasRatio != 0)*/ false )
//  {
    // Selection in the entire CSpace and
    // biased to the Comp of the goal configuration
    if ( m_goal_bias && (p3d_random(0., 1.) <= 0.25) )
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
  
  if( m_tube_bias )
  {
    q = sampleInTube();
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
    if ( neigh_nodes[i] == min_node || neigh_nodes[i]->parent() == NULL ) { 
      continue;
    }
    
//    cout << "****************************" << endl;
//    if( min_node->parent() )
//      cout << "min_node->parent()->getId() : " << min_node->parent()->getId() << endl;
//    cout << "min_node->getId() : " << min_node->getId() << endl;
//    cout << "new_node->getId() : " << new_node->getId() << endl;
//    cout << "neigh_nodes[i]->getId() : " << neigh_nodes[i]->getId() << endl;
//    cout << "neigh_nodes[i]->parent()->getId() : " << neigh_nodes[i]->parent()->getId() << endl;
    
    LocalPath path( q_new, neigh_nodes[i]->getConfiguration() );
    
    // Compute the cost to get to the new node from each neighbor
    // and change edge if superior
    double neigh_sum_cost = neigh_nodes[i]->sumCost();
    double new_sum_cost = new_node->sumCost();
    double path_cost = path.cost();
//    cout << "neigh_sum_cost : " << neigh_sum_cost << endl;
//    cout << "new_sum_cost : " << new_sum_cost << endl;
//    cout << "path_cost : " << path_cost << endl;
    
    if ( neigh_sum_cost > (new_sum_cost+path_cost) )
    {
      m_Graph->removeEdges( neigh_nodes[i], neigh_nodes[i]->parent() );
      m_Graph->addEdges( neigh_nodes[i], new_node, false, path.getParamMax(), false, path.cost() );
      
      neigh_nodes[i]->parent() = new_node;
      new_node->isLeaf() = false;
      
      if( print_rewiring )
      {
        cout << "Rewiring : " << ++m_nb_rewiring << endl;
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
int StarExpansion::extendExpandProcess( Node* expansionNode, confPtr_t directionConfig, Node* directionNode )
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
  
  bool is_valid = true;
  if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
  {
    is_valid = extensionLocalpath.isValid();
  }
  
	if ( is_valid && !failed )
	{
		// Create new node and add it to the graph
		node_new = new Node( m_Graph, extensionLocalpath.getEnd() );
		nbCreatedNodes++;
    
//    double radius = rrgBallRadius() * m_RrgRadiusFactor;
    double radius = rrgBallRadius() * PlanEnv->getDouble( PlanParam::starRadius );
    
//    if( print_exploration )
//    {
//      cout << "radius : " << radius  << " , number of nodes : " << m_Graph->getNumberOfNodes() << endl;
//    }
    
    int K = m_Graph->getNumberOfNodes();
    //int K = m_K_Nearest;

		vector<Node*> near_nodes = m_compco->KNearestWeightNeighbour(node_new->getConfiguration(), 
                                                                 K, radius, false, ENV.getInt(Env::DistConfigChoice));
    
    if( print_exploration )
    {
      cout << "Number of Neighboors : " << near_nodes.size() << endl;
    }
    
		// The normal RRT procedure
		Node* node_min = expansionNode;
    confPtr_t q_new = node_new->getConfiguration();
    
		std::vector<Node*> valid_nodes;
    
		// Compute the node that minimizes the Sum of Cost
    double min_path_cost = extensionLocalpath.cost();
    double min_path_dist = extensionLocalpath.getParamMax();
    double min_sum_of_cost = expansionNode->sumCost(true)+extensionLocalpath.cost();
    
		for (int i=0; i<int(near_nodes.size()); i++) 
		{
			LocalPath path( near_nodes[i]->getConfiguration(), q_new );
			
      is_valid = true;
      if( PlanEnv->getBool(PlanParam::trajComputeCollision) )
      {
        is_valid = path.isValid();
      }
      
      if ( is_valid  )
      {
        valid_nodes.push_back( near_nodes[i] );
        
        double cost_to_node = near_nodes[i]->sumCost(true)+path.cost();
        
        if ( cost_to_node < min_sum_of_cost )
        {
          node_min = near_nodes[i];
          min_sum_of_cost = cost_to_node;
          min_path_cost = path.cost();
          min_path_dist = path.getParamMax();
        }
      }
		}
    
    if ( print_lower_connect && (expansionNode != node_min ))
    {
      cout << "Lower cost connect" << endl;
    }
    
    // Add node_new to the graph, and add the edge n_min -> n_new
    m_last_added_node = node_new;
		m_Graph->addNode( node_new );
		m_Graph->addEdges( node_min, node_new, false, min_path_dist, false, min_path_cost );
    
    // Merge compco with minNode and set as parent
		node_new->merge( node_min );
		node_new->sumCost() = min_sum_of_cost;
		node_new->parent() = node_min;
    node_min->isLeaf() = false;
    
    // Call to rewire function
    if( PlanEnv->getBool(PlanParam::starRewire) ) 
    {
      rewireGraph( node_new, node_min, valid_nodes );
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
		if(( directionNode != NULL )&&( node_new == directionNode ))
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
unsigned StarExpansion::expandProcess(Node* expansionNode,
																 confPtr_t directionConfig,
																 Node* directionNode,
																 Env::expansionMethod method)
{
	switch (method) 
	{
//		case Env::Connect:
//			return connectExpandProcess( expansionNode, directionConfig, directionNode );
			
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
  m_current_traj = NULL;
}

/** 
 * Destructor
 */
StarRRT::~StarRRT()
{
	saveConvergenceToFile();
  delete m_current_traj;
}

/**
 * Initialzation of the plannificator
 * @return the number of node added during the init phase
 */
unsigned StarRRT::init()
{
	int added = TreePlanner::init();
	_expan = new StarExpansion(_Graph);
  dynamic_cast<StarExpansion*>(_expan)->setInitAndGoal( _q_start, _q_goal );
//  _expan = _expan = new RRTExpansion(_Graph);
  dynamic_cast<StarExpansion*>(_expan)->setInitialCompco( _Start->getConnectedComponent() );
  
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
bool StarRRT::connectNodeToCompco( Node* node, Node* compNode )
{
	LocalPath path( node->getConfiguration(), compNode->getConfiguration() );
	
	if( path.getParamMax() <= PlanEnv->getDouble(PlanParam::starFinish)*_expan->step() )
	{
		if ( path.getParamMax() == 0.0 && print_exploration ) 
    {
      cout << "path.getParamMax() == 0.0 in " << __func__ << endl;
			node->print();
			compNode->print();
		}
		
		if( path.isValid() ) 
    {
			//_expan->addNode( node, path, 1.0, nearestNode, nbCreatedNodes );
      //m_Graph->addEdges( node, nearestNode, false, path.getParamMax(), false, path.cost() );
      _Graph->linkNodeAndMerge( node, compNode, true );
      
      compNode->parent() = node;
      compNode->isLeaf() = true;
      
			cout << "Path Valid Connected" << endl;
			return true;
		}
		else {
			return false;
		}
	}
	
	return false;
}

void StarRRT::saveConvergenceToFile()
{
  std::ostringstream oss;
  std::ofstream s;
  
    oss << getenv("HOME_MOVE3D") << "/statFiles/convergence_rrt_star_" << std::setfill('0') << std::setw(4) << m_runId << ".csv";
  
	const char *res = oss.str().c_str();
	
	s.open(res);
	cout << "Opening save file : " << res << endl;
	
  s << "TIME" << ";";
  s << "LENGTH" << ";";
  s << "MAX" << ";";
  s << "AVERAGE" << ";";
  s << "INTEGRAL" << ";";
  s << "MECHA WORK" << ";";
  s << endl;
  
	for (int i=0; i<int(m_convergence_rate.size()); i++)
	{    
    s << m_convergence_rate[i].first << ";";
    s << m_convergence_rate[i].second.length << ";";
    s << m_convergence_rate[i].second.max << ";";
    s << m_convergence_rate[i].second.average << ";";
    s << m_convergence_rate[i].second.integral << ";";
    s << m_convergence_rate[i].second.mecha_work << ";";
    s << endl;
	}
  
  s.close();
  cout << "Closing save file" << endl;
}

void StarRRT::extractTrajectory()
{
  API::Trajectory* traj = _Graph->extractAStarShortestPathsTraj( _q_start, _q_goal );
  
  if( traj ) 
  {
    if( m_current_traj == NULL || (*m_current_traj) != (*traj) )
    {
      TrajectoryStatistics stat;
      
      double cost = traj->costStatistics(stat);
      
      cout << "time : " << getTime() << " , traj cost : " << cost << endl;
      //    traj->costDeltaAlongTraj();
      //    traj->replaceP3dTraj(); 
      
      m_convergence_rate.push_back( std::make_pair( getTime(), stat )  );
      
      if( m_current_traj == NULL ) 
        m_current_traj = new API::Trajectory( _Graph->getRobot() );
      
      // replace current trajectory by new trajectory
      (*m_current_traj) = (*traj);
      
      if(  (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
        g3d_draw_allwin_active();
    }
  }
  else {
    cout << "no traj" << endl;
  }
  
  delete traj;
}

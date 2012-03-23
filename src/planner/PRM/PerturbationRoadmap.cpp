//
//  PerturbationRoadmap.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/02/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#include "PerturbationRoadmap.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"

#include "trajectoryOptim.hpp"

#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/planEnvironment.hpp"

#include "p3d/env.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

PerturbationRoadmap::PerturbationRoadmap(Robot* R, Graph* G)  : PRM(R,G)
{
  m_K_Nearest = 3;
  m_traj = NULL;
  m_sampled_on_traj = false;
  m_descent = true;
  m_max_dist_to_traj = std::numeric_limits<double>::min();
  m_use_rejection = false;
  
  m_std_dev_trans = 1000.0;
  m_std_dev_rot =3.14/10;
  
  m_radian_steps = 1;
  m_transl_steps = 1;
}

PerturbationRoadmap::~PerturbationRoadmap()
{
  
}

int PerturbationRoadmap::init()
{
  int added = PRM::init();
  
  cout << "Init PerturbationRoadmap" << endl;
  
  if( m_traj ) {
    delete m_traj;
    m_traj = NULL;
  }
  
//  if( !traj_optim_initScenario()) {
//    cout << "Error!!!" << endl;
//    return added;
//  }
  
  API::Trajectory T(_Robot);
  
  if( _Robot->getTrajStruct() )
  {
    T = _Robot->getCurrentTraj();
  }
  else {
    T =  API::Trajectory( traj_optim_create_sraight_line_traj() );
  }
  
  m_qi = T.getBegin();
  m_qf = T.getEnd();
  addTrajectory( T );
  
  m_traj = new API::CostOptimization( T );
  
  return added;
}

void PerturbationRoadmap::addTrajectory( const API::Trajectory& T )
{
  double range_max = T.getRangeMax();
  m_delta = range_max/20;
  
  Node* node_1 = m_main_compco = _Start;
  Node* node_2;
  
  for (double s=m_delta; s<(range_max-m_delta); s+=m_delta) 
  {
    node_2 = new Node( _Graph, T.configAtParam(s), true);
    _Graph->addNode( node_2 );
    _Graph->addEdges( node_1, node_2, 
                     false, node_1->getConfiguration()->dist(*node_2->getConfiguration()), 
                     true, NULL );
    node_1 = node_2;
  }
  node_2 = new Node( _Graph, T.getEnd(), true);
  _Graph->addNode( node_2 );
  _Graph->addEdges( node_1, node_2, 
                   false, node_1->getConfiguration()->dist(*T.getEnd()), 
                   true , NULL );
  
  _Graph->rebuildCompcoFromBoostGraph();
}

Node* PerturbationRoadmap::getClosestOnTraj( confPtr_t q_rand )
{
  vector<Node*> nodes = _Graph->extractAStarShortestNodePaths( m_qi, m_qf );
  if( nodes.empty() ) {
    return NULL;
  }
  
  Graph::sortNodesByDist( nodes, q_rand );
  return nodes[0];
}

double PerturbationRoadmap::distToTraj( confPtr_t q_new )
{
  vector<Node*> nodes = _Graph->extractAStarShortestNodePaths( m_qi, m_qf );
  if( nodes.empty() ) {
    return NULL;
  }
  
  Graph::sortNodesByDist( nodes, q_new );
  return nodes[0]->getNodeStruct()->dist_Nnew;
}

bool PerturbationRoadmap::testPerturb( confPtr_t q_new, vector<Node*>& vect_nodes )
{  
  bool is_valid = true;
  if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
  {
    is_valid = !q_new->isInCollision();
  }
  
  if ( !is_valid ) {
    return false;
  }
  
  vector< pair< int, shared_ptr<LocalPath> > > edges;
	Node* node_new = NULL;
  
  for( int i=0;i<int(vect_nodes.size());i++) {
    
    shared_ptr<LocalPath> path(new LocalPath(vect_nodes[i]->getConfiguration(), q_new));
    
    // if the path is valid, check for cost
    if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
    {
      is_valid = path->isValid();
    }
    
    if ( is_valid )
    {
      edges.push_back(make_pair(i,path));
      
      if( edges.size() == 1 )
      {
        node_new = new Node( _Graph, q_new, true);
        _Graph->addNode( node_new );
      }
    }
  }
  
  for( int i=0;i<int(edges.size());i++) 
  {
    double max_param = edges[i].second->getParamMax();
    
    _Graph->addEdges( vect_nodes[edges[i].first], node_new, false, max_param, true, NULL);
    _Graph->rebuildCompcoFromBoostGraph();
    is_valid = true;
  }

  return is_valid;
}

bool PerturbationRoadmap::addPerturbation( confPtr_t q_rand )
{
  Node* node_near = _Graph->nearestWeightNeighbour( m_main_compco, q_rand, false, ENV.getInt(Env::DistConfigChoice));
  
  confPtr_t q_new = API::CostOptimization::perturbCurrent( node_near->getConfiguration(), q_rand, m_delta, m_descent );
  
  vector<Node*> nodes = _Graph->KNearestWeightNeighbour( q_new, m_K_Nearest, P3D_HUGE, 
                                                        false, ENV.getInt(Env::DistConfigChoice));
  
  if( int(nodes.size()) < m_K_Nearest )
    return false;
  
  vector<Node*> nodes_neigh(2);
  nodes_neigh[0] = nodes[1];
  nodes_neigh[1] = nodes[2];
  
  bool success = testPerturb( q_new, nodes_neigh );
  return success;
}

bool PerturbationRoadmap::expandPerturbation( confPtr_t q_rand )
{
  Node* node_near = _Graph->nearestWeightNeighbour( m_main_compco, q_rand, false, ENV.getInt(Env::DistConfigChoice));
  if (node_near == NULL) {
    return false;
  }
  
//  Node* node_near = getClosestOnTraj( q_rand );
//  if( node_near == NULL ) {
//    return false;
//  }
  
  LocalPath path( node_near->getConfiguration(), q_rand );
  
  // Dividing the step n times to stay in the bounds
  const double minStep = 2.0;
	const int	max_div = 3;
  
  confPtr_t q_new;
  
  double step = m_delta/2.0;
	bool is_q_out_of_bounds = true;
  bool is_q_in_collision = true;
  int ith_div =0;
  
	for (; ith_div<max_div && (is_q_out_of_bounds && is_q_in_collision) ; ith_div++) 
	{
    q_new = path.configAtParam( step );
    is_q_out_of_bounds = q_new->isOutOfBounds();
    
    if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
    {
      if( !is_q_out_of_bounds ) {
        is_q_in_collision = q_new->isInCollision();
      }
    } else {
      is_q_in_collision = false;
    }
    step /= minStep; 
  }
  
  //cout << "ith_div : " << ith_div << endl;
  
  if( is_q_out_of_bounds || is_q_in_collision ) {
    //cout << "is_q_out_of_bounds : " << is_q_out_of_bounds ;
    //cout << " , is_q_in_collision : " << is_q_in_collision << endl;
    return false;
  }
  
  double dist=0.0;
  
  if( m_use_rejection ) 
  {
    double dist = distToTraj(q_new);
    
    if( (dist > m_max_dist_to_traj) && (p3d_random(0.0,1.0) > 0.01) ) {
      //cout << "reject : " << endl;
      return false;
    }
  }
  
  vector<Node*> nodes = _Graph->KNearestWeightNeighbour( q_new, 5, P3D_HUGE, 
                                                        false, ENV.getInt(Env::DistConfigChoice));
  
  Graph::sortNodesByDist( nodes, q_new );
  
  //for (int i=0; i<int(nodes.size()) ; i++)  {
    //cout << "node["<< i <<"] : " << nodes[i]->getNodeStruct()->dist_Nnew << endl;
  //}
  
//  if( int(nodes.size()) < 2 ) {
//    return false;
//  }
//  nodes.resize(2);
  
  //confPtr_t q_new = API::CostOptimization::perturbCurrent( q_rand, _Robot->shoot(), m_delta, false );
  
  bool success = testPerturb( q_new, nodes );
  
  if( m_use_rejection ) 
  {
    if ( success && (dist > m_max_dist_to_traj)) { 
      m_max_dist_to_traj = dist;
    }
  }
  return success;
}


confPtr_t PerturbationRoadmap::getExpansionConfiguration(bool sample_on_traj)
{
  if( !sample_on_traj )
  {
    return _Robot->shoot();
  }
  else {
    double dist;
    return m_traj->getRandConfAlongTraj( dist, true );
  }
}

confPtr_t PerturbationRoadmap::trajShootConfiguration()
{
  double dist=0.0;
  confPtr_t q1( m_traj->getRandConfAlongTraj( dist, false ));
  confPtr_t q2( new Configuration(*q1));
  
  p3d_gaussian_config2_specific(q1->getRobot()->getRobotStruct(), 
                                q1->getConfigStruct(), 
                                q2->getConfigStruct(), 
                                m_std_dev_trans, m_std_dev_rot, false);
  
  return q2;
}

void PerturbationRoadmap::expandOneStep()
{
  // Get the random direction
  // confPtr_t q = getExpansionConfiguration(false);
  confPtr_t q = trajShootConfiguration();
  
  // Get the expansion perturbation
  bool valid_perturb = expandPerturbation(q);
  
  if( valid_perturb ) {
    
		m_nbConscutiveFailures = 0;
		m_nbAddedNode++;
    
    //cout << "m_std_dev : " << m_std_dev << endl;
    m_std_dev_trans = 1000/m_transl_steps;
    m_std_dev_rot = 3.14/m_radian_steps;
    m_radian_steps += 0.1;
    m_transl_steps += 0.1;
    
    // 1000 - 50 = 950, 950/300 
    // 15 - 50 = 35, 35/300 => 0.1 
    
    if( m_traj ) {
      delete m_traj;
    }
    API::Trajectory* traj = _Graph->extractAStarShortestPathsTraj( m_qi, m_qf ); 
    
    if( traj != NULL ) {
      
      m_traj = new API::CostOptimization(*traj);
      delete traj;
      cout << "trajectory cost : " << m_traj->cost() << endl;
    }
    else {
      cout << "Trajectory not in graph" << endl;
    }
    
		if (ENV.getBool(Env::drawExploration)) {
      
      _Robot->setAndUpdate( *m_qf );
      
			(*_draw_func)();
      
      // Wait for window event
      // while ( PlanEnv->getBool(PlanParam::nextIterWaitForGui) );
      // PlanEnv->setBool(PlanParam::nextIterWaitForGui, true ); 
		}
	}
	else {
		m_nbConscutiveFailures++;
	}
}
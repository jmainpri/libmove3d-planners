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

using namespace std;
using namespace tr1;

PerturbationRoadmap::PerturbationRoadmap(Robot* R, Graph* G)  : PRM(R,G)
{
  m_K_Nearest = 2;
}

PerturbationRoadmap::~PerturbationRoadmap()
{
  
}
 
int PerturbationRoadmap::init()
{
  int added = PRM::init();
  
  cout << "Init PerturbationRoadmap" << endl;
  
  if( !traj_optim_initScenario()) {
    cout << "Error!!!" << endl;
    return added;
  }
  
  API::Trajectory T( traj_optim_create_sraight_line_traj() );
  
  m_qi = T.getBegin();
  m_qf = T.getEnd();
  
  addTrajectory( T );
  
  return added;
}

void PerturbationRoadmap::addTrajectory( const API::Trajectory& T )
{
  double range_max = T.getRangeMax();
  m_delta = range_max/10;
  
  Node* node_1 = m_main_compco = _Start;
  
  for (double s=m_delta; s<range_max; s+=m_delta) 
  {
    Node* node_2 = new Node( _Graph, T.configAtParam(s), true);
    _Graph->addNode( node_2 );
    
    _Graph->addEdges( node_1, node_2, node_1->getConfiguration()->dist(*node_2->getConfiguration()));
    node_1 = node_2;
  }
  _Graph->rebuildCompcoFromBoostGraph();
}

bool PerturbationRoadmap::testPerturb( confPtr_t q_new, vector<Node*>& vect_nodes )
{  
  bool in_collision=false;
  
	//Get 3 configurations at random along the trajectory
	confPtr_t q_prev = vect_nodes[0]->getConfiguration();
	confPtr_t q_next = vect_nodes[1]->getConfiguration();
	
	// If qNew is free then
	// Check the triangle localPath
	if ( !q_new->isInCollision() )
	{
		shared_ptr<LocalPath> first_half(new LocalPath(q_prev, q_new));
		shared_ptr<LocalPath> secon_half(new LocalPath(q_new, q_next));
		
		bool isValid = true;
		// If the path is valid, check for cost
		if ( PlanEnv->getBool(PlanParam::trajComputeCollision) )
		{
			isValid = first_half->isValid();
      
      if( isValid ) {
        isValid = secon_half->isValid();
      }
		}
		
		if ( isValid )
		{
      double max_param1 = first_half->getParamMax();
      double max_param2 = secon_half->getParamMax();
      
      Node* node_node = new Node( _Graph, q_new, true);
      _Graph->addNode( node_node );
      _Graph->addEdges( vect_nodes[0], node_node, max_param1);
      _Graph->addEdges( node_node, vect_nodes[1], max_param2);
      _Graph->rebuildCompcoFromBoostGraph();
		}
    else {
      in_collision=true;
    }
	}
	else {
		in_collision=true;
	}
  
  return in_collision;
}

bool PerturbationRoadmap::addPerturbation( confPtr_t q_rand )
{
  Node* node_near = _Graph->nearestWeightNeighbour( m_main_compco, q_rand, false, ENV.getInt(Env::DistConfigChoice));

  confPtr_t q_new = API::CostOptimization::perturbCurrent( node_near->getConfiguration(), q_rand, m_delta, true );
  
  vector<Node*> nodes = _Graph->KNearestWeightNeighbour( q_new, m_K_Nearest, P3D_HUGE, 
                                                         false, ENV.getInt(Env::DistConfigChoice));
  
  if( nodes.size() < 2 )
    return false;
  
  bool success = testPerturb( q_new, nodes );
  return success;
}

void PerturbationRoadmap::expandOneStep()
{
  confPtr_t q = _Robot->shoot();
  
  if( addPerturbation(q) ) {

		m_nbConscutiveFailures = 0;
		m_nbAddedNode++;
		
		if (ENV.getBool(Env::drawExploration)) {
      _Graph->extractAStarShortestPathsTraj( m_qi, m_qf );
			(*_draw_func)();
		}
	}
	else {
		m_nbConscutiveFailures++;
	}
}

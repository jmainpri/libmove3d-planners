//
//  PerturbationRoadmap.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/02/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef PERTURBATION_ROADMAP_hpp
#define PERTURBATION_ROADMAP_hpp

#include "planner/PRM/PRM.hpp"

#include "API/Trajectory/trajectory.hpp"

class PerturbationRoadmap : public PRM
{
public:
	/**
	 * Creates a perturbation roadmap from a given robot 
   * and a given graph
	 * @param Robot
   * @param Graph
	 */
	PerturbationRoadmap(Robot* R, Graph* G);
	
	/**
	 * Deletes a perturbation planner
	 */
	~PerturbationRoadmap();
	
	/**
	 * initialize the Planner
	 * @return le number of nodes added to the graph
	 */
	virtual int init();
	
  /**
   * adds a trajectory
   */
  void addTrajectory(const API::Trajectory& T);
  
  /**
   * Test if the perturbation is valid
   */
  bool testPerturb( confPtr_t q_new, std::vector<Node*>& vect_conf );
  
  /**
   * Add a perturbation to the graph
   */
  bool addPerturbation( confPtr_t q_rand );
  
	/**
	 * Adds nodes to Graph
	 */
	virtual void expandOneStep();
	
protected:
	/**
	 * Members
	 */
  Node* m_main_compco;
  double m_delta;
  bool m_descent;
  int m_K_Nearest;
  
  confPtr_t m_qi;
  confPtr_t m_qf;
};

#endif

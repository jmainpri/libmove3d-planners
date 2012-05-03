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
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

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
  
protected:
  
  /**
   * Compute the translation bounds of the robot
   */
  void getTranslationBounds();
	
  /**
   * Adds a trajectory
   */
  void addTrajectory(const API::Trajectory& T);
  
  /**
   * Test if the perturbation is valid
   */
  bool testPerturb( confPtr_t q_new, std::vector<Node*>& vect_conf );
  
  /**
   * Find the best cycle in the graph
   */
  bool findBestCycle( confPtr_t q_new, double dist, std::vector<Node*>& vect_conf );
  
  /**
   * Add a perturbation to the graph
   */
  bool addPerturbation( confPtr_t q_rand );
  
  /**
   *
   */
  bool expandPerturbation( confPtr_t q_rand );
  
  
  /**
   * Get the expansion direction
   */
  confPtr_t trajShootConfiguration();
  
  /**
   * Get the expansion direction
   */
  confPtr_t getExpansionConfiguration(bool sample_on_traj);
  
  /**
   * Get colest on traj
   */
  Node* getClosestOnTraj( confPtr_t q_rand );
  
  /**
   * Get colest on traj
   */
  double distToTraj( confPtr_t q_rand );
  
	/**
	 * Adds nodes to Graph
	 */
	virtual void expandOneStep();
	
private:
	/**
	 * Members
	 */
  Node* m_main_compco;
  double m_delta;
  bool m_descent;
  bool m_sampled_on_traj;
  double m_max_dist_to_traj;
  int m_K_Nearest;
  
  bool m_use_rejection;
  double m_std_dev_trans;
  double m_std_dev_rot;
  double m_radian_steps;
  double m_transl_steps;
  double m_transl_max;
  
  confPtr_t m_qi;
  confPtr_t m_qf;
  
  API::CostOptimization* m_traj;
};

#endif

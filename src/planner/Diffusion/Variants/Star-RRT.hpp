/*
 *  Star-RRT.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 31/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef STAR_RRT_HPP_
#define STAR_RRT_HPP_

/**
 * @ingroup Diffusion
 *
 * This class implements a RRT that stays under a given threshold of cost
 */

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/Variants/Threshold-RRT.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/cspace.hpp"
#include "API/Roadmap/compco.hpp"
/**
 @ingroup Diffusion
 */
class StarExpansion : public RRTExpansion 
{
public:
	/**
	 * Constructors with default values
	 */
	StarExpansion(Graph* G);
  
  /**
   * Set the init and goal
   */
  void setInitAndGoal( confPtr_t q_init, confPtr_t q_goal );
  
  /**
   * Set the CSpace distance
   */
  void initCSpace();
  
  /**
   * Set the CSpace distance
   */
  void initStomp();
	
	/**
	 * Destructor
	 */
	~StarExpansion();
	
	/**
	 * Checks that the localpath 
	 * between epansionNode and directionConfig can expanded in the costspace
	 */
	bool expandToGoal(Node* expansionNode, 
										std::tr1::shared_ptr<Configuration> directionConfig);
  
  /**
   * Sample in tube
   */
  confPtr_t sampleInTube();
  
  /**
   * Get the extepansion direction
   */
  confPtr_t getExpansionDirection(Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode);
	
	/**
	 * Connect expansion method
	 */
	int connectExpandProcess(Node* expansionNode, 
													 std::tr1::shared_ptr<Configuration> directionConfig, 
													 Node* directionNode);
	
	/**
	 * Extend expansion method
	 */
	int extendExpandProcess(Node* expansionNode, 
													std::tr1::shared_ptr<Configuration> directionConfig, 
													Node* directionNode);
  
  /**
   * Set the initial compco
   */
  void setInitialCompco( ConnectedComponent* compco );
  
  /**
   * Get the rgg ball raduis
   */
  double rrgBallRadius();
  
  /**
   *
   */
  void rewireGraph(Node* new_node, Node* min_node, const std::vector<Node*>& neigh_nodes);
	
	/** 
	 * expandProcess 
	 *
	 * checks the validity of the local path in one direction and adds nodes 
	 * to the trees with a different behaviour depending on the method variable
	 *
	 * @param expansionNode
	 * @param directionConfig
	 * @param directionNode
	 * @param method
	 *
	 * @return the number of nodes created
	 */
	int expandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode,
										Env::expansionMethod method);
	
private:
  
  bool m_goal_bias;
  bool m_start_bias;
  bool m_tube_bias;
  int m_Iteration;
  int m_biasRatio;
  std::vector<std::vector<confPtr_t> > m_biasTrajectory;
  int m_ith_on_traj;
  int m_ith_trajectory;
  
  int m_nb_rewiring;
	int m_K_Nearest;
  double m_RrgRadiusFactor;
  double m_step;
  
  ConnectedComponent* m_compco;
  
  // cspace
  CSpace* m_cspace;
  
  
  // init and goal configurations
  confPtr_t m_q_init;
  confPtr_t m_q_goal;
  
};


class StarRRT : public RRT
{
	
public:
	/** 
	 * Constructor from a WorkSpace object
	 * @param WS the WorkSpace
	 */
	StarRRT(Robot* R, Graph* G);
	
	/** 
	 * Destructor
	 */
	virtual ~StarRRT();
		
	/**
	 * Initialzation of the plannificator
	 * @return the number of node added during the init phase
	 */
	virtual int init();
  
  /**
   * This function prunes the tree from the configuration
   */
  void pruneTreeFromNode(Node* node); 
  
  /**
   * Exctracts the trajectory if it exists
   * this function is called when a connection to goal is found
   */
  void extractTrajectory();
  
  /**
   * save convergence to file
   */
  void saveConvergenceToFile();

	/**
	 * Connects the two connected components
   * if the seconed is within a certain reach and there exists
   * a collision free path
	 */
	bool connectNodeToCompco(Node* N, Node* CompNode);
  
private:
  
  std::vector< std::pair<double,double> > m_convergence_rate;

};

#endif
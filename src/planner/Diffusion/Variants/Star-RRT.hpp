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
   * Set the CSpace distance
   */
  void initCSpace();
	
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
  int m_nb_rewiring;
	int m_K_Nearest;
  double m_RrgRadiusFactor;
  double m_step;
  // cspace
  CSpace* m_cspace;
	
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
	 * TODO
	 */
	bool connectNodeToCompco(Node* N, Node* CompNode);
};

#endif
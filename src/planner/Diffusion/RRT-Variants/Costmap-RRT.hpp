/*
 *  Costmap-RRT.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef COSTMAP_RRT_HPP
#define COSTMAP_RRT_HPP

/**
 * @ingroup Diffusion
 *
 * This class implements a RRT that stays under a given threshold of cost
 */

#include "RRT.hpp"
#include "Expansion/CostmapExpansion.hpp"

class CostmapRRT : public RRT
{
public:
	/** 
	 * Constructor from a WorkSpace object
	 * @param WS the WorkSpace
	 */
	CostmapRRT(Robot* R, Graph* G);
	
	/** 
	 * Destructor
	 */
	virtual ~CostmapRRT();
	
	/**
	 * Set the threshold
	 */
	void setLastBestPath( double pathCost ) 
	{ 
		dynamic_cast<CostmapExpansion*>(_expan)->setLastBestPath(pathCost); 
	}
	
	/**
	 * Get the threshold
	 */
	double getLastBestPath() 
	{ 
		return dynamic_cast<CostmapExpansion*>(_expan)->getLastBestPath(); 
	}
	
	/**
	 * Initialzation of the plannificator
	 * @return the number of node added during the init phase
	 */
	virtual int init();
	
	/**
	 * Function called to set the
	 * tree variables when connecting to the final node
	 */
	void addFinalNode(Node* node, Node* final);
	
	/**
	 * costConnectNodeToComp
	 * Try to connect a node to a given component
	 * taking into account the cost space
	 *
	 * @param: node a node frome the graph
	 * @param: compNode is a connected component form the graph
	 * @return: true if the node and the componant have been connected.
	 */
	virtual bool connectNodeToCompco(Node* node, Node* compNode);
	
};

#endif

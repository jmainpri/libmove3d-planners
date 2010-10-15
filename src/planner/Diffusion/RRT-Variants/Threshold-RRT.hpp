/*
 *  ThresholdRRT.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef THRESHOLD_RRT_HPP_
#define THRESHOLD_RRT_HPP_

/**
 * @ingroup Diffusion
 *
 * This class implements a RRT that stays under a given threshold of cost
 */

#include "RRT.hpp"
#include "Expansion/ThresholdExpansion.hpp"

class ThresholdRRT : public RRT
{
	
public:
	/** 
	 * Constructor from a WorkSpace object
	 * @param WS the WorkSpace
	 */
	ThresholdRRT(Robot* R, Graph* G);
	
	/** 
	 * Destructor
	 */
	virtual ~ThresholdRRT();
	
	/**
	 * Set the threshold
	 */
	void setThreshold(double thresh) { dynamic_cast<ThresholdExpansion*>(_expan)->setThreshold(thresh); }
	
	/**
	 * Get the threshold
	 */
	double getThreshold() { return dynamic_cast<ThresholdExpansion*>(_expan)->getThreshold(); }
	
	/**
	 * Initialzation of the plannificator
	 * @return the number of node added during the init phase
	 */
	virtual int init();
	
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
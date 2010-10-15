/*
 *  Star-RRT.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 31/07/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
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
#include "Expansion/ThresholdExpansion.hpp"

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
	 * TODO
	 */
	bool connectNodeToCompco(Node* N, Node* CompNode);
};

#endif
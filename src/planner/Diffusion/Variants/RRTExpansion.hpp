/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef RRTEXPANSION_HPP
#define RRTEXPANSION_HPP

#include "planner/Diffusion/Variants/BaseExpansion.hpp"

namespace Move3D
{

/**
  @ingroup Diffusion
  * RRT Expansion Methods
  *
  * Functions to expand a tree in the CSpace
  * Collision Checking, Random sampling and Biasing
  */
class RRTExpansion: public BaseExpansion
{

public:

    RRTExpansion();
    RRTExpansion(Graph* prtGraph);
    ~RRTExpansion();


    /**
     * Shoots a direction (includes the biasing)
     *
     * @param Expanding component
     * @param Goal Component
     * @param Sampling passive mode
     * @param Direction node
     */
    virtual confPtr_t getExpansionDirection( Node* expandComp, Node* goalComp, bool samplePassive,Node*& directionNode);

    /**
     * Gets the nearest node in the graph
     *
     * @param compNode Expanding component
     * @param direction Direction of expansion
     * @param Sampling passive mode
     */
    virtual Node* getExpansionNode( Node* compNode, confPtr_t direction, int distance);
	
	/**
     * Expands towards the goal
     *
     * @param expansionNode     Node expanded
     * @param directionConfig   Direction
     */
    virtual bool expandToGoal( Node* expansionNode, confPtr_t directionConfig);

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
    virtual unsigned expandProcess(Node* expansionNode, confPtr_t directionConfig, Node* directionNode, Env::expansionMethod method);

};

}

#endif // RRTEXPANSION_HPP

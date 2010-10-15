/*
 *  Multi-TRRT.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 09/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef MULTITRANSITIONRRT_HPP_
#define MULTITRANSITIONRRT_HPP_

#include "planner/Diffusion/RRT-Variants/Multi-RRT.hpp"

/**
 @ingroup Diffusion
 */
class MultiTRRT : public MultiRRT 
{	
public:
	
    MultiTRRT(Robot* R, Graph* G);
	
    ~MultiTRRT();
	
    virtual int init();
	
	void initalizeRoot(Node* rootNode);
	
    /**
	 * costConnectNodeToComp
	 * Try to connect a node to a given component
	 * taking into account the fact that the space
	 * is a cost space
	 * @return: TRUE if the node and the componant have
	 * been connected.
	 */
    bool connectNodeToCompco(Node* N, Node* CompNode);
	
};

#endif /* MULTITRANSITIONRRT_HPP_ */
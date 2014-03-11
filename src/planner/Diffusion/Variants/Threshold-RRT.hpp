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

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/Variants/RRTExpansion.hpp"

namespace Move3D {

/**
 @ingroup Diffusion
 */
class ThresholdExpansion : public RRTExpansion {
	
public:
	/**
	 * Constructors with default values
	 */
	ThresholdExpansion();
	ThresholdExpansion(Graph* G);
	
	/**
	 * Destructor
	 */
	~ThresholdExpansion();
	
	/**
	 * Set threshold
	 */
	void setThreshold(double thresh) { m_threshold = thresh; }
	
	/**
	 * Get Threshold
	 */
	double getThreshold() { return m_threshold; }
	
	
	/**
	 * Verifies that the localpath 
	 * between epansionNode and directionConfig can expanded in the cost space
	 */
	bool expandToGoal(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig);
	
	/**
	 * Connect expansion method
	 */
	int connectExpandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, Node* directionNode);
	
	/**
	 * Extend expansion method
	 */
	int extendExpandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, Node* directionNode);
	
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
	unsigned expandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, Node* directionNode,
                    Env::expansionMethod method);
	
private:
	double m_threshold;
	
};

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
	virtual unsigned init();
	
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

}

#endif

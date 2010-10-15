/*
 *  ThresholdExpansion.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef THRESHOLD_EXPANSION_HPP_
#define THRESHOLD_EXPANSION_HPP_

#include "RRTExpansion.hpp"

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
	bool expandToGoal(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig);
	
	/**
	 * Connect expansion method
	 */
	int connectExpandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode);
	
	/**
	 * Extend expansion method
	 */
	int extendExpandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode);
	
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
	double m_threshold;
	
};

#endif /* TRANSTIONEXPANSION_HPP_ */

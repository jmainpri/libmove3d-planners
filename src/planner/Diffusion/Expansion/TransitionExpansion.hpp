/*
 * TranstionExpansion.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TRANSTIONEXPANSION_HPP_
#define TRANSTIONEXPANSION_HPP_

#include "RRTExpansion.hpp"

/**
  @ingroup Diffusion
  */
class TransitionExpansion : public RRTExpansion {

public:
	TransitionExpansion();
	TransitionExpansion(Graph* G);

	~TransitionExpansion();
	
	/**
	 * Compute the gradient of the 2D cost sapce at a configuration q
	 */
	std::tr1::shared_ptr<Configuration> computeGradient(std::tr1::shared_ptr<Configuration> q);
	
	/**
	 * Mix the direction with the gradient 
	 * direction
	 */
	bool mixWithGradient(Node* expansionNode,std::tr1::shared_ptr<Configuration> directionConfig);

   /**
	* Shoots a direction (includes the biasing)
    *
    * @param Expanding component
    * @param Goal Component
    * @param Sampling passive mode
    * @param Direction node
    */
	virtual std::tr1::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive,
            Node*& directionNode);


	/**
	 *
	 */
	bool costTestSucceeded(Node* previousNode, std::tr1::shared_ptr<
			Configuration> currentConfig, double currentCost);

	/**
	 *
	 */
	bool costTestSucceededConf(
			std::tr1::shared_ptr<Configuration>& previousConfig,
			std::tr1::shared_ptr<Configuration>& currentConfig);

	/**
	 *
	 */
	bool transitionTest(Node& fromNode,LocalPath& extensionLocalpath);

	/**
	 *
	 */
	bool expandToGoal(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig);
	/**
	 *
	 */
	void adjustTemperature(bool accepted, Node* node);
	
	/**
	 *
	 */
	bool expandCostConnect(Node& expansionNode, std::tr1::shared_ptr<Configuration> directionConfig,
			Node* directionNode, bool toGoal);
	
	/**
	 *
	 */
	int extendExpandProcess(Node* expansionNode,
						 std::tr1::shared_ptr<Configuration> directionConfig,
						 Node* directionNode);
	

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
	int expandProcess(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode,
			Env::expansionMethod method);

};

#endif /* TRANSTIONEXPANSION_HPP_ */

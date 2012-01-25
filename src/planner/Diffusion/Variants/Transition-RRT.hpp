/*
 * Transition-RRT.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TRANSITIONRRT_HPP_
#define TRANSITIONRRT_HPP_

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/Variants/RRTExpansion.hpp"

/**
 @ingroup Diffusion
 */
class TransitionExpansion : public RRTExpansion {
  
public:
	TransitionExpansion();
	TransitionExpansion(Graph* G);
  
	~TransitionExpansion();
	
  void setInitAndGoalNodes( Node* init, Node* goal );
  
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
  
private:
  Node* m_initNode;
  Node* m_goalNode;
  
};

/**
 @ingroup Diffusion
 */
class TransitionRRT : public RRT {
	
public:
	
	TransitionRRT(Robot* R, Graph* G);
	
	~TransitionRRT();
	
	/**
	 * Initialize the Transition RRT variables
	 */
	virtual int init();
	
	/**
	 * Set the node cost
	 */
	void setNodeCost(Node* node);
	
	/**
	 * costConnectNodeToComp
	 * Try to connect a node to a given component
	 * taking into account the fact that the space
	 * is a cost space
	 * @return: TRUE if the node and the componant have
	 * been connected.
	 */
	virtual bool connectNodeToCompco(Node* N, Node* CompNode);
	
};

#endif /* TRANSITIONRRT_HPP_ */

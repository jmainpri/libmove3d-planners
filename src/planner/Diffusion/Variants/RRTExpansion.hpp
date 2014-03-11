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

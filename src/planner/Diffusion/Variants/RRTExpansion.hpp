#ifndef RRTEXPANSION_HPP
#define RRTEXPANSION_HPP

#include "planner/Diffusion/Variants/BaseExpansion.hpp"

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
    virtual MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive,Node*& directionNode);

    /**
     * Gets the nearest node in the graph
     *
     * @param compNode Expanding component
     * @param direction Direction of expansion
     * @param Sampling passive mode
     */
    virtual Node* getExpansionNode(
            Node* compNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> direction, int distance);
	
	/**
     * Expands towards the goal
     *
     * @param expansionNode     Node expanded
     * @param directionConfig   Direction
     */
    virtual bool expandToGoal(
			Node* expansionNode,MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig);

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
    virtual unsigned expandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig,
                       Node* directionNode,
                       Env::expansionMethod method);

};

#endif // RRTEXPANSION_HPP

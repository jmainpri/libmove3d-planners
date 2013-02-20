/*
 * TreeExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_TREE_EXPANSION_HPP
#define P3D_TREE_EXPANSION_HPP

#include "planner/Diffusion/Variants/RRTExpansion.hpp"

/**
  @ingroup Diffusion
  * Tree Expansion Methods
  *
  * Functions to expand a tree in the CSpace
  * Collision Checking, Random sampling and Biasing
  */
class ESTExpansion: public RRTExpansion
{

public:

    ESTExpansion();
    ESTExpansion(Graph* prtGraph);

    ~ESTExpansion();


    /**
     * EST Special case
     */
    Node* getExpansionNode(std::vector<Node*>& nodes);

    /**
     * EST Special case
     */
    MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> getExpansionDirection(Node* expansionNode,Node* toComp);

    /**
     * EST Special case
     */
    Node* expandProcessEST( Node* expansionNode,
                            MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig,
                            int& nbOfNodesAdded);

    /**
     * EST
     */
    void printAllNodes(std::vector<Node*>& nodes);


};

#endif

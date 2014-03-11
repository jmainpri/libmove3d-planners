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
namespace Move3D {

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
    confPtr_t getExpansionDirection( Node* expansionNode, Node* toComp );

    /**
     * EST Special case
     */
    Node* expandProcessEST( Node* expansionNode, confPtr_t, int& nbOfNodesAdded );

    /**
     * EST
     */
    void printAllNodes(std::vector<Node*>& nodes);
};

}

#endif

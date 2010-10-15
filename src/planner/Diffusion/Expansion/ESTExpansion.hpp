/*
 * TreeExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_TREE_EXPANSION_HPP
#define P3D_TREE_EXPANSION_HPP

#include "RRTExpansion.hpp"

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
    std::tr1::shared_ptr<Configuration> getExpansionDirection(Node* expansionNode,Node* toComp);

    /**
     * EST Special case
     */
    Node* expandProcessEST( Node* expansionNode,
                            std::tr1::shared_ptr<Configuration> directionConfig,
                            int& nbOfNodesAdded);

    /**
     * EST
     */
    void printAllNodes(std::vector<Node*>& nodes);


};

#endif

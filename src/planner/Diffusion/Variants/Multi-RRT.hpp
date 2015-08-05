/*
 *  Multi-RRT.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 09/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef MULTI_RRT_HPP_
#define MULTI_RRT_HPP_

#include "planner/Diffusion/RRT.hpp"

/**
 @ingroup Diffusion
 */
namespace Move3D
{

class MultiRRT : public RRT {

public:

    MultiRRT(Robot* R, Graph* G);
    ~MultiRRT();

    unsigned init();

    /**
     * Adds a Seed to the algorithm
     */
    bool addSeed(confPtr_t q);

    /**
   * Get random compco for expansion
   */
    Node* getRandomCompcoForExpansion(Node* fromNode);

    /**
     * Returns the least expanded component
     */
    Node* getInitCompco();

    /**
     * Returns a Goal Compco
     */
    Node* getGoalCompco();

    /**
     * Main function of multi RRT
     */
    unsigned int run();

protected:
    std::vector<Node*> m_Roots;

    Node* m_initNode;
    Node* m_goalNode;
};

}

#endif /* MULTI_RRT_HPP_ */

/*
 *  Multi-TRRT.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 09/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef MULTITRANSITIONRRT_HPP_
#define MULTITRANSITIONRRT_HPP_

#include "planner/Diffusion/Variants//Multi-RRT.hpp"

namespace Move3D {

extern std::vector<confPtr_t> multi_rrt_configs;

/**
 @ingroup Diffusion
 */
class MultiTRRT : public MultiRRT 
{	
public:

    MultiTRRT(Robot* R, Graph* G);

    ~MultiTRRT();

    virtual unsigned init();
    void initalizeRoot(Node* rootNode, int tree);
    Node* closestNeighInTree(Node* N, int tree);
    bool connectNodeToCompco(Node* N, Node* CompNode);
    bool testAddEdge(Node* source, Node* target);
    void extractTrajectory();

    unsigned int run();

    std::vector< std::vector<Node* > >& getTrees() { return m_trees; }

private:

    int                                 m_tree_id;
    std::vector< confPtr_t >            m_seeds;
    std::vector< double >               m_temperature;
    std::vector< std::vector<Node* > >  m_trees;
    Move3D::Trajectory*                 m_current_traj;
};

/**
 @ingroup Diffusion
 */
class MultiTransitionExpansion : public RRTExpansion {

public:
    MultiTransitionExpansion();
    MultiTransitionExpansion(Graph* G);

    ~MultiTransitionExpansion() { }

    bool costTestSucceeded(Node* previousNode, double currentCost, double temperature);
    bool transitionTest(Node& fromNode,LocalPath& extensionLocalpath);
    void adjustTemperature(bool accepted, Node* node, double& temperature );
    bool expandToGoal(Node* expansionNode, confPtr_t directionConfig);

    Node* getExpansionNode( Node* compNode, confPtr_t direction, int distance);

    unsigned expandProcess(Node* expansionNode, MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig, Node* directionNode,
                           Env::expansionMethod method);

    MultiTRRT* rrt;
    int m_tree_id;
    double m_temperature;
};

}

#endif /* MULTITRANSITIONRRT_HPP_ */

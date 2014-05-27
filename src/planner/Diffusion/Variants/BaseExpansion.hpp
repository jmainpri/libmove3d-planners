/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
/*
 * BaseExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_BASE_EXPANSION_HPP
#define P3D_BASE_EXPANSION_HPP

#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

#include <libmove3d/p3d/env.hpp>

namespace Move3D {

#ifndef GRAPH_HPP
class Graph;
#endif

#ifndef NODE_HPP
class Node;
#endif

/**
 * @ingroup Diffusion
 *
 * The expansion class holds the method and local variables
 * allowing to expand a Tree in a given configuration space
 * methods such as chose a direction, check the validity in that direction and
 * add nodes to the tree
 */
class BaseExpansion 
{
public:

    /**
     * Constructor
     */
    BaseExpansion();
    BaseExpansion(Graph* prtGraph);

    /**
     * Destructor
     */
    ~BaseExpansion();

    /**
     * Set the graph that is beeing expanded
     */
    void setGraph(Graph* G) { m_Graph = G; }

    /**
     * Get the graph that is beeing expanded
     */
    Graph* getGraph() { return m_Graph; }

    /**
     * Get Node Expansion Method
     */
    int getDirectionMethod() { return m_ExpansionDirectionMethod; }

    /**
     * Set Expansion node Method
     */
    void setDirectionMethod(int directionExpansion) { m_ExpansionDirectionMethod = directionExpansion; }

    /**
     * Get Node Expansion Method
     */
    int getNodeMethod() { return m_ExpansionNodeMethod; }

    /**
     * Set Expansion node Method
     */
    void setNodeMethod(int nodeExpansion) { m_ExpansionNodeMethod = nodeExpansion; }

    /**
     * Set the From Connected Component
     */
    void setFromComp(Node* fromComp) { m_fromComp = fromComp; }

    /**
     * Get the From Connected Component
     */
    Node* getFromComp() { return m_fromComp; }

    /**
     * Set the To Connected Component
     */
    void setToComp(Node* toComp) { m_toComp = toComp; }

    /**
     * Get the To Connected Component
     */
    Node* getToComp() { return m_toComp; }

    /**
     * Expansion Step (Delta)
     */
    double step();

    /**
     * Position on localpath in [0,1]
     */
    double positionAlongPath(LocalPath& path, double param);

    /**
     * Return the path parameter that is
     * the closest to step()
     */
    double pathDelta(LocalPath& path);

    /**
     * Computes a localpath parameter used for expansion
     * of at max step() of length
     */
    pathPtr_t getExtensiontPath( confPtr_t qi, confPtr_t qf, double max_param = -1.0 );

    /**
     * Function called when a node can not be connected
     * @param the node which has not been connected
     */
    void expansionFailed( Node& node );

    /**
     * Adds a node to a connected component
     */
    virtual Node* addNode( Node* currentNode, LocalPath& path, double pathDelta,
                           Node* directionNode, int& nbCreatedNodes);

    /**
     * Function that balances the ratio of
     * Exploration towards refinement
     */
    bool expandControl( LocalPath& path, Node& compNode );

    /**
     * Returns a valid configuration on the local path
     */
    bool nextStep( LocalPath& path, confPtr_t& directionConfig, double& pathDelta, pathPtr_t& newPath,
                  Env::expansionMethod method);

    /**
     * Returns a valid configuration on the local path
     */
    bool nextStep( LocalPath& path, Node* directionNode, double& pathDelta, pathPtr_t& newPath,
                  Env::expansionMethod method);

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
    virtual unsigned expandProcess( Node* expansionNode, confPtr_t directionConfig,
                                    Node* directionNode, Env::expansionMethod method) = 0;

    /**
   * Return last added node
   */
    Node* getLasAddedNode() { return m_last_added_node; }

protected:

    int m_ExpansionNodeMethod;
    int m_MaxExpandNodeFailure;
    int m_kNearestPercent;

    int m_ExpansionDirectionMethod; // = GLOBAL_CS_EXP;
    //	double GoalBias; //= 0.1;
    //	bool IsGoalBias; //= FALSE;
    bool m_IsDirSampleWithRlg; //= FALSE;

    Graph* m_Graph;

    Node* m_last_added_node;

    Node* m_fromComp;
    Node* m_toComp;
};

}

#endif

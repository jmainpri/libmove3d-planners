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
 * TreePlanner.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TREEPLANNER_HPP_
#define TREEPLANNER_HPP_

#include "planner/planner.hpp"

/**
  @ingroup NEW_CPP_MODULE
  @defgroup Diffusion
  @brief Tree planner module
  \image html RRT_graph2.png
  */

namespace Move3D
{

/**
  @ingroup Diffusion
  */
class TreePlanner : public Planner {

public:
    /**
     * Constructor
     */
    TreePlanner(Robot* R, Graph* G);

    /**
     * Destructor
     */
    ~TreePlanner();

    /**
     * Initializes Planner
     */
    virtual unsigned init();

    /**
     * Checks out the Stop condition
     */
    virtual bool checkStopConditions();

    /**
     * Checks out the Pre-conditions
     * - start and goal are not in collision
     * - start and goal are different configurations
     */
    virtual bool preConditions();

    /**
     * Tries to connect a node to the other
     * connected component of the graph
     *
     * @param currentNode The node that will be connected
     * @param ComNode The Connected Component
     */
    virtual bool connectNodeToCompco(Node* N, Node* CompNode);

    /**
     * Main function to connect to the other Connected Component
     */
    virtual bool connectionToTheOtherCompco(Node* toNode);

    /**
     * Expands tree from component fromComp
     * to component toComp
     * @param fromComp the starting connex component
     * @param toComp the arriving connex component
     * @return the number of node created
     */
    virtual int expandOneStep(Node* fromComp, Node* toComp) = 0 ;

    /**
     * Main function of the Tree process
     * @return the number of Nodes added to the Graph
     */
    virtual unsigned int run();

    /**
   * Extract trajectory
   */
    virtual void extractTrajectory() { }

    /**
     * Returns number of consecutive failure
     * during plannification
     */
    unsigned int getNumberOfConsecutiveFail()
    {
        return m_nbConscutiveFailures;
    }

    /**
     * Returns number of expansion
     * during plannification
     */
    unsigned int getNumberOfExpansion()
    {
        return m_nbExpansion;
    }

    /**
     * Returns number of expansion failure
     * during plannification
     */
    unsigned int getNumberOfFailedExpansion()
    {
        return m_nbFailedExpansion;
    }

    /**
     * Returns number the initial number of nodes
     * of plannification
     */
    unsigned int getNumberOfInitialNodes()
    {
        return m_nbInitNodes;
    }

    /**
     * Returns the last node added to the graph
     */
    Node* getLastNode()
    {
        return m_last_node;
    }

protected:

    unsigned int m_nbConscutiveFailures;
    unsigned int m_nbExpansion;
    unsigned int m_nbFailedExpansion;
    unsigned int m_nbInitNodes;

    Node* m_last_node;
};

}

#endif /* TREEPLANNER_HPP_ */

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
#ifndef GRAPHSTATE_HPP
#define GRAPHSTATE_HPP

#include "API/Search/AStar/State.hpp"

#include <libmove3d/include/Planner-pkg.h>
/**
  * @ingroup CPP_API
  * @defgroup SEARCH Graph search
  * @brief Astar and Dijsktra
  */

/**
   * @ingroup SEARCH
   * @brief Graph state interface for the AStar class
   */

class GraphState : public Move3D::State
{
public:
    GraphState();
    GraphState(p3d_node* n);

    std::vector<State*> getSuccessors();

    p3d_node* getGraphNode() { return _GraphNode; }

    bool equal(State* other);
    bool isLeaf();
    void print();

    void setClosed(std::vector<State*>& closedStates,std::vector<State*>& openStates);
    bool isColsed(std::vector<State*>& closedStates);

    void setOpen(std::vector<State*>& openStates);
    bool isOpen(std::vector<State*>& openStates);

protected:
    double computeLength(Move3D::State *parent);       /* g */
    double computeHeuristic(Move3D::State *parent);    /* h */

private:
    p3d_node* _GraphNode;
};

#endif // GRAPHSTATE_HPP

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
 * EST.hpp
 *
 *  Created on: Sep 16, 2009
 *      Author: jmainpri
 */

#ifndef EST_HPP_
#define EST_HPP_

#include "planner/Diffusion/Variants/ESTExpansion.hpp"
#include "planner/Diffusion/TreePlanner.hpp"

/**
  @ingroup Diffusion
 * Expansive State Space Tree Planner
 */
namespace Move3D
{

class EST: public TreePlanner
{

public:
    /**
     * Constructor from a WorkSpace object
     * @param WS the WorkSpace
     */
    EST(Robot* R, Graph* G);

    /**
     * Destructor
     */
    ~EST();

    /**
     * Initialzation of the plannificator
     * @return the number of node added during the init phase
     */
    virtual unsigned init();

    /**
     * Checks out the Stop condition
     */
    bool checkStopConditions();

    /**
     * Checks out the preconditions
     */
    bool preConditions();

    /**
     * Add node to sorted set
     */
    void addNodeToSet(Node* extentionNode);

    /**
     * Three phases One Step Expansion
     *  - Direction
     *  - Node
     *  - Process
     *
     *  @param fromComp the component which is expanded
     *  @param toComp the goal component
     */
    virtual int expandOneStep(Node* fromComp, Node* toComp);

    /**
     * EST Connect node using Cost
     */
    bool connectNodeToCompco(Node* node, Node* compNode);

    /**
    * Returns number of consecutive failure
    * during plannification
    */
    ESTExpansion* getExpansion()
    {
        return _Expan;
    }

protected:
    ESTExpansion* _Expan;
    std::vector<Node*> _SortedNodes;
};

}

#endif /* EST_HPP_ */

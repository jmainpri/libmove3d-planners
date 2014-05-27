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
#ifndef RRT_HPP
#define RRT_HPP

#include "planner/Diffusion/Variants/RRTExpansion.hpp"
#include "planner/Diffusion/TreePlanner.hpp"

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

/**
 * @ingroup Diffusion
 *
 * This class implements the following RRT algorithms:
 * RRT, T-RRT and ML-RRT.
 * The expansion can be mono- or bi-directional,
 * with or without a goal.
 * The possible expansion methods are:
 * "extend", "extend n steps" and "connect".
 * There are some restrictions on the use of those methods:
 * connect cannot be used with T-RRT,
 * while ML-RRT should be used with connect.
 */

namespace Move3D
{

class RRT: public TreePlanner
{

public:
    /** Constructor from a WorkSpace object
     * @param WS the WorkSpace
     */
    RRT(Robot* R, Graph* G);

    /**
     * Destructor
     */
    ~RRT();

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
     * Shoots a new configuration randomly at a fix step
     * @param qCurrent la Configuration limitant la distance
     * @return la Configuration tirée
     */
    confPtr_t diffuseOneConf(confPtr_t qCurrent)
    {
        LocalPath path( qCurrent, _Robot->shoot());
        return path.configAtParam( std::min(path.length(), _expan->step()) );
    }

    /**
     * Returns number of consecutive failure
     * during plannification
     */
    RRTExpansion* getExpansion()
    {
        return _expan;
    }


protected:

    RRTExpansion* _expan;

};

}

#endif

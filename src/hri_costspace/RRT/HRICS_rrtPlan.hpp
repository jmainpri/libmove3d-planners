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
#ifndef HRICS_RRTPLAN_H
#define HRICS_RRTPLAN_H

#include "planner/Diffusion/RRT.hpp"
#include "../grid/HRICS_two_d_grid.hpp"

namespace HRICS {


/**
  @ingroup HRICS
  @brief Special RRT implentation for the HRICS
  */
class HRICS_RRTPlan : public Move3D::RRT
{
public:
    HRICS_RRTPlan( Move3D::Robot* R, Move3D::Graph* G );

    /**
      * Sets the grid pointer
      */
    void setGrid( HRICS::PlanGrid* G );

    /**
      * Sets the cell path
      */
    void setCellPath(std::vector<Move3D::TwoDCell*> cellPath);

   /**
     * Initialzation of the plannificator
     * @return the number of node added during the init phase
     */
    virtual unsigned init();

    /**
      * Intents to connect a node to the compco
      */
    bool connectNodeToCompco( Move3D::Node* node, Move3D::Node* compNode);

    /**
      * Finds the nearest neighbour in the cell
      * @return Other wise returns NULL
      */
    Move3D::Node* nearestNeighbourInCell( Move3D::Node* node, std::vector<Move3D::Node*> neigbour);

    /**
      * @return the cell in which is the given node
      */
    Move3D::TwoDCell* getCellFromNode( Move3D::Node* node );

 private:
    HRICS::PlanGrid* mGrid;

};

}

#endif // HRICS_RRT_H

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
#ifndef HRICS_RRTEXPANSION_H
#define HRICS_RRTEXPANSION_H

#include "Diffusion/Variants/Transition-RRT.hpp"
#include "hri_costspace/grid/HRICS_grid.hpp"

namespace HRICS
{

/**
  @ingroup HRICS
  @brief Special RRT Expansion method for the HRICS
  */
class HRICS_rrtExpansion : public Move3D::TransitionExpansion
{
public:
    HRICS_rrtExpansion();
    HRICS_rrtExpansion(Move3D::Graph* G);

    /**
      * Initializes some variables for the expansion
      * method
      */
    void init();

     /**
      * Sets the grid
      */
    void setGrid(Move3D::ThreeDGrid* grid) { _3DGrid = dynamic_cast<HRICS::Grid*>(grid); }

     /**
      * Sets the cell path
      */
    void setCellPath(std::vector<Move3D::ThreeDCell*> cellPath);

    /**
      * Direction used in RRT one step
      */
    Move3D::confPtr_t getExpansionDirection( Move3D::Node* expandComp, Move3D::Node* goalComp, bool samplePassive, Move3D::Node*& directionNode);

    /**
      * Configuration from the next cell along the 3dPath
      */
    Move3D::confPtr_t getConfigurationInNextCell(Move3D::Node* node);

    /**
      * Adds a node to a conected component
      */
    Move3D::Node* addNode(Move3D::Node* currentNode, Move3D::LocalPath& path, double pathDelta, Move3D::Node* directionNode, int& nbCreatedNodes);

    /**
      * Checks it the cell is after the given cell on the
      * 3D path
      */
    bool on3DPathAndAfter(Move3D::ThreeDCell* cell);

private:
    HRICS::Grid*             _3DGrid;
    std::vector<Move3D::ThreeDCell*>  _3DCellPath;

    Move3D::ThreeDCell*               _LastForward;
    Move3D::ThreeDCell*               _LastBackward;

    int         mIndexObjectDof;

    bool                     _forward;
    bool                     _biasing;

    double*                  _Box;

};

}

#endif // HRICS_RRTEXPANSION_H

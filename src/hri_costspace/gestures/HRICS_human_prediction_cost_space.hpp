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
#ifndef HRICS_WS_OCCUPANCYCOSTSPACE_HPP
#define HRICS_WS_OCCUPANCYCOSTSPACE_HPP

#include "HRICS_workspace_occupancy.hpp"

double HRICS_getPredictionOccupancyCost(Move3D::Configuration& q);

namespace HRICS
{

class HumanPredictionCostSpace {

public:
    HumanPredictionCostSpace( Move3D::Robot* robot, WorkspaceOccupancyGrid* occup_grid);
    ~HumanPredictionCostSpace();

    void draw();

    double getCost( Move3D::Configuration& q ) const;
    double getCostFromActiveJoints( Move3D::Configuration& q ) const;
    double getCurrentOccupationCost( Move3D::Configuration& q ) const;

    void setActiveJoints( const std::vector<Move3D::Joint*>& active_joints ) { m_active_joints = active_joints; }

    void computeCurrentOccupancy() { m_ws_occupancy->computeCurrentOccupancy(); }

private:
    void draw_sampled_points();
    void sampleRobotPoints();
    void setActiveJoints();

    Move3D::Robot* m_robot;
    WorkspaceOccupancyGrid* m_ws_occupancy;
    Move3D::BodySurfaceSampler* m_surface_sampler;
    std::vector<Move3D::Joint*> m_active_joints;
};

}

extern HRICS::HumanPredictionCostSpace* global_humanPredictionCostSpace;

#endif // HRICS_WS_OCCUPANCYCOSTSPACE_HPP

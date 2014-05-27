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
#include "HRICS_detours.hpp"

#include "planner/planEnvironment.hpp"
#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include <boost/bind.hpp>

#include "Graphic-pkg.h"

using namespace Move3D;
using namespace HRICS;
using std::cout;
using std::endl;

Detours::Detours()
{
    robot_ = global_Project->getActiveScene()->getActiveRobot();

    if( global_DrawModule )
    {
        global_DrawModule->addDrawFunction( "Detours", boost::bind( &Detours::draw, this) );
        global_DrawModule->enableDrawFunction( "Detours" );
    }
}

Detours::~Detours()
{
    if( global_DrawModule )
    {
        global_DrawModule->deleteDrawFunction( "Detours" );
    }
}

void Detours::planAStar()
{
    AStarPlanner planner( robot_ );
    planner.set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
    planner.init();

    grid_ = planner.getGrid();

    confPtr_t q_init = robot_->getInitPos();
    confPtr_t q_goal = robot_->getGoalPos();

    Move3D::Trajectory* traj = planner.computeRobotTrajectory( q_init, q_goal );

    if( traj != NULL )
        cout << "AStar planning OK" << endl;
    else
        cout << "Error in run AStar planner" << endl;
}

void Detours::draw()
{
    cout << "DRAW DETOURS" << endl;
    const double height = 0.0;
    for( int i=0; i<grid_->getNumberOfCells(); i++ )
    {
        Eigen::Vector2d center = grid_->getCell(i)->getCenter();
        Eigen::Vector2d cell_size = grid_->getCellSize();
        g3d_draw_solid_sphere(center[0], center[1], height, cell_size[0]/5, 20);
    }
}

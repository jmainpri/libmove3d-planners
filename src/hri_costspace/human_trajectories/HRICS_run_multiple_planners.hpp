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
#ifndef HRICS_RUN_MULTIPLE_STOMP_HPP
#define HRICS_RUN_MULTIPLE_STOMP_HPP

#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

namespace HRICS
{
enum planner_t {
    stomp=0,
    astar=1,
    rrt=2 };

class MultiplePlanners
{
public:
    MultiplePlanners( Move3D::Robot* robot);
    ~MultiplePlanners() { }

    bool run();
    void multipleRun( std::string folder, int nb_runs );
    void saveTrajsToFile( std::string folder );
    void loadTrajsFromFile( std::string folder, int nb_max_files=-1 );

    void initializeNoisy();

    const std::vector<Move3D::Trajectory>& getBestTrajs() { return best_traj_; }
    const std::vector<Move3D::Trajectory>& getAllTrajs();
    void setPlannerType( planner_t planner ) { planner_type_ = planner; }
    void clearTrajs() { best_traj_.clear(); }
    void setStompInit( const Move3D::Trajectory& t ) { init_stomp_ = t; }

    void draw();

private:

    bool runStomp();
    bool runAStar();
    bool runRRT();

    Move3D::Robot* robot_;
    std::vector<Move3D::Trajectory> best_traj_;
    std::vector<Move3D::Trajectory> all_traj_;
    planner_t planner_type_;
    Move3D::Trajectory init_stomp_;
};

}

#endif // HRICS_RUN_MULTIPLE_STOMP_HPP

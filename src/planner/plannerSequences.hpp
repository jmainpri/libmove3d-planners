#ifndef PLANNERSEQUENCES_HPP
#define PLANNERSEQUENCES_HPP

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

#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

namespace Move3D
{
enum planner_t {
    stomp=0,
    astar=1,
    rrt=2 };

class SequencesPlanners
{
public:
    SequencesPlanners( Move3D::Robot* robot);
    ~SequencesPlanners() { }

    bool run();
    void multipleRun( std::string folder, int nb_runs );
    void saveTrajsToFile( std::string folder );
    void loadTrajsFromFile( const std::vector<std::string>& files );

    const std::vector<Move3D::Trajectory>& getBestTrajs() { return best_traj_; }
    const std::vector<Move3D::Trajectory>& getAllTrajs();
    void setPlannerType( planner_t planner ) { planner_type_ = planner; }
    void playTrajs() ; //const;
    void clearTrajs() { best_traj_.clear(); }
    void setStompInit( const Move3D::Trajectory& t ) { init_stomp_ = t; }

    std::vector<confPtr_t> getStoredConfig() const;
    void runSequence();
    void runSequence( const std::vector<confPtr_t>& configs );

    void draw();

private:

    bool runStomp(double duration);
    bool runAStar();
    bool runRRT();

    Move3D::Robot* robot_;
    std::vector<Move3D::Trajectory> best_traj_;
    std::vector<Move3D::Trajectory> all_traj_;
    planner_t planner_type_;
    Move3D::Trajectory init_stomp_;
};

}

#endif // PLANNERSEQUENCES_HPP

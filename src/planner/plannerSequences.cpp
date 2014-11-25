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
#include "plannerSequences.hpp"

#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"
#include "utils/misc_functions.hpp"

#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/planEnvironment.hpp"

#include "collision_space/collision_space_factory.hpp"

#include <libmove3d/include/Graphic-pkg.h>

#include <iomanip>
#include <sstream>

using namespace Move3D;
using std::cout;
using std::endl;

SequencesPlanners::SequencesPlanners(Robot* robot)
{
    // robot_ = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
    robot_ = robot;
    cout << "PLanner initialized for robot : " << robot->getName() << endl;

    //    folder_ = "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/squares";
    //    folder_ = "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/per_feature_square/ASTAR";

    planner_type_ = astar;

//    if( global_DrawModule )
//    {
//        global_DrawModule->addDrawFunction( "MultiplePlanner", boost::bind( &SequencesPlanners::draw, this) );
//        global_DrawModule->enableDrawFunction( "MultiplePlanner" );
//    }
//    else{
//        cout << "Draw module not initialized" << endl;
//    }
}


void SequencesPlanners::multipleRun( std::string folder, int nb_runs )
{
    best_traj_.clear();

    for( int i=0;i<nb_runs;i++)
        run();

    saveTrajsToFile( folder );
}

void SequencesPlanners::saveTrajsToFile( std::string folder )
{
    std::stringstream ss;

    for( int i=0; i<int(best_traj_.size()); i++ )
    {
        ss.str("");
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";

        best_traj_[i].saveToFile( folder + "/" + ss.str() );

        cout << "save planner result " << i << " : " << ss.str() << endl;
    }
}

void SequencesPlanners::loadTrajsFromFile( const std::vector<std::string>& files )
{
    best_traj_.clear();

    for( int i=0; i<int(files.size()); i++ )
    {
        Move3D::Trajectory traj( robot_ );

        if( traj.loadFromFile( files[i] ) )
        {
            best_traj_.push_back( traj );
            cout << "load planner result " << i << " : " << files[i] << endl;
        }
    }
}

std::vector<confPtr_t> SequencesPlanners::getStoredConfig() const
{
    std::vector<confPtr_t> configs;

    if( robot_->getUseLibmove3dStruct() )
    {
        p3d_rob* robot = robot_->getP3dRobotStruct();

        for( int i=0; i<robot->nconf; i++ ) // Add all configs stored in pr3_rob
        {
            cout << "Add config : " << robot->conf[i]->name << endl;
            confPtr_t q( new Configuration( robot_ , robot->conf[i]->q ) );
            configs.push_back( q );
        }
    }

     return configs;
}

void SequencesPlanners::runSequence()
{
    std::vector<confPtr_t> configs = getStoredConfig();
    confPtr_t q_init = robot_->getInitPos();

    robot_->setAndUpdate( *q_init );

    g3d_draw_allwin_active();

    for( int i=0; i<configs.size() ; i++ )
    {
        robot_->setInitPos( *q_init );
        robot_->setGoalPos( *configs[i] );
        run();

        robot_->setInitPos( *configs[i] );
        robot_->setGoalPos( *q_init );
        run();
    }

    robot_->setInitPos( *q_init );
}

void SequencesPlanners::playTrajs() //const
{
    for( int i=0; i<best_traj_.size(); i++ )
    {
        if( best_traj_[i].size() > 0 )
        {
            best_traj_[i].replaceP3dTraj();
            best_traj_[i].show();

            if( PlanEnv->getBool(PlanParam::stopPlanner) )
            {
                return;
            }

            best_traj_[i].resetIsValid();
            if( !best_traj_[i].isValid() )
            {
                cout << "Trajectory " << i << " in collision" << endl;
                return;
            }
        }
    }
}


bool SequencesPlanners::run()
{
    switch( planner_type_ )
    {
    case stomp:
        return runStomp(PlanEnv->getDouble(PlanParam::trajDuration));
    case astar:
        return runAStar();
    case rrt:
        return runRRT();
    default: cout << "Error : planner not defined!!!" << endl;
        return false;
    }
}

const std::vector<Move3D::Trajectory>& SequencesPlanners::getAllTrajs()
{
    if( planner_type_ != stomp )
    {
        return best_traj_;
    }
    else {
        return all_traj_;
    }
}

bool SequencesPlanners::runStomp(double duration)
{
    if( robot_ == NULL )
    {
        cout << "robot not defined" << endl;
        return false;
    }

    std::vector<Robot*> robots;
    robots.push_back( robot_ );

    // TODO see to remove this
    traj_optim_initScenario();

    std::vector<int> planner_joints = traj_optim_get_planner_joints();
    const CollisionSpace* coll_space = traj_optim_get_collision_space();
    std::vector<CollisionPoint> collision_points = traj_optim_get_collision_points();

    stomp_motion_planner::stompRun pool( coll_space, planner_joints, collision_points );
    pool.setPool( robots );
    // Uncomment for parallel stomps
    // pool.setRobotPool( 0, robots );

    Move3D::Trajectory T( robot_ );

    if( init_stomp_.getNbOfViaPoints() == 0 )
    {
        T.push_back( robot_->getInitPos() );
        T.push_back( robot_->getGoalPos() );
    }
    else{
       T = init_stomp_;
    }

    pool.run( 0, T, duration );

    all_traj_ = pool.getContext(0)->getStompOptimizer()->getAllTrajs();
    best_traj_.push_back( pool.getBestTrajectory( 0 ) );

    return true;
}

bool SequencesPlanners::runRRT()
{
    bool succeed = false;

    try
    {
        succeed = p3d_run_rrt( robot_->getP3dRobotStruct() );

        if( !ENV.getBool(Env::drawDisabled) ) {
            g3d_draw_allwin_active();
        }
    }
    catch (std::string str)
    {
        std::cerr << "Exeption in run rrt : " << endl;
        std::cerr << str << endl;
    }
    catch (...)
    {
        std::cerr << "Exeption in run qt_runDiffusion" << endl;
    }

    if( succeed ){
        best_traj_.push_back( p3d_get_last_trajectory() );
        return true;
    } else {
        return false;
    }
}

bool SequencesPlanners::runAStar()
{
    // Warning leaking
    AStarPlanner planner( robot_ );
    planner.set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
    planner.init();

    confPtr_t q_init = robot_->getInitPos();
    confPtr_t q_goal = robot_->getGoalPos();

    cout << "PLANNING ASTAR" << endl;
    Move3D::Trajectory* traj = planner.computeRobotTrajectory( q_init, q_goal );

    if( traj != NULL ){
        best_traj_.push_back(*traj);
        return true;
    } else {
        return false;
    }
}

void SequencesPlanners::draw()
{
//    for( size_t i=0;i<best_traj_.size();i++)
//        best_traj_[i].draw(100);
}

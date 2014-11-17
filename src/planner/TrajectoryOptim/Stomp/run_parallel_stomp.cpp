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
#include "run_parallel_stomp.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "collision_space/collision_space_factory.hpp"

#include "API/project.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/p3d/env.hpp>

#include <boost/thread/thread.hpp>

using namespace Move3D;
using namespace stomp_motion_planner;
using std::cout;
using std::endl;

stompRun* stomp_motion_planner::global_stompRun = NULL;

stompContext::stompContext( int id, Robot* robot, const CollisionSpace* coll_space,  const std::vector<int>& planner_joints, const std::vector<CollisionPoint>& collision_points )
{
    m_id = id;

    m_chomptraj = NULL;
    m_chompplangroup = NULL;
    m_stompparams = NULL;

    m_robot = robot;
    m_coll_space = coll_space;
    m_planner_joints = planner_joints;
    m_collision_points = collision_points;

    m_parallel_robots.clear();

    m_runid = 0;
    m_use_costspace = ENV.getBool( Env::isCostSpace );
    m_use_iteration_limit = PlanEnv->getBool( PlanParam::trajStompWithIterLimit );
    m_max_iterations = PlanEnv->getInt( PlanParam::stompMaxIteration );
    m_use_time_limit = PlanEnv->getBool( PlanParam::trajStompWithTimeLimit );
    m_max_time = PlanEnv->getDouble( PlanParam::trajStompTimeLimit );
    m_nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );
}

stompContext::~stompContext()
{
    delete m_chomptraj;
    delete m_chompplangroup;
    delete m_stompparams;

    m_stomp->resetSharedPtr();
}

void stompContext::setParallelRobots( const std::vector<Robot*>& robots )
{
    m_parallel_robots = robots;
}

bool stompContext::initRun( Move3D::Trajectory& T, double duration )
{
    if( T.getNbOfPaths() == 0 )
    {
        cout << "Error in trajectory, contains no localpath" << endl;
        return false;
    }

    if( !T.cutTrajInSmallLP( m_nb_points-1 ) )
    {
        cout << "Error in cutTrajInSmallLP" << endl;
        return false;
    }

    cout << "m_nb_points : " << m_nb_points << " , nb of paths : " << T.getNbOfPaths() << endl;

    delete m_chomptraj;
    delete m_chompplangroup;
    delete m_stompparams;

    for (int i=0; i<int(m_planner_joints.size()); i++) {
        cout << m_planner_joints[i] << endl;
    }

    m_chompplangroup = new ChompPlanningGroup( m_robot, m_planner_joints );
    m_chompplangroup->collision_points_ = m_collision_points;

    m_chomptraj = new ChompTrajectory( T, DIFF_RULE_LENGTH, *m_chompplangroup, duration );
    cout << "Chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;
    cout << "Chomp Trajectory duration : " << duration << endl;

    // Save passive dof to generate the Move3D trajectory
    std::vector<confPtr_t> passive_dofs = T.getVectorOfConfiguration();

    // Create Optimizer
    // ----------------
    m_stompparams = new stomp_motion_planner::StompParameters;
    m_stompparams->init();

    cout << "Initialize optimizer" << endl;
    m_stomp.reset(new stomp_motion_planner::StompOptimizer( m_chomptraj, m_stompparams, m_chompplangroup, m_coll_space, m_id ));
    m_stomp->setSource( T.getBegin() );
    m_stomp->setPassiveDofs( passive_dofs );
    m_stomp->setSharedPtr( m_stomp );
    m_stomp->setTrajColor( m_color );


    //    if( m_sce == traj_optim::HumanAwareManip && m_robot->getName() == "PR2_ROBOT")
    //    {
    m_stomp->setUseCostSpace( m_use_costspace );
    //    }

    m_stomp->setRobotPool( m_parallel_robots );

    cout << "Optimizer created" << endl;
    return true;
}

void stompContext::run()
{
    m_stompparams->init();

    if( m_use_iteration_limit )
    {
        m_stomp->setUseIterationLimit( true );
        m_stompparams->max_iterations_ = m_max_iterations;
    }

    if( m_use_time_limit )
    {
        m_stomp->setUseTimeLimit( true );
        m_stompparams->max_time_ = m_max_time;
    }

    m_stomp->runDeformation( 0, ++m_runid );
    m_stomp->resetSharedPtr();
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------

stompRun::stompRun( const CollisionSpace* coll_space, std::vector<int> planner_joints, const std::vector<CollisionPoint>& collision_points )
{
    m_coll_space = coll_space;
    m_planner_joints = planner_joints;
    m_collision_points = collision_points;
}

stompRun::~stompRun()
{
    for( int i=0;i<int(m_stomps.size());i++)
    {
        delete m_stomps[i];
    }
}

void stompRun::setPool(const std::vector<Robot*>& robots )
{
    m_robots = robots;

    for( int i=0; i<int(m_robots.size()); i++ )
    {
        m_stomps.push_back( new stompContext( i, m_robots[i], m_coll_space, m_planner_joints, m_collision_points ) );
        m_is_thread_running.push_back( false );
    }
}

bool stompRun::setParallelStompEnd(int id)
{
    if( m_is_thread_running.size() == 1 )
    {
        // See for openrave may need to comment back in
        // m_mtx_multi_end.unlock(); // TODO make sure it's ok
        return m_is_thread_running[id];
    }

    m_mtx_set_end.lock();
    m_is_thread_running[id] = false;

    for( int i=0; i<int(m_is_thread_running.size()); i++)
    {
        if( m_is_thread_running[i] )
        {
            m_mtx_set_end.unlock();
            return true;
        }
    }

    m_mtx_multi_end.unlock();
    m_mtx_set_end.unlock();
    return false;
}

void stompRun::start()
{
    m_mtx_multi_end.lock();
}

void stompRun::isRunning()
{
    m_mtx_multi_end.lock();
    m_mtx_multi_end.unlock();
}

void stompRun::setPathColor( int id, const std::vector<double>& color )
{
    m_stomps[id]->setPathColor( color );
}

void stompRun::setRobotPool( int id, const std::vector<Robot*>& robots )
{
    m_stomps[id]->setParallelRobots( robots );
}

Move3D::Trajectory stompRun::getBestTrajectory( int id )
{
    return m_stomps[id]->getStompOptimizer()->getBestTraj();
}

bool stompRun::run( int id, Move3D::Trajectory& T, double duration )
{
    if( id >= int(m_stomps.size()) )
    {
        cout << "run id does not exist" << endl;
        return false;
    }

    // impossible to parallize the localpath class
    if( m_stomps[id]->getRobot()->getUseLibmove3dStruct() )
        m_mtx_set_end.lock();

    bool succeed = m_stomps[id]->initRun( T, duration );

    if( m_stomps[id]->getRobot()->getUseLibmove3dStruct() )
        m_mtx_set_end.unlock();

    if( succeed )
    {
        m_is_thread_running[id] = true;
        m_stomps[id]->run();
    }
    else {
        return false;
    }

    setParallelStompEnd( id );
    cout << "end running thread : " << id << endl;
    return true;
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void stomp_motion_planner::srompRun_MultipleParallel()
{
    std::vector<Robot*> robots;
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob1") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob2") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob3") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob4") );
    // robots.push_back( global_Project->getActiveScene()->getRobotByName("rob5") );
    // robots.push_back( global_Project->getActiveScene()->getRobotByName("rob6") );
    // robots.push_back( global_Project->getActiveScene()->getRobotByName("rob7") );

    traj_optim_initScenario();
    std::vector<int> planner_joints = traj_optim_get_planner_joints();
    std::vector<CollisionPoint> collision_points;

    stompRun* pool = new stompRun( NULL, planner_joints, collision_points );
    pool->setPool( robots );

    // Move3D::Trajectory T( robots[0] );
    // T.push_back( robots[0]->getInitPos() );
    // T.push_back( robots[0]->getGoalPos() );
    // pool->run( 0, T );

    std::vector<Move3D::Trajectory> trajs;
    for( int i=0;i<int(robots.size()); i++)
    {
        trajs.push_back( Move3D::Trajectory( robots[i] ) );
        trajs.back().push_back( robots[i]->getInitPos() );
        trajs.back().push_back( robots[i]->getGoalPos() );
    }

    cout << "spawns: " <<  robots.size() << " threads" << endl;

    stomp_motion_planner::global_stompRun = pool;

    pool->start();

    for( int i=0;i<int(robots.size()); i++)
    {
        boost::thread( &stompRun::run, pool, i, trajs[i], PlanEnv->getDouble(PlanParam::trajDuration) );
        //global_MultiStomplinesToDraw[robots[i]].clear();
        robots[i]->getP3dRobotStruct()->display_mode = P3D_ROB_NO_DISPLAY;
    }

    pool->isRunning();

    cout << "Pool of stomps has ended" << endl;
    delete pool;
    stomp_motion_planner::global_stompRun = NULL;
}

void stomp_motion_planner::srompRun_OneParallel()
{
    std::vector<Robot*> robots;
    robots.push_back( global_Project->getActiveScene()->getRobotByNameContaining("ROBOT") );

    traj_optim_initScenario();
    std::vector<int> planner_joints = traj_optim_get_planner_joints();
    const CollisionSpace* coll_space = traj_optim_get_collision_space();
    std::vector<CollisionPoint> collision_points = traj_optim_get_collision_points();

    stompRun* pool = new stompRun( coll_space, planner_joints, collision_points );
    pool->setPool( robots );

    robots.clear();
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob1") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob2") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob3") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob4") );
    pool->setRobotPool( 0, robots );

    robots.clear();
    robots.push_back( global_Project->getActiveScene()->getRobotByNameContaining("ROBOT") );

    Move3D::Trajectory T( robots[0] );
    T.push_back( robots[0]->getInitPos() );
    T.push_back( robots[0]->getGoalPos() );

    pool->run( 0, T, PlanEnv->getDouble(PlanParam::trajDuration) );
}

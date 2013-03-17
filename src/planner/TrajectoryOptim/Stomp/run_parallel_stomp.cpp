#include "run_parallel_stomp.hpp"

#include "p3d/env.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "API/project.hpp"

#include <boost/thread/thread.hpp>

stompRun* global_stompRun = NULL;
std::map< Robot*, std::vector<Eigen::Vector3d> > global_MultiStomplinesToDraw;

stompContext::stompContext(Robot* robot, const CollisionSpace* coll_space,  const std::vector<int>& planner_joints, const std::vector<CollisionPoint>& collision_points )
{
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

bool stompContext::initRun( API::Trajectory& T )
{
    if( T.getNbOfPaths() == 0 )
    {
        cout << "Error in trajectory, contains no localpath" << endl;
        return false;
    }

    if( !T.cutTrajInSmallLP( m_nb_points ) )
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

    m_chomptraj = new ChompTrajectory( T, DIFF_RULE_LENGTH, *m_chompplangroup );
    cout << "Chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;

    // Save passive dof to generate the Move3D trajectory
    std::vector<confPtr_t> passive_dofs = T.getVectorOfConfiguration();

    // Create Optimizer
    // ----------------
    m_stompparams = new stomp_motion_planner::StompParameters;
    m_stompparams->init();

    cout << "Initialize optimizer" << endl;
    m_stomp.reset(new stomp_motion_planner::StompOptimizer( m_chomptraj, m_stompparams, m_chompplangroup, m_coll_space ));
    m_stomp->setSource( T.getBegin() );
    m_stomp->setPassiveDofs( passive_dofs );
    m_stomp->setSharedPtr( m_stomp );

    if( PlanEnv->getBool(PlanParam::trajStompWithTimeLimit) )
    {
        m_stomp->setTimeLimit( PlanEnv->getDouble(PlanParam::trajStompTimeLimit));
    }

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
        m_stomps.push_back( new stompContext( m_robots[i], m_coll_space, m_planner_joints, m_collision_points ) );
        m_is_thread_running.push_back( false );
    }
}

bool stompRun::setParallelStompEnd(int id)
{
    if( m_is_thread_running.size() == 1 )
    {
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

void stompRun::setRobotPool( int id, const std::vector<Robot*>& robots )
{
    m_stomps[id]->setParallelRobots( robots );
}

API::Trajectory stompRun::getBestTrajectory( int id )
{
    return m_stomps[id]->getBestTrajectory();
}

void stompRun::run( int id, API::Trajectory& T )
{
    if( id >= int(m_stomps.size()) )
    {
        cout << "run id does not exist" << endl;
        return;
    }

    // impossible to parallize the localpath class
    m_mtx_set_end.lock();
    bool succeed = m_stomps[id]->initRun( T );
    m_mtx_set_end.unlock();

    if( succeed )
    {
        m_is_thread_running[id] = true;
        m_stomps[id]->run();
    }

    setParallelStompEnd( id );
    cout << "end running thread : " << id << endl;
}

//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
//----------------------------------------------------------------------------------
void srompRun_MultipleParallel()
{
    std::vector<Robot*> robots;
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob1") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob2") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob3") );
    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob4") );
//    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob5") );
//    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob6") );
//    robots.push_back( global_Project->getActiveScene()->getRobotByName("rob7") );

    traj_optim_initScenario();
    std::vector<int> planner_joints = traj_optim_get_planner_joints();
    std::vector<CollisionPoint> collision_points;

    stompRun* pool = new stompRun( NULL, planner_joints, collision_points );
    pool->setPool( robots );

//    API::Trajectory T( robots[0] );
//    T.push_back( robots[0]->getInitialPosition() );
//    T.push_back( robots[0]->getGoTo() );
//    pool->run( 0, T );

    std::vector<API::Trajectory> trajs;
    for( int i=0;i<int(robots.size()); i++)
    {
        trajs.push_back( API::Trajectory( robots[i] ) );
        trajs.back().push_back( robots[i]->getInitialPosition() );
        trajs.back().push_back( robots[i]->getGoTo() );
    }

    cout << "spawns: " <<  robots.size() << " threads" << endl;

    global_stompRun = pool;

    pool->start();

    for( int i=0;i<int(robots.size()); i++)
    {
        boost::thread( &stompRun::run, pool, i, trajs[i] );
        global_MultiStomplinesToDraw[robots[i]].clear();
        robots[i]->getRobotStruct()->display_mode = P3D_ROB_NO_DISPLAY;
    }

    pool->isRunning();

    cout << "Pool of stomps has ended" << endl;
    delete pool;
    global_stompRun = NULL;
}

void srompRun_OneParallel()
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

    API::Trajectory T( robots[0] );
    T.push_back( robots[0]->getInitialPosition() );
    T.push_back( robots[0]->getGoTo() );

    pool->run( 0, T );
}

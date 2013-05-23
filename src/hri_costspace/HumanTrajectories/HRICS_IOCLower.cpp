#include "HRICS_IOCLower.hpp"

#include "API/Device/robot.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"

using namespace HRICS;

IOCLower::IOCLower()
{
    m_is_scenario_init = false;
}

IOCLower::~IOCLower()
{

}

void IOCLower::setTrajectory()
{

}

void IOCLower::setActiveRobot(Robot* robot)
{
    m_robot = robot;
    m_start_goal_config.clear();
    confPtr_t q_s = m_robot->getInitPos();
    confPtr_t q_g = m_robot->getGoalPos();
    m_start_goal_config.push_back( std::make_pair( q_s, q_g ) );
}

void IOCLower::lauchOptimization()
{
    cout << "run parallel stomps" << endl;

    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, 100 );

    confPtr_t q_init = m_robot->getInitPos();
    confPtr_t q_goal = m_robot->getGoalPos();

    m_robot->setAndUpdate( *q_init );

    API::Trajectory traj;

    traj.push_back( q_init );
    traj.push_back( q_goal );

    if( !m_is_scenario_init )
    {
        traj_optim_add_human_to_collision_space( true );
        traj_optim_initScenario();
        m_is_scenario_init = true;
    }

    std::vector<Robot*> robots; robots.push_back( m_robot );

    stompRun* pool = new stompRun( traj_optim_get_collision_space(),
                                   traj_optim_get_planner_joints(),
                                   traj_optim_get_collision_points() );

    pool->setPool( robots );
    pool->setRobotPool( 0, robots );
    pool->run( 0, traj );

    delete pool;
}

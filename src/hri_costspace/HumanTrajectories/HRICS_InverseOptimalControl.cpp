#include "HRICS_InverseOptimalControl.hpp"

#include "API/Device/robot.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

using namespace HRICS;

InverseOptimalControl::InverseOptimalControl()
{

}

InverseOptimalControl::~InverseOptimalControl()
{

}

void InverseOptimalControl::setTrajectory()
{

}

void InverseOptimalControl::setActiveRobot(Robot* robot)
{
    m_robot = robot;
    m_start_goal_config.clear();
    confPtr_t q_s = m_robot->getInitPos();
    confPtr_t q_g = m_robot->getGoalPos();
    m_start_goal_config.push_back( std::make_pair( q_s, q_g ) );
}

void InverseOptimalControl::lauchOptimization()
{
    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, 100 );
    // traj_optim_set_discretize( true );
    // traj_optim_set_discretization( 0.015 );

    m_robot->setAndUpdate( *m_start_goal_config[0].first );

//    if( iter>0 )
//    {
//        const API::Trajectory& current_traj = m_paths[m_best_path_id];
//        const double parameter = m_robot_steps_per_exection*m_robot_step;
//        API::Trajectory optimi_traj;

//        if( id_goal == m_best_path_id )
//        {
//            optimi_traj = current_traj.extractSubTrajectory( parameter, current_traj.getRangeMax(), false );
//        }
//        else
//        {
//            API::CostOptimization traj( m_paths[id_goal] );
//            traj.connectConfigurationToBegin( current_traj.configAtParam( parameter ), parameter/5, true );
//            optimi_traj = traj;
//        }

//        traj_optim_set_use_extern_trajectory( true );
//        traj_optim_set_extern_trajectory( optimi_traj );
//    }
//    else
//    {
        m_robot->setInitPos( *m_start_goal_config[0].first );
        m_robot->setGoalPos( *m_start_goal_config[0].second );
        traj_optim_set_use_extern_trajectory( false );
//    }

    traj_optim_set_use_iteration_limit(true);
    traj_optim_set_iteration_limit( PlanEnv->getInt(PlanParam::stompMaxIteration) );
    traj_optim_runStomp(0);

//    m_paths[id_goal] = global_optimizer->getBestTraj();
}

#include "HRICS_run_multiple_planners.hpp"

#include "API/project.hpp"
#include "API/misc_functions.hpp"

#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"
#include "planner/AStar/AStarPlanner.hpp"
#include "planner/planEnvironment.hpp"

#include <Graphic-pkg.h>

#include <iomanip>
#include <sstream>

using namespace HRICS;
using std::cout;
using std::endl;

MultiplePlanners::MultiplePlanners(Robot* robot)
{
    // robot_ = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
    robot_ = robot;
    cout << "PLanner initialized for robot : " << robot->getName() << endl;

//    folder_ = "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/squares";
//    folder_ = "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/per_feature_square/ASTAR";

    planner_type_ = astar;
}

void MultiplePlanners::initializeNoisy()
{

}

void MultiplePlanners::multipleRun( std::string folder, int nb_runs )
{
    best_traj_.clear();

    for( int i=0;i<nb_runs;i++)
        run();

    saveTrajsToFile( "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/per_feature_square/ASTAR" );
}

void MultiplePlanners::saveTrajsToFile( std::string folder )
{
    for( int i=0;i<int(best_traj_.size());i++)
    {
        // Set file names
        std::stringstream ss;
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        std::string filename = folder + "/" + ss.str();

        best_traj_[i].replaceP3dTraj();

        p3d_save_traj( filename.c_str(), robot_->getRobotStruct()->tcur );
        cout << "save planner result " << i << " : " << ss.str() << endl;
    }
}

void MultiplePlanners::loadTrajsFromFile( std::string folder )
{
    cout << "--------------------------------------------" << endl;
    cout << " load trajs trom file " << endl;

    std::vector<std::string>  files = move3d_get_files_in_folder( folder , "traj" );

    best_traj_.resize( files.size() ); // only loads 10
    global_trajToDraw.clear();

    std::stringstream ss;

    int nb_of_loaded_traj = 0;

    for( int i=0;i<int(files.size());i++)
    {
        ss.str(""); // clear stream
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";

        if ( !p3d_read_traj( ( folder + "/" + files[i] ).c_str()) )
        {
            cout << "could not load trajectory" << endl;
            continue;
        }

        API::Trajectory T( robot_, (p3d_traj*)p3d_get_desc_curid(P3D_TRAJ) );
        T.setColor( i%8 );

        //cout << "color : " << i%8 << endl;
        best_traj_[i] = T;
        global_trajToDraw.push_back( T );

        nb_of_loaded_traj++;
        // cout << "load traj : " << ss.str() << " , with : " << T.getNbOfViaPoints() << endl;
    }

    cout << "Successfully loaded "  << nb_of_loaded_traj << " trajectories from : "  << folder << "!!!" << endl;
}

bool MultiplePlanners::run()
{
    switch( planner_type_ )
    {
    case stomp:
        return runStomp();
    case astar:
        return runAStar();
    case rrt:
        return runRRT();
    default: cout << "Error : planner not defined!!!" << endl;
        return false;
    }
}

bool MultiplePlanners::runStomp()
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

    stompRun pool( coll_space, planner_joints, collision_points );
    pool.setPool( robots );

    robots.clear();
    robots.push_back( robots[0] );
    pool.setRobotPool( 0, robots );

    API::Trajectory T( robot_ );
    T.push_back( robot_->getInitPos() );
    T.push_back( robot_->getGoalPos() );

    pool.run( 0, T );

    best_traj_.push_back( pool.getBestTrajectory( 0 ) );

    return true;
}

bool MultiplePlanners::runRRT()
{
    bool succeed = false;

    try
    {
        succeed = p3d_run_rrt( robot_->getRobotStruct() );

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

bool MultiplePlanners::runAStar()
{
    // Warning leaking
    AStarPlanner planner( robot_ );
    planner.set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
    planner.init();

    confPtr_t q_init = robot_->getInitPos();
    confPtr_t q_goal = robot_->getGoalPos();

    API::Trajectory* traj = planner.computeRobotTrajectory( q_init, q_goal );

    if( traj != NULL ){
        best_traj_.push_back(*traj);
        return true;
    } else {
        return false;
    }
}

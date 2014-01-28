#include "HRICS_run_multiple_stomp.hpp"

#include "API/project.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp"

#include <iomanip>
#include <sstream>

using namespace HRICS;
using std::cout;
using std::endl;

MultipleStomp::MultipleStomp()
{
    robot_ = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
    folder_ = "/home/jmainpri/workspace/move3d/move3d-launch/matlab/stomp_trajs/squares";
}

void MultipleStomp::initializeNoisy()
{

}

void MultipleStomp::multipleRun( int nb_runs )
{
    best_traj_.clear();

    for( int i=0;i<nb_runs;i++)
    {
        run();
    }
}

void MultipleStomp::saveTrajsToFile()
{
    for( int i=0;i<int(best_traj_.size());i++)
    {
        // Set file names
        std::stringstream ss;
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";
        std::string filename = folder_ + "/" + ss.str();

        best_traj_[i].replaceP3dTraj();

        p3d_save_traj( filename.c_str(), robot_->getRobotStruct()->tcur );
        cout << "save stomp result " << i << " : " << ss.str() << endl;
    }
}

void MultipleStomp::loadTrajsFromFile()
{
    best_traj_.resize(10); // only loads 10
    global_trajToDraw.clear();

    std::stringstream ss;

    for( int i=0;i<int(best_traj_.size());i++)
    {
        ss.str(""); // clear stream
        ss << "trajectory" << std::setw(3) << std::setfill( '0' ) << i << ".traj";

        if ( !p3d_read_traj( ( folder_ + "/" + ss.str() ).c_str()) )
        {
            cout << "could not load trajectory" << endl;
            continue;
        }

        API::Trajectory T( robot_, (p3d_traj*)p3d_get_desc_curid(P3D_TRAJ) );
        T.setColor( i%8 );

        //cout << "color : " << i%8 << endl;
        best_traj_[i] = T;
        global_trajToDraw.push_back( T );

        cout << "load traj : " << ss.str() << " , with : " << T.getNbOfViaPoints() << endl;
    }
}

bool MultipleStomp::run()
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

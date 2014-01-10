#include "HRICS_detours.hpp"

#include "planner/planEnvironment.hpp"
#include "API/project.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

Detours::Detours()
{
    robot_ = global_Project->getActiveScene()->getActiveRobot();
}

void Detours::planAStar()
{
    AStarPlanner planner( robot_ );
    planner.set_pace( PlanEnv->getDouble(PlanParam::grid_pace) );
    planner.init();

    grid_ = planner.getGrid();

    confPtr_t q_init = robot_->getInitPos();
    confPtr_t q_goal = robot_->getGoalPos();

    API::Trajectory* traj = planner.computeRobotTrajectory( q_init, q_goal );

    if( traj != NULL )
        cout << "AStar planning OK" << endl;
    else
        cout << "Error in run AStar planner" << endl;
}

void Detours::draw()
{

}

#include "HRICS_detours.hpp"

#include "planner/planEnvironment.hpp"
#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include <boost/bind.hpp>

#include "Graphic-pkg.h"

using namespace HRICS;
using std::cout;
using std::endl;

Detours::Detours()
{
    robot_ = global_Project->getActiveScene()->getActiveRobot();

    global_DrawModule->addDrawFunction( "Detours", boost::bind( &Detours::draw, this) );
    global_DrawModule->enableDrawFunction( "Detours" );
}

Detours::~Detours()
{
    global_DrawModule->deleteDrawFunction( "Detours" );
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
    cout << "DRAW DETOURS" << endl;
    const double height = 0.0;
    for( int i=0; i<grid_->getNumberOfCells(); i++ )
    {
        Eigen::Vector2d center = grid_->getCell(i)->getCenter();
        Eigen::Vector2d cell_size = grid_->getCellSize();
        g3d_draw_solid_sphere(center[0], center[1], height, cell_size[0]/5, 20);
    }
}

#include "HRICS_HumanPredictionCostSpace.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

using namespace std;
using namespace HRICS;

HumanPredictionCostSpace* global_HumanPredictionCostSpace = NULL;


HumanPredictionCostSpace::HumanPredictionCostSpace( const std::string& robotname, WorkspaceOccupancyGrid* occup_grid ) : m_ws_occupancy(occup_grid)
{
    cout << "Create HumanPredictionCostSpace" << endl;

    m_robot = global_Project->getActiveScene()->getRobotByName( robotname );

    if( m_robot == NULL )
    {
        cout << "No robot named " << robotname <<  " in " << __func__ << endl;
        return;
    }

    m_surface_sampler = new BodySurfaceSampler(0.10);

    sampleRobotPoints();
}

HumanPredictionCostSpace::~HumanPredictionCostSpace()
{

}

double HumanPredictionCostSpace::cost(Configuration& q)
{
    return 0.0;
}

void HumanPredictionCostSpace::draw()
{
    if( PlanEnv->getBool(PlanParam::drawSampledPoints) )
        draw_sampled_points();
}

void HumanPredictionCostSpace::sampleRobotPoints()
{
    m_surface_sampler->sampleRobotBodiesSurface( m_robot );
}

void HumanPredictionCostSpace::draw_sampled_points()
{
    for(int i=0; i<int(m_robot->getNumberOfJoints()); i++)
    {
        Joint* jnt = m_robot->getJoint(i);
        p3d_obj* obj = jnt->getJointStruct()->o;

        if( obj )
        {
            m_surface_sampler->getPointCloud( obj ).drawAllPoints( jnt->getMatrixPos() );
        }
    }
}


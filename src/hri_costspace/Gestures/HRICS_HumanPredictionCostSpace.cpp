#include "HRICS_HumanPredictionCostSpace.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

using namespace std;
using namespace HRICS;

HumanPredictionCostSpace* global_humanPredictionCostSpace = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Cost Function

double HRICS_getPredictionOccupancyCost(Configuration& q)
{
//    return 1.0;
//    return global_humanPredictionCostSpace->getCostFromActiveJoints(q);
    return global_humanPredictionCostSpace->getCost(q);
}


//----------------------------------------------------------------------
//----------------------------------------------------------------------

HumanPredictionCostSpace::HumanPredictionCostSpace( Robot* robot, WorkspaceOccupancyGrid* occup_grid ) : m_robot(robot), m_ws_occupancy(occup_grid)
{
    cout << "Create HumanPredictionCostSpace" << endl;

    m_surface_sampler = new BodySurfaceSampler( 0.50 );

    sampleRobotPoints();
    setActiveJoints();
}

HumanPredictionCostSpace::~HumanPredictionCostSpace()
{

}

double HumanPredictionCostSpace::getCost(Configuration& q)
{
    Robot* robot = q.getRobot();

    robot->setAndUpdate(q); // TODO remove this when necessary

    double cost = 0.0;

    int nb_points = 0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        Joint* jnt = m_robot->getJoint( m_active_joints[i] );
        p3d_obj* obj = jnt->getJointStruct()->o;

        if( obj )
        {
            //cout << "compute cost for joint " << jnt->getName() << endl;
            Eigen::Transform3d T = robot->getJoint( m_active_joints[i] )->getMatrixPos();
            PointCloud& pc = m_surface_sampler->getPointCloud( obj );

            for( int j=0; j<int(pc.size()); j++ )
            {
                cost += m_ws_occupancy->getOccupancyCombination( T*pc[j] );
                //cost += m_ws_occupancy->getOccupancy( T*pc[j] );
                nb_points++;
            }
        }
    }

    //cout << "cost : " << cost << endl;
    //cout << "occupancy computed for " << nb_points << endl;
    //cout << "HumanPredictionCostSpace cost : " << cost << endl;
    return cost;
}

double HumanPredictionCostSpace::getCostFromActiveJoints(Configuration& q)
{
     Robot* robot = q.getRobot();
//    robot->setAndUpdate(q);

    double cost=0.0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        cost += 10*m_ws_occupancy->getOccupancyCombination( robot->getJoint( m_active_joints[i] )->getVectorPos() );
    }

    return cost;
}

void HumanPredictionCostSpace::draw()
{
//    if( PlanEnv->getBool(PlanParam::drawSampledPoints) )
        draw_sampled_points();
}

void HumanPredictionCostSpace::sampleRobotPoints()
{
    m_surface_sampler->sampleRobotBodiesSurface( m_robot );
}

void HumanPredictionCostSpace::draw_sampled_points()
{
    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        Joint* jnt = m_robot->getJoint( m_active_joints[i] );
        p3d_obj* obj = jnt->getJointStruct()->o;

        if( obj )
        {
            m_surface_sampler->getPointCloud( obj ).drawAllPoints( jnt->getMatrixPos() );
        }
    }
}

void HumanPredictionCostSpace::setActiveJoints()
{
    if( m_robot->getName() == "PR2_ROBOT" )
    {
        // Set the planner joints
        m_active_joints.clear();
        // right arm
//        m_active_joints.push_back( 6 );
//        m_active_joints.push_back( 7 );

        m_active_joints.push_back( 8 );
        m_active_joints.push_back( 9 );
        m_active_joints.push_back( 10 );
        m_active_joints.push_back( 11 );
        m_active_joints.push_back( 12 );
//        m_active_joints.push_back( 14 );
//        m_active_joints.push_back( 15 );
    }

    int nb_points = 0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        Joint* jnt = m_robot->getJoint( m_active_joints[i] );
        p3d_obj* obj = jnt->getJointStruct()->o;

        if( obj )
        {
            PointCloud& pc = m_surface_sampler->getPointCloud( obj );
            nb_points += pc.size();
        }
    }

    cout << "HumanPredictionCostSpace => add " << nb_points << " to the cost function" << endl;

//    for( int i=0;i<int(m_active_joints.size());i++)
//    {
//        cout << m_robot->getJoint( m_active_joints[i] )->getName() << endl;
//    }
}


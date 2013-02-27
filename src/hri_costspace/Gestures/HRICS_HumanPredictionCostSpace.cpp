#include "HRICS_HumanPredictionCostSpace.hpp"

#include "API/project.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include <boost/bind.hpp>

using namespace std;
using namespace HRICS;

HumanPredictionCostSpace* global_humanPredictionCostSpace = NULL;

//----------------------------------------------------------------------
//----------------------------------------------------------------------
// Cost Function

void HRICS_initOccupancyPredictionFramework()
{
    std::vector<double> size = global_Project->getActiveScene()->getBounds();

    Robot* robot = global_Project->getActiveScene()->getRobotByName( "PR2_ROBOT" );
    Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );

    if( robot == NULL || human == NULL )
    {
        cout << "human or robot not defined" << endl;
        return;
    }

    RecordMotion* recorder = new RecordMotion( human );

    if( recorder->loadRegressedFromCSV() )
    {
        cout << "Motions loaded successfully" << endl;
    }
    else {
         cout << "Error loading motions" << endl;
         delete recorder;
         return;
    }

    ClassifyMotion* classifier = new ClassifyMotion();

    if( classifier->load_model() )
    {
        cout << "GMMs loaded successfully" << endl;
    }
    else {
         cout << "Error loading GMMs" << endl;
         delete recorder;
         delete classifier;
         return;
    }

    WorkspaceOccupancyGrid* occupancyGrid = new WorkspaceOccupancyGrid( human, 0.05, size, classifier );
    occupancyGrid->setRegressedMotions( recorder->getStoredMotions() );
    occupancyGrid->computeOccpancy();
    occupancyGrid->setClassToDraw(1);

    // Create the prediction costspace
    HumanPredictionCostSpace* predictionSpace = new HumanPredictionCostSpace( robot, occupancyGrid );

    // GUI global variables
    global_motionRecorder = recorder;
    global_classifyMotion = classifier;
    global_workspaceOccupancy = occupancyGrid;
    global_humanPredictionCostSpace = predictionSpace;


    // Define cost functions
    global_costSpace->addCost( "costHumanPredictionOccupancy" , boost::bind(HRICS_getPredictionOccupancyCost, _1) );
    global_costSpace->setCost( "costHumanPredictionOccupancy" );
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

double HRICS_getPredictionOccupancyCost(Configuration& q)
{
//    return global_humanPredictionCostSpace->getCostFromActiveJoints(q);
    double cost = 0.0;
    cost = global_humanPredictionCostSpace->getCost(q);
    if( PlanEnv->getBool(PlanParam::trajStompComputeColl ))
    {
        if (q.isInCollision()) {
            cost += 1000;
        }
    }
    return cost;
}

//----------------------------------------------------------------------
//----------------------------------------------------------------------

HumanPredictionCostSpace::HumanPredictionCostSpace( Robot* robot, WorkspaceOccupancyGrid* occup_grid ) : m_robot(robot), m_ws_occupancy(occup_grid)
{
    cout << "Create HumanPredictionCostSpace" << endl;

    m_surface_sampler = new BodySurfaceSampler(0.07);

    sampleRobotPoints();
    setActiveJoints();
}

HumanPredictionCostSpace::~HumanPredictionCostSpace()
{

}

double HumanPredictionCostSpace::getCost(Configuration& q)
{
    m_robot->setAndUpdate(q);

    double cost = 0.0;

    int nb_points = 0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        Joint* jnt = m_robot->getJoint( m_active_joints[i] );
        p3d_obj* obj = jnt->getJointStruct()->o;

        if( obj )
        {
            //cout << "compute cost for joint " << jnt->getName() << endl;
            Eigen::Transform3d T = jnt->getMatrixPos();
            PointCloud& pc = m_surface_sampler->getPointCloud( obj );

            for(int j=0;j<int(pc.size());j++)
            {
                cost += m_ws_occupancy->getOccupancy( T*pc[j] );
                nb_points++;
            }
        }
    }

    //cout << "occupancy compueded for " << nb_points << endl;
    //cout << "HumanPredictionCostSpace cost : " << cost << endl;

    return cost;
}

double HumanPredictionCostSpace::getCostFromActiveJoints(Configuration& q)
{
    double cost=0.0;

    for(int i=0; i<int(m_active_joints.size()); i++)
    {
        cost += 10*m_ws_occupancy->getOccupancy( m_robot->getJoint( m_active_joints[i] )->getVectorPos() );
    }

    return cost;
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


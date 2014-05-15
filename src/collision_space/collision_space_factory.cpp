#include "collision_space_factory.hpp"

#include "body_surface_sampler.hpp"
#include "collision_space.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/libmove3d_api.hpp"
#include "API/libmove3d_simple_api.hpp"

#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planner/planEnvironment.hpp"

#include "hri_costspace/HRICS_costspace.hpp"

#include "utils/multilocalpath_utils.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Planner-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

CollisionSpace* global_collSpace=NULL;

static std::vector<int> m_active_joints;
static std::vector<int> m_planner_joints;
static std::vector<CollisionPoint> m_collision_points;

static bool m_add_human = false;
static bool m_init=false;

static Robot* m_robot=NULL;

void traj_optim_add_human_to_collision_space(bool add)
{
    m_add_human = add;
}

const CollisionSpace* traj_optim_get_collision_space()
{
    return global_collSpace;
}

std::vector<CollisionPoint> traj_optim_get_collision_points()
{
    return m_collision_points;
}

std::vector<int> traj_optim_get_active_joints()
{
    return m_active_joints;
}

std::vector<int> traj_optim_get_planner_joints()
{
    return m_planner_joints;
}

//! initializes the collision points
// --------------------------------------------------------
bool traj_optim_init_collision_points()
{
    // Generate Bounding volumes for active joints
    BodySurfaceSampler* sampler = global_collSpace->getBodySampler();

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    std::vector<Joint*> joints;
    joints.clear();

    for (unsigned int i=0; i<m_active_joints.size(); i++)
    {
        joints.push_back( m_robot->getJoint( m_active_joints[i] ) );
    }
    /*double maxRadius =*/ sampler->generateRobotBoudingCylinder( m_robot, joints );

    // Get all planner joint and compute collision points
    std::vector<int> planner_joints_id;
    for (unsigned int i=0; i<m_planner_joints.size(); i++)
    {
        planner_joints_id.push_back( m_planner_joints[i] );
    }
    m_collision_points = sampler->generateRobotCollisionPoints( m_robot, m_active_joints, planner_joints_id );

    cout << "nb of collision point are " << m_collision_points.size() << endl;

    // Set the collision space as global (drawing)
    global_collisionSpace = global_collSpace;
    return true;
}

//! initializes the collision space
// --------------------------------------------------------
void traj_optim_init_collision_space()
{
    //    std::vector<double> env_size = global_Project->getActiveScene()->getBounds();
    //    double pace = env_size[1] - env_size[0];
    //    pace = max( env_size[3] - env_size[3], pace);
    //    pace = max( env_size[4] - env_size[5], pace);
    //    pace /= ENV.getInt(Env::nbCells);


    //    ChronoTimeOfDayOn();

    global_collSpace = new CollisionSpace( m_robot, double(ENV.getInt(Env::nbCells))/100, global_Project->getActiveScene()->getBounds() );
    global_collSpace->resetPoints();

    // Warning
    // If not human not add bodies
    if( m_robot->getName().find("HERAKLES") == std::string::npos )
    {
//        cout << "robot name : " << m_robot->getName() << endl;
//        cout << "Add robot bodies exit " << endl; exit(0);

        for (unsigned int joint_id=0; joint_id<m_robot->getNumberOfJoints(); joint_id++)
        {
            if ( find( m_active_joints.begin(), m_active_joints.end(), joint_id ) == m_active_joints.end() )
            {
                global_collSpace->addRobotBody( m_robot->getJoint(joint_id) );
            }
        }
    }

    if( m_add_human )
    {
        Scene* sc = global_Project->getActiveScene();

        for (unsigned int i=0; i<sc->getNumberOfRobots(); i++)
        {
            Robot* rob = sc->getRobot(i);

            if (  ( m_robot != rob ) && rob->getName().find("HERAKLES") != std::string::npos )
            {
                global_collSpace->addRobot( rob );
            }
        }
    }

    // Add all moving and static obstacles
    global_collSpace->addEnvPoints();

    // Adds the sampled points to the distance field
    global_collSpace->propagateDistance();

    //    double time=0.0;
    //    ChronoTimeOfDayTimes(&time);
    //    ChronoTimeOfDayOff();
    //    cout << " collision space computed in : " << time << endl;
}

//****************************************************************
//* Default example
//****************************************************************

bool traj_optim_default_init()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    m_active_joints.clear();
    m_planner_joints.clear();

    m_active_joints = m_robot->getActiveJointsIds();
    m_planner_joints = m_active_joints;

//    for (int i=1; i<int(m_robot->getNumberOfJoints()); i++)
//    {
//        // Set the active joints (links)
//        m_active_joints.push_back( i );

//        // Set the planner joints
//        m_planner_joints.push_back( i );
//    }

    global_collSpace = NULL;

    if( ENV.getBool(Env::isCostSpace) )
    {
        return (global_costSpace != NULL);
    }
    if( move3d_use_api_functions_collision_space() )
    {
        if( global_collisionSpace == NULL )
        {
            traj_optim_init_collision_space();
            traj_optim_init_collision_points();
        }
        else {
            global_collSpace = global_collisionSpace;
        }
    }

    return true;
}

//****************************************************************
//* 2D Simple example
//****************************************************************

//! Initializes the optimization for a costspace
// --------------------------------------------------------
bool traj_optim_simple_init()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back( 1 );

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back( 1 );

    ENV.setInt( Env::jntToDraw, 1 );

    global_collSpace = NULL;
    return (global_costSpace != NULL);
}

//****************************************************************
//* 2D Costmap example
//****************************************************************

//! Initializes the optimization for a costspace
// --------------------------------------------------------
bool traj_optim_costmap_init()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back( 1 );

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back( 1 );

    ENV.setInt( Env::jntToDraw, 1 );

    global_collSpace = NULL;

    bool valid_costspace = global_costSpace->setCost("costMap2D");
    return valid_costspace;
}

//****************************************************************
//* Navigation example
//****************************************************************


//! Sets the point on the navigation DoF
void traj_optim_navigation_generate_points()
{
    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back( 1 );

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back( 1 );

    // Generate Bounding volumes for active joints
    BodySurfaceSampler* sampler = global_collSpace->getBodySampler();

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    std::vector<Joint*> joints;
    joints.clear();
    for (unsigned int i=0; i<m_active_joints.size(); i++)
    {
        joints.push_back( m_robot->getJoint( m_active_joints[i] ) );
    }
    sampler->generateRobotBoudingCylinder( m_robot, joints );

    // Get all planner joint and compute collision points
    std::vector<int> planner_joints_id;
    for (unsigned int i=0; i<m_planner_joints.size(); i++)
    {
        planner_joints_id.push_back( m_planner_joints[i] );
    }
    m_collision_points = sampler->generateRobotCollisionPoints( m_robot, m_active_joints, planner_joints_id );

    //  sampler->generateRobotBoudingCylinder( m_robot, m_robot->getAllJoints() );
    //  m_collision_points = sampler->generateAllRobotCollisionPoints( m_robot );
}

//****************************************************************
//* HRICS example
//****************************************************************

//! Sets the point on the navigation DoF
void traj_optim_hrics_generate_points()
{
    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back( 1 );

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back( 1 );

    // Generate Bounding volumes for active joints
    BodySurfaceSampler* sampler = global_collSpace->getBodySampler();

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    std::vector<Joint*> joints;
    joints.clear();
    for (unsigned int i=0; i<m_active_joints.size(); i++)
    {
        joints.push_back( m_robot->getJoint( m_active_joints[i] ) );
    }
    sampler->generateRobotBoudingCylinder( m_robot, joints );

    // Get all planner joint and compute collision points
    std::vector<int> planner_joints_id;
    for (unsigned int i=0; i<m_planner_joints.size(); i++)
    {
        planner_joints_id.push_back( m_planner_joints[i] );
    }
    m_collision_points = sampler->generateRobotCollisionPoints( m_robot, m_active_joints, planner_joints_id );

    //  sampler->generateRobotBoudingCylinder( m_robot, m_robot->getAllJoints() );
    //  m_collision_points = sampler->generateAllRobotCollisionPoints( m_robot );
}

//! initializes the collision space
// --------------------------------------------------------
void traj_optim_manip_init_joints()
{
    global_collSpace = NULL;

    if( m_robot->getName() == "PR2_ROBOT" ) {
        // Set the active joints (links)
        m_active_joints.clear();
        m_active_joints.push_back( 6 );
        m_active_joints.push_back( 7 );
        m_active_joints.push_back( 8 );
        m_active_joints.push_back( 9 );
        m_active_joints.push_back( 10 );
        m_active_joints.push_back( 11 );
        m_active_joints.push_back( 12 );
        m_active_joints.push_back( 14 );
        m_active_joints.push_back( 15 );

        // Set the planner joints
        m_planner_joints.clear();
        m_planner_joints.push_back( 6 );
        m_planner_joints.push_back( 7 );
        m_planner_joints.push_back( 8 );
        m_planner_joints.push_back( 9 );
        m_planner_joints.push_back( 10 );
        m_planner_joints.push_back( 11 );
        m_planner_joints.push_back( 12 );
    }

    if( m_robot->getName() == "JUSTIN_ROBOT" ) {
        // Set the active joints (links)
        m_active_joints.clear();
        m_active_joints.push_back( 2 );
        m_active_joints.push_back( 3 );
        m_active_joints.push_back( 4 );
        m_active_joints.push_back( 5 );
        m_active_joints.push_back( 17 );
        m_active_joints.push_back( 18 );
        m_active_joints.push_back( 19 );
        m_active_joints.push_back( 20 );
        m_active_joints.push_back( 21 );
        m_active_joints.push_back( 22 );

        m_active_joints.push_back( 23 );

        // Set the planner joints
        m_planner_joints.clear();
        m_planner_joints.push_back( 2 );
        m_planner_joints.push_back( 3 );
        m_planner_joints.push_back( 4 );
        m_planner_joints.push_back( 5 );
        m_planner_joints.push_back( 18 );
        m_planner_joints.push_back( 19 );
        m_planner_joints.push_back( 20 );
        m_planner_joints.push_back( 21 );
        m_planner_joints.push_back( 22 );
        m_planner_joints.push_back( 23 );
        m_planner_joints.push_back( 24 );
        //    m_planner_joints.clear();
        //    m_planner_joints.push_back( 2 );
        //    m_planner_joints.push_back( 3 );
        //    m_planner_joints.push_back( 4 );
        //    m_planner_joints.push_back( 30 );
    }
}

//! initializes the collision space
//! and points
// --------------------------------------------------------
void traj_optim_hrics_mobile_manip_init_joints()
{
    // Set the active joints (links)
    m_active_joints.clear();
    m_active_joints.push_back( 1 );
    m_active_joints.push_back( 6 );
    m_active_joints.push_back( 7 );
    m_active_joints.push_back( 8 );
    m_active_joints.push_back( 9 );
    m_active_joints.push_back( 10 );
    m_active_joints.push_back( 11 );
    m_active_joints.push_back( 12 );

    m_active_joints.push_back( 14 );
    m_active_joints.push_back( 15 );

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back( 1 );
    m_planner_joints.push_back( 6 );
    m_planner_joints.push_back( 7 );
    m_planner_joints.push_back( 8 );
    m_planner_joints.push_back( 9 );
    m_planner_joints.push_back( 10 );
    m_planner_joints.push_back( 11 );
    m_planner_joints.push_back( 12 );

    global_collSpace = NULL;
}

//! initializes the collision space
//! and points
// --------------------------------------------------------
void traj_optim_hrics_human_trajectory_manip_init_joints()
{
    // Set the active joints (links)

    m_active_joints.clear();
    m_active_joints.push_back( 1 ); // Pelvis
//    m_active_joints.push_back( 2 ); // TorsoX
//    m_active_joints.push_back( 3 ); // TorsoY
    m_active_joints.push_back( 4 ); // TorsoZ
//    m_active_joints.push_back( 8 ); // rShoulderX
//    m_active_joints.push_back( 9 ); // rShoulderZ
    m_active_joints.push_back( 10 ); // rShoulderY
//    m_active_joints.push_back( 11 ); // rArmTrans
    m_active_joints.push_back( 12 ); // rElbowZ
//    active_joints_.push_back(14); // joint name : rWristX
//    active_joints_.push_back(15); // joint name : rWristY
    m_active_joints.push_back(16); // joint name : rWristZ

    // Set the planner joints
    m_planner_joints.clear();
    m_planner_joints.push_back( 1 );
    m_planner_joints.push_back( 2 );
    m_planner_joints.push_back( 3 );
    m_planner_joints.push_back( 4 );
    m_planner_joints.push_back( 8 );
    m_planner_joints.push_back( 9 );
    m_planner_joints.push_back( 10 );
    m_planner_joints.push_back( 11 );
    m_planner_joints.push_back( 12 );
//    m_planner_joints.push_back( 13 );

    global_collSpace = NULL;
}

//****************************************************************
//* Common functions
//****************************************************************

//! Get current robot
//! Initializes the costspace and multi localpath
// --------------------------------------------------------
bool traj_optim_init_collision_spaces( traj_optim::ScenarioType sce, Robot* rob )
{
    if( m_init == true )
        return true;

    m_robot = rob;

    cout << "Robot is : " << m_robot->getName() << " in " << __PRETTY_FUNCTION__ << endl;

    if( m_robot == NULL )
        return false;

    switch( sce )
    {
    case traj_optim::Default:
        cout << "Init with default parameters" << endl;
        if( !traj_optim_default_init() )
            return false;
        break;

    case traj_optim::CostMap:

        cout << "Init with 2D costmap" << endl;
        if( !traj_optim_costmap_init() )
            return false;
        // PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.1);
        // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
        // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
        // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.001);
        break;

    case traj_optim::Simple:

        cout << "Init Simple Nav" << endl;
        if( !traj_optim_simple_init() )
            return false;
        // PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.030000);
        // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
        // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
        // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.000005);
        break;

    case traj_optim::Shelf:

        cout << "Init Shelf" << endl;
        cout << "Set robot, localpath and cntrts with " << m_robot->getName() << endl;

        if( move3d_use_api_functions() )
        {
            // traj_optim_set_MultiLP();
            // traj_optim_invalidate_cntrts();
            traj_optim_manip_init_joints();
            traj_optim_shelf_set_localpath_and_cntrts();

            if( !global_collisionSpace )
            {
                traj_optim_init_collision_space();
                traj_optim_init_collision_points();
            }
            else {
                global_collSpace = global_collisionSpace;
            }
        }
        else {
            traj_optim_default_init();
        }

        // PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
        // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,15);
        // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
        // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);

        // PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
        // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
        // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,30);
        // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.01);
        break;

    case traj_optim::HumanAwareNav:

        cout << "Init HumanAwareNav" << endl;
        cout << "Set robot, localpath and cntrts with ";
        cout << m_robot->getName() << endl;

        traj_optim_set_MultiLP();
        traj_optim_invalidate_cntrts();
        traj_optim_navigation_set_localpath_and_cntrts();

        if( !HRICS_humanCostMaps ) {
            cout << "No Collspace" << endl;
            return false;
        }
        else {
            global_collSpace = global_collisionSpace;
            traj_optim_navigation_generate_points();
        }

        PlanEnv->setDouble(PlanParam::trajOptimStdDev,3);
        PlanEnv->setInt(PlanParam::nb_pointsOnTraj,30);
        PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,20);
        PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);
        break;

    case traj_optim::HumanAwareManip:

        cout << "Init HumanAwareManip" << endl;
        cout << "Set robot, localpath and cntrts with ";
        cout << m_robot->getName() << endl;

        traj_optim_set_MultiLP();

        if( m_robot->getName() == "PR2_ROBOT" )
        {
            traj_optim_invalidate_cntrts();
            traj_optim_shelf_set_localpath_and_cntrts();
            traj_optim_manip_init_joints();

            // Init collspace
            if( !global_collisionSpace )
            {
                traj_optim_init_collision_space();
                traj_optim_init_collision_points();
            }
            else {
                global_collSpace = global_collisionSpace;
            }
        }
        else {
            traj_optim_init_mlp_cntrts_and_fix_joints( m_robot );
            traj_optim_manip_init_joints();
        }

        // PlanEnv->setDouble(PlanParam::trajOptimStdDev,3);
        // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,30);
        // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,20);
        // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);
        break;

    case traj_optim::HumanAwareMobileManip:

        cout << "Init HumanAwareMobileManip" << endl;
        cout << "Set robot, localpath and cntrts with ";
        cout << m_robot->getName() << endl;

        traj_optim_set_MultiLP();
        traj_optim_invalidate_cntrts();
        traj_optim_hrics_mobile_manip_localpath_and_cntrts();
        traj_optim_hrics_mobile_manip_init_joints();
        break;

    case traj_optim::HumanSimulation:

        cout << "Init Human Simulation (Trajectory costspace)" << endl;

        traj_optim_hrics_human_trajectory_manip_init_joints();

        // Init collspace
        if( !global_collisionSpace )
        {
            traj_optim_init_collision_space();
            traj_optim_init_collision_points();
        }
        else {
            global_collSpace = global_collisionSpace;
        }

        break;

    case traj_optim::Navigation:

        cout << "Init Navigation" << endl;
        cout << "Set robot, localpath and cntrts with ";
        cout << m_robot->getName() << endl;

        traj_optim_set_MultiLP();
        traj_optim_invalidate_cntrts();
        traj_optim_navigation_set_localpath_and_cntrts();

        if( !global_collisionSpace )
        {
            cout << "No Collspace" << endl;
            return false;
        }
        else
        {
            global_collSpace = global_collisionSpace;
            traj_optim_navigation_generate_points();
        }

        PlanEnv->setDouble(PlanParam::trajOptimStdDev,3);
        PlanEnv->setInt(PlanParam::nb_pointsOnTraj,30);
        PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,20);
        PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);
        break;

    case traj_optim::LegiblePlane:

        cout << "Init LegiblePlane" << endl;
        cout << "Set robot, localpath and cntrts with ";
        cout << m_robot->getName() << endl;

        if( !traj_optim_default_init() )
            return false;
        break;

//        traj_optim_set_MultiLP();
//        traj_optim_invalidate_cntrts();
//        traj_optim_navigation_set_localpath_and_cntrts();
        break;
    }

    return true;
}

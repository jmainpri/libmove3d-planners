//  trajectoryOptim.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//
// généraliser chomp => init codé en dur et ne tenant pas entièrement compte de la GUI
// régler les prob de freeze
// regarder le collision checker : trajectoire valide mais en collision à l'éxécution
// cinématique inverse à fond
// scénario du facteur + costmap

#include "trajectoryOptim.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "collision_space/CollisionSpace.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include "Chomp/chompTrajectory.hpp"
#include "Chomp/chompOptimizer.hpp"
#include "Chomp/chompParameters.hpp"

#include "Stomp/stompOptimizer.hpp"
#include "Stomp/stompParameters.hpp"

#include "hri_costspace/HRICS_costspace.hpp"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif
#include "Graphic-pkg.h"
#include "move3d-headless.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

//--------------------------------------------------------
// Variables
//--------------------------------------------------------
static bool m_init = false;
static bool m_add_human = false;
static bool m_plan_for_human = false;

static Robot* m_robot = NULL;

namespace traj_optim
{
enum ScenarioType {
    Default,
    CostMap,
    Simple,
    Shelf,
    Navigation,
    HumanAwareNav,
    HumanAwareManip,
    HumanAwareMobileManip,
    HumanSimulation,
    LegiblePlane };
}

static traj_optim::ScenarioType m_sce;

enum PlanningType { DEFAULT = -1, NAVIGATION = 0, MANIPULATION = 1, MOBILE_MANIP = 2 };
static PlanningType m_planning_type; 

static CollisionSpace* m_coll_space=NULL;

static ChompPlanningGroup* m_chompplangroup= NULL;
static ChompTrajectory* m_chomptraj=NULL;
static ChompParameters* m_chompparams=NULL;

stomp_motion_planner::StompParameters* m_stompparams=NULL;

static int m_ArmId=0;

static int m_BaseMLP=0;
static int m_BaseSmMLP=0;
static int m_HeadMLP=0;
static int m_UpBodyMLP=0;
static int m_UpBodySmMLP=0;

static vector<int> m_active_joints;
static vector<int> m_planner_joints;
static vector<CollisionPoint> m_collision_points;

// External variable
vector< vector <double> > traj_optim_to_plot;
vector< vector <double> > traj_optim_convergence;

CollisionSpace* global_collSpace=NULL;

static bool m_use_iteration_limit=false;
static double m_max_iteration;

static bool m_use_external_trajectory=false;
static API::Trajectory m_external_trajectory;

static bool m_discretize=false;
static double m_discretization=0.0;

//--------------------------------------------------------
// External init method
//--------------------------------------------------------
//void set_robot_active_joints()
//{    
//  if( PlanEnv->getBool(PlanParam::setActiveDofs) ) 
//  {
//    // This function sets 
//    // the type of dof that are going to be active
//    traj_optim_init_mlp_cntrts_and_fix_joints();
//  }
//}

//--------------------------------------------------------
// General method
//--------------------------------------------------------

//! set mlp for this robot
bool traj_optim_set_MultiLP()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    for (int i = 0; m_robot && i < m_robot->getRobotStruct()->mlp->nblpGp; i++) {
        if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
            m_BaseMLP = i;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "baseSm")) {
            m_BaseSmMLP = i;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
            m_HeadMLP = i;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
            m_UpBodyMLP = i;
        } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
            m_UpBodySmMLP = i;
        }
    }

    return true;
}

//! invalidate all constraints
// --------------------------------------------------------
bool traj_optim_invalidate_cntrts()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    p3d_rob* rob = m_robot->getRobotStruct();
    p3d_cntrt* ct;

    // over all constraints
    for(int i=0; i<rob->cntrt_manager->ncntrts; i++)
    {
        string name = rob->cntrt_manager->cntrts[i]->namecntrt;

        if ( m_robot->getName() == "JUSTIN_ROBOT" && (name == "p3d_min_max_dofs" || name == "p3d_lin_rel_dofs" || name == "p3d_fixed_jnt" ) ) {
            continue;
        }

        if( name == "p3d_fix_jnts_relpos" ){
            continue;
        }

        cout << "deactivate : " << m_robot->getName() << " , " << name << endl;

        // get constraint from the cntrts manager
        ct = rob->cntrt_manager->cntrts[i];
        p3d_desactivateCntrt( rob, ct );
    }

    return true;
}

//! cntrts and fix
// --------------------------------------------------------
bool traj_optim_init_mlp_cntrts_and_fix_joints()
{
    m_robot = global_Project->getActiveScene()->getActiveRobot();

    cout << "Initialize robot : " << m_robot->getName() << endl;

    if( !traj_optim_set_MultiLP() ) {
        return false;
    }

    if( true /*m_robot->getName() != "JUSTIN_ROBOT"*/ )
    {
        if( !traj_optim_invalidate_cntrts() ) {
            return false;
        }

        //    if( !traj_optim_switch_cartesian_mode(false) ) {
        //      return false;
        //    }
    }

    switch( ENV.getInt(Env::setOfActiveJoints) )
    {
    case 0 : // Navigation
        cout << "Set navigation parameters" << endl;
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);
        fixAllJointsExceptBase( m_robot->getRobotStruct() );
        break;

    case 1 : // Manipulation
    {
        cout << "Set manipulation parameters" << endl;
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);

        m_ArmId = 0;
        if( m_robot->getName() == "JUSTIN_ROBOT" )
            m_ArmId = 1;

        fixAllJointsWithoutArm( m_robot->getRobotStruct() , m_ArmId );

        if( m_robot->getName() == "JUSTIN_ROBOT" )
        {
            unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->joints[2] );
            unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->joints[3] );
            unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->joints[4] );
            unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->joints[5] );
            // Unfix virtual joint
            //unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->joints[30] );
        }
    }
        break;

    case 2 : // Mobile Manipulation
        cout << "Set mobile-manipulation parameters" << endl;
        p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
        p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
        fixAllJointsWithoutArm( m_robot->getRobotStruct() , 0 );
        unFixJoint(m_robot->getRobotStruct(), m_robot->getRobotStruct()->baseJnt);
        break;
    }

    return true;
}

//! Virtual object
// --------------------------------------------------------
bool traj_optim_switch_cartesian_mode(bool cartesian) {

    if( m_robot == NULL ) {
        cout << "m_robot == NULL : traj_optim not initilized" << endl;
        return false;
    }

    cout << "Set Cartesian (" << cartesian << ")" << endl;

    p3d_rob* object = NULL;
    p3d_rob* robot = m_robot->getRobotStruct();

    configPt q = p3d_get_robot_config(robot);

    // For Each Arm
    for (int i=0; i < int((*robot->armManipulationData).size()); i++)
    {
        if(  m_robot->getName() == "JUSTIN_ROBOT" && m_ArmId != i )
            continue;

        ArmManipulationData& armData  = (*robot->armManipulationData)[i];

        if( cartesian && m_ArmId==i )
            armData.setCartesian( true );
        else
            armData.setCartesian( false );

        desactivateTwoJointsFixCntrt(robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);

        if (armData.getCartesian())
        {
            cout << "Arm Data " << i <<  " is set cartesian" << endl;
            // Uptdate the Virual object for inverse kinematics
            // Be carfull the Arm will be unfixed
            //p3d_update_virtual_object_config_for_arm_ik_constraint(robot, i, q);
            activateCcCntrts(robot, i, false);
            ManipulationUtils::unfixManipulationJoints(robot, i);
            shootTheObjectArroundTheBase(robot, robot->baseJnt, armData.getManipulationJnt(), 2.0);

            if(object)
                armData.getManipulationJnt()->dist = object->joints[1]->dist;
            else
                armData.getManipulationJnt()->dist  = 0.10;

        } else {
            deactivateCcCntrts(robot, i);
            //p3d_update_virtual_object_config_for_arm_ik_constraint(robot, i, q);
            p3d_set_and_update_this_robot_conf(robot, q);
            setAndActivateTwoJointsFixCntrt(robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
            ManipulationUtils::fixManipulationJoints(robot, i, q, NULL);
            shootTheObjectInTheWorld(robot, armData.getManipulationJnt());
        }
    }
    p3d_set_and_update_this_robot_conf(robot, q);
    p3d_get_robot_config_into(robot, &q);
    p3d_destroy_config(robot, q);
    return true;
}

//! Create an initial Move3D trajectory
//! it generates a straigt line between the two configuration init 
//! and goal
// --------------------------------------------------------
API::Trajectory traj_optim_create_sraight_line_traj()
{
    if (!m_robot) {
        cout << "robot not initialized in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
    }

    confPtr_t q_init( m_robot->getInitPos() );
    confPtr_t q_goal( m_robot->getGoalPos() );

    if( q_init->equal( *q_goal ) )
    {
        cout << "init equal q_goal in file "
             << __FILE__ << " ,  " << __PRETTY_FUNCTION__ << endl;
    }

    vector<confPtr_t> confs(2);

    confs[0] = q_init;
    confs[1] = q_goal;

    API::Trajectory T( confs );

    return T;
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

    for (int i=1; i<int(m_robot->getNumberOfJoints()); i++)
    {
        // Set the active joints (links)
        m_active_joints.push_back( i );

        // Set the planner joints
        m_planner_joints.push_back( i );
    }

    m_coll_space = NULL;
    return (global_costSpace != NULL);
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

    m_coll_space = NULL;
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

    m_coll_space = NULL;

    bool valid_costspace = global_costSpace->setCost("costMap2D");
    return valid_costspace;
}

//! Generate A Soft Motion Trajectory
// --------------------------------------------------------
bool traj_optim_generate_pointsOnTraj()
{
    //  if ( m_robot->getName() != traj.getRobot()->getName() )
    //  {
    //    cout << "trajectory not for propper robot" << endl;
    //    return false;
    //  }
    cout << "Generate Points on Vias" << endl;

    if ( m_robot == NULL )
        m_robot = global_Project->getActiveScene()->getActiveRobot();

    API::Trajectory traj = m_robot->getCurrentTraj();

    int nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );
    int nb_via = traj.getNbOfViaPoints()-2;

    if( nb_via < 1 )
    {
        cout << "not enought via" << endl;
        return false;
    }

    int nb_points_per_via = ceil( nb_points/nb_via );

    API::Trajectory new_traj( m_robot );

    double param=0.0;
    double delta = 0.0;
    double density = 4;
    double step = traj.getRangeMax()/(nb_points*density);

    for (int i=0; i<traj.getNbOfPaths(); i++ )
    {
        param += traj.getLocalPath( i )->getParamMax();
        delta = -step*density;

        for(int j=0; j<nb_points_per_via; j++ )
        {
            new_traj.push_back( traj.configAtParam( param+delta ));
            delta += step;
        }
    }
    traj = new_traj;
    traj.replaceP3dTraj();
    return true;
}

//! Generate A Soft Motion Trajectory
// --------------------------------------------------------
bool traj_optim_generate_softMotion()
{
    if ( m_robot == NULL )
        m_robot = global_Project->getActiveScene()->getActiveRobot();

    if( !traj_optim_set_MultiLP() )
        return false;

    API::Trajectory T = m_robot->getCurrentTraj();

    if( T.isEmpty() )
    {
        cout << "The robot has no current traj!!!" << endl;
        return false;
    }

    cout << "m_UpBodyMLP = " << m_UpBodyMLP << endl;

    // T.cutTrajInSmallLP( 10 );
    // T.replaceP3dTraj();
    // T.print();

    MANPIPULATION_TRAJECTORY_CONF_STR confs;
    SM_TRAJ smTraj;

    p3d_multiLocalPath_disable_all_groupToPlan(m_robot->getRobotStruct(), FALSE);
    p3d_multiLocalPath_set_groupToPlan(m_robot->getRobotStruct(), m_UpBodyMLP, 1, FALSE);
    smTraj.clear();

    API::Trajectory TSaved = T;

    //  p3d_traj * trajPt
    //  bool param_write_file
    //  bool approximate
    //  std::vector < int >&lp
    //  std::vector < std::vector <double > > &positions
    //  SM_TRAJ & smTraj
    p3d_convert_traj_to_softMotion(m_robot->getRobotStruct()->tcur,
                                   ENV.getBool(Env::smoothSoftMotionTraj),
                                   true,
                                   false,
                                   confs.first, confs.second, smTraj);

    //smTraj.plot();

    T = m_robot->getCurrentTraj();
    double delta = T.getRangeMax() / (100-1) ;

    p3d_multiLocalPath_disable_all_groupToPlan(m_robot->getRobotStruct(), FALSE);
    p3d_multiLocalPath_set_groupToPlan(m_robot->getRobotStruct(), m_UpBodyMLP, 1, FALSE);

    API::Trajectory newT(m_robot);
    cout << "delta = " << delta << endl;

    double t = 0.0;

    for (int i=0; i<100; i++ )
    {
        newT.push_back( T.configAtParam(t) );
        t += delta;

    }
    newT.replaceP3dTraj();

    //  // Set to plot
    //  traj_optim_to_plot.clear();
    //  vector<double> x,y;
    //  x.resize(100);
    //  y.resize(100);
    //
    //  // SoftMotion
    //  int i=0;
    //  delta = T.getRangeMax()/100;
    //
    //  for (double t=0; t<=T.getRangeMax(); t += delta )
    //  {
    //    shared_ptr<Configuration> q = T.configAtParam(t);
    //    x[i] = (*q)[6];
    //    y[i] = (*q)[7];
    //    i++;
    //
    //    if (i >= int(x.size()) || i >= int(y.size()))
    //      break;
    //  }
    //  traj_optim_to_plot.push_back( x );
    //  traj_optim_to_plot.push_back( y );
    //
    //  // Initial traj
    //  i=0;
    //  delta = TSaved.getRangeMax()/100;
    //
    //  for (double t=0; t<=TSaved.getRangeMax(); t += delta )
    //  {
    //    shared_ptr<Configuration> q = TSaved.configAtParam(t);
    //    x[i] = (*q)[6];
    //    y[i] = (*q)[7];
    //    i++;
    //
    //    if (i >= int(x.size()) || i >= int(y.size()))
    //      break;
    //  }
    //  traj_optim_to_plot.push_back( x );
    //  traj_optim_to_plot.push_back( y );
    return true;
}

//****************************************************************
//* Shelf example
//****************************************************************

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_shelf_set_localpath_and_cntrts()
{
    cout << "Set robot, localpath and cntrts" << endl;
#ifdef MULTILOCALPATH
    traj_optim_set_MultiLP();
    traj_optim_invalidate_cntrts();

    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);

    fixAllJointsWithoutArm( m_robot->getRobotStruct() , 0 );
#endif
    //ENV.setInt( Env::jntToDraw, 28 );
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

    m_coll_space = new CollisionSpace( m_robot, double(ENV.getInt(Env::nbCells))/100, global_Project->getActiveScene()->getBounds() );

    m_coll_space->resetPoints();

    // Warning
    // If not human not add bodies
    if( m_robot->getName().find("HERAKLES") == string::npos )
    {
//        cout << "robot name : " << m_robot->getName() << endl;
//        cout << "Add robot bodies exit " << endl; exit(0);

        for (unsigned int joint_id=0; joint_id<m_robot->getNumberOfJoints(); joint_id++)
        {
            if ( find (m_active_joints.begin(), m_active_joints.end(), joint_id ) == m_active_joints.end() )
            {
                m_coll_space->addRobotBody( m_robot->getJoint(joint_id) );
            }
        }
    }

    if( m_add_human )
    {
        Scene* sc = global_Project->getActiveScene();

        for (unsigned int i=0; i<sc->getNumberOfRobots(); i++)
        {
            Robot* rob = sc->getRobot(i);

            if (  ( m_robot != rob ) && rob->getName().find("HERAKLES") != string::npos )
            {
                m_coll_space->addRobot( rob );
            }
        }
    }

    // Add all moving and static obstacles
    m_coll_space->addEnvPoints();

    // Adds the sampled points to the distance field
    m_coll_space->propagateDistance();

    //    double time=0.0;
    //    ChronoTimeOfDayTimes(&time);
    //    ChronoTimeOfDayOff();
    //    cout << " collision space computed in : " << time << endl;
}

//! initializes the collision points
// --------------------------------------------------------
bool traj_optim_init_collision_points()
{
    // Generate Bounding volumes for active joints
    BodySurfaceSampler* sampler = m_coll_space->getBodySampler();

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    vector<Joint*> joints;
    joints.clear();

    for (unsigned int i=0; i<m_active_joints.size(); i++)
    {
        joints.push_back( m_robot->getJoint( m_active_joints[i] ) );
    }
    /*double maxRadius =*/ sampler->generateRobotBoudingCylinder( m_robot, joints );

    // Get all planner joint and compute collision points
    vector<int> planner_joints_id;
    for (unsigned int i=0; i<m_planner_joints.size(); i++)
    {
        planner_joints_id.push_back( m_planner_joints[i] );
    }
    m_collision_points = sampler->generateRobotCollisionPoints( m_robot, m_active_joints, planner_joints_id );

    cout << "nb of collision point are " << m_collision_points.size() << endl;

    // Set the collision space as global (drawing)
    global_collisionSpace = m_coll_space;
    return true;
}

//****************************************************************
//* Navigation example
//****************************************************************

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_navigation_set_localpath_and_cntrts()
{
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);

    fixAllJointsExceptBase( m_robot->getRobotStruct() );
#endif

    ENV.setInt( Env::jntToDraw, 1 );
}

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
    BodySurfaceSampler* sampler = m_coll_space->getBodySampler();

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    vector<Joint*> joints;
    joints.clear();
    for (unsigned int i=0; i<m_active_joints.size(); i++)
    {
        joints.push_back( m_robot->getJoint( m_active_joints[i] ) );
    }
    sampler->generateRobotBoudingCylinder( m_robot, joints );

    // Get all planner joint and compute collision points
    vector<int> planner_joints_id;
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

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_hrics_set_localpath_and_cntrts()
{
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);

    fixAllJointsExceptBase( m_robot->getRobotStruct() );
#endif

    ENV.setInt( Env::jntToDraw, 1 );
}

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
    BodySurfaceSampler* sampler = m_coll_space->getBodySampler();

    // Get all joints active in the motion planning
    // and compute bounding cylinders
    vector<Joint*> joints;
    joints.clear();
    for (unsigned int i=0; i<m_active_joints.size(); i++)
    {
        joints.push_back( m_robot->getJoint( m_active_joints[i] ) );
    }
    sampler->generateRobotBoudingCylinder( m_robot, joints );

    // Get all planner joint and compute collision points
    vector<int> planner_joints_id;
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
    m_coll_space = NULL;

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
//        m_active_joints.push_back( 14 );
//        m_active_joints.push_back( 15 );

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

//! initializes localpaths and cntrts for mobile manip
// --------------------------------------------------------
void traj_optim_hrics_mobile_manip_localpath_and_cntrts()
{
    cout << "Set mobile-manipulation parameters" << endl;
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
    fixAllJointsWithoutArm( m_robot->getRobotStruct() , 0 );
    unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->baseJnt );
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

    m_coll_space = NULL;
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

    m_coll_space = NULL;
}

//****************************************************************
//* Common functions 
//****************************************************************

//! Get current robot
//! Initializes the costspace and multi localpath
// --------------------------------------------------------
bool traj_optim_init_collision_spaces()
{
    if (m_init == true)
        return true;

    // THE ROBOT IS SET HERE!!!!
    m_robot = global_Project->getActiveScene()->getActiveRobot();
    //m_robot = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");

    cout << "Robot is : " << m_robot->getName() << " in " << __PRETTY_FUNCTION__ << endl;

    if( m_robot == NULL )
        return false;

    switch( m_sce )
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
            m_coll_space = global_collisionSpace;
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
            m_coll_space = global_collisionSpace;
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
                m_coll_space = global_collisionSpace;
            }
        }
        else {
            traj_optim_init_mlp_cntrts_and_fix_joints();
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
            m_coll_space = global_collisionSpace;
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
            m_coll_space = global_collisionSpace;
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

//! Set the type of scenario
//! Depending on global variables
// --------------------------------------------------------
bool traj_optim_set_scenario_type()
{  
    if( ENV.getBool(Env::isCostSpace) && ( global_costSpace == NULL )) {
        cout << "Cost space not initialized!!!" << endl;
        return false;
    }

    if( m_planning_type == DEFAULT )
    {
        if( ENV.getBool(Env::isCostSpace) && global_costSpace->getSelectedCostName() == "costMap2D" )
        {
            m_sce = traj_optim::CostMap;
        }
        //    else if( ENV.getBool(Env::isCostSpace) &&
        //            ( global_costSpace->getSelectedCostName() == "costIsInCollision" ||
        //             global_costSpace->getSelectedCostName() == "costDistToObst"  ))
        //    {
        //      m_sce = Simple;
        //    }
        else
        {
            m_sce = traj_optim::Default;
        }
    }
    else if( ENV.getBool(Env::isCostSpace) &&
             ( global_costSpace->getSelectedCostName() == "costHumanGrids" ||
               global_costSpace->getSelectedCostName() == "costHumanPredictionOccupancy" ) )
    {
        if( m_planning_type == NAVIGATION )
        {
            m_sce = traj_optim::HumanAwareNav;
        }
        if( m_planning_type == MANIPULATION )
        {
            m_sce = traj_optim::HumanAwareManip;
        }
        if( m_planning_type == MOBILE_MANIP )
        {
            m_sce = traj_optim::HumanAwareMobileManip;
        }
    }
    else if ( ENV.getBool(Env::isCostSpace) &&
              global_costSpace->getSelectedCostName() == "costHumanTrajecoryCost" )
    {
        if( m_planning_type == MANIPULATION )
        {
            cout << "Set human cost function" << endl;
            m_sce = traj_optim::HumanSimulation;
        }
    }
    else if (  PlanEnv->getBool(PlanParam::useLegibleCost) )
    {
        if( m_planning_type == NAVIGATION )
        {
            cout << "Set legible cost function" << endl;
            cout << "Set default" << endl;
            m_sce = traj_optim::LegiblePlane;
        }
    }
    else
    {
        const bool navigation = false;

        if( navigation )
            m_sce = traj_optim::Navigation;
        else
            m_sce = traj_optim::Shelf;
    }

    return true;
}

//! Choose what part of the robot are active in the planning phase
//! Three type of planning to chose from
//! Navigation, Manipulation, Mobile-anipulation
void traj_optim_init_planning_type(int type)
{
    switch (type)
    {
    case 0:
        m_planning_type = NAVIGATION;
        cout << "NAVIGATION planner type" << endl;
        break;

    case 1:
        m_planning_type = MANIPULATION;
        cout << "MANIPULATION planner type" << endl;
        break;

    case 2:
        m_planning_type = MOBILE_MANIP;
        cout << "MOBILE_MANIP planner type" << endl;
        break;

    default:
        m_planning_type = DEFAULT;
        cout << "DEFAULT planner type" << endl;
        break;
    }
}

//****************************************************************
//*   Run Functions 
//****************************************************************

bool traj_optim_initScenario()
{
    traj_optim_init_planning_type( ENV.getInt(Env::setOfActiveJoints) );

    if(!traj_optim_set_scenario_type()) {
        cout << "Not well initialized" << endl;
        return false;
    }

    if(!traj_optim_init_collision_spaces()){
        cout << "Not well initialized" << endl;
        return false;
    }

    m_init = true;
    return true;
}

//!
//! Get Initial trajectory
// --------------------------------------------------------
bool traj_optim_InitTraj(API::Trajectory& T)
{
    if( !m_init )
    {
        if(!traj_optim_initScenario())
        {
            return false;
        }
    }

    if( m_use_external_trajectory )
    {
        T = m_external_trajectory;
    }
    else if( PlanEnv->getBool(PlanParam::withCurrentTraj) )
    {
        T = m_robot->getCurrentTraj();
    }
    else
    {
        T = traj_optim_create_sraight_line_traj();
    }

    if( T.getNbOfPaths() == 0 )
        return false;

    int nb_points = 0;

    if( m_discretize )
    {
        nb_points = floor(T.getRangeMax() / m_discretization );
    }
    else
    {
        nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );
    }

    T.cutTrajInSmallLP( nb_points );
    T.replaceP3dTraj();

    cout << "End Init traj" << endl;
    return true;
}

//! Chomp
// --------------------------------------------------------
bool traj_optim_runChomp()
{
    API::Trajectory T(m_robot);

    if( !traj_optim_InitTraj(T) ){
        return false;
    }

    int nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );

    T.cutTrajInSmallLP( nb_points );
    T.replaceP3dTraj();

    g3d_draw_allwin_active();

    // Create Optimizer
    // ----------------
    m_chompparams = new ChompParameters;
    m_chompparams->init();

    m_chompplangroup = new ChompPlanningGroup( m_robot, m_planner_joints );
    m_chompplangroup->collision_points_ = m_collision_points;

    m_chomptraj = new ChompTrajectory( T, DIFF_RULE_LENGTH, *m_chompplangroup );
    m_chomptraj->print();
    cout << "chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;
    cout << "Initialize optimizer" << endl;

    ChompOptimizer optimizer( m_chomptraj, m_chompparams, m_chompplangroup, m_coll_space );
    cout << "Optimizer created" << endl;

    optimizer.runDeformation(0,0);
    return true;
}

//! Init Stomp
// --------------------------------------------------------
bool traj_optim_initStomp()
{
    cout << "----------------------------------" << endl;
    cout << " Init Stomp ----------------------" << endl;
    API::Trajectory T(m_robot);

    if( !traj_optim_InitTraj(T) )
    {
        cout << "Error in traj_optim_InitTraj" << endl;
        return false;
    }

    // Save passive dof to generate the Move3D trajectory
    std::vector<confPtr_t> passive_dofs = T.getVectorOfConfiguration();

    //  if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
    //  {
    //    g3d_draw_allwin_active();
    //  }

    delete m_stompparams;
    delete m_chompplangroup;
    delete m_chomptraj;

    // Create Optimizer
    // ----------------
    m_stompparams = new stomp_motion_planner::StompParameters;
    m_stompparams->init();

//    for (int i=0; i<int(m_planner_joints.size()); i++) {
//        cout << m_planner_joints[i] << endl;
//    }
    m_chompplangroup = new ChompPlanningGroup( m_robot, m_planner_joints );
    m_chompplangroup->collision_points_ = m_collision_points;

    m_chomptraj = new ChompTrajectory( T, DIFF_RULE_LENGTH, *m_chompplangroup );
    //m_chomptraj->print();
    cout << "Chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;

    cout << "Initialize optimizer" << endl;
    global_optimizer.reset(new stomp_motion_planner::StompOptimizer( m_chomptraj, m_stompparams, m_chompplangroup, m_coll_space));
    global_optimizer->setSource( T.getBegin() );
    global_optimizer->setSharedPtr( global_optimizer );
    global_optimizer->setPassiveDofs( passive_dofs );

    if( PlanEnv->getBool(PlanParam::trajStompWithTimeLimit) )
    {
        global_optimizer->setUseTimeLimit( true );
        m_stompparams->max_time_ = PlanEnv->getDouble(PlanParam::trajStompTimeLimit);
    }

    if( (m_sce == traj_optim::HumanAwareManip && m_robot->getName() == "PR2_ROBOT") ||
        (m_sce == traj_optim::Default) ||
        (m_sce == traj_optim::CostMap) )
    {
        global_optimizer->setUseCostSpace(true);
    }

    cout << "Optimizer created" << endl;

    GlobalCostSpace::initialize();
    // std::cout << "Initializing the collision space function" << std::endl;
    // global_costSpace->addCost("CollisionSpace",boost::bind(computeCollisionSpaceCost, _1));
    // global_costSpace->setCost("CollisionSpace");

    m_robot->setAndUpdate( *m_robot->getInitPos() );

    return true;
}

bool traj_optim_runStomp( int runId )
{
//    cout << "Robot is : " << m_robot->getName() << endl;

    if(!traj_optim_initStomp() )
    {
        cout << "Could not init stomp in : " << __PRETTY_FUNCTION__ << endl;
        return false;
    }

    m_stompparams->init();

    if( m_use_iteration_limit )
    {
        global_optimizer->setUseIterationLimit( true );
        m_stompparams->max_iterations_ = m_max_iteration;
    }

    if( PlanEnv->getBool(PlanParam::trajStompWithTimeLimit) )
    {
        global_optimizer->setUseTimeLimit( true );
        m_stompparams->max_time_ = PlanEnv->getDouble(PlanParam::trajStompTimeLimit);
    }
    else {
        global_optimizer->setUseTimeLimit( false );
    }

    if(!PlanEnv->getBool(PlanParam::trajOptimTestMultiGauss))
    {
        global_optimizer->runDeformation( 0, runId );
        //optimizer->generateNoisyTrajectories();
    }
    else {
        global_optimizer->testMultiVariateGaussianSampler();
    }

    global_optimizer->resetSharedPtr();
    return true;
}

bool traj_optim_runStompNoInit( int runId, const API::Trajectory& traj )
{
    cout << "run stomp no init" << endl;

    if( PlanEnv->getBool(PlanParam::trajStompWithTimeLimit) )
    {
        global_optimizer->setUseTimeLimit( true );
        m_stompparams->max_time_ = PlanEnv->getDouble(PlanParam::trajStompTimeLimit);
    }
    else {
        global_optimizer->setUseTimeLimit( false );
    }

    cout << "set iteration limit" << endl;

    if( m_use_iteration_limit )
    {
        global_optimizer->setUseIterationLimit( true );
        m_stompparams->max_iterations_ = m_max_iteration;
    }

    global_optimizer->setUseOtp( PlanEnv->getBool(PlanParam::trajUseOtp) );
    global_optimizer->initializeFromNewTrajectory( traj );

    cout << "run deformation" << endl;

    global_optimizer->runDeformation( 0, runId );
    return true;
}

bool traj_optim_runStompNoReset(int runId)
{
    traj_optim_switch_cartesian_mode( false );

    API::Trajectory traj( m_robot );

    if( !traj_optim_InitTraj( traj ) ){
        return false;
    }

    //  API::Trajectory traj = m_robot->getCurrentTraj();
    return traj_optim_runStompNoInit( runId, traj );
}

//! Run Stomp
// --------------------------------------------------------
void traj_optim_plan_for_human(bool use)
{
    m_plan_for_human = use;
}

void traj_optim_add_human_to_collision_space(bool add)
{
    m_add_human = add;
}

void traj_optim_set_use_iteration_limit(bool use)
{
    m_use_iteration_limit = use;
}

void traj_optim_set_iteration_limit(int max_iter)
{
    m_max_iteration = max_iter;
}

void traj_optim_set_use_extern_trajectory( bool use )
{
    m_use_external_trajectory = use;
}

void traj_optim_set_extern_trajectory( const API::Trajectory& traj )
{
    m_external_trajectory = traj;
}

void traj_optim_set_discretize( bool discretize )
{
    m_discretize = discretize;
}

void traj_optim_set_discretization( double discretization )
{
    m_discretization = discretization;
}

std::vector<int> traj_optim_get_planner_joints()
{
    return m_planner_joints;
}

const CollisionSpace* traj_optim_get_collision_space()
{
    return m_coll_space;
}

std::vector<CollisionPoint> traj_optim_get_collision_points()
{
    return m_collision_points;
}

// --------------------------------------------------------
// Draw Functions
// --------------------------------------------------------
void traj_optim_draw_collision_points()
{
    if (m_chompplangroup && !m_chompplangroup->collision_points_.empty())
    {
        m_chompplangroup->draw();
    }
}

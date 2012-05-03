//
//  trajectoryOptim.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 01/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "trajectoryOptim.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "planner/Greedy/CollisionSpace.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include "Chomp/chompTrajectory.hpp"
#include "Chomp/chompOptimizer.hpp"
#include "Chomp/chompParameters.hpp"

#include "Stomp/stompOptimizer.hpp"
#include "Stomp/stompParameters.hpp"

#include "HRI_costspace/HRICS_costspace.hpp"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif
#include "Graphic-pkg.h"
#include "move3d-headless.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace tr1;

//--------------------------------------------------------
// Variables
//--------------------------------------------------------
bool m_init = false;
bool m_add_human = true;

static Robot* m_robot = NULL;

enum ScenarioType { CostMap, Simple, Shelf, Navigation , HumanAwareNav, HumanAwareManip, HumanAwareMobileManip };
static ScenarioType m_sce;

enum PlanningType { NAVIGATION = 0, MANIPULATION = 1, MOBILE_MANIP = 2 };
static PlanningType m_planning_type; 

CollisionSpace* global_collSpace=NULL;
CollisionSpace* m_coll_space=NULL;

ChompPlanningGroup* m_chompplangroup= NULL;
ChompTrajectory* m_chomptraj=NULL;
ChompParameters* m_chompparams=NULL;

stomp_motion_planner::StompParameters* m_stompparams=NULL;

int m_BaseMLP=0;
int m_BaseSmMLP=0;
int m_HeadMLP=0;
int m_UpBodyMLP=0;
int m_UpBodySmMLP=0;

vector<int> m_active_joints;
vector<int> m_planner_joints;
vector<CollisionPoint> m_collision_points;

vector< vector <double> > traj_optim_to_plot;

//--------------------------------------------------------
// General method
//--------------------------------------------------------

//! set mlp for this robot
bool traj_optim_set_MultiLP()
{
  if (!m_robot) {
    cout << "robot not initialized in file " 
    << __FILE__ << " ,  " << __func__ << endl;
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
    << __FILE__ << " ,  " << __func__ << endl;
    return false;
  }
  
  p3d_rob* rob = m_robot->getRobotStruct();
  p3d_cntrt* ct;
  
  // over all constraints
	for(int i=0; i<rob->cntrt_manager->ncntrts; i++) 
	{
    // get constraint from the cntrts manager
    ct = rob->cntrt_manager->cntrts[i];
    p3d_desactivateCntrt( rob, ct );
  }
  
  return true;
}

//! Virtual object
// --------------------------------------------------------
bool traj_optim_set_cartesian_mode() {

  p3d_rob* object = NULL;
  p3d_rob* robot = m_robot->getRobotStruct();

  configPt q = p3d_get_robot_config(robot);
  
  // For Each Arm
  for (int i=0; i < int((*robot->armManipulationData).size()); i++) 
  {
    ArmManipulationData& armData  = (*robot->armManipulationData)[i];
    
    desactivateTwoJointsFixCntrt(robot,armData.getManipulationJnt(), 
                                 armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
    
    if (armData.getCartesian()) {
      cout << "Arm Data " << i <<  " is set cartesian" << endl;
      // Uptdate the Virual object for inverse kinematics 
      // Be carfull the Arm will be unfixed 
      p3d_update_virtual_object_config_for_arm_ik_constraint(robot, i, q);
      activateCcCntrts(robot, i, false);
      ManipulationUtils::unfixManipulationJoints(robot, i);
      if(object){
        armData.getManipulationJnt()->dist = object->joints[1]->dist;
      }
    } else {
      deactivateCcCntrts(robot, i);
      p3d_update_virtual_object_config_for_arm_ik_constraint(robot, i, q);
      p3d_set_and_update_this_robot_conf(robot, q);
      setAndActivateTwoJointsFixCntrt(robot,armData.getManipulationJnt(),
                                      armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
    }
  }
  p3d_set_and_update_this_robot_conf(robot, q);
  p3d_get_robot_config_into(robot, &q);
  p3d_destroy_config(robot, q);
  return true;
}

//--------------------------------------------------------
// Main function to fix pr2 joints
//--------------------------------------------------------
bool traj_optim_init_mlp_cntrts_and_fix_joints()
{
  m_robot = global_Project->getActiveScene()->getActiveRobot();
  
  cout << "Initialize robot : " << m_robot->getName() << endl;
  
  if( !traj_optim_set_MultiLP() ) {
    return false;
  }
  
  if( !traj_optim_invalidate_cntrts() ) {
    return false;
  }
  
  if( !traj_optim_set_cartesian_mode() ) {
    return false;
  }
  
  switch( PlanEnv->getInt(PlanParam::setOfActiveJoints) )
  {
    case 0 : // Navigation
      cout << "Set navigation parameters" << endl;
      p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
      p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);
      fixAllJointsExceptBase( m_robot->getRobotStruct() );
      break;
      
    case 1 : // Manipulation
      cout << "Set manipulation parameters" << endl;
      p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
      p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
      fixAllJointsWithoutArm( m_robot->getRobotStruct() , 0 );
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

//! Create an initial Move3D trajectory
//! it generates a straigt line between the two configuration init 
//! and goal
// --------------------------------------------------------
API::Trajectory traj_optim_create_sraight_line_traj()
{
  if (!m_robot) {
    cout << "robot not initialized in file " 
    << __FILE__ << " ,  " << __func__ << endl;
  }
  
  confPtr_t q_init( m_robot->getInitialPosition() );
  confPtr_t q_goal( m_robot->getGoTo() );
  
  if( q_init->equal( *q_goal ) )
  {
    cout << "init equal q_goal in file "
    << __FILE__ << " ,  " << __func__ << endl; 
  }
  
  vector<confPtr_t> confs(2);
  
  confs[0] = q_init;
  confs[1] = q_goal;
  
  API::Trajectory T( confs );
  
  return T;
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
    << __FILE__ << " ,  " << __func__ << endl;
    return false;
  }
  
  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back( 1 );
  
  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back( 1 );
  
  p3d_set_user_drawnjnt(1);
  
  m_coll_space = NULL;
  return (global_costSpace != NULL);
}

//****************************************************************
//* 2D Costmap example
//****************************************************************

//! Initializes the optimization for a costspace
// --------------------------------------------------------
bool traj_optim_costspace_init()
{
  if (!m_robot) {
    cout << "robot not initialized in file " 
    << __FILE__ << " ,  " << __func__ << endl;
    return false;
  }
  
  // Set the active joints (links)
  m_active_joints.clear();
  m_active_joints.push_back( 1 );
  
  // Set the planner joints
  m_planner_joints.clear();
  m_planner_joints.push_back( 1 );
  
  p3d_set_user_drawnjnt(1);
  
  m_coll_space = NULL;
  
  bool valid_costspace = global_costSpace->setCost("costMap2D");
  return valid_costspace;
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
  
  //T.cutTrajInSmallLP( 10 );
  T.replaceP3dTraj();
  //T.print();
  
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
  double delta = T.getRangeMax() / 50 ;
  
  p3d_multiLocalPath_disable_all_groupToPlan(m_robot->getRobotStruct(), FALSE);
  p3d_multiLocalPath_set_groupToPlan(m_robot->getRobotStruct(), m_UpBodyMLP, 1, FALSE);
  
  API::Trajectory newT(m_robot);
  
  cout << "delta = " << delta << endl;
  
//  double param_first = T.
  
  for (double t=0; t<=T.getRangeMax(); t += delta ) 
  {
    newT.push_back( T.configAtParam(t) );
  }
  newT.replaceP3dTraj();
  
  // Set to plot
  traj_optim_to_plot.clear();
  vector<double> x,y;
  x.resize(100);
  y.resize(100);

  // SoftMotion
  int i=0;
  delta = T.getRangeMax()/100;

  for (double t=0; t<=T.getRangeMax(); t += delta ) 
  {
    shared_ptr<Configuration> q = T.configAtParam(t);
    x[i] = (*q)[6];
    y[i] = (*q)[7];
    i++;
    
    if (i >= int(x.size()) || i >= int(y.size()))
      break;
  }
  traj_optim_to_plot.push_back( x );
  traj_optim_to_plot.push_back( y );
  
  // Initial traj
  i=0;
  delta = TSaved.getRangeMax()/100;
  
  for (double t=0; t<=TSaved.getRangeMax(); t += delta ) 
  {
    shared_ptr<Configuration> q = TSaved.configAtParam(t);
    x[i] = (*q)[6];
    y[i] = (*q)[7];
    i++;
    
    if (i >= int(x.size()) || i >= int(y.size()))
      break;
  }
  traj_optim_to_plot.push_back( x );
  traj_optim_to_plot.push_back( y );
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
  p3d_set_user_drawnjnt(28);
}

//! initializes the collision space
// --------------------------------------------------------
void traj_optim_shelf_init_collision_space()
{
  m_coll_space = new CollisionSpace(m_robot);
  
  // Set the active joints (links)
  m_active_joints.clear();
  //  m_active_joints.push_back( 5 );
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
  //m_planner_joints.push_back( 5 );
  m_planner_joints.push_back( 6 );
  m_planner_joints.push_back( 7 );
  m_planner_joints.push_back( 8 );
  m_planner_joints.push_back( 9 );
  m_planner_joints.push_back( 10 );
  m_planner_joints.push_back( 11 );
  m_planner_joints.push_back( 12 );
  
  for (unsigned int joint_id=0; joint_id<m_robot->getNumberOfJoints(); joint_id++) 
  {
    if ( find (m_active_joints.begin(), m_active_joints.end(), joint_id ) 
        == m_active_joints.end() ) 
    {
      m_coll_space->addRobotBody( m_robot->getJoint(joint_id) ); 
    }
  }
  
  if( m_add_human )
  {
    Scene* sc = global_Project->getActiveScene();
    
    for (unsigned int i=0; i<sc->getNumberOfRobots(); i++) 
    {
      Robot* rob = sc->getRobot(i);
      
      if ( rob->getName().find("HERAKLES") != string::npos )
      {
        m_coll_space->addRobot( rob );
      }
    }
  }
  
  // Adds the sampled points to the distance field
  m_coll_space->addAllPointsToField();
}

//! initializes the collision points
// --------------------------------------------------------
bool traj_optim_shelf_init_collision_points()
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

  p3d_set_user_drawnjnt(1);
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
  
  p3d_set_user_drawnjnt(1);
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
void traj_optim_hrics_manip_init_joints()
{
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
  
  m_coll_space = NULL;
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
  
  m_robot = global_Project->getActiveScene()->getActiveRobot();
  
  switch (m_sce) 
  {
    case CostMap:
      
      cout << "Init CostMap" << endl;
      if( !traj_optim_costspace_init() )
        return false;
      
//      PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.1);
//      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
//      PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
//      PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.001);
      break;
      
    case Simple:
      
      cout << "Init Simple" << endl;
      if( !traj_optim_simple_init() )
        return false;
      
      //      PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.030000);
      //      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
      //      PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
      //      PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.000005);
      break;
      
    case Shelf:
      
      cout << "Init Shelf" << endl;
      cout << "Set robot, localpath and cntrts with " << m_robot->getName() << endl;
      
      traj_optim_set_MultiLP();
      traj_optim_invalidate_cntrts();
      traj_optim_shelf_set_localpath_and_cntrts();
      
      if( !global_collisionSpace ) 
      {
        traj_optim_shelf_init_collision_space();
        traj_optim_shelf_init_collision_points();
      }
      else {
        m_coll_space = global_collisionSpace;
      }
      
//      PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
//      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,15);
//      PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,10);
//      PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);
      
//      PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
//      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
//      PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,30);
//      PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.01);
      break;
      
    case HumanAwareNav:
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
      
    case HumanAwareManip:
      cout << "Init HumanAwareManip" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;
      
      traj_optim_set_MultiLP();
      traj_optim_invalidate_cntrts();
      traj_optim_shelf_set_localpath_and_cntrts();
      traj_optim_hrics_manip_init_joints();
      
      // PlanEnv->setDouble(PlanParam::trajOptimStdDev,3);
      // PlanEnv->setInt(PlanParam::nb_pointsOnTraj,30);
      // PlanEnv->setDouble(PlanParam::trajOptimObstacWeight,20);
      // PlanEnv->setDouble(PlanParam::trajOptimSmoothWeight,0.1);
      break;
      
    case HumanAwareMobileManip:
      
      cout << "Init HumanAwareMobileManip" << endl;
      cout << "Set robot, localpath and cntrts with ";
      cout << m_robot->getName() << endl;
      
      traj_optim_set_MultiLP();
      traj_optim_invalidate_cntrts();
      traj_optim_hrics_mobile_manip_localpath_and_cntrts();
      traj_optim_hrics_mobile_manip_init_joints();
      break;
      
    case Navigation:
      
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
  }
  
  return true;
}

//! Set the type of scenario
//! Depending on global variables
// --------------------------------------------------------
bool traj_optim_set_scenario_type()
{  
  if( ENV.getBool(Env::isCostSpace) && ( global_costSpace == NULL )){
    cout << "Cost space not initialized!!!" << endl;
    return false;
  }
  
  if( ENV.getBool(Env::isCostSpace) &&
     global_costSpace->getSelectedCostName() == "costMap2D")
  {
    m_sce = CostMap;
  }
  else if( ENV.getBool(Env::isCostSpace) &&
     ( global_costSpace->getSelectedCostName() == "costIsInCollision" || 
       global_costSpace->getSelectedCostName() == "costDistToObst"  ))
  {
    m_sce = Simple;
  } 
  else if( ENV.getBool(Env::isCostSpace) &&
          global_costSpace->getSelectedCostName() == "costHumanGrids" ) {
    
    if( m_planning_type == NAVIGATION )
    {
      m_sce = HumanAwareNav;
    }
    if( m_planning_type == MANIPULATION )
    {
      m_sce = HumanAwareManip;
    }
    if( m_planning_type == MOBILE_MANIP )
    {
      m_sce = HumanAwareMobileManip;
    }
  }
  else 
  {
    const bool navigation = false;
    
    if( navigation )
      m_sce = Navigation;
    else
      m_sce = Shelf;
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
      break;
      
    case 1:
      m_planning_type = MANIPULATION;
      break;
      
    case 2:
      m_planning_type = MOBILE_MANIP;
      break;
      
    default:
      cout << "Planner type not implemented" << endl;
      break;
  }
}

//****************************************************************
//*   Run Functions 
//****************************************************************

bool traj_optim_initScenario()
{
  traj_optim_init_planning_type( PlanEnv->getInt(PlanParam::setOfActiveJoints) );
  
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

//! Chomp
// --------------------------------------------------------
bool traj_optim_runChomp()
{
  if( !m_init )
  {
    traj_optim_initScenario();
  }
  else
  {
    if ( m_coll_space && m_sce == Shelf ) 
    {
      traj_optim_shelf_set_localpath_and_cntrts();
    }
  }
  
  // Get Initial trajectory
  // ----------------------
  API::Trajectory T;
  
  if( PlanEnv->getBool(PlanParam::withCurrentTraj) )
  {
    T = m_robot->getCurrentTraj();
  }
  else 
  {
    T = traj_optim_create_sraight_line_traj(); 
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

//!
//! Stomp
// --------------------------------------------------------
bool traj_optim_initStomp()
{
  if( !m_init )
  {
    traj_optim_initScenario();
  }
  
  // Get Initial trajectory
  // ----------------------
  API::Trajectory T;
  
  if( PlanEnv->getBool(PlanParam::withCurrentTraj) )
  {
    T = m_robot->getCurrentTraj();
//    PlanEnv->setInt( PlanParam::nb_pointsOnTraj, T.getNbOfViaPoints() );
    T.cutTrajInSmallLP( PlanEnv->getInt( PlanParam::nb_pointsOnTraj ) );
    T.replaceP3dTraj();
    T.print();
  }
  else 
  {
    T = traj_optim_create_sraight_line_traj(); 
    T.cutTrajInSmallLP( PlanEnv->getInt( PlanParam::nb_pointsOnTraj ) );
    T.replaceP3dTraj();
    T.print();
  }
  
  g3d_draw_allwin_active();
  
  // Create Optimizer
  // ----------------
  m_stompparams = new stomp_motion_planner::StompParameters;
  m_stompparams->init();
  
  m_chompplangroup = new ChompPlanningGroup( m_robot, m_planner_joints );
  m_chompplangroup->collision_points_ = m_collision_points;
  
  m_chomptraj = new ChompTrajectory( T, DIFF_RULE_LENGTH, *m_chompplangroup );
  m_chomptraj->print();
  cout << "Chomp Trajectory has npoints : " << m_chomptraj->getNumPoints() << endl;
  
  cout << "Initialize optimizer" << endl;
  optimizer.reset(new stomp_motion_planner::StompOptimizer(m_chomptraj,
                                                           m_stompparams,
                                                           m_chompplangroup,
                                                           m_coll_space));
  
  optimizer->setSharedPtr(optimizer);
  cout << "Optimizer created" << endl;
  
  GlobalCostSpace::initialize();
//  std::cout << "Initializing the collision space function" << std::endl;
//  global_costSpace->addCost("CollisionSpace",boost::bind(computeCollisionSpaceCost, _1));
//  global_costSpace->setCost("CollisionSpace");
    
  return true;
}

bool traj_optim_runStomp()
{
  if(!traj_optim_initStomp())
  {
    return false;
  }
  
  if(!PlanEnv->getBool(PlanParam::trajOptimTestMultiGauss))
  {
    optimizer->runDeformation(0,0);
    //optimizer->generateNoisyTrajectories();
  }
  else
  {
    optimizer->testMultiVariateGaussianSampler();
  }
  optimizer->resetSharedPtr();
  return true;
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



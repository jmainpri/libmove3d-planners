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

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif
#include "Graphic-pkg.h"
#include "move3d-headless.h"

using namespace std;
using namespace tr1;

//--------------------------------------------------------
// Variables
//--------------------------------------------------------
bool m_init = false;
bool m_add_human = true;

static Robot* m_robot = NULL;

enum ScenarioType { CostMap, Shelf, Navigation };
static ScenarioType m_sce;

CollisionSpace* m_coll_space=NULL;

ChompPlanningGroup* m_chompplangroup= NULL;
ChompTrajectory* m_chomptraj=NULL;
ChompParameters* m_chompparams=NULL;

stomp_motion_planner::StompParameters* m_stompparams=NULL;

int m_BaseMLP=0;
int m_HeadMLP=0;
int m_UpBodyMLP=0;
int m_UpBodySmMLP=0;

vector<int> m_active_joints;
vector<int> m_planner_joints;
vector<CollisionPoint> m_collision_points;

//--------------------------------------------------------
// General method
//--------------------------------------------------------

#ifdef MULTILOCALPATH
//! set mlp for this robot
void traj_optim_set_MultiLP()
{
  for (int i = 0; m_robot && i < m_robot->getRobotStruct()->mlp->nblpGp; i++) {
    if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
      m_BaseMLP = i;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
      m_HeadMLP = i;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
      m_UpBodyMLP = i;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
      m_UpBodySmMLP = i;
    }
  }
}
#endif

//! invalidate all constraints
// --------------------------------------------------------
void traj_optim_invalidate_cntrts()
{
  if (!m_robot) {
    cout << "robot not initialized in file " 
    << __FILE__ << " ,  " << __func__ << endl;
    return;
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
  m_robot = global_Project->getActiveScene()->getActiveRobot();

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
  //  m_planner_joints.push_back( 5 );
  m_planner_joints.push_back( 6 );
  m_planner_joints.push_back( 7 );
  m_planner_joints.push_back( 8 );
  m_planner_joints.push_back( 9 );
  m_planner_joints.push_back( 10 );
  m_planner_joints.push_back( 11 );
  m_planner_joints.push_back( 12 );
  
  for (unsigned int joint_id=0; 
       joint_id<m_robot->getNumberOfJoints(); joint_id++) 
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
  global_CollisionSpace = m_coll_space;
  return true;
}

//****************************************************************
//* 2D Costmap example
//****************************************************************

//! Initializes the optimization for a costspace
// --------------------------------------------------------
bool traj_optim_costspace_init()
{
  m_robot = global_Project->getActiveScene()->getActiveRobot();
  
  if (m_robot == NULL) 
  {
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

//****************************************************************
//* Navigation example
//****************************************************************

//! Sets the robot and the local method to be used
//! Also sets the constraints and fixes the joints
//! which are not used durring the planning/optimization phaze
// --------------------------------------------------------
void traj_optim_navigation_set_localpath_and_cntrts()
{
  cout << "Set robot, localpath and cntrts" << endl;
  m_robot = global_Project->getActiveScene()->getActiveRobot();

#if defined(MULTILOCALPATH) && defined(LIGHT_PLANNER)
  traj_optim_set_MultiLP();
  traj_optim_invalidate_cntrts();
  
  p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
  p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);
  
  fixAllJointsExceptBase( m_robot->getRobotStruct() );
#endif

  p3d_set_user_drawnjnt(1);
}

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
}

//****************************************************************
//* Common functions 
//****************************************************************

//! Create initial Move3D trajectory
//! this function creates a straigt trajectory
// --------------------------------------------------------
API::Trajectory traj_optim_create_sraight_line_traj()
{
  shared_ptr<Configuration> q_init( m_robot->getInitialPosition() );
  shared_ptr<Configuration> q_goal( m_robot->getGoTo() );
  
  vector< shared_ptr<Configuration> > confs(2);
  
  confs[0] = q_init;
  confs[1] = q_goal;
  
  API::Trajectory T( confs );
  
  return T;
}

//! Get current robot
//! Initializes the costspace and multi localpath
// --------------------------------------------------------
bool traj_optim_init()
{
  if (m_init == true)
    return true;
  
  switch (m_sce) 
  {
    case CostMap:
      
      if( !traj_optim_costspace_init() )
        return false;
      
      PlanEnv->setDouble(PlanParam::trajOptimStdDev,0.2);
      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,50);
      break;
      
    case Shelf:
      
      traj_optim_shelf_set_localpath_and_cntrts();
      
      if( !global_CollisionSpace ) 
      {
        traj_optim_shelf_init_collision_space();
        traj_optim_shelf_init_collision_points();
      }
      else {
        m_coll_space = global_CollisionSpace;
      }
      
      PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,15);
      break;
      
    case Navigation:
      
      if( !global_CollisionSpace )
        return false;
      else
      {
        m_coll_space = global_CollisionSpace;
        
        traj_optim_navigation_set_localpath_and_cntrts();
        traj_optim_navigation_generate_points();
      }
      
      PlanEnv->setDouble(PlanParam::trajOptimStdDev,2);
      PlanEnv->setInt(PlanParam::nb_pointsOnTraj,15);
      break;
  }
  
  return true;
}

//! Set the type of scenario
//! Depending on global variables
// --------------------------------------------------------
void traj_set_scenario_type()
{
  if( ENV.getBool(Env::isCostSpace) )
  {
    m_sce = CostMap;
  }
  else
  {
    const bool navigation = true;
    
    if( navigation )
    {
      m_sce = Shelf;
    }
    else
    {
      m_sce = Navigation;
    }
  }
}


//****************************************************************
//*   Run Functions 
//****************************************************************

//! Chomp
// --------------------------------------------------------
bool traj_optim_runChomp()
{
  if( !m_init )
  {
    traj_optim_init();
    m_init = true;
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
bool traj_optim_runStomp()
{
  if( !m_init )
  {
    traj_optim_init();
    m_init = true;
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
  T.print();
  
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
  
  cout << "Optimizer created" << endl;
  
  if(!PlanEnv->getBool(PlanParam::trajOptimTestMultiGauss))
  {
    optimizer->setSharedPtr(optimizer);
    optimizer->runDeformation(0,0);
    optimizer->resetSharedPtr();
  }
  else
  {
    optimizer->setSharedPtr(optimizer);
    optimizer->testMultiVariateGaussianSampler();
    optimizer->resetSharedPtr();
  }
  
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



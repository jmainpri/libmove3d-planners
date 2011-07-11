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

#include "Chomp/chompTrajectory.hpp"
#include "Chomp/chompOptimizer.hpp"
#include "Chomp/chompParameters.hpp"

#include "Stomp/stompOptimizer.hpp"
#include "Stomp/stompParameters.hpp"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"

using namespace std;
using namespace tr1;

//--------------------------------------------------------
// Variables
//--------------------------------------------------------
bool m_init = false;
bool m_add_human = true;

Robot* m_robot = NULL;

CollisionSpace* m_coll_space=NULL;

ChompPlanningGroup* chompplangroup= NULL;
ChompTrajectory* chomptraj=NULL;
ChompParameters* chompparams=NULL;

stomp_motion_planner::StompParameters* stompparams=NULL;

int m_BaseMLP=0;
int m_HeadMLP=0;
int m_UpBodyMLP=0;
int m_UpBodySmMLP=0;

vector<int> active_joints;
vector<int> planner_joints;
vector<CollisionPoint> collision_points;

//--------------------------------------------------------
// Init Functions
//--------------------------------------------------------

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

//! invalidate all constraints
// -----------------------------------------------
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

//! initializes the collision space
// -----------------------------------------------
void traj_optim_init_collision_space()
{
  m_coll_space = new CollisionSpace(m_robot);
  
  // Set the active joints (links)
  active_joints.clear();
  active_joints.push_back( 6 );
  active_joints.push_back( 7 );
  active_joints.push_back( 8 );
  active_joints.push_back( 9 );
  active_joints.push_back( 10 );
  active_joints.push_back( 11 );
  active_joints.push_back( 12 );
  
  active_joints.push_back( 14 );
  active_joints.push_back( 15 );
  
  // Set the planner joints
  planner_joints.clear();
  planner_joints.push_back( 6 );
  planner_joints.push_back( 7 );
  planner_joints.push_back( 8 );
  planner_joints.push_back( 9 );
  planner_joints.push_back( 10 );
  planner_joints.push_back( 11 );
  planner_joints.push_back( 12 );
  
  for (unsigned int joint_id=0; 
       joint_id<m_robot->getNumberOfJoints(); joint_id++) 
  {
    if ( find (active_joints.begin(), active_joints.end(), joint_id ) 
        == active_joints.end() ) 
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
// -----------------------------------------------
bool traj_optim_init_collision_points()
{
  // Generate Bounding volumes for active joints
  BodySurfaceSampler* sampler = m_coll_space->getBodySampler();
  
  // Get all joints active in the motion planning
  // and compute bounding cylinders
  vector<Joint*> joints;
  joints.clear();
  for (unsigned int i=0; i<active_joints.size(); i++) 
  {
    joints.push_back( m_robot->getJoint( active_joints[i] ) );
  }
  /*double maxRadius =*/ sampler->generateRobotBoudingCylinder( m_robot, joints );
  
  // Get all planner joint 
  // and compute collision points
  vector<int> planner_joints_id;
  for (unsigned int i=0; i<planner_joints.size(); i++) 
  {
    planner_joints_id.push_back( planner_joints[i] );
  }
  collision_points = sampler->generateRobotCollisionPoints( m_robot, active_joints, planner_joints_id );
  
  // Set the collision space as global (drawing)
  global_CollisionSpace = m_coll_space;
  return true;
}

//! Get current robot
//! Initializes the multi-localpaths
// -----------------------------------------------
void traj_optim_init()
{
  if (m_init == true)
    return;
  
  m_robot = global_Project->getActiveScene()->getActiveRobot();
  
  traj_optim_set_MultiLP();
  traj_optim_invalidate_cntrts();
  
  // If the collision space exists we use it
  if( !global_CollisionSpace ) 
  {
    traj_optim_init_collision_space();
  }
  else {
    m_coll_space = global_CollisionSpace;
  }
  
  traj_optim_init_collision_points();
}

//! Create initial Move3D trajectory
// -----------------------------------------------
API::Trajectory traj_optim_create_sraight_line_traj()
{
  shared_ptr<Configuration> q_init( m_robot->getInitialPosition() );
  shared_ptr<Configuration> q_goal( m_robot->getGoTo() );
  
  p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
  p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
  
  vector< shared_ptr<Configuration> > confs(2);
  
  confs[0] = q_init;
  confs[1] = q_goal;
  
  API::Trajectory T( confs );
  
  T.cutTrajInSmallLP( 20 );
  T.replaceP3dTraj();
  
  g3d_draw_allwin_active();
  
  return T;
}

//--------------------------------------------------------
// Run Functions
//--------------------------------------------------------
bool traj_optim_runChomp()
{
  if( !m_init )
  {
    traj_optim_init();
    m_init = true;
  }
  
  API::Trajectory T;
  
  if( PlanEnv->getBool(PlanParam::withCurrentTraj) )
  {
    return true;
  }
  else {
    T = traj_optim_create_sraight_line_traj(); 
  }
  
  // Create Optimizer
  // -----------------------------------------------
  chomptraj = new ChompTrajectory(T,DIFF_RULE_LENGTH, planner_joints );
  chomptraj->print();
  cout << "chomp Trajectory has npoints : " << chomptraj->getNumPoints() << endl;
  cout << "Initialize optimizer" << endl;
  
  chompparams = new ChompParameters;
  chompparams->init();
  
  chompplangroup = new ChompPlanningGroup( m_robot, planner_joints );
  chompplangroup->collision_points_ = collision_points;
  
  ChompOptimizer optimizer(chomptraj,chompparams,chompplangroup,m_coll_space);
  cout << "Optimizer created" << endl;
  
  optimizer.runDeformation(0,0);
  return true;
}

//--------------------------------------------------------
// Run Functions
//--------------------------------------------------------
bool traj_optim_runStomp()
{
  if( !m_init )
  {
    traj_optim_init();
    m_init = true;
  }
  
  API::Trajectory T;
  
  if( PlanEnv->getBool(PlanParam::withCurrentTraj) )
  {
    return true;
  }
  else {
    T = traj_optim_create_sraight_line_traj(); 
  }
  
  // Create Optimizer
  // -----------------------------------------------
  chomptraj = new ChompTrajectory(T,DIFF_RULE_LENGTH, planner_joints );
  chomptraj->print();
  cout << "stomp Trajectory has npoints : " << chomptraj->getNumPoints() << endl;
  cout << "Initialize optimizer" << endl;
  
  stompparams = new stomp_motion_planner::StompParameters;
  stompparams->init();
  
  chompplangroup = new ChompPlanningGroup( m_robot, planner_joints );
  chompplangroup->collision_points_ = collision_points;
  
  boost::shared_ptr<stomp_motion_planner::StompOptimizer> optimizer;
  
  optimizer.reset(new stomp_motion_planner::StompOptimizer(chomptraj,
                                                 stompparams,
                                                 chompplangroup,
                                                 m_coll_space));
  cout << "Optimizer created" << endl;
  
  optimizer->setSharedPtr(optimizer);
  optimizer->runDeformation(0,0);
  optimizer->resetSharedPtr();
  
  return true;
}

//--------------------------------------------------------
// Draw Functions
//--------------------------------------------------------
void traj_optim_draw_collision_points()
{
  if (chompplangroup && !chompplangroup->collision_points_.empty()) 
  {
    chompplangroup->draw();
  } 
}



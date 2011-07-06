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
#include "API/trajectory/trajectory.hpp"

#include "planner/Greedy/CollisionSpace.hpp"

#include "Chomp/chompTrajectory.hpp"
#include "Chomp/chompOptimizer.hpp"
#include "Chomp/chompParameters.hpp"

using namespace std;
using namespace tr1;

//--------------------------------------------------------
//--------------------------------------------------------
bool m_init = false;

Robot* m_robot = NULL;

CollisionSpace* m_coll_space=NULL;

ChompPlanningGroup* chompplangroup= NULL;
ChompTrajectory* chomptraj=NULL;
ChompParameters* chompparams=NULL;

int m_BaseMLP=0;
int m_HeadMLP=0;
int m_UpBodyMLP=0;
int m_UpBodySmMLP=0;

vector<int> active_joints;
vector<int> planner_joints;
vector<CollisionPoint> collision_points;

//--------------------------------------------------------
//--------------------------------------------------------


bool traj_optim_init_collision_space();

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

//! Get current robot
//! Initializes the multi-localpaths
void traj_optim_init()
{
  m_robot = global_Project->getActiveScene()->getActiveRobot();
  
  traj_optim_set_MultiLP();
}

bool traj_optim_runChomp()
{
  if( !m_init )
  {
    traj_optim_init();
    m_init = true;
  }
  
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
  
  // Create base Trajectory
  // -----------------------------------------------
  shared_ptr<Configuration> q_init( m_robot->getInitialPosition() );
  shared_ptr<Configuration> q_goal( m_robot->getGoTo() );
  
  p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
  p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
  
  vector< shared_ptr<Configuration> > confs(2);
  
  confs[0] = q_init;
  confs[1] = q_goal;
  
  API::Trajectory T( confs );
  
  T.cutTrajInSmallLP(8);
  T.replaceP3dTraj(); 
  
  // Initialize collision space
  // -----------------------------------------------
  traj_optim_init_collision_space();
  
  // Create Optimizer
  // -----------------------------------------------
  chomptraj = new ChompTrajectory(T,DIFF_RULE_LENGTH,planner_joints);
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

bool traj_optim_init_collision_space()
{
  if (m_coll_space != NULL) {
    cout << "Delete collision space" << endl;
    delete m_coll_space;
  }
  
  m_coll_space = new CollisionSpace(m_robot);
  
  m_coll_space->addAllPointsToField();
  
  for (unsigned int joint_id=0; 
       joint_id<m_robot->getNumberOfJoints(); joint_id++) 
  {
    if ( find (active_joints.begin(), active_joints.end(), joint_id ) 
        == active_joints.end() ) 
    {
      m_coll_space->addRobotBody( m_robot->getJoint(joint_id) ); 
    }
  }
  
  // Get all joints active in the motion planning
  vector<Joint*> joints;
  joints.clear();
  for (unsigned int i=0; i<active_joints.size(); i++) 
  {
    joints.push_back( m_robot->getJoint( active_joints[i] ) );
  }
  
  // Generate Bounding volumes for active joints
  BodySurfaceSampler* sampler = m_coll_space->getBodySampler();
  
  sampler->generateRobotBoudingCylinder( m_robot, joints );
  
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

void traj_optim_draw_collision_points()
{
  if (chompplangroup && !chompplangroup->collision_points_.empty()) 
  {
    chompplangroup->draw();
  } 
}



//
//  trajectoryOptimSTL.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 21/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "trajectoryOptim.hpp"

#include <Eigen/StdVector>

#include "planner/Greedy/collision_space.hpp"

using namespace std;

static vector<CollisionPoint> m_collision_points;

//! initializes the collision points
// -----------------------------------------------
bool traj_optim_init_collision_points()
{
  
  Robot* m_robot = traj_optim_getRobot();
  BodySurfaceSampler* sampler = traj_optim_get_colspace()->getBodySampler();
  
  //---------------------------------------------------
  // Generate Bounding volumes for active joints
  vector<int> active_joints;
  
  int id = 0;
  unsigned int size = traj_optim_get_active_joints(id);
  for (unsigned int i=0; i<size; i++) {
    active_joints.push_back( traj_optim_get_active_joints(id) );
  }
  
  // Get all joints active in the motion planning
  // and compute bounding cylinders
  vector<Joint*> joints;
  joints.clear();
  for (unsigned int i=1; i<=active_joints.size(); i++) 
  {
    joints.push_back( m_robot->getJoint( active_joints[i] ) );
  }
  /*double maxRadius =*/ sampler->generateRobotBoudingCylinder( m_robot, joints );
  
  //---------------------------------------------------
  // Get all planner joint and compute collision points
  vector<int> planner_joints;
  vector<int> planner_joints_id;
  
  id = 0;
  size = traj_optim_get_active_joints(id);
  
  for (unsigned int i=1; i<=size; i++) {
    planner_joints.push_back( traj_optim_get_planner_joints(id) );
  }
  
  for (unsigned int i=0; i<planner_joints.size(); i++) 
  {
    planner_joints_id.push_back( planner_joints[i] );
  }
  m_collision_points = sampler->generateRobotCollisionPoints( m_robot, active_joints, planner_joints_id );
  
  for (unsigned int i=0; i<m_collision_points.size(); i++) 
  {
    traj_optim_add_coll_point( &m_collision_points[i] );
  }
  
  m_collision_points.clear();
  
  // Set the collision space as global (drawing)
  global_CollisionSpace = traj_optim_get_colspace();
  return true;
}


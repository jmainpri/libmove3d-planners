//
//  chompPlanningGroup.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 04/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "chompPlanningGroup.hpp"

#include "P3d-pkg.h"

using namespace std;

ChompPlanningGroup::ChompPlanningGroup(Robot* rob, const std::vector<int>& active_joints )
{
  m_robot = rob;
  num_joints_ = active_joints.size();
  chomp_joints_.clear();
  
  for (int i=0; i<num_joints_; i++) 
  {
    ChompJoint jnt;
    
    jnt.move3d_joint_ = m_robot->getJoint( active_joints[i] );
    jnt.move3d_joint_index_ = active_joints[i];
    jnt.chomp_joint_index_ = i;
    jnt.joint_name_ = jnt.move3d_joint_->getName();
    //jnt.link_name_ = 
    jnt.wrap_around_ = false;      
    jnt.has_joint_limits_ = !p3d_jnt_is_dof_circular(jnt.move3d_joint_->getJointStruct(),0);
    
    double min,max;
    jnt.move3d_joint_->getDofBounds(0,min,max);
    
    jnt.joint_limit_min_ = min;
    jnt.joint_limit_max_ = max;
    
    jnt.joint_update_limit_ = 0.10; 
    
    chomp_joints_.push_back( jnt );
  }
}

bool ChompPlanningGroup::addCollisionPoint(CollisionPoint& collision_point)
{
  // create the new parent joints indexing vector:
  std::vector<int> parent_joints(num_joints_, 0);
  
  //ROS_INFO_STREAM("Num joints is " << num_joints_ << " parent size is " << collision_point.getParentJoints().size());
  
  // check if this collision point is controlled by any joints which belong to the group
  bool add_this_point=false;
  for (int i=0; i<num_joints_; i++)
  {
    if (collision_point.isParentJoint(chomp_joints_[i].move3d_joint_index_))
    {
      add_this_point = true;
      break;
    }
  }
  
  if (!add_this_point)
    return false;
  
  collision_points_.push_back(CollisionPoint(collision_point, collision_point.getParentJoints()));
  return true;
}

void ChompPlanningGroup::draw() const
{
  //cout << "Number of collision points : " << collision_points_.size() << endl;
  for (unsigned int i=0; i<collision_points_.size(); i++) 
  {
//    if (i>=38) 
//      continue;
    
    Eigen::Transform3d T = m_robot->getJoint( collision_points_[i].getParentJoints().back() )->getMatrixPos();
    
    collision_points_[i].draw(T);
  }
}

void ChompPlanningGroup::draw(std::vector<Eigen::Transform3d>& segment) const
{
  for (unsigned int i=0; i<collision_points_.size(); i++) 
  {
    Eigen::Transform3d T = segment[collision_points_[i].getSegmentNumber()];
    collision_points_[i].draw( T );
  }
}


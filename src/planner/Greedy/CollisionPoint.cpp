/*
 *  CollisionPoint.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CollisionPoint.hpp"
#include <cmath>

#include "Graphic-pkg.h"

CollisionPoint::CollisionPoint(
                               const std::vector<int>& parent_joints, 
                               double radius, 
                               double clearance,
                               int segment_number, 
                               const Eigen::Vector3d& position) :
      m_parent_joints(parent_joints),
      m_radius(radius),
      m_volume((4.0/3.0)*M_PI*m_radius*m_radius*m_radius),
      m_clearance(clearance),
      m_inv_clearance(1.0/m_clearance),
      m_segment_number(segment_number),
      m_position(position)
{

}

CollisionPoint::CollisionPoint(const CollisionPoint &point, const std::vector<int>& parent_joints):
  m_parent_joints(parent_joints),
  m_radius(point.m_radius),
  m_volume((4.0/3.0)*M_PI*m_radius*m_radius*m_radius),
  m_clearance(point.m_clearance),
  m_inv_clearance(1.0/m_clearance),
  m_segment_number(point.m_segment_number),
  m_position(point.m_position)
{
}

CollisionPoint::~CollisionPoint()
{
}

void CollisionPoint::getJacobian(std::vector<Eigen::Map<Eigen::Vector3d> >& joint_pos, 
                                 std::vector<Eigen::Map<Eigen::Vector3d> >& joint_axis,
                                 Eigen::Map<Eigen::Vector3d>& collision_point_pos, 
                                 Eigen::MatrixXd& jacobian, 
                                 const std::vector<int>& group_joint_to_kdl_joint_index) const
{
  for(unsigned int joint = 0; joint < group_joint_to_kdl_joint_index.size(); joint++) 
  {
    if(!isParentJoint(group_joint_to_kdl_joint_index[joint])) {
      // since the joint is not active, fill the jacobian column with zeros
      jacobian.col(joint).setZero();
    }
    else
    {
      int kj = group_joint_to_kdl_joint_index[joint];
      jacobian.col(joint) = joint_axis[kj].cross(collision_point_pos - joint_pos[kj]);
    }
  }
}

void CollisionPoint::draw(const Eigen::Transform3d& T)
{
  Eigen::Vector3d point = T*m_position; 
  
  double colorvector[4];
  colorvector[0] = 1.0;
  colorvector[1] = 1.0;
  colorvector[2] = 0.0;
  colorvector[3] = 0.7;
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  glColor4dv(colorvector);
  g3d_draw_solid_sphere(point[0], 
                        point[1],
                        point[2], m_radius, 20);
  
  glDisable(GL_BLEND);
}


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

using namespace Move3D;

using std::cout;
using std::endl;

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

void CollisionPoint::getJacobian(std::vector</*Eigen::Map<*/Eigen::Vector3d> /*>*/& joint_pos, 
                                 std::vector</*Eigen::Map<*/Eigen::Vector3d> /*>*/& joint_axis,
                                 /*Eigen::Map<*/Eigen::Vector3d/*>*/& collision_point_pos,
                                 Eigen::MatrixXd& jacobian,
                                 const std::vector<int>& group_joint_to_move3d_joint_index) const
{
    for(unsigned int joint = 0; joint < group_joint_to_move3d_joint_index.size(); joint++)
    {
        if(!isParentJoint(group_joint_to_move3d_joint_index[joint])) {
            // since the joint is not active, fill the jacobian column with zeros
            jacobian.col(joint).setZero();
        }
        else
        {
            //int kj = group_joint_to_move3d_joint_index[joint];
            jacobian.col(joint) = joint_axis[joint].cross(collision_point_pos - joint_pos[joint]);
            //jacobian.col(joint) = Eigen::Vector3d::Zero();
        }
    }
}

void CollisionPoint::draw(const Eigen::Transform3d& T, bool yellow) const
{
    Eigen::Vector3d point = T*m_position;

    // cout << "point : " << point.transpose() << endl;

    // yellow
    double colorvector[4];
    colorvector[0] = 1.0;
    colorvector[1] = 1.0;
    colorvector[2] = 0.0;
    colorvector[3] = 0.7;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (yellow)
    {
        glColor4dv(colorvector);
    }

    g3d_draw_solid_sphere( point[0], point[1], point[2], m_radius, 20 );
    glDisable(GL_BLEND);
}

BoundingCylinder::BoundingCylinder(const Eigen::Vector3d& vect1, const Eigen::Vector3d& vect2,
                                   const Eigen::Vector3d& vect5, const Eigen::Vector3d& vect6)
{
    m_p1 = 0.5*(vect1+vect2);
    m_p2 = 0.5*(vect5+vect6);

    m_radius = 0.5*( ( vect1 - vect2 ).norm() );
}

void BoundingCylinder::draw( const Eigen::Transform3d& T )
{
    double p1[3], p2[3];

    Eigen::Vector3d p1Trans;
    Eigen::Vector3d p2Trans;

    //  if (withTransform) {
    //    p1Trans = m_p1;
    //    p2Trans = m_p2;
    //  }
    //  else {
    p1Trans = T*m_p1;
    p2Trans = T*m_p2;
    //  }

    p1[0] = p1Trans[0];
    p1[1] = p1Trans[1];
    p1[2] = p1Trans[2];

    p2[0] = p2Trans[0];
    p2[1] = p2Trans[1];
    p2[2] = p2Trans[2];

    double colorvector[4];
    colorvector[0] = 1.0;
    colorvector[1] = 1.0;
    colorvector[2] = 0.0;
    colorvector[3] = 0.7;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4dv(colorvector);
    g3d_draw_cylinder(p1, p2, m_radius, 20 );

    glDisable(GL_BLEND);
}


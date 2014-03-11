/*
 *  CollisionPoint.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef COLLISION_POINT_HPP_
#define COLLISION_POINT_HPP_

#include <vector>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

#include "planner/TrajectoryOptim/Chomp/chompUtils.hpp"

namespace Move3D
{

class BoundingCylinder  
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    BoundingCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double radius) :
        m_p1(p1), m_p2(p2), m_radius(radius)
    {}

    BoundingCylinder(const Eigen::Vector3d& vect1, const Eigen::Vector3d& vect2,
                     const Eigen::Vector3d& vect5, const Eigen::Vector3d& vect6);

    double getLength() { return ( m_p1 - m_p2 ).norm(); }
    double getRadius() { return m_radius; }

    Eigen::Vector3d& getPoint1() { return m_p1; }
    Eigen::Vector3d& getPoint2() { return m_p2; }

    void draw( const Eigen::Transform3d& T );

private:
    Eigen::Vector3d m_p1;
    Eigen::Vector3d m_p2;
    double m_radius;
};

class CollisionPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CollisionPoint(const std::vector<int>& parent_joints, double radius, double clearance, int segment_number, const Eigen::Vector3d& position);

    CollisionPoint(const CollisionPoint &point, const std::vector<int>& parent_joints);

    virtual ~CollisionPoint();

    bool isParentJoint(int joint) const;
    double getRadius() const;
    double getVolume() const;
    double getClearance() const;
    double getInvClearance() const;
    int getSegmentNumber() const;

    const Eigen::Vector3d& getPosition() const;

    const std::vector<int>& getParentJoints() const
    {
        return m_parent_joints;
    }

    void getTransformedPosition(std::vector<Eigen::Transform3d>& segment_frames, Eigen::Vector3d& position) const;
    void getTransformedPosition(std::vector<std::vector<double> >& segment_frames, Eigen::Vector3d& position) const;

    void getJacobian(std::vector</*Eigen::Map<*/Eigen::Vector3d> /*>*/& joint_pos,
                     std::vector</*Eigen::Map<*/Eigen::Vector3d> /*>*/& joint_axis,
                     /*Eigen::Map<*/Eigen::Vector3d/*>*/& collision_point_pos,
                     Eigen::MatrixXd& jacobian,
                     const std::vector<int>& group_joint_to_move3d_joint_index) const;

    void draw(const Eigen::Transform3d& T, bool yellow=true) const;

    bool m_is_colliding;                  /**< Collision point in collision */

private:
    std::vector<int> m_parent_joints;      /**< Which joints can influence the motion of this point */
    double m_radius;                       /**< Radius of the sphere */
    double m_volume;                       /**< Volume of the sphere */
    double m_clearance;                    /**< Extra clearance required while optimizing */
    double m_inv_clearance;                /**< 1/clearance_ pre-computed */
    int m_segment_number;                  /**< Which segment does this point belong to */
    Eigen::Vector3d m_position;            /**< Vector of this point in the frame of the above segment */
};

inline bool CollisionPoint::isParentJoint(int joint) const
{
    return(find(m_parent_joints.begin(),
                m_parent_joints.end(), joint) != m_parent_joints.end());
}

inline double CollisionPoint::getRadius() const
{
    return m_radius;
}

inline double CollisionPoint::getVolume() const
{
    return m_volume;
}

inline double CollisionPoint::getClearance() const
{
    return m_clearance;
}

inline double CollisionPoint::getInvClearance() const
{
    return m_inv_clearance;
}

inline int CollisionPoint::getSegmentNumber() const
{
    return m_segment_number;
}

inline const Eigen::Vector3d& CollisionPoint::getPosition() const
{
    return m_position;
}

inline void CollisionPoint::getTransformedPosition(std::vector<Eigen::Transform3d>& segment_frames, 
                                                   Eigen::Vector3d& position) const
{
    position = segment_frames[m_segment_number] * m_position;
}

inline void CollisionPoint::getTransformedPosition(std::vector<std::vector<double> >& segment_frames, 
                                                   Eigen::Vector3d& position) const
{
    Eigen::Transform3d T;
    stdVectorToEigenTransform( segment_frames[m_segment_number], T );
    position = T*m_position;
}

}

#endif /* COLLISION_POINT_HPP */

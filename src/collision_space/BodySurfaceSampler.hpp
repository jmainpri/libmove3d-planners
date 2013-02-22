/*
 *  BodySurfaceSampler.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 23/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef BODY_SURFACE_SAMPLING_H
#define BODY_SURFACE_SAMPLING_H

#include "p3d.h"
#include "environment.h"

#include "API/Device/robot.hpp"
#include "API/Grids/PointCloud.hpp"

#include "CollisionPoint.hpp"

#include <map>

#ifndef _P3D_H
typedef struct obj;
typedef struct env;
#endif

class BodySurfaceSampler 
{
public:
    BodySurfaceSampler(double step);
    ~BodySurfaceSampler();

    // Sample on the object surface
    PointCloud& sampleObjectSurface( obj* obj, bool isRobot = false );
    void sampleStaticObjectsSurface();
    void sampleRobotBodiesSurface(Robot* rob);
    void sampleAllRobotsBodiesSurface();

    bool isPointInEnvironment( const Eigen::Vector3d& point );
    bool isPointOverGround( const Eigen::Vector3d& point );

    // Access to the sampled point cloud given a robot object
    PointCloud& getPointCloud( obj* o )
    {
        return *m_objectToPointCloudMap[o];
    }

    BoundingCylinder* generateBoudingCylinder( p3d_obj* obj );

    //! returns the max radius
    double generateRobotBoudingCylinder(Robot* rob, const std::vector<Joint*>& activeJoints);
    void generateAllRobotsBoundingCylinders();

    // Generate collision points
    std::vector<CollisionPoint> getLinksCollisionPoints(Joint* jnt, int segment_number , const std::vector<int>& parent_joints );
    std::vector<CollisionPoint> generateJointCollisionPoints(Robot* robot, int id, const std::vector<int>& active_joints, const std::vector<int>& planner_joints);
    std::vector<CollisionPoint> generateRobotCollisionPoints(Robot* robot, const std::vector<int>& active_joints, const std::vector<int>& planner_joints);
    std::vector<CollisionPoint> generateAllRobotCollisionPoints(Robot* robot);

    // Access a collision point vector
    // Given a joint
    std::vector<CollisionPoint>& getCollisionPoints(Joint* jnt)
    {
        return m_jointToCollisionPoint[jnt];
    }

    double getDefaultClearance() { return m_collision_clearance_default; }

    void draw();

private:
    env*                                            m_env;
    double                                          m_step;
    double                                          m_collision_clearance_default;
    PointCloud                                      m_staticPoints;
    std::map<obj*,PointCloud*>                      m_objectToPointCloudMap;
    std::map<obj*,BoundingCylinder*>                m_objectToBoCylinderMap;
    std::map<Joint*, std::vector<CollisionPoint> >  m_jointToCollisionPoint;
};

#endif
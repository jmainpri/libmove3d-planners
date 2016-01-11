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

#include <libmove3d/include/p3d.h>

#include "API/Device/robot.hpp"
#include "API/Grids/PointCloud.hpp"

#include "collision_point.hpp"

#include <map>

#ifndef _P3D_H
typedef struct obj;
typedef struct env;
#endif

namespace Move3D {

class BodySurfaceSampler {
 public:
  BodySurfaceSampler(double step, double clearance);
  ~BodySurfaceSampler();

  //-----------------------------------------------------
  // Sample body surface points
  //-----------------------------------------------------
  PointCloud& sampleObjectSurface(obj* obj, bool isRobot = false);
  void sampleObjectInside(p3d_obj* obj, PointCloud& pc, bool isRobot = false);
  void sampleStaticObjectsSurface();
  void sampleRobotBodiesSurface(Robot* rob);
  void sampleAllRobotsBodiesSurface();

  // Access to the sampled point cloud given a robot object
  PointCloud& getPointCloud(obj* o);

  //-----------------------------------------------------
  // Generate cylinders
  //-----------------------------------------------------
  BoundingCylinder* generateBoudingCylinder(p3d_obj* obj);
  double generateRobotBoudingCylinder(Robot* rob,
                                      const std::vector<Joint*>& activeJoints);
  void generateAllRobotsBoundingCylinders();

  //-----------------------------------------------------
  // Generate collision points
  //-----------------------------------------------------
  std::vector<CollisionPoint> getLinksCollisionPoints(
      Joint* jnt, int segment_number, const std::vector<int>& parent_joints);
  std::vector<CollisionPoint> generateJointCollisionPoints(
      Robot* robot,
      int id,
      const std::vector<int>& active_joints,
      const std::vector<int>& planner_joints);
  std::vector<CollisionPoint> generateRobotCollisionPoints(
      Robot* robot,
      const std::vector<int>& active_joints,
      const std::vector<int>& planner_joints,
      int id_first_active_joint = 2);
  std::vector<CollisionPoint> generateAllRobotCollisionPoints(Robot* robot);

  // Access a collision point vector
  // Given a joint
  std::vector<CollisionPoint>& getCollisionPoints(Joint* jnt);

  //-----------------------------------------------------
  // Draws sampled and collision points
  //-----------------------------------------------------
  void draw();

 private:
  bool isPointInEnvironment(const Eigen::Vector3d& point);
  bool isPointOverGround(const Eigen::Vector3d& point);

  double m_step;
  double m_collision_clearance_default;
  std::map<obj*, PointCloud> m_objectToPointCloudMap;
  std::map<obj*, BoundingCylinder*> m_objectToBoCylinderMap;
  std::map<Joint*, std::vector<CollisionPoint> > m_jointToCollisionPoint;
};
}

#endif

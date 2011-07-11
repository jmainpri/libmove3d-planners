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
	
	void computeObjectPointCloud( obj* obj, PointCloud& pointCloud, bool isRobot = false );
	void computeStaticObjectsPointCloud();
	void computeRobotBodiesPointCloud(Robot* rob);
	void computeAllRobotsBodiesPointCloud();
  
  bool isPointInEnvironment( const Eigen::Vector3d& point );
  
  PointCloud& getPointCloud(obj* o) 
  { 
    return*m_objectToPointCloudMap[o]; 
  }
  
  BoundingCylinder* generateBoudingCylinder(p3d_obj* obj);
  
  //! returns the max radius
  double generateRobotBoudingCylinder(Robot* rob, const std::vector<Joint*>& activeJoints);
  void generateAllRobotsBoundingCylinders();
  
  std::vector<CollisionPoint> generateRobotCollisionPoints(Robot* robot, const std::vector<int>& active_joints, const std::vector<int>& planner_joints);
  
  std::vector<CollisionPoint> getLinksCollisionPoints(Joint* jnt, int segment_number , const std::vector<int>& parent_joints );
  
  double getDefaultClearance() { return m_collision_clearance_default; } 
  
  void draw();
	
private:
	env*      m_env;
	double		m_step;
  
  std::map<obj*,PointCloud*>  m_objectToPointCloudMap;
  
  PointCloud m_staticPoints;
  std::vector<PointCloud*> m_robotsPoints;
  
  std::map<obj*,BoundingCylinder*>    m_objectToBoCylinderMap;
  
  std::vector<CollisionPoint*> m_collisionPoints;
  
  double m_collision_clearance_default;
};

#endif
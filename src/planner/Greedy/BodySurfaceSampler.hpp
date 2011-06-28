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

class BoundingCylinder  
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  BoundingCylinder(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double radius) :
  m_p1(p1), m_p2(p2), m_radius(radius)
  {}
  
  BoundingCylinder(const Eigen::Vector3d& vect1, const Eigen::Vector3d& vect2,
                   const Eigen::Vector3d& vect5, const Eigen::Vector3d& vect6);
  
  void draw( const Eigen::Transform3d& T );
  
private:
  Eigen::Vector3d m_p1;
  Eigen::Vector3d m_p2;
  double m_radius;
};

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
  void generateRobotBoudingCylinder(Robot* rob);
  void generateAllRobotsBoundingCylinders();
  
  std::vector<CollisionPoint> generateAllCollisionPoints(Robot* rob);
  std::vector<CollisionPoint> generateCollisionPointsForLink(Robot* rob,int link_id);
  
  void draw();
	
private:
	env*      m_env;
	double		m_step;
  
  std::map<obj*,PointCloud*>  m_objectToPointCloudMap;
  
  PointCloud m_staticPoints;
  std::vector<PointCloud*> m_robotsPoints;
  
  std::map<obj*,BoundingCylinder*>    m_objectToBoCylinderMap;
  
  std::vector<CollisionPoint*> m_collisionPoints;
};

#endif
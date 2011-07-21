/*
 *  BodySurfaceSampler.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 23/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "BodySurfaceSampler.hpp"
#include "project.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"

using namespace std;

BodySurfaceSampler::BodySurfaceSampler(double step) : 
m_step(step),
m_collision_clearance_default(0.10)
{
	
}

BodySurfaceSampler::~BodySurfaceSampler()
{
  cout << "Delete BodySurfaceSampler" << endl;
  
  for (map<p3d_obj*,PointCloud*>::iterator it = m_objectToPointCloudMap.begin();
       it != m_objectToPointCloudMap.end();it++) 
  {
    if ( it->second )
    {
      delete it->second;
    }
  }
  
  for (map<p3d_obj*,BoundingCylinder*>::iterator it = m_objectToBoCylinderMap.begin();
       it != m_objectToBoCylinderMap.end();it++) 
  {
    if ( it->second )
    {
      delete it->second;
    }
  }
}

void BodySurfaceSampler::computeObjectPointCloud( p3d_obj* obj, PointCloud& pointCloud, bool isRobot )
{
  //  cout << "Sample Object : " << obj->name << endl;
  m_objectToPointCloudMap[obj] = new PointCloud( 0.02 );
  pointCloud = *m_objectToPointCloudMap[obj];
  
  string objName = obj->name;
  
	for(int i = 0; i < obj->np; i++)
	{
		if(obj->pol[i]->TYPE != P3D_GRAPHIC)
		{
			p3d_polyhedre* poly = obj->pol[i]->poly;
			p3d_vector3 the_points[poly->nb_points];
      
			for(unsigned int j = 0; j < poly->nb_points; j++)
			{
				the_points[j][0] = poly->the_points[j][0];
				the_points[j][1] = poly->the_points[j][1];
				the_points[j][2] = poly->the_points[j][2];
        
				if (obj->type == P3D_OBSTACLE)
				{//real point position
					p3d_xformPoint(obj->pol[i]->pos0, poly->the_points[j], the_points[j]);
				}
				else
				{
					p3d_matrix4 inv_pos, mat;
					p3d_matInvertXform( obj->jnt->pos0, inv_pos );
					p3d_matMultXform(inv_pos, obj->pol[i]->pos0, mat);
					p3d_xformPoint(mat, poly->the_points[j], the_points[j]);
				}
			}
			
			for(unsigned int j = 0; j < poly->nb_faces; j++)
			{
				unsigned int nbPoints = 0;
				
				p3d_vector3* tmp = sample_triangle_surface(
                                                   the_points[poly->the_faces[j].the_indexs_points[0] - 1], 
                                                   the_points[poly->the_faces[j].the_indexs_points[1] - 1], 
                                                   the_points[poly->the_faces[j].the_indexs_points[2] - 1], m_step, &nbPoints);
        
        //#ifdef DPG
        
        //				obj->pointCloud = MY_REALLOC(obj->pointCloud, 
        //                                     p3d_vector3, 
        //                                     obj->nbPointCloud, 
        //                                     obj->nbPointCloud + nbPoints);
        
				for(unsigned int k = 0; k < nbPoints; k++)
				{
          Eigen::Vector3d point;
          
          point(0) = tmp[k][0];
          point(1) = tmp[k][1];
          point(2) = tmp[k][2];
          
          //					obj->pointCloud[obj->nbPointCloud + k][0] = point(0);
          //					obj->pointCloud[obj->nbPointCloud + k][1] = point(1);
          //					obj->pointCloud[obj->nbPointCloud + k][2] = point(2);
          if( isRobot || isPointInEnvironment( point ) )
          { 
            (*m_objectToPointCloudMap[obj]).push_back( point ); 
          }
          //          else {
          //            cout << "Point not in environment, Object : " << obj->name << endl;
          //          }
          //          cout << point << endl;
				}
        //				obj->nbPointCloud += nbPoints;
        //#endif
				free(tmp);
			}
		}
	}
}

void BodySurfaceSampler::computeStaticObjectsPointCloud()
{
  m_staticPoints.clear();
  
	for(int i = 0; i < XYZ_ENV->no; i++)
	{
		computeObjectPointCloud( XYZ_ENV->o[i], m_staticPoints );
	}
}

void BodySurfaceSampler::computeRobotBodiesPointCloud(Robot* robot)
{
  m_robotsPoints.push_back( new PointCloud );
  
	for(unsigned int i=0; i<robot->getNumberOfJoints(); i++)
	{
    p3d_obj* obj = robot->getJoint(i)->getJointStruct()->o;
    
		if(obj)
    { computeObjectPointCloud( obj, *m_robotsPoints.back() , true ); }
	}
}

void BodySurfaceSampler::computeAllRobotsBodiesPointCloud()
{
  for (unsigned int i=0; i<m_robotsPoints.size(); i++) 
  {
    m_robotsPoints[i]->clear();
  }
  
  m_robotsPoints.clear();
  
  Scene* sc = global_Project->getActiveScene();
  
	for(unsigned int i=0;i<sc->getNumberOfRobots(); i++)
	{
 		computeRobotBodiesPointCloud( sc->getRobot(i) );
	}
}

bool BodySurfaceSampler::isPointInEnvironment( const Eigen::Vector3d& point ) 
{
  vector<double> bounds = global_Project->getActiveScene()->getBounds();
  
  if ( (point[0] < bounds[0]) || (point[0] > bounds[1]) ) {
    return false;
  }
  if ( (point[1] < bounds[2]) || (point[1] > bounds[3]) ) {
    return false;
  }
  if ( (point[2] < bounds[4]) || (point[2] > bounds[5]) ) {
    return false;
  }
  
  return true;
}

BoundingCylinder* BodySurfaceSampler::generateBoudingCylinder(p3d_obj* obj)
{
  if(!obj)
  { 
    cout << "Problem object is NILL" << endl;
    return NULL; }
  
  double box[8][3];
  std::vector<Eigen::Vector3d> cuboide(8);
  
  double radius;
  Eigen::Vector3d p1,p2;
  
  if ( pqp_get_OBB_first_level(obj,box) )
  {
    for (unsigned int i=0; i<8; i++) {
      for(unsigned int j=0; j<3; j++) {
        cuboide[i][j] = box[i][j];
      }
    }
    
    p1 = 0.5*(cuboide[1]+cuboide[2]);
    p2 = 0.5*(cuboide[5]+cuboide[6]);
    
    radius = 0.5*( ( cuboide[1] - cuboide[2] ).norm() );
  }
  else 
  { 
    //cout << "No Bounding Box" << endl;
    return NULL; 
  }
  
  return new BoundingCylinder(p1,p2,radius);
}

double BodySurfaceSampler::generateRobotBoudingCylinder(Robot* robot,const vector<Joint*>& activeJoints)
{
  double maxRadius= numeric_limits<double>::min();
  
  for(unsigned int i=0;i<activeJoints.size();i++)
  {
    p3d_obj* obj = activeJoints[i]->getJointStruct()->o;
    
    if(obj)
    { 
      BoundingCylinder* bc = generateBoudingCylinder(obj);
      
      if (bc) 
      {
        m_objectToBoCylinderMap[obj] = bc;
        if ( maxRadius < bc->getRadius() ) 
        {
          maxRadius = bc->getRadius();
        }
      }
    }
  }
  
  return maxRadius;
}

void BodySurfaceSampler::generateAllRobotsBoundingCylinders()
{
  Scene* sc = global_Project->getActiveScene();
  Robot* rob;
  
	for(unsigned int i=0;i<sc->getNumberOfRobots(); i++)
	{
    rob = sc->getRobot(i);
 		generateRobotBoudingCylinder( rob , rob->getAllJoints() );
	}
}

std::vector<CollisionPoint> BodySurfaceSampler::generateRobotCollisionPoints(Robot* robot, const std::vector<int>& active_joints, const std::vector<int>& planner_joints )
{
  std::vector<CollisionPoint> all_points;
  all_points.clear();
  
  std::vector<int> parent_joints;
  
  for (unsigned int i=1; i<active_joints.size(); i++) 
  {
    int joint = active_joints[i];
    
    // Compute the planner joints, this doesn't handle trees
    // only handles chains, the planner joints before the active 
    // joints are taken into account
//    parent_joints.clear();
//    for (unsigned int j=0; j<planner_joints.size(); j++) 
//    {
//      if ( planner_joints[j] <= joint ) 
//      {
//        parent_joints.push_back( planner_joints[j] );
//      }
//    }
    
    parent_joints.clear();
    for (unsigned int j=0; j<active_joints.size(); j++) 
    {
      if ( active_joints[j] <= joint ) 
      {
        parent_joints.push_back( active_joints[j] );
      }
    }
    
//    if ( !find(parent_joints.begin(), parent_joints.end(), joint ) 
//        != parent_joints.end()); ) 
//    {
//        
//    }
    
    // Get collision points of the link of the assciated joint
    Joint* jnt = robot->getJoint( joint );
    
    int segment;
    
    // Becarefull!!!
//    if ( i >= (planner_joints.size()-1) ) {
//      segment = planner_joints.size()-1;
//    }
//    else {
      segment = i;
//    }
    
    std::vector<CollisionPoint> points = getLinksCollisionPoints( jnt, segment, parent_joints );
    
    // Stores the collision point in a map of the corresponding joint
    m_jointToCollisionPoint[jnt] = points;
    
    for (unsigned int j=0; j<points.size(); j++) 
    {
      all_points.push_back( points[j] );
    }
  }
  
  return all_points;
}

std::vector<CollisionPoint> BodySurfaceSampler::getLinksCollisionPoints(Joint* jnt, int segment_number , const std::vector<int>& parent_joints )
{
  std::vector<CollisionPoint> collision_points;
  
  if (m_objectToBoCylinderMap.empty())
    return collision_points;
  
  BoundingCylinder* bc = m_objectToBoCylinderMap[jnt->getJointStruct()->o];
  
  if (bc == NULL)
    return collision_points;
  
  double radius = bc->getRadius();
  double length = bc->getLength();
  
//  Eigen::Vector3d p(0,0,0);
//  Eigen::Vector3d p2;
  
  Eigen::Vector3d p1 = bc->getPoint1();
  Eigen::Vector3d p2 = bc->getPoint2();
  
  Eigen::Vector3d p;
  
  double spacing = radius/2.0;
  int num_points = ceil(length/spacing)+1;
  spacing = length/(num_points-1.0);
  
//  cout << "spacing = " << spacing << endl;
//  cout << "num_points = " << num_points << endl;
  
  for (int i=0; i<num_points; ++i) 
  {
//    p(2) = -length/2.0 + i*spacing;
//    p2 = f*p;
    
    Eigen::Vector3d p = p1 + ((double)i/(double)num_points)*(p2-p1);
    
//    cout << "Center(" << i << ") :  " << endl << p << endl;
    
    collision_points.push_back(CollisionPoint(parent_joints, radius, m_collision_clearance_default, segment_number, p));
  }
  
  return collision_points;
}

void BodySurfaceSampler::draw()
{
	for(int i = 0; i < XYZ_ENV->no; i++)
	{
		//m_objectToPointCloudMap[XYZ_ENV->o[i]]->drawAllPoints();
	}  
  
  Joint* jnt;
  Robot* rob;
  Scene* sc = global_Project->getActiveScene();
  
	for(unsigned int j=0; j<sc->getNumberOfRobots(); j++)
	{
		rob = sc->getRobot(j) ;
    
    for( unsigned int i=0; i<rob->getNumberOfJoints(); i++ )
    {
      jnt = rob->getJoint(i);
      
      Eigen::Transform3d t = jnt->getMatrixPos();
//      vector<CollisionPoint> points = m_jointToCollisionPoint[jnt];
//      
//      for( unsigned int i=0; i<points.size(); i++ )
//      {
//        points[i].draw(t);
//      }
      
      p3d_obj* obj = jnt->getJointStruct()->o;
      
      if( obj )
      {
        m_objectToPointCloudMap[obj]->drawAllPoints(t,NULL);
        
        if (rob->getName().find("PR2") == string::npos )
          continue;
        
        BoundingCylinder* bc = m_objectToBoCylinderMap[obj];
        
        if (bc) 
        {
          bc->draw(t);
        }
      }
    }
	}
}

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

BodySurfaceSampler::BodySurfaceSampler(double step) : m_step(step)
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

void BodySurfaceSampler::generateRobotBoudingCylinder(Robot* robot)
{
  for(unsigned int i=0;i<robot->getNumberOfJoints();i++)
  {
    p3d_obj* obj = robot->getJoint(i)->getJointStruct()->o;
    
		if(obj)
    { 
      BoundingCylinder* bc = generateBoudingCylinder(obj);
      
      if (bc) {
        m_objectToBoCylinderMap[obj] = bc;
      }
    }
  }
}

void BodySurfaceSampler::generateAllRobotsBoundingCylinders()
{
  Scene* sc = global_Project->getActiveScene();
  
	for(unsigned int i=0;i<sc->getNumberOfRobots(); i++)
	{
 		generateRobotBoudingCylinder( sc->getRobot(i) );
	}
}

std::vector<CollisionPoint> BodySurfaceSampler::generateAllCollisionPoints(Robot* robot)
{
  std::vector<CollisionPoint> points;

  
  return points;
}

std::vector<CollisionPoint> BodySurfaceSampler::generateCollisionPointsForLink(Robot* rob, int link_id)
{
  std::vector<CollisionPoint> points;
  Joint* jnt = rob->getJoint(link_id);
  double box[8][3];
  p3d_obj* obj = jnt->getJointStruct()->o;
  
  return points;
}

void BodySurfaceSampler::draw()
{
//	for(int i = 0; i < XYZ_ENV->no; i++)
//	{
//		m_objectToPointCloudMap[XYZ_ENV->o[i]]->drawAllPoints();
//	}
  
  Joint* jnt;
  Robot* robot;
  Scene* sc = global_Project->getActiveScene();
  
	for(unsigned int j=0; j<sc->getNumberOfRobots(); j++)
	{
		robot = sc->getRobot(j) ;
    
    if (robot->getName().find("PR2") == string::npos )
      continue;
        
    for( unsigned int i=2; i<robot->getNumberOfJoints(); i++ )
    {
      jnt = robot->getJoint(i);
      p3d_obj* obj = jnt->getJointStruct()->o;
      
      if(obj)
      {
        Eigen::Transform3d t = jnt->getMatrixPos();
        
        //m_objectToPointCloudMap[obj]->drawAllPoints(t);
    
        BoundingCylinder* bc = m_objectToBoCylinderMap[obj];
        if (bc) {
          bc->draw(t);
        }
      }
    }
	}
}

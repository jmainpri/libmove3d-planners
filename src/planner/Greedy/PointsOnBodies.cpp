/*
 *  PointsOnBodies.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 23/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "P3d-pkg.h"
#include "PointsOnBodies.hpp"
#include "project.hpp"

using namespace std;

BodySurfaceSampler::BodySurfaceSampler(double step) : m_step(step)
{
	
}

void BodySurfaceSampler::computeObjectPointCloud(p3d_obj* obj)
{
	for(int i = 0; i < obj->np; i++)
	{
		cout << "Object : " << obj->name << endl;
		
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
				
#ifdef DPG
				obj->pointCloud = MY_REALLOC(obj->pointCloud, p3d_vector3, obj->nbPointCloud, obj->nbPointCloud + nbPoints);
				
				for(unsigned int k = 0; k < nbPoints; k++)
				{
					obj->pointCloud[obj->nbPointCloud + k][0] = tmp[k][0];
					obj->pointCloud[obj->nbPointCloud + k][1] = tmp[k][1];
					obj->pointCloud[obj->nbPointCloud + k][2] = tmp[k][2];
				}
				obj->nbPointCloud += nbPoints;
#endif
				free(tmp);
			}
		}
	}
}

void BodySurfaceSampler::computeStaticObjectsPointCloud()
{
	for(int i = 0; i < XYZ_ENV->no; i++)
	{
		computeObjectPointCloud(XYZ_ENV->o[i]);
	}
}

void BodySurfaceSampler::computeRobotBodiesPointCloud(Robot* robot)
{
	for(int i = 0; i <= robot->getRobotStruct()->njoints; i++)
	{
		if(robot->getRobotStruct()->joints[i]->o)
		{
			computeObjectPointCloud(robot->getRobotStruct()->joints[i]->o);
		}
	}
}

void BodySurfaceSampler::computeAllRobotsBodiesPointCloud()
{
	for(unsigned int i = 0; i < global_Project->getActiveScene()->getNumberOfRobots(); i++)
	{
		computeRobotBodiesPointCloud(global_Project->getActiveScene()->getRobot(i));
	}
}

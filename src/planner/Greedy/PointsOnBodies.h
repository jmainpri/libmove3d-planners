/*
 *  PointsOnBodies.h
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

#ifndef _P3D_H
typedef struct obj;
typedef struct env;
#endif

class BodySurfaceSampler 
{
public:
	BodySurfaceSampler(double step);
	
	void computeObjectPointCloud(obj* obj);
	void computeStaticObjectsPointCloud();
	void computeRobotBodiesPointCloud(Robot* rob);
	void computeAllRobotsBodiesPointCloud();
	
private:
	env*		m_env;
	double		m_step;
};

#endif
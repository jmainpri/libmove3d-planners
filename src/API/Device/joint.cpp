/*
 *  joint.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/05/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "API/Device/robot.hpp"

#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry> 
#include <Eigen/LU>

#include <string>

#include "P3d-pkg.h"

using namespace std;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

Joint::Joint(Robot *R, p3d_jnt* jntPt, bool copy ) : m_Robot(R)
{
	m_Joint = jntPt;
	
	string name(jntPt->name);
    m_Name = name;
	
}

Vector3d Joint::getVectorPos()
{
	Vector3d v;
	
	v(0) = m_Joint->abs_pos[0][3];
	v(1) = m_Joint->abs_pos[1][3];
	v(2) = m_Joint->abs_pos[2][3];
	
	return v;
}

Transform3d Joint::getMatrixPos()
{
	Transform3d t;
	
	t(0,0) = m_Joint->abs_pos[0][0];
	t(1,0) = m_Joint->abs_pos[1][0];
	t(2,0) = m_Joint->abs_pos[2][0];
	t(3,0) = 0.0;
	
	t(0,1) = m_Joint->abs_pos[0][1];
	t(1,1) = m_Joint->abs_pos[1][1];
	t(2,1) = m_Joint->abs_pos[2][1];
	t(3,1) = 0.0;
	
	t(0,2) = m_Joint->abs_pos[0][2];
	t(1,2) = m_Joint->abs_pos[1][2];
	t(2,2) = m_Joint->abs_pos[2][2];
	t(3,2) = 0.0;
	
	t(0,3) = m_Joint->abs_pos[0][3];
	t(1,3) = m_Joint->abs_pos[1][3];
	t(2,3) = m_Joint->abs_pos[2][3];
	t(3,3) = 1.0;
	
//	cout << "t of joint " << m_Joint->num << " = " << endl << t.matrix() << endl;
//	
//	p3d_mat4Print(m_Joint->abs_pos, "Abs Pos");
	
	//cout << "Warning: Joint::getAbsPos() undefined" << endl;
	return t;
}

p3d_matrix4* Joint::getAbsPos()
{ 
	return &(m_Joint->abs_pos); 
}

void Joint::shoot(Configuration& q,bool sample_passive)
{
	for(int j=0; j<m_Joint->dof_equiv_nbr; j++) 
	{
		int k = m_Joint->index_dof + j;
	
		if ( 
			sample_passive ||
			(p3d_jnt_get_dof_is_user(m_Joint, j) && p3d_jnt_get_dof_is_active_for_planner(m_Joint,j)) &&
			(q.getRobot()->getRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) 
		{
			double vmin,vmax;
			p3d_jnt_get_dof_rand_bounds(m_Joint, j, &vmin, &vmax);
			q[k] = p3d_random(vmin, vmax);
			//std::cout << "Sample Passive = "<<sample_passive<<" , Sampling q["<<k<<"] = "<<q[k]<< std::endl;
		} 
		else
		{ 
			q[k] = p3d_jnt_get_dof(m_Joint, j); 
		}
	}
}

double Joint::getJointDof(int ithDoF)
{
	return p3d_jnt_get_dof(m_Joint,ithDoF);
}

void Joint::setJointDof(int ithDoF, double value)
{
	p3d_jnt_set_dof(m_Joint,ithDoF,value);
}

void Joint::getDofBounds(int ithDoF, double& vmin, double& vmax)
{
	vmin = m_Joint->dof_data[ithDoF].vmin;
	vmax = m_Joint->dof_data[ithDoF].vmax;
}

unsigned int Joint::getNumberOfDof()
{
	return m_Joint->dof_equiv_nbr;
}

unsigned int Joint::getIndexOfFirstDof()
{
	return m_Joint->index_dof;
}

void Joint::setConfigFromDofValues(Configuration& q)
{
	for(int j=0; j<m_Joint->dof_equiv_nbr; j++) 
	{
		int k = m_Joint->index_dof + j;
		q[k] = getJointDof(j); 
	}
}

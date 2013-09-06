/*
 *  joint.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 12/05/10.
 *  Copyright 2010 LAAS-CNRS. All rights reserved.
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

Joint::Joint(Robot *R, p3d_jnt* jntPt, int id, bool copy ) : 
    m_Robot(R),
    m_id(id)
{
    m_Joint = jntPt;
    m_Name = jntPt->name;
}

Vector3d Joint::getVectorPos() const
{
    Vector3d v;

    pp3d_matrix4 mat = &(m_Joint->abs_pos);

    v(0) = (*mat)[0][3];
    v(1) = (*mat)[1][3];
    v(2) = (*mat)[2][3];

    return v;
}

Transform3d Joint::getMatrixPos() const
{
    Transform3d t;

    pp3d_matrix4 mat = &(m_Joint->abs_pos);

    t(0,0) = (*mat)[0][0];
    t(1,0) = (*mat)[1][0];
    t(2,0) = (*mat)[2][0];
    t(3,0) = 0.0;

    t(0,1) = (*mat)[0][1];
    t(1,1) = (*mat)[1][1];
    t(2,1) = (*mat)[2][1];
    t(3,1) = 0.0;

    t(0,2) = (*mat)[0][2];
    t(1,2) = (*mat)[1][2];
    t(2,2) = (*mat)[2][2];
    t(3,2) = 0.0;

    t(0,3) = (*mat)[0][3];
    t(1,3) = (*mat)[1][3];
    t(2,3) = (*mat)[2][3];
    t(3,3) = 1.0;

    return t;
}

pp3d_matrix4 Joint::getAbsPos()
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
                ((p3d_jnt_get_dof_is_user(m_Joint, j) && p3d_jnt_get_dof_is_active_for_planner(m_Joint,j)) &&
                 (q.getRobot()->getRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) )
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

double Joint::getJointDof(int ithDoF) const
{
    return p3d_jnt_get_dof(m_Joint,ithDoF);
}

void Joint::setJointDof(int ithDoF, double value)
{
    p3d_jnt_set_dof(m_Joint,ithDoF,value);
}

bool Joint::setFreeFlyerFromMatrix( const Eigen::Transform3d& T )
{
    if( m_Joint->type == P3D_FREEFLYER )
    {
        Eigen::Vector3d trans = T.translation();
        p3d_jnt_set_dof(m_Joint,0,trans[0]);
        p3d_jnt_set_dof(m_Joint,1,trans[1]);
        p3d_jnt_set_dof(m_Joint,2,trans[2]);

        cout << "trans : " << endl << trans << endl;

        Eigen::Vector3d rot = T.linear().eulerAngles(0, 1, 2);
        p3d_jnt_set_dof(m_Joint,3,rot[0]);
        p3d_jnt_set_dof(m_Joint,4,rot[1]);
        p3d_jnt_set_dof(m_Joint,5,rot[2]);
        return true;
    }
    else
        return false;
}

bool Joint::isJointDofUser(int ithDoF) const
{  
    return p3d_jnt_get_dof_is_user(m_Joint,ithDoF);
}

void Joint::getDofBounds(int ithDoF, double& vmin, double& vmax) const
{
    vmin = m_Joint->dof_data[ithDoF].vmin;
    vmax = m_Joint->dof_data[ithDoF].vmax;
}

void Joint::getDofRandBounds(int ithDoF, double& vmin, double& vmax) const
{
    vmin = m_Joint->dof_data[ithDoF].vmin_r;
    vmax = m_Joint->dof_data[ithDoF].vmax_r;
}

unsigned int Joint::getNumberOfDof() const
{
    return m_Joint->dof_equiv_nbr;
}

unsigned int Joint::getIndexOfFirstDof() const
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

Joint* Joint::getPreviousJoint()
{
    Joint* prevJnt=NULL; int found=0;

    for (unsigned int i=0; i<m_Robot->getNumberOfJoints(); i++ )
    {
        Joint* jnt = m_Robot->getJoint(i);

        if ( m_Joint->prev_jnt == jnt->m_Joint )
        {
            found++;
            prevJnt = jnt;
        }
    }

    if (found == 1) {
        return prevJnt;
    }
    else if (found > 1 ) {
        cout << "Found : " << found << " prev. joints!!!" << endl;
    }

    return NULL;
}

std::vector<Joint*> Joint::getAllPrevJoints()
{
    Joint* jnt( getPreviousJoint() );

    std::vector<Joint*> prevJoints;
    prevJoints.clear();

    cout << "Prev Joints ---------" << endl;
    while (jnt != NULL)
    {
        cout << "jnt->m_id = " << jnt->m_id << endl;
        prevJoints.push_back( jnt );
        jnt = jnt->getPreviousJoint();
    }

    return prevJoints;
}

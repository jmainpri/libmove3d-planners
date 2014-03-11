/*
 *  HRICS_Visbility.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"

#include "HRICS_Visibility.hpp"
#include "HRICS_costspace.hpp"

#include "P3d-pkg.h"

#include <Eigen/Core>
#include <Eigen/Geometry> 

using namespace std;
using namespace HRICS;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE


#define DISTANCE3D(x1,y1,z1,x2,y2,z2) (sqrt(((x2)-(x1))*((x2)-(x1))+((y2)-(y1))*((y2)-(y1))+((z2)-(z1))*((z2)-(z1))))

/*!
 * Constructor
 * Sets the Human structure
 */
Visibility::Visibility(Robot* H)
{
    if (H->getName().find("HUMAN") == string::npos)
    {
        cout << "HRICS::Visibility::Warning Human's name does not contain HUMAN" << endl;
        return;
    }

    m_Human = H;
}

/* The coordinates change from one human model to the other */
/* Change these numbers when you change the human model */
/* The rest will follow these changes */
//#define HUMANq_X 6
//#define HUMANq_Y 7
//#define HUMANq_Z 8
//#define HUMANq_RZ 11
//#define HUMANq_NECKZ 63
//#define HUMANq_PAN 64
//#define HUMANq_TILT 65

const double HRICS_HRI_EYE_TOLERANCE_TILT=0.3;
const double HRICS_HRI_EYE_TOLERANCE_PAN=0.3;

const bool Old_Achile =false;

/*!
 * Get the visibility Cost
 * This function computes an angle beetween a WSPoint and the Human gaze direction
 */
double Visibility::getWorkspaceCost(const Vector3d& WSPoint)
{
    Transform3d t = m_Human->getJoint(HRICS_HUMANj_NECK_TILT)->getMatrixPos();

    Vector3d gazeDir;
    gazeDir(0) = t(0,0);
    gazeDir(1) = t(1,0);
    gazeDir(2) = t(2,0);
    gazeDir.normalize();

    Vector3d pointDir;
    pointDir(0) = WSPoint(0) - t(0,3);
    pointDir(1) = WSPoint(1) - t(1,3);
    pointDir(2) = WSPoint(2) - t(2,3);
    pointDir.normalize();

    double alpha = gazeDir.dot( pointDir );

    return (acos(alpha)+1)/2;
}

double Visibility::getOldWorkspaceCost(const Vector3d& WSPoint)
{
    double phi,theta;
    double Dphi, Dtheta;
    p3d_vector4 realcoord,newcoord;
    p3d_matrix4 inv;

    realcoord[0] = WSPoint[0];
    realcoord[1] = WSPoint[1];
    realcoord[2] = WSPoint[2];
    realcoord[3] = 1;

    if (Old_Achile)
    {
        // get the right frame
        p3d_matrix4 newABS;
        p3d_matrix4 rotation ={
            {1,0,0,0},
            {0,1,0,0},
            {0,0,1,0},
            {0,0,0,1}};
        //Matrix4d newAbsPos; = m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos

        p3d_mat4Mult(m_Human->getRobotStruct()->joints[HRICS_HUMANj_NECK_TILT]->abs_pos,rotation,newABS);

        // Invert frame and get the point in this frame
        p3d_matInvertXform(newABS, inv);
        p3d_matvec4Mult(inv, realcoord, newcoord);
    }
    else
    {
        double Ccoord[6];
        p3d_mat4ExtractPosReverseOrder(m_Human->getRobotStruct()->joints[5]->abs_pos,
                                       Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);

        p3d_matInvertXform(m_Human->getRobotStruct()->joints[5]->abs_pos, inv);
        p3d_matvec4Mult(inv, realcoord, newcoord);
    }

    // Compute the angle the point makes with the
    // Angular coord of the vector
    Vector3d newCoordVect;
    newCoordVect[0] = newcoord[0];
    newCoordVect[1] = newcoord[1];
    newCoordVect[2] = newcoord[2];

    phi = ABS(atan2( newCoordVect[1],newCoordVect[0]));
    //theta = ABS(acos( newCoordVect[2]/newCoordVect.norm() )- M_PI_2);

    Vector2d newCoord2D;
    newCoord2D[0] = newcoord[0];
    newCoord2D[1] = newcoord[1];

    theta = ABS(atan2( newCoordVect[2],newCoord2D.norm() ));

    // Compute delta cost for PAN
    if(phi < HRICS_HRI_EYE_TOLERANCE_PAN/2.)
        Dphi = 0;
    else
        Dphi = phi - HRICS_HRI_EYE_TOLERANCE_PAN/2.;

    // Compute delta cost for TILT
    if(theta < HRICS_HRI_EYE_TOLERANCE_TILT/2.)
        Dtheta = 0;
    else
        Dtheta = theta - HRICS_HRI_EYE_TOLERANCE_TILT/2.;

    m_Cost = (1/0.65)*((Dtheta+Dphi)/(M_2PI-(HRICS_HRI_EYE_TOLERANCE_TILT/2.)-(HRICS_HRI_EYE_TOLERANCE_PAN/2.)));

    // Add coeff from original formula
    m_Cost *= 2.;
    //    cout << "Visib =  "  << cost << endl;
    return m_Cost;
}

/*!
 * Get the visibility Cost
 * Old Akin function
 */
double Visibility::akinVisibilityCost(const Vector3d& WSPoint)
{
    double phi,theta;
    double Dphi, Dtheta;
    //	double Ccoord[6];
    p3d_vector4 realcoord,newcoord;
    p3d_matrix4 inv;

    realcoord[0] = WSPoint[0];
    realcoord[1] = WSPoint[1];
    realcoord[2] = WSPoint[2];
    realcoord[3] = 1;

    // get the right frame
    p3d_matrix4 newABS;
    p3d_matrix4 rotation ={	{1,0,0,0},
                            {0,0,-1,0},
                            {0,1,0,0},
                            {0,0,0,1}};
    //Matrix4d newAbsPos; = m_Human->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos

    p3d_mat4Mult(m_Human->getRobotStruct()->joints[HRICS_HUMANj_NECK_TILT]->abs_pos,rotation,newABS);

    //	p3d_mat4ExtractPosReverseOrder(m_Human->getRobotStruct()->joints[6]->abs_pos,
    //								   Ccoord, Ccoord+1, Ccoord+2,Ccoord+3, Ccoord+4, Ccoord+5);

    p3d_matInvertXform(newABS,inv);
    p3d_matvec4Mult(inv, realcoord, newcoord);

    {
        double x =newcoord[0];
        double y =newcoord[1];
        double z =newcoord[2];

        double originx=0;
        double originy=0;
        double originz=0;

        double distance = DISTANCE3D(x,y,z,originx,originy,originz);

        phi = atan2( (y-originy),(x-originx) );
        theta = acos( (z-originz)/distance );
    }

    phi = ABS(phi);
    theta = ABS(theta - M_PI_2);

    if(phi < HRICS_HRI_EYE_TOLERANCE_PAN/2.)
        Dphi = 0;
    else
        Dphi = phi - HRICS_HRI_EYE_TOLERANCE_PAN/2.;

    if(theta < HRICS_HRI_EYE_TOLERANCE_TILT/2.)
        Dtheta = 0;
    else
        Dtheta = theta - HRICS_HRI_EYE_TOLERANCE_TILT/2.;

    return (Dtheta+Dphi)/(M_2PI-(HRICS_HRI_EYE_TOLERANCE_TILT/2.)-(HRICS_HRI_EYE_TOLERANCE_PAN/2.))/0.65;
    //return Dphi/M_PI;
    //return Dtheta/M_PI_2;
}

/*!
 * Get the Gaze direction to show in the OpenGl viewer
 */
std::vector<double> Visibility::getGaze()
{
    Vector3d point;
    p3d_vector3 gazeOrigin;

    const double length = 1.5;

    Transform3d t = m_Human->getJoint(HRICS_HUMANj_NECK_TILT)->getMatrixPos();

    gazeOrigin[0] = t(0,3);
    gazeOrigin[1] = t(1,3);
    gazeOrigin[2] = t(2,3);

    Vector3d xAxis;
    xAxis[0] = gazeOrigin[0] + length*t(0,0);
    xAxis[1] = gazeOrigin[1] + length*t(1,0);
    xAxis[2] = gazeOrigin[2] + length*t(2,0);

    m_VectGaze.clear();
    m_VectGaze.push_back(gazeOrigin[0]);
    m_VectGaze.push_back(gazeOrigin[1]);
    m_VectGaze.push_back(gazeOrigin[2]);

    m_VectGaze.push_back(xAxis[0]);
    m_VectGaze.push_back(xAxis[1]);
    m_VectGaze.push_back(xAxis[2]);

    return m_VectGaze;
}

Vector2d Visibility::get2dPointAlongGaze(double dist)
{
    getGaze();

    Vector2d origin; Vector2d direct;

    origin(0) = m_VectGaze[0];
    origin(1) = m_VectGaze[1];

    direct(0) = m_VectGaze[3];
    direct(1) = m_VectGaze[4];

    Vector2d x = (direct - origin) / (direct - origin).norm();

    return dist*x;
}

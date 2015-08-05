/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "PointCloud.hpp"
#include "Graphic-pkg.h"

// import most common Eigen types 
//using namespace Eigen;
using namespace Eigen;

PointCloud* PointsToDraw=NULL;

PointCloud::PointCloud()
{
    m_CubeSize[0] = 0.01;
    m_CubeSize[1] = 0.01;
    m_CubeSize[2] = 0.01;
}

// Cube size used for drawing
PointCloud::PointCloud(double PointSize)
{
    m_CubeSize[0] = PointSize;
    m_CubeSize[1] = PointSize;
    m_CubeSize[2] = PointSize;
}

// Adds a point to the cloud
void PointCloud::push_back(const Eigen::Vector3d& point)
{
    m_AllPoints.push_back( point );
}

void PointCloud::drawAllPoints(double* color)
{
    double colorvector[4];

    if (color == NULL)
    {
        colorvector[0] = 1.0;       //red
        colorvector[1] = 0.0;       //green
        colorvector[2] = 0.0;       //blue
        colorvector[3] = 1.0;       //transparency
    }
    else
    {
        colorvector[0] = color[0];       //red
        colorvector[1] = color[1];       //green
        colorvector[2] = color[2];       //blue
        colorvector[3] = color[3];       //transparency
    }

    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);

    Eigen::Transform3d t(Eigen::Transform3d::Identity());

    //std::cout << "Drawing " << m_AllPoints.size() << " points" << std::endl;
    for(unsigned i=0; i<m_AllPoints.size(); i++)
    {
        glColor4dv(colorvector);
        this->drawOnePoint( false, t, i );
    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
    glPopAttrib();
} 

void PointCloud::drawAllPoints(const Eigen::Transform3d& t, double* color)
{
    double colorvector[4];

    if (color == NULL)
    {
        colorvector[0] = 1.0;       //red
        colorvector[1] = 0.0;       //green
        colorvector[2] = 0.0;       //blue
        colorvector[3] = 1.0;       //transparency
    }
    else
    {
        colorvector[0] = color[0];       //red
        colorvector[1] = color[1];       //green
        colorvector[2] = color[2];       //blue
        colorvector[3] = color[3];       //transparency
    }

    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);

    //std::cout << "Drawing " << m_AllPoints.size() << " points" << std::endl;
    for(unsigned i=0; i<m_AllPoints.size(); i++)
    {
        glColor4dv(colorvector);
        this->drawOnePoint(true,t,i);
    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
    glPopAttrib();
}

void PointCloud::drawOnePoint(bool withTransform, const Eigen::Transform3d& t, int i )
{
    double _corner[3];

    Eigen::Vector3d point = m_AllPoints[i];

    if (withTransform == true)
    {
        point = t * point;
    }

    _corner[0] = point[0] - m_CubeSize[0] / 2;
    _corner[1] = point[1] - m_CubeSize[1] / 2;
    _corner[2] = point[2] - m_CubeSize[2] / 2;

    double _v0[3]; double _v1[3]; double _v2[3]; double _v3[3];
    double _v4[3]; double _v5[3]; double _v6[3]; double _v7[3];

    //  _v0 = new double[3]; _v1 = new double[3]; _v2 = new double[3]; _v3 = new double[3];
    //  _v4 = new double[3]; _v5 = new double[3]; _v6 = new double[3]; _v7 = new double[3];

    _v0[0] = _corner[0] + m_CubeSize[0];
    _v0[1] = _corner[1] + m_CubeSize[1];
    _v0[2] = _corner[2] + m_CubeSize[2];

    _v1[0] = _corner[0] ;
    _v1[1] = _corner[1] + m_CubeSize[1];
    _v1[2] = _corner[2] + m_CubeSize[2];

    _v2[0] = _corner[0] ;
    _v2[1] = _corner[1] ;
    _v2[2] = _corner[2] + m_CubeSize[2];

    _v3[0] = _corner[0] + m_CubeSize[0];
    _v3[1] = _corner[1] ;
    _v3[2] = _corner[2] + m_CubeSize[2];

    _v4[0] = _corner[0] + m_CubeSize[0];
    _v4[1] = _corner[1] ;
    _v4[2] = _corner[2] ;

    _v5[0] = _corner[0] + m_CubeSize[0];
    _v5[1] = _corner[1] + m_CubeSize[1];
    _v5[2] = _corner[2] ;

    _v6[0] = _corner[0] ;
    _v6[1] = _corner[1] + m_CubeSize[1];
    _v6[2] = _corner[2] ;

    _v7[0] = _corner[0] ;
    _v7[1] = _corner[1] ;
    _v7[2] = _corner[2] ;

    glNormal3f(0,0,1);
    glVertex3dv(_v0);    // front face
    glVertex3dv(_v1);
    glVertex3dv(_v2);
    glVertex3dv(_v3);

    glNormal3f(1,0,0);
    glVertex3dv(_v0);    // right face
    glVertex3dv(_v3);
    glVertex3dv(_v4);
    glVertex3dv(_v5);

    glNormal3f(0,1,0);
    glVertex3dv(_v0);    // up face
    glVertex3dv(_v5);
    glVertex3dv(_v6);
    glVertex3dv(_v1);

    glNormal3f(-1,0,0);
    glVertex3dv(_v1);
    glVertex3dv(_v6);
    glVertex3dv(_v7);
    glVertex3dv(_v2);

    glNormal3f(0,0,-1);
    glVertex3dv(_v4);
    glVertex3dv(_v7);
    glVertex3dv(_v6);
    glVertex3dv(_v5);

    glNormal3f(0,-1,0);
    glVertex3dv(_v7);
    glVertex3dv(_v4);
    glVertex3dv(_v3);
    glVertex3dv(_v2);
}

#include "ThreeDPoints.hpp"
#include "Graphic-pkg.h"

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

ThreeDPoints* PointsToDraw=NULL;

ThreeDPoints::ThreeDPoints()
{
    m_CubeSize[0] = 0.01;
    m_CubeSize[1] = 0.01;
    m_CubeSize[2] = 0.01;
}

void ThreeDPoints::push_back(Vector3d point)
{
    m_AllPoints.push_back(point);
}

void ThreeDPoints::drawAllPoints()
{
    double colorvector[4];

    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.0;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 1.0;       //transparency

    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);

    for(unsigned i=0; i<m_AllPoints.size(); i++)
    {
        glColor4dv(colorvector);
        this->drawOnePoint(i);
    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
    glPopAttrib();
}

void ThreeDPoints::drawOnePoint(int i)
{
    double* _corner = new double[3];

    _corner[0] = m_AllPoints.at(i)[0];
    _corner[1] = m_AllPoints.at(i)[1];
    _corner[2] = m_AllPoints.at(i)[2];

    double* _v0; double* _v1; double* _v2; double* _v3;
    double* _v4; double* _v5; double* _v6; double* _v7;

    _v0 = new double[3]; _v1 = new double[3]; _v2 = new double[3]; _v3 = new double[3];
    _v4 = new double[3]; _v5 = new double[3]; _v6 = new double[3]; _v7 = new double[3];

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

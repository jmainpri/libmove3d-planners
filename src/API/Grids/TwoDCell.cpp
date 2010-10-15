#include "TwoDCell.hpp"

#include "TwoDGrid.hpp"
#include <iostream>
#include "Graphic-pkg.h"

#include <Eigen/Array>

using namespace std;
using namespace API;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
TwoDCell::TwoDCell()
{
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
TwoDCell::TwoDCell(int i, Vector2d corner, TwoDGrid* grid) :
        _index(i),
        _corner(corner),
        _grid(grid)
{
    //    cout << " ThreeDCell " << i << ", Cornner = "<<  _corner.at(0) <<  _corner.at(1) <<  _corner.at(2) << ", Grid = " << _grid << endl;
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
TwoDCell::~TwoDCell()
{

}

/*!
 * \brief Function is inside cell
 *
 * \param 3D point vector
 */
bool TwoDCell::isInsideCell(Vector2d point)
{
  return false;
}


/*!
 * \brief Function to get the center of the cell
 *
 * \param 3D point vector
 */
Vector2d TwoDCell::getCenter()
{
    //    cout << "getCenter()" << endl;

    Vector2d dimentions = _grid->getCellSize();

    for(int i=0;i< dimentions.size(); i++)
    {
        dimentions[i] = dimentions[i]/2 + _corner[i];
    }

    return dimentions;
}

/**
  * Random Point In ThreeDCell
  */
Vector2d TwoDCell::getRandomPoint()
{
    Vector2d X = Vector2d::Random();

    Matrix2d A = Matrix2d::Zero();

    A(0,0) = _grid->getCellSize()[0]/2;
    A(1,1) = _grid->getCellSize()[1]/2;
//    A(2,2) = _grid->getCellSize()[2]/2;

    X = A*X;

    Vector2d B = this->getCenter();

//    cout << "X + B =" << endl << X + B << endl;

    return  X + B;
}

/**
  * Gets the cell size
  */
Vector2d TwoDCell::getCellSize()
{
     return _grid->getCellSize();
}

void TwoDCell::draw()
{
    double* _v0; double* _v1; double* _v2; double* _v3;
    double* _v4; double* _v5; double* _v6; double* _v7;

    _v0 = new double[3]; _v1 = new double[3]; _v2 = new double[3]; _v3 = new double[3];
    _v4 = new double[3]; _v5 = new double[3]; _v6 = new double[3]; _v7 = new double[3];

    _v0[0] = _corner[0] + _grid->getCellSize()[0];
    _v0[1] = _corner[1] + _grid->getCellSize()[1];
    _v0[2] = _corner[2] + _grid->getCellSize()[2];

    _v1[0] = _corner[0] ;
    _v1[1] = _corner[1] + _grid->getCellSize()[1];
    _v1[2] = _corner[2] + _grid->getCellSize()[2];

    _v2[0] = _corner[0] ;
    _v2[1] = _corner[1] ;
    _v2[2] = _corner[2] + _grid->getCellSize()[2];

    _v3[0] = _corner[0] + _grid->getCellSize()[0];
    _v3[1] = _corner[1] ;
    _v3[2] = _corner[2] + _grid->getCellSize()[2];

    _v4[0] = _corner[0] + _grid->getCellSize()[0];
    _v4[1] = _corner[1] ;
    _v4[2] = _corner[2] ;

    _v5[0] = _corner[0] + _grid->getCellSize()[0];
    _v5[1] = _corner[1] + _grid->getCellSize()[1];
    _v5[2] = _corner[2] ;

    _v6[0] = _corner[0] ;
    _v6[1] = _corner[1] + _grid->getCellSize()[1];
    _v6[2] = _corner[2] ;

    _v7[0] = _corner[0] ;
    _v7[1] = _corner[1] ;
    _v7[2] = _corner[2] ;

    double colorvector[4];

    colorvector[0] = 0.0;       //red
    colorvector[1] = 0.0;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.2;       //transparency

    glColor4dv(colorvector);

//    cout << "Drawing cell" << endl;
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);

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

    glEnd();
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    glPopAttrib();
}

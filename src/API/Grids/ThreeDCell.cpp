#include "ThreeDCell.hpp"
#include "ThreeDGrid.hpp"

#include <iostream>
#include "Graphic-pkg.h"
#include "P3d-pkg.h"

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

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
ThreeDCell::ThreeDCell()
{
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
ThreeDCell::ThreeDCell(int i, ThreeDGrid* grid) :
	_index(i),
	_grid(grid)
{
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
ThreeDCell::ThreeDCell(int i, Vector3d corner, ThreeDGrid* grid) :
        _index(i),
        _corner(corner),
        _grid(grid)
{
    //cout << " ThreeDCell " << i << ", Cornner = "<<  _corner[0] <<  _corner[1] <<  _corner[2] << ", Grid = " << _grid << endl;
}

/*!
 * \brief Constructor of cell
 *
 * \param integer index
 */
ThreeDCell::~ThreeDCell()
{

}


/*!
 * \brief Function to get the center of the cell
 *
 * \param 3D point vector
 */
Vector3d ThreeDCell::getCenter()
{
//    cout << "_grid = " << _grid << endl;

    Vector3d dimentions = _grid->getCellSize();
//	
//	cout << "dimentions = " << endl << dimentions << endl;

    for(int i=0;i< dimentions.size(); i++)
    {
        dimentions[i] = dimentions[i]/2 + _corner[i];
    }

    return dimentions;
}

/**
  * Random Point In ThreeDCell
  */
Vector3d ThreeDCell::getRandomPoint()
{
    Vector3d X = Vector3d::Random();

    Matrix3d A = Matrix3d::Zero();

    A(0,0) = _grid->getCellSize()[0]/2;
    A(1,1) = _grid->getCellSize()[1]/2;
    A(2,2) = _grid->getCellSize()[2]/2;
    
    X = A*X;

    Vector3d B = this->getCenter();
    
//    cout << "X + B =" << endl << X + B << endl;

    return  X + B;
}

/**
  * Gets the cell size
  */
Vector3d ThreeDCell::getCellSize()
{
     return _grid->getCellSize();
}


//     V1 -- V0
//    /      / |     Z  Y
//   V2 -- V3 V5     |/
//   |      | /       -- X
//   V7 -- V4
//
void ThreeDCell::getVerticies(vector<Eigen::Vector3d>& verticies)
{
  const Eigen::Vector3d& cell_size_origin = _grid->getCellSize();
  Eigen::Vector3d cell_size = cell_size_origin / 2;
  
  verticies[0][0] = _corner[0] + cell_size[0];
  verticies[0][1] = _corner[1] + cell_size[1];
  verticies[0][2] = _corner[2] + cell_size[2];
  
  verticies[1][0] = _corner[0] ;
  verticies[1][1] = _corner[1] + cell_size[1];
  verticies[1][2] = _corner[2] + cell_size[2];
  
  verticies[2][0] = _corner[0] ;
  verticies[2][1] = _corner[1] ;
  verticies[2][2] = _corner[2] + cell_size[2];
  
  verticies[3][0] = _corner[0] + cell_size[0];
  verticies[3][1] = _corner[1] ;
  verticies[3][2] = _corner[2] + cell_size[2];
  
  verticies[4][0] = _corner[0] + cell_size[0];
  verticies[4][1] = _corner[1] ;
  verticies[4][2] = _corner[2] ;
  
  verticies[5][0] = _corner[0] + cell_size[0];
  verticies[5][1] = _corner[1] + cell_size[1];
  verticies[5][2] = _corner[2] ;
  
  verticies[6][0] = _corner[0] ;
  verticies[6][1] = _corner[1] + cell_size[1];
  verticies[6][2] = _corner[2] ;
  
  verticies[7][0] = _corner[0] ;
  verticies[7][1] = _corner[1] ;
  verticies[7][2] = _corner[2] ;
}

void ThreeDCell::draw()
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

//    double colorvector[4];

//    colorvector[0] = 0.0;       //red
//    colorvector[1] = 0.0;       //green
//    colorvector[2] = 0.0;       //blue
//    colorvector[3] = 0.2;       //transparency

//    glColor4dv(colorvector);

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

//! @value should be between min and max
//! @min the min value that value can take
//! @max the max value that value can take
//! @inverse set the color gradient to be the inverse of 0 => blue, 1 => red
void ThreeDCell::drawColorGradient( double value, double min, double max, bool inverse )
{
  double colorvector[4];
  double alpha = (value-min) / (max-min);
  double diagonal = getCellSize().minCoeff();
  Vector3d center = getCenter();
  
  if (alpha < 0.0) 
  { alpha = 0.0; }
  
  if (alpha > 1.0) 
  { alpha = 1.0; }
  
  if ( inverse ) 
  { alpha = 1 - alpha; }
	
  colorvector[3] = 0.05; //transparency
  GroundColorMixGreenToRed( colorvector, alpha );
  g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/3, 10);
  glColor4dv(colorvector);
}

bool ThreeDCell::writeToXml(xmlNodePtr _XmlCellNode_)
{
	stringstream ss;
	string str; 
	
	str.clear(); ss << getCost(); ss >> str; ss.clear();
	xmlNewProp (_XmlCellNode_, xmlCharStrdup("Cost"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << _corner[0] ; ss >> str; ss.clear();
	xmlNewProp (_XmlCellNode_, xmlCharStrdup("CornerX"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << _corner[1]; ss >> str; ss.clear();
	xmlNewProp (_XmlCellNode_, xmlCharStrdup("CornerY"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << _corner[2]; ss >> str; ss.clear();
	xmlNewProp (_XmlCellNode_, xmlCharStrdup("CornerZ"), xmlCharStrdup(str.c_str()));
	
	return true;
}

bool ThreeDCell::readCellFromXml(xmlNodePtr cur)
{
	return true;
}



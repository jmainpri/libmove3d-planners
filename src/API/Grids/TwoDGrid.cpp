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
#include "TwoDGrid.hpp"

#include <iostream>
#include "Graphic-pkg.h"

#define EIGEN2_SUPPORT
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <Eigen/Array>

#include <iostream>

using namespace std;
using namespace Move3D;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;


TwoDGrid* API_activeRobotGrid = NULL;

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


/*!
 * \brief Constructor
 *
 * \param X number of cells
 * \param Y number of cells
 */
TwoDGrid::TwoDGrid()
{

}

/*!
 * \brief Destructor
 */
TwoDGrid::~TwoDGrid()
{
    //    for(unsigned int i=0;i<_cells.size();i++)
    //    {
    //        delete _cells.at(i);
    //    }
}

/*!
 * \brief Initializes the grid with a pace
 *
 * \param vector int size (number of cells in X, Y)
 * \param vector envSize XMin Xmax YMin YMax
 */
TwoDGrid::TwoDGrid( Vector2i numCell, vector<double> envSize )

{
    envSize.resize(4);
    setEnvSizeAndNumCell( numCell[0], numCell[1], envSize );
}


/*!
 * \brief Initializes the grid with a certain pace
 *
 * \param double pace : sizes of the squared cells IMPORTANT Cells are squared
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
TwoDGrid::TwoDGrid( double samplingRate, vector<double> envSize )
{
    envSize.resize(4);

    for(unsigned int i= 0; i< envSize.size() ; i++)
    {
        cout << envSize.at(i) << " ";
    }
    cout << endl;

    if(((int)samplingRate) != 0 )
    {
        if( ( ((int) (envSize.at(1) - envSize.at(0))) % (int)samplingRate ) != 0 )
        {
            cout << "TwoDGrid Warning : not good X disctretization " << endl;
        }

        if( ( ((int) (envSize.at(3) - envSize.at(2))) % (int)samplingRate ) != 0 )
        {
            cout << "TwoDGrid Warning : not good Y disctretization " << endl;
        }
    }

    //    _cellSize.push_back( (envSize.at(1) - envSize.at(0))/pace );
    //    _cellSize.push_back( (envSize.at(3) - envSize.at(2))/pace );

    _cellSize[0] =  samplingRate ;
    _cellSize[1] =  samplingRate ;

    _nbCellsX =  (envSize.at(1) - envSize.at(0)) / samplingRate ;
    _nbCellsY =  (envSize.at(3) - envSize.at(2)) / samplingRate ;

    cout << " _nbCellsX = " << _nbCellsX << endl;
    cout << " _nbCellsY = " << _nbCellsY << endl;

    _originCorner[0] = envSize.at(0);
    _originCorner[1] = envSize.at(2);

    cout << "_originCorner[0] = " << _originCorner[0] <<  endl;
    cout << "_originCorner[1] = " << _originCorner[1] <<  endl;

}

void TwoDGrid::setEnvSizeAndNumCell( int x, int y, std::vector<double> envSize  )
{
    _nbCellsX = x;
    _nbCellsY = y;

    _cellSize[0] = (envSize.at(1) - envSize.at(0)) / _nbCellsX ;
    _cellSize[1] = (envSize.at(3) - envSize.at(2)) / _nbCellsY ;

    _originCorner[0] = envSize.at(0);
    _originCorner[0] = envSize.at(2);

    //    cout << "_originCorner[0] = " << _originCorner.at(0) <<  endl;
    //    cout << "_originCorner[1] = " << _originCorner.at(1) <<  endl;
}

/*!
 * \brief Creates All Cells
 *
 * \param vector envSize XMin Xmax YMin YMax
 */
void TwoDGrid::createAllCells()
{
    unsigned int nbCells = _nbCellsX * _nbCellsY ;

    _cells.resize(nbCells);

    unsigned int x=0;
    unsigned int y=0;

    for(unsigned int i = 0; i<nbCells; i++)
    {
        // cout << "("<< x << "," << y << ")" << endl;

        TwoDCell* ptrCell = createNewCell(i,x,y);
        _cells[i] = ptrCell;

        x++;
        if( x >= _nbCellsX )
        {
            y++;
            x=0;
            if( y >= _nbCellsY )
            {
                //cout << "TwoDGrid : Error Size of TwoDGrid " << endl;
                return;
            }
        }
    }
    //    cout << "Finished"<< endl;
}

/*!
 * \brief Get Cell
 *
 * \param index
 */
TwoDCell* TwoDGrid::getCell(const Vector2i& coord)
{
    return dynamic_cast<TwoDCell*>( _cells[ coord[0] + coord[1]*_nbCellsX ] );
}



/*!
 * \brief Retruns the Cell at (x,y)
 *
 * \param integers x, y
 */
TwoDCell* TwoDGrid::getCell(int x, int y)
{
    Vector2i  coord;
    coord[0] = x;
    coord[1] = y;

    return getCell(coord);
}

/*!
 * \brief Get Cell in 3D ThreeDGrid
 *
 * \param index
 */
TwoDCell* TwoDGrid::getCell(Vector2d point)
{
    Vector2i  coord;
    coord[0] = (int)floor((abs(point[0]-_originCorner[0]))/_cellSize[0]);
    coord[1] = (int)floor((abs(point[1]-_originCorner[1]))/_cellSize[1]);

    //    cout << "( "<<x<<" , "<<y<<" ) "<< endl;

    if( !isCellCoordInGrid(coord) )
    {
        cout << "TwoDGrid::OutBounds" << endl;
        return 0x0;
    }

    return getCell(coord);
}

/*!
 * \brief Get Cell in 3D ThreeDGrid
 *
 * \param index
 */
TwoDCell* TwoDGrid::getCell(double* pos)
{
    Vector2i  coord;
    coord[0] = (int)((pos[0]-_originCorner[0])/_cellSize[0]);
    coord[1] = (int)((pos[1]-_originCorner[1])/_cellSize[1]);

    //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;

    if( !isCellCoordInGrid(coord) )
    {
        cout << "ThreeDGrid::OutBounds" << endl;
        return 0x0;
    }

    return getCell(coord);
}

/*!
 * \brief Get Cell
 *
 * \param index
 */
TwoDCell* TwoDGrid::getCell(unsigned int index)
{
    return dynamic_cast<TwoDCell*>(_cells[index]);
}

/**
 * \brief Is a Coord inside the Grid (used to debug)
 * \param index vector
 */
bool TwoDGrid::isCellCoordInGrid(const Vector2i& coord)
{
    return !( coord[0] >=((int)_nbCellsX) ||  coord[1]>= ((int)_nbCellsY) || coord[0] <0 || coord[1] <0 );
}

/*!
 * \brief Get place in grid
 *
 * \param index
 */
Vector2i TwoDGrid::getCellCoord(TwoDCell* ptrCell)
{
    Vector2i coord;

    int i = ptrCell->getIndex();

    coord[0] = (i/1) % 2 - 1 ; // x
    coord[1] = (i/2) % 2 - 1 ; // y

    return coord;
}


/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
Move3D::TwoDCell* TwoDGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    if (index == 0)
    {
        return new TwoDCell( 0, _originCorner , this );
    }
    TwoDCell* newCell = new TwoDCell( index, computeCellCorner(x,y) , this );
    //Vector2d corner = newCell->getCorner();
    //    cout << " = (" << corner[0] <<"," << corner[1] << ")" << endl;
    return newCell;
}

/*!
 * \brief Computes the corner of a cell
 *
 * \param integer index
 */
Vector2d TwoDGrid::computeCellCorner(int x, int y )
{
    Vector2d corner;

    corner[0] = _originCorner[0] + x*(_cellSize[0]*1.0);
    corner[1] = _originCorner[1] + y*(_cellSize[1]*1.0);

    //    cout << " = (" << x <<"," << y << "," << z << ")" << endl;
    //    cout << " = (" << corner[0] <<"," << corner[1] << ")" << endl;

    return corner;
}


/*!
 * \brief Get Number Of Cells
 */
int TwoDGrid::getNumberOfCells()
{
    return _cells.size();
}


/*!
 * \brief Get Neighboor Cell
 */
TwoDCell* TwoDGrid::getNeighbour( const Vector2i& pos, int i)
{
    if( i<0 || i>8 )
    {
        return 0x0;
    }
    else
    {
        if(i>=4) i++;

        int dx =  (i/1) % 3 - 1 ;
        int dy =  (i/3) % 3 - 1 ;

        //    cout << "( "<<dx<<" , "<<dy<<" ) "<< endl;

        Vector2i coord;

        coord[0] = pos[0] + dx ;
        coord[1] = pos[1] + dy ;

        if( ! this->isCellCoordInGrid(coord)){
            return 0x0;
        }
        else {
            return getCell(coord);
        }
    }
}

/**
 * Retrive the X Y Z coordinate of the cell from its index
 */
Vector2d TwoDGrid::getCoordinates(TwoDCell* cell)
{
    Vector2d coordinates;
    int index = cell->getIndex();
    int sizeXY = _nbCellsX * _nbCellsY;
    coordinates[2] = floor(index / sizeXY);
    coordinates[1] = floor((index - coordinates[2]*sizeXY) / _nbCellsX);
    //  coordinates[0] = floor(index - coordinates[2]*sizeXY - coordinates[1] * _nbCellsX);
    return coordinates;
}

void TwoDGrid::draw()
{
//    double colorvector[4];

//    colorvector[0] = 1.0;       //red
//    colorvector[1] = 0.5;       //green
//    colorvector[2] = 0.0;       //blue
//    colorvector[3] = 0.05;       //transparence

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);

    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);

    double depth = 0.20;

    cout << "Drawing 2D Grid"  << endl;


    //    int nbCells = this->getNumberOfCells();

    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {

            if ((x+y)&0x00000001) //modulo 2
                glColor3f(1.0f,1.0f,1.0f); //white
            else
                glColor3f(0.0f,0.0f,0.0f); //black

            Vector2d center = this->getCell(x,y)->getCenter();

            glVertex3d( (double)(center[0] - _cellSize[0]/2) , (double)(center[1] - _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] + _cellSize[0]/2) , (double)(center[1] - _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] + _cellSize[0]/2) , (double)(center[1] + _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] - _cellSize[0]/2) , (double)(center[1] + _cellSize[1]/2), depth );
        }
    }


    //    for(int i=0; i<nbCells; i++)
    //    {
    //        TwoDCell* cell = static_cast<TwoDCell*>(getCell(i));
    //        glColor4dv(colorvector);
    //        cell->draw();
    //    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
}

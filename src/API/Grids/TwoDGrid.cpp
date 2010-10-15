#include "TwoDGrid.hpp"

#include "Graphic-pkg.h"

#include <iostream>

using namespace std;
using namespace API;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

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
    for(unsigned int i=0;i<_cells.size();i++)
    {
        delete _cells.at(i);
    }
}

/*!
 * \brief Initializes the grid with a pace
 *
 * \param vector int size (number of cells in X, Y)
 * \param vector envSize XMin Xmax YMin YMax
 */
TwoDGrid::TwoDGrid( Vector2i size, vector<double> envSize )

{
    _nbCellsX = size[0];
    _nbCellsY = size[1];

    _cellSize[0] = (envSize.at(1) - envSize.at(0)) / _nbCellsX ;
    _cellSize[1] = (envSize.at(3) - envSize.at(2)) / _nbCellsY ;

    _originCorner[0] = envSize.at(0);
    _originCorner[0] = envSize.at(2);

    //    cout << "_originCorner[0] = " << _originCorner.at(0) <<  endl;
    //    cout << "_originCorner[1] = " << _originCorner.at(1) <<  endl;
}


/*!
 * \brief Initializes the grid with a certain pace
 *
 * \param double pace : sizes of the squared cells IMPORTANT Cells are squared
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
TwoDGrid::TwoDGrid( double samplingRate, vector<double> envSize )
{
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

/*!
 * \brief Creates All Cells
 *
 * \param vector envSize XMin Xmax YMin YMax
 */
void TwoDGrid::createAllCells()
{

    unsigned int _nbCells = _nbCellsX * _nbCellsY ;

    _cells.resize(_nbCells);

    unsigned int x=0;
    unsigned int y=0;

    for(unsigned int i = 0; i < _nbCells; i++)
    {
        //        cout << "("<< x << "," << y << ")" << endl;

        TwoDCell* ptrCell = createNewCell(i,x,y);
        _cells[i] = ptrCell;

        x++;
        if( x >= _nbCellsX )
        {
            y++;
            x=0;
            if( y >= _nbCellsY )
            {
                cout << "ThreeDGrid : Error Size of ThreeDGrid " << endl;
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
        cout << "TwoDGrid:: OutBands " << endl;
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
        cout << "ThreeDGrid:: OutBands " << endl;
        return 0x0;
    }

    return getCell(coord);
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
API::TwoDCell* TwoDGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    if (index == 0)
    {
        return new TwoDCell( 0, _originCorner , this );
    }
    TwoDCell* newCell = new TwoDCell( index, computeCellCorner(x,y) , this );
    Vector2d corner = newCell->getCorner();
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

    corner[0] = _originCorner[0] + x*_cellSize[0];
    corner[1] = _originCorner[1] + y*_cellSize[1];

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

//        cout << "( "<<dx<<" , "<<dy<<" ) "<< endl;

        Vector2i coord;

        coord[0] = pos[0] + dx ;
        coord[1] = pos[1] + dy ;

        if( ! this->isCellCoordInGrid(coord) )
        {
            //            cout << "( "<<x<<" , "<<y<< ) "<< endl;
            //            cout << "OutBands" << endl;
            return 0x0;
        }
        else
        {
            //            cout << "( "<<x<<" , "<<y<< ) "<< endl;
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
    double colorvector[4];

    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency


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

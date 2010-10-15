#include "NDCell.hpp"
#include "NDGrid.hpp"

#include <iostream>
#include "Graphic-pkg.h"

#include <iostream>
//#include <Eigen/Array>

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
 * \param Z number of cells
 */
template <int _nDimGrid_Dimension_ >
nDimGrid<_nDimGrid_Dimension_>::nDimGrid()
{
	
}

/*!
 * \brief Destructor
 */
template <int _nDimGrid_Dimension_ >
nDimGrid<_nDimGrid_Dimension_>::~nDimGrid()
{
	
}

/*!
 * \brief Initializes the grid with a pace
 *
 * \param vector int size (number of cells in X, Y, Z)
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
template <int _nDimGrid_Dimension_ >
nDimGrid<_nDimGrid_Dimension_>::nDimGrid( Matrix<    int, _nDimGrid_Dimension_ , 1 > size, std::vector<double> envSize )

{
	m_nbOfCell = size;
	
	for (unsigned int  i=0; i< envSize.size() ; i += 2) 
	{
		m_cellSize[i] = (envSize.at(i+1) - envSize.at(i)) / size[i] ;
		m_originCorner[i/2] = envSize.at(i);
	}
}


/*!
 * \brief Initializes the grid with a certain pace
 *
 * \param double pace : sizes of the squared cells IMPORTANT Cells are squared
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
template <int _nDimGrid_Dimension_ >
nDimGrid<_nDimGrid_Dimension_>::nDimGrid( double samplingRate, vector<double> envSize )
{
    for(unsigned int i= 0; i< envSize.size() ; i++)
    {
        cout << envSize.at(i) << " ";
    }
    cout << endl;
	
    if(((int)samplingRate) != 0 )
    {
		for (unsigned int i=0; i< envSize.size() ; i += 2) 
		{
			if( ( ((int) (envSize.at(i+1) - envSize.at(i))) % (int)samplingRate ) != 0 )
			{
				cout << "nDimGrid Warning : not good  disctretization on dimention :" << i << endl;
			}
		}
	}
	
	for (unsigned int  i=0; i< (envSize.size()/2); i++) 
	{
		m_cellSize[i] =  samplingRate ;
	}
	
	for (unsigned int  i=0; i< envSize.size() ; i += 2) 
	{
		m_nbOfCell[i/2] =  (envSize.at(i+1) - envSize.at(i)) / samplingRate ;
		m_originCorner[i/2] = envSize.at(i);
	}
	
    cout << " m_nbOfCellX = " << m_nbOfCell[0] << endl;
    cout << " m_nbOfCellY = " << m_nbOfCell[1] << endl;
	
	if( envSize.size() > 4 )
	{
		cout << " _nbCellsZ = " << m_nbOfCell[2] << endl;
	}
	
    cout << "m_originCorner[0] = " << m_originCorner[0] <<  endl;
    cout << "m_originCorner[1] = " << m_originCorner[1] <<  endl;
	
	if( envSize.size() > 4 )
	{
		cout << " m_originCorner[3] = " << m_nbOfCell[2] << endl;
	}
	
}

/*!
 * \brief Creates All Cells
 *
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
template <int _nDimGrid_Dimension_ >
void nDimGrid<_nDimGrid_Dimension_>::createAllCells()
{	
    int tot_nbCells = 1;
	
	for(int i=0;i<m_nbOfCell.size();i++)
	{
		tot_nbCells *= m_nbOfCell[i];
	}
	
    m_cells.resize(tot_nbCells);
	
	Matrix< int, _nDimGrid_Dimension_ , 1 > currentCellCoordinate = Matrix< int, _nDimGrid_Dimension_ , 1 >::Zero();
	
    for(int i = 0; i < tot_nbCells; i++)
    {
		//        cout << "("<< x << "," << y << "," << z << ")" << endl;
        nDimCell* ptrCell = createNewCell(i,currentCellCoordinate);
        m_cells[i] = ptrCell;
		
		int j=0;
		
		currentCellCoordinate[j]++;
		
        while((currentCellCoordinate[j] % m_nbOfCell[j]) == 0 )
        {
			j++;
			currentCellCoordinate[j+1]++;
			currentCellCoordinate[j] = 0;
			
			if ((currentCellCoordinate[j] % m_nbOfCell[j]) != 0) 
			{
				j = 0;
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
template <int _nDimGrid_Dimension_ >
nDimCell* nDimGrid<_nDimGrid_Dimension_>::getCell(unsigned int i)
{
    return m_cells[i];
}
/*!
 * \brief Retruns the Cell at (x,y,z)
 *
 * \param integers x, y, z
 */
template <int _nDimGrid_Dimension_ >
nDimCell* nDimGrid<_nDimGrid_Dimension_>::getCell( const Matrix<    int, _nDimGrid_Dimension_ , 1 > & coordinate)
{
	unsigned int j=0;
	unsigned int coeff=1;
	
	for (unsigned int i=0; i<m_nbOfCell.size(); i++) 
	{
		if(coordinate[i]<0 || coordinate[i] >= m_nbOfCell[i])
		{
			cout << "nDimGrid Error : out of bands in dimension "<< i << endl;
			return 0x0;
		}
		
		j += coeff*coordinate[i];
		coeff *= m_nbOfCell[i];
	}
	
    return getCell(j);
}

/*!
 * \brief Get Cell in 3D nDimGrid
 *
 * \param index
 */
template <int _nDimGrid_Dimension_ >
nDimCell* nDimGrid<_nDimGrid_Dimension_>::getCell(const Matrix<    double, _nDimGrid_Dimension_ , 1 > & point)
{
	Matrix< int, _nDimGrid_Dimension_ , 1 > coordinate;
	
	for (unsigned int i=0; i<m_nbOfCell.size(); i++) 
	{
		coordinate[i] = (unsigned int)floor((abs(point[i]-m_originCorner[i]))/m_cellSize[i]);
	}
	
    return getCell(coordinate);
}

/*!
 * \brief Get Cell in 3D nDimGrid
 *
 * \param index
 */
template <int _nDimGrid_Dimension_ >
nDimCell* nDimGrid<_nDimGrid_Dimension_>::getCell(double* point)
{
	Matrix< int, _nDimGrid_Dimension_ , 1 > coordinate;
	
	for (unsigned int i=0; i<m_nbOfCell.size(); i++) 
	{
		coordinate[i] = (unsigned int)floor((abs(point[i]-m_originCorner[i]))/m_cellSize[i]);
	}
	
	return getCell(coordinate);
}


/*!
 * \brief Get place in grid from index
 *
 * \param index
 */
template <int _nDimGrid_Dimension_ >
Matrix< int, _nDimGrid_Dimension_ , 1 > nDimGrid<_nDimGrid_Dimension_>::getCellCoord(unsigned int index)
{
	Matrix<    int, _nDimGrid_Dimension_ , 1 > coordinate;
	
	unsigned int coeff=1.0;
	
	for (unsigned int j=0; j<m_nbOfCell.size(); j++) 
	{
		coordinate[j] = (index/coeff) % (coeff*m_nbOfCell[j]) - 1 ;
		coeff *= m_nbOfCell[j];
	}
	
	return coordinate;
}


/*!
 * \brief Get place in grid
 *
 * \param index
 */
template <int _nDimGrid_Dimension_ >
Matrix< int, _nDimGrid_Dimension_ , 1 > nDimGrid<_nDimGrid_Dimension_>::getCellCoord(nDimCell* ptrCell)
{
	//return getCellCoord(ptrCell->getIndex());
	return 0x00;
}


/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
template <int _nDimGrid_Dimension_ >
nDimCell* nDimGrid<_nDimGrid_Dimension_>::createNewCell(int index, const Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate )
{
    if (index == 0)
    {
        return new nDimCell( 0, m_originCorner , this );
    }
    nDimCell* newCell = new nDimCell( index, computeCellCorner(coordinate) , this );
	// Matrix< double, _nDimGrid_Dimension_ , 1 > corner = newCell->getCorner();
	//    cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;
    return newCell;
}

/*!
 * \brief Computes the corner of a cell
 *
 * \param integer index
 */
template <int _nDimGrid_Dimension_ >
Matrix< double, _nDimGrid_Dimension_ , 1 > nDimGrid<_nDimGrid_Dimension_>::computeCellCorner( 
					const Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate)
{
    Matrix<    double, _nDimGrid_Dimension_ , 1 > corner;
	
	for (unsigned int i=0; i<m_nbOfCell.size(); i++) 
	{
		corner[i] = m_originCorner[i] + coordinate[i]*m_cellSize[i];
	}
//    corner[0] = m_originCorner[0] + x*m_cellSize[0];
//    corner[1] = m_originCorner[1] + y*m_cellSize[1];
//    corner[2] = m_originCorner[2] + z*m_cellSize[2];
	
	//    cout << " = (" << x <<"," << y << "," << z << ")" << endl;
	//    cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;
    return corner;
}


/*!
 * \brief Get Number Of Cells
 */
template <int _nDimGrid_Dimension_ >
unsigned int nDimGrid<_nDimGrid_Dimension_>::getNumberOfCells()
{
    return m_cells.size();
}


/*!
 * \brief Get Neighboor Cell
 */
template <int _nDimGrid_Dimension_ >
nDimCell* nDimGrid<_nDimGrid_Dimension_>::getNeighbour( const Matrix< int, _nDimGrid_Dimension_ , 1 >& coordinate, int ith_neigh)
{
	unsigned int totDirections=1;
	unsigned int dimension = m_nbOfCell.size();
	
	for (int i=0; i<dimension; i++) 
	{
		totDirections *= m_nbOfCell[i];
	}
	
	
    if( ith_neigh<0 || ith_neigh>(totDirections-1) )
    {
        return 0x0;
    }
    else
    {
        if(ith_neigh>=(totDirections/2)) ith_neigh++;
		
		unsigned int coeff=1;
		
		Matrix< int, _nDimGrid_Dimension_ , 1 >& NeighboorCoord;
		
		for (int i=0; i<dimension; i++) 
		{
			NeighboorCoord[i] = coordinate + ((ith_neigh/coeff) % 3 - 1);
			coeff *= 3;
		}
		
		return getCell(NeighboorCoord);
    }
}

/**
 * Retrive the X Y Z coordinate of the cell from its index
 */
//template <int _nDimGrid_Dimension_ >
//Matrix<    double, _nDimGrid_Dimension_ , 1 > nDimGrid<_nDimGrid_Dimension_>::getCoordinates(nDimCell* cell)
//{
//	Matrix<    double, _nDimGrid_Dimension_ , 1 > coordinates;
//	int index = cell->getIndex();
//	int sizeXY = _nbCellsX * _nbCellsY;
//	coordinates[2] = floor(index / sizeXY);
//	coordinates[1] = floor((index - coordinates[2]*sizeXY) / _nbCellsX);
//	coordinates[0] = floor(index - coordinates[2]*sizeXY - coordinates[1] * _nbCellsX);
//	return coordinates;
//}

/**
 * Draws the grid (function is virtual
 * Example for 2D and 3D 
 */
template <int _nDimGrid_Dimension_ >
void nDimGrid<_nDimGrid_Dimension_>::draw()
{
    double colorvector[4];
	
    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency
	
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT);	

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
	
    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);
	
    int nbCells = this->getNumberOfCells();
	
    for(int i=0; i<nbCells; i++)
    {
        nDimCell* cell = dynamic_cast<nDimCell*>( getCell(i) );
        glColor4dv(colorvector);
        cell->draw();
    }
	
    glEnd();
	
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
	
    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
    glPopAttrib();
}




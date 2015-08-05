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
#ifndef NDGRID_HPP
#define NDGRID_HPP

#include "API/Grids/NDCell.hpp"
#include <vector>
////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>


/**
 * Base class for a Grid
 */
namespace Move3D
{
	template <int _nDimGrid_Dimension_ > 
	class nDimGrid
	{
		
	public:
		/**
		 * Constructors
		 */
		nDimGrid();
		nDimGrid( Eigen::Matrix< int, _nDimGrid_Dimension_ , 1 > size, std::vector<double> envSize );
		nDimGrid( double samplingRate, std::vector<double> envSize );
		
		/**
		 * Destructor
		 */
		virtual ~nDimGrid();
		
		/**
		 * Creates all cells in
		 * the grid calling the function create new cell
		 */
		void createAllCells();
		
		/**
		 * Returns the dimension of on cell
		 */
		Eigen::Matrix< double, _nDimGrid_Dimension_ , 1 > getCellSize() { return m_cellSize; }
		
		/**
		 * Accessors to cells
		 */
		nDimCell* getCell(unsigned int i);
		nDimCell* getCell(const Eigen::Matrix<    int, _nDimGrid_Dimension_ , 1 > & coordinate);
		nDimCell* getCell(const Eigen::Matrix< double, _nDimGrid_Dimension_ , 1 > & pos);
		nDimCell* getCell(double* pos);
		
		
		/**
		 * Returns the coordinate of a cell in the grid
		 */
		Eigen::Matrix< int, _nDimGrid_Dimension_ , 1 > getCellCoord(unsigned int index);
		Eigen::Matrix< int, _nDimGrid_Dimension_ , 1 > getCellCoord(nDimCell* ptrCell);
		
		/**
		 * Returns the number of cell in the grid 
		 */
		unsigned int getNumberOfCells();
		
		/**
		 * Returns the neighbor cells
		 * of the cell at coordinate 'pos'
		 */
		nDimCell* getNeighbour( const Eigen::Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate, int i);
		
		/**
		 * Function to display
		 * the grid
		 */
		virtual void draw();
		
	protected:
		
		/**
		 * Allocates one cell
		 */
		virtual nDimCell* createNewCell(int index, const Eigen::Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate );
		
		/**
		 * Compute the cell's corner as the 
		 * center is stored inside the cell and the size 
		 * is stored in the Grid
		 */
		Eigen::Matrix< double, _nDimGrid_Dimension_ , 1 > computeCellCorner( const Eigen::Matrix< int, _nDimGrid_Dimension_ , 1 > & coordinate );
		
		
	private:
		/**
		 * All cells are stored
		 * in this array
		 */
		std::vector<nDimCell*> m_cells;
		
		/**
		 * Origin corner coordinate
		 */
		Eigen::Matrix< double, _nDimGrid_Dimension_ , 1 > m_originCorner;
		
		/**
		 * One cell dimension (Size)
		 */
		Eigen::Matrix< double, _nDimGrid_Dimension_ , 1 > m_cellSize;
		
		/**
		 * Number of Cell per dimension
		 */
		Eigen::Matrix<    int, _nDimGrid_Dimension_ , 1 > m_nbOfCell;
	};
	
};

#endif // NDGRID_HPP

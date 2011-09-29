#ifndef NDGRID_HPP
#define NDGRID_HPP

#include "API/Grids/NDCell.hpp"
#include <vector>
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>


/**
 * Base class for a Grid
 */
namespace API
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

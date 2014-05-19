#ifndef TWODGRID_HPP
#define TWODGRID_HPP

#include <vector>

#include <Eigen/Core>

#include "API/Grids/BaseGrid.hpp"

/*!
 @ingroup GRID

* \brief Base class for 2D grid based algorithms
*
* Deriving the Grid class and the Cell class permits to generates
* easier grid algorithm. The function createNewCell is virtual just reimplement
* this function in the new class as well as the constructors which allready
* call the base one.
*/
namespace Move3D
{

    class TwoDGrid;
    /**
      * @ingroup CPP_API
      * @defgroup GRID Grid over the WS
      */

    /**
      @ingroup GRID
      */
    class TwoDCell : public BaseCell
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TwoDCell();
        TwoDCell(int i, Eigen::Vector2d corner, TwoDGrid* grid);
        virtual ~TwoDCell();

        Eigen::Vector2d getCenter();
        Eigen::Vector2d getCorner() { return _corner; }
        Eigen::Vector2d getRandomPoint();
        Eigen::Vector2d getCellSize();

        int getIndex() { return _index; }

        virtual void draw();

        bool operator==( TwoDCell otherCell) { return ((otherCell._index) == (this->_index)); }

    protected:
        int _index;
        Eigen::Vector2d _corner;
        TwoDGrid* _grid;
    };

    class TwoDGrid : public BaseGrid
   {
   public:
    	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
       TwoDGrid();
       TwoDGrid( Eigen::Vector2i numCell, std::vector<double> envSize );
       TwoDGrid( double samplingRate, std::vector<double> envSize );

       void setEnvSizeAndNumCell( int x, int y, std::vector<double> envSize  );

       ~TwoDGrid();

       void createAllCells();

       Eigen::Vector2d getCellSize() { return _cellSize; }

       TwoDCell* getCell(const Eigen::Vector2i& cell);
       TwoDCell* getCell(int x, int y );
       TwoDCell* getCell(Eigen::Vector2d pos);
       TwoDCell* getCell(double* pos);
       TwoDCell* getCell(unsigned int index);

       bool isCellCoordInGrid(const Eigen::Vector2i& coord);

       Eigen::Vector2i getCellCoord(TwoDCell* ptrCell);
       int getNumberOfCells();
       TwoDCell* getNeighbour(const Eigen::Vector2i& pos, int i);
       Eigen::Vector2d getCoordinates(TwoDCell* cell);

       virtual void draw();

   protected:
       virtual TwoDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y );
       Eigen::Vector2d computeCellCorner(int x, int y );

       Eigen::Vector2d _originCorner;
       Eigen::Vector2d _cellSize;

       unsigned int _nbCellsX;
       unsigned int _nbCellsY;

   };
}

#endif // TWODGRID_HPP

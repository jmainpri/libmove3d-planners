#ifndef GRID_HPP
#define GRID_HPP

#include <vector>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

#include "API/Grids/ThreeDCell.hpp"
#include "API/Grids/BaseGrid.hpp"

/*!
 @ingroup GRID
 
 * \brief Base class for 3D grid based algorithms
 *
 * Deriving the Grid class and the Cell class permits to generates
 * easier grid (Voxel) algorithms. The function createNewCell is virtual just reimplement
 * this function in the new class as well as the constructors which allready
 * call the base one.
 */
namespace API
{
  class ThreeDGrid : public BaseGrid
  {
		
  public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		
    ThreeDGrid();
    ThreeDGrid( Eigen::Vector3i size,    std::vector<double> envSize );
    ThreeDGrid( double samplingRate,     std::vector<double> envSize );
		ThreeDGrid( const ThreeDGrid& grid );
    
    virtual ~ThreeDGrid();
    
    void createAllCells();
    
    Eigen::Vector3d getCellSize() { return _cellSize; }
		
		unsigned int getXNumberOfCells() const {return _nbCellsX;}
		unsigned int getYNumberOfCells() const {return _nbCellsY;}
		unsigned int getZNumberOfCells() const {return _nbCellsZ;}
    
    ThreeDCell* getCell(unsigned int x, unsigned int y, unsigned int z) const;
    ThreeDCell* getCell(Eigen::Vector3i cell)  const;
    ThreeDCell* getCell(const Eigen::Vector3d& pos)  const;
    ThreeDCell* getCell(double* pos)  const;
    
    Eigen::Vector3i getCellCoord(ThreeDCell* ptrCell) const;
    ThreeDCell* getNeighbour(const Eigen::Vector3i& pos, unsigned int i) const;
    Eigen::Vector3d getCoordinates(ThreeDCell* cell) const;
		
		unsigned int getXlineOfCell(unsigned int ith);
		unsigned int getYlineOfCell(unsigned int ith);
		unsigned int getZlineOfCell(unsigned int ith);
    
    virtual void draw();
		
		bool writeToXmlFile(std::string file);
		bool loadFromXmlFile(std::string file);
    
  protected:
    virtual ThreeDCell* createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z );
    Eigen::Vector3d computeCellCorner(unsigned int x, unsigned int y, unsigned int z);
    
    Eigen::Vector3d _originCorner;
    Eigen::Vector3d _cellSize;
    
    unsigned int _nbCellsX;
    unsigned int _nbCellsY;
    unsigned int _nbCellsZ;
  };
}

#endif // GRID_HPP

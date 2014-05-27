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
#ifndef GRID_HPP
#define GRID_HPP

#include <vector>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/Geometry>

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
namespace Move3D
{

class ThreeDGrid;


class ThreeDCell : public BaseCell
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ThreeDCell();
    ThreeDCell(int i, ThreeDGrid* grid);
    ThreeDCell(int i, Eigen::Vector3d corner, ThreeDGrid* grid);

    virtual ~ThreeDCell();

    virtual double getCost() { return 0; }

    void getVerticies(std::vector<Eigen::Vector3d>& verticies);

    Eigen::Vector3d getCenter();
    Eigen::Vector3d getCorner() { return _corner; }
    Eigen::Vector3d getRandomPoint();
    Eigen::Vector3d getCellSize();

    int getIndex() { return _index; }

    void setCorner(const Eigen::Vector3d& corner) { _corner = corner; }
    void setGrid( ThreeDGrid* grid ) { _grid = grid; }

    virtual void draw();
    void drawColorGradient( double value, double min, double max , bool inverse = false );
    void drawColorGradient( double value, double min, double max , const Eigen::Transform3d& t, bool inverse = false );

    virtual bool writeToXml(xmlNodePtr cur);
    virtual bool readCellFromXml(xmlNodePtr cur);

    bool operator==( ThreeDCell otherCell) { return ((otherCell._index) == (this->_index)); }

protected:

    int _index;
    Eigen::Vector3d _corner;
    ThreeDGrid* _grid;
  };

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
    
    const Eigen::Vector3d& getCellSize() { return _cellSize; }

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

#ifndef CELL_HPP
#define CELL_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include "API/Grids/BaseCell.hpp"

#include <Eigen/Core>
#include <libxml/parser.h>
#include <vector>

/**
 * @ingroup CPP_API
 * @defgroup GRID Grid over the WS
 */

/**
 @ingroup GRID
 */
namespace API
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
		
		bool writeToXml(xmlNodePtr cur);
		bool readCellFromXml(xmlNodePtr cur);
    
    bool operator==( ThreeDCell otherCell) { return ((otherCell._index) == (this->_index)); }
    
  protected:
    int _index;
    Eigen::Vector3d _corner;
    ThreeDGrid* _grid;
  };
  
}
#endif // CELL_HPP

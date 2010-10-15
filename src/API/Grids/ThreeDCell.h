#ifndef CELL_HPP
#define CELL_HPP

#include <Eigen/Core>

#include "BaseCell.hpp"

#include <libxml/parser.h>

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
		
		virtual double getCost() { return 0; };

        bool isInsideCell(Eigen::Vector3d point);

        Eigen::Vector3d getCenter();
        Eigen::Vector3d getCorner() { return _corner; }
        Eigen::Vector3d getRandomPoint();
        Eigen::Vector3d getCellSize();

        int getIndex() { return _index; }

		void setCorner(const Eigen::Vector3d& corner) { _corner = corner; }
		void setGrid( ThreeDGrid* grid ) { _grid = grid; }
		
        virtual void draw();
		
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

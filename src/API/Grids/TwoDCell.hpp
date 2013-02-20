#ifndef TWODCELL_HPP
#define TWODCELL_HPP

#include <Eigen/Core>

#include "API/Grids/BaseCell.hpp"

namespace API
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

}

#endif // TWODCELL_HPP

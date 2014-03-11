#ifndef GRIDTOGRAPH_H
#define GRIDTOGRAPH_H

#include "API/Grids/ThreeDGrid.hpp"

#include "API/Device/robot.hpp"
#include "API/Roadmap/graph.hpp"

namespace Move3D
{

/**
  @ingroup GRID
  */
class GridToGraph : public Move3D::ThreeDGrid
{
public:
    GridToGraph();
    GridToGraph( Eigen::Vector3i size );
    GridToGraph( double pace, std::vector<double> envSize );

    Move3D::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );

    void putGridInGraph();

private:
    Robot* _Robot;
    Graph* _Graph;
};

}

#endif // GRIDTOGRAPH_H

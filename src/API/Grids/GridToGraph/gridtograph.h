#ifndef GRIDTOGRAPH_H
#define GRIDTOGRAPH_H

#include "ThreeDGrid.h"

#include "Device/robot.hpp"
#include "Roadmap/graph.hpp"

/**
  @ingroup GRID
  */
class GridToGraph : public API::ThreeDGrid
{
public:
    GridToGraph();
    GridToGraph( Eigen::Vector3i size );
    GridToGraph( double pace, std::vector<double> envSize );

    API::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );

    void putGridInGraph();

private:
    Robot* _Robot;
    Graph* _Graph;
};

#endif // GRIDTOGRAPH_H

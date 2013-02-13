#ifndef HRICS_WORKSPACEOCCUPANCY_HPP
#define HRICS_WORKSPACEOCCUPANCY_HPP

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Grids/ThreeDGrid.hpp"

#include "Graphic-pkg.h"

namespace HRICS
{
class WorkspaceOccupancyGrid;

class WorkspaceOccupancyCell : public API::ThreeDCell
{
public:
    WorkspaceOccupancyCell();
    WorkspaceOccupancyCell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, WorkspaceOccupancyGrid* grid);

    ~WorkspaceOccupancyCell();

    void draw();

};


class WorkspaceOccupancyGrid : public API::ThreeDGrid
{
public:
    WorkspaceOccupancyGrid(double pace, std::vector<double> envSize);
    ~WorkspaceOccupancyGrid();

    API::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );

    void draw();

private:

    void init_drawing();

    Robot* m_Human;
    bool   m_drawing;
    GLuint m_triangleVBO;

};
}

extern HRICS::WorkspaceOccupancyGrid* workspace_grid;


#endif // HRICS_WORKSPACEOCCUPANCY_HPP

#ifndef HRICS_WORKSPACEOCCUPANCY_HPP
#define HRICS_WORKSPACEOCCUPANCY_HPP

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Grids/ThreeDGrid.hpp"

#include "collision_space/CollisionSpace.hpp"

#include "HRICS_RecordMotion.hpp"

#include "Graphic-pkg.h"

namespace HRICS
{
class WorkspaceOccupancyGrid;

class WorkspaceOccupancyCell : public API::ThreeDCell
{
public:
    WorkspaceOccupancyCell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, WorkspaceOccupancyGrid* grid);
    ~WorkspaceOccupancyCell();

//    void draw(/*bool transform*/);

    Eigen::Vector3d m_center;
    bool m_visited;
    std::vector<bool> m_class_occupies;
};

class WorkspaceOccupancyGrid : public API::ThreeDGrid
{
public:
    WorkspaceOccupancyGrid(double pace, std::vector<double> envSize);
    ~WorkspaceOccupancyGrid();

    API::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );

    void setRegressedMotions(const std::vector<motion_t>& motions);
    void computeOccpancy();
    void setClassToDraw(int id_class);
    void simple_draw();
    void draw();

private:

    void setSortedCellIdsToDraw( const std::vector<WorkspaceOccupancyCell*>& occupied_voxels );
    void getCellsOccupiedByHuman( std::vector<WorkspaceOccupancyCell*>& occupied_voxels, int id_class );
//    void draw_all_cubes();
//    void init_one_cube();
    void init_drawing();
    void transform_cubes();
    void draw_voxels( const vector<unsigned int>& indices );
    void draw_sampled_points();

    bool areAllCellsBlank(int id);

    // OpenGL Drawing
    bool   m_drawing;
    GLuint m_trianglebuffer;
    GLuint m_elementbuffer;
    std::vector<unsigned int> m_indices;
    std::vector<GLfloat> m_grid_cells;
    std::vector<unsigned int> m_ids_to_draw;
    int m_nbframes;
    double m_lasttime;

    // Motions
    Robot* m_human;
    BodySurfaceSampler* m_sampler;
    std::vector<motion_t> m_motions;
    std::vector< std::vector<WorkspaceOccupancyCell*> > m_occupied_cells;
    int m_id_class_to_draw;
};
}

extern HRICS::WorkspaceOccupancyGrid* global_workspaceOccupancy;

#endif // HRICS_WORKSPACEOCCUPANCY_HPP

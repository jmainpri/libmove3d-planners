#ifndef HRICS_WORKSPACEOCCUPANCY_HPP
#define HRICS_WORKSPACEOCCUPANCY_HPP

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Grids/ThreeDGrid.hpp"

#include "collision_space/CollisionSpace.hpp"

#include "HRICS_RecordMotion.hpp"
#include "HRICS_ClassifyMotion.hpp"

#include <libmove3d/include/Graphic-pkg.h>

namespace HRICS
{
class WorkspaceOccupancyGrid;

class WorkspaceOccupancyCell : public Move3D::ThreeDCell
{
public:
    WorkspaceOccupancyCell(int i, Eigen::Vector3i pos , Eigen::Vector3d corner, WorkspaceOccupancyGrid* grid);
    ~WorkspaceOccupancyCell();

//    void draw(/*bool transform*/);

    Eigen::Vector3d m_center;
    bool m_visited;
    std::vector<bool> m_occupies_class;
    bool m_currently_occupied;
};

class WorkspaceOccupancyGrid : public Move3D::ThreeDGrid
{
public:
    WorkspaceOccupancyGrid( Move3D::Robot* human, double pace, std::vector<double> envSize );

    ~WorkspaceOccupancyGrid();

    Move3D::ThreeDCell* createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z );

    void setRegressedMotions( const std::vector<motion_t>& motions );
    void computeCurrentOccupancy();
    bool computeOccpancy();
    void setClassToDraw( int id_class );
    int classifyMotion( const motion_t& motions );
    void setLikelihood( const std::vector<double>& likelyhood );
    double getOccupancy( const Eigen::Vector3d& point ) const;
    double getOccupancyCombination( const Eigen::Vector3d& point ) const;
    double geCurrentOccupancy( const Eigen::Vector3d& point ) const;
    void drawSampledPoints();
    void draw();

private:

    void simple_draw_one_class();
    void simple_draw_combined();
    void simple_draw_current_occupancy();
    void set_all_occupied_cells();
    void set_sorted_cell_ids_to_draw( const std::vector<WorkspaceOccupancyCell*>& occupied_voxels );
    void get_cells_occupied_by_human( std::vector<WorkspaceOccupancyCell*>& occupied_voxels, int id_class, bool checkclass=true );
//    void draw_all_cubes();
//    void init_one_cube();
    void init_drawing();
    void transform_cubes();
    void draw_voxels( const std::vector<unsigned int>& indices );
    bool are_all_cells_blank(int id);

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
    Move3D::Robot* m_human;
    Move3D::BodySurfaceSampler* m_sampler;
    std::vector<motion_t> m_motions;
    std::vector< std::vector<WorkspaceOccupancyCell*> > m_occupied_cells;
    int m_id_class_to_draw;
    ClassifyMotion* m_classifier;
    RecordMotion* m_motion_recorder;
    std::vector<double> m_likelihood;
    double m_min_likelihood;
    double m_max_likelihood;
    std::vector<WorkspaceOccupancyCell*> m_all_occupied_cells;
    std::vector<WorkspaceOccupancyCell*> m_current_occupied_cells;
};
}

extern HRICS::WorkspaceOccupancyGrid* global_workspaceOccupancy;

#endif // HRICS_WORKSPACEOCCUPANCY_HPP

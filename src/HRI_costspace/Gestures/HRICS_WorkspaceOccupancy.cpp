#include <GL/glew.h>

#include "HRICS_WorkspaceOccupancy.hpp"

#include "gridsAPI.hpp"

#include <PolyVoxCore/SurfaceMesh.h>
#include <PolyVoxCore/CubicSurfaceExtractorWithNormals.h>
#include <PolyVoxCore/MarchingCubesSurfaceExtractor.h>
#include <PolyVoxCore/SurfaceMesh.h>
#include <PolyVoxCore/SimpleVolume.h>

//#define GL_GLEXT_PROTOTYPES
//#include <GL/gl.h>
//#include <GL/glext.h>


using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace HRICS;

// import most common Eigen types
// USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

HRICS::WorkspaceOccupancyGrid* global_workspaceGrid = NULL;

WorkspaceOccupancyCell::WorkspaceOccupancyCell()
{

}

WorkspaceOccupancyCell::WorkspaceOccupancyCell(int i, Vector3i coord , Vector3d corner, WorkspaceOccupancyGrid* grid) :
    API::ThreeDCell(i,corner,grid)
{

}


WorkspaceOccupancyCell::~WorkspaceOccupancyCell()
{

}

void WorkspaceOccupancyCell::draw()
{
//    double Cost = 0.0;
    double diagonal = 0.07;
    double colorvector[4];

    colorvector[0] = 0.0;       //red
    colorvector[1] = 0.0;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.01;       //transparency


    Vector3d center = getCenter();

    g3d_set_color(Any,colorvector);
    g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal, 10);
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
WorkspaceOccupancyGrid::WorkspaceOccupancyGrid(double pace, vector<double> envSize) :
    API::ThreeDGrid( pace ,envSize ), m_drawing(false)
{
    cout << "AgentGrid::createAllCells" << endl;
    createAllCells();
}

//! @brief Virtual function that creates a new cell
//! @param integer index
//! @param integer x position in the grid
//! @param integer y position in the grid
//! @param integer z position in the grid
API::ThreeDCell* WorkspaceOccupancyGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
{
    Vector3i pos;
    pos[0] = x; pos[1] = y; pos[2] = z;

    if ( index == 0 )
    {
        return new WorkspaceOccupancyCell( 0, pos ,_originCorner , this );
    }
    return new WorkspaceOccupancyCell( index, pos , computeCellCorner(x,y,z) , this );
}

WorkspaceOccupancyGrid::~WorkspaceOccupancyGrid()
{

}

void WorkspaceOccupancyGrid::init_drawing()
{
    //Initialise VBO - do only once, at start of program
    //Create a variable to hold the VBO identifier
//    GLuint triangleVBO;

    double* grid_cells = new double[3*_cells.size()];

//    std::vector<Eigen::Vector3d> data;

    // Draws all cells
    for( int i=0; i<int(_cells.size()); i++)
    {

        Eigen::Vector3d center = dynamic_cast<API::ThreeDCell*>(_cells[i])->getCenter();
//        data.push_back(center);
//        cout << "center : " << center.transpose() << endl;
        grid_cells[3*i+0] = center[0];
        grid_cells[3*i+1] = center[1];
        grid_cells[3*i+2] = center[2];
    }

    //    Vertices of a triangle (counter-clockwise winding)
    //    float data[] = {1.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 1.0};
    //    cout << "data size : " << sizeof(data) << endl;

    //Create a new VBO and use the variable id to store the VBO id
    glGenBuffers( 1, &m_triangleVBO );

    //Make the new VBO active
    glBindBuffer( GL_ARRAY_BUFFER, m_triangleVBO );

    //Upload vertex data to the video device
//    glBufferData(GL_ARRAY_BUFFER, sizeof(data), data, GL_STATIC_DRAW);
//    glBufferData(GL_ARRAY_BUFFER, data.size()*sizeof(Eigen::Vector3d), &data[0], GL_STATIC_DRAW);
    glBufferData( GL_ARRAY_BUFFER, 3*_cells.size()*sizeof(double), grid_cells, GL_STATIC_DRAW );

    //Draw Triangle from VBO - do each time window, view point or data changes
    //Establish its 3 coordinates per vertex with zero stride in this array; necessary here
    glVertexPointer(3, GL_DOUBLE, 0, NULL);

     delete grid_cells;
}

void WorkspaceOccupancyGrid::draw()
{
//    cout << "data size : " << sizeof(data) << endl;
//    cout << "vector size : " << sizeof(Eigen::Vector3d) << endl;
//    cout << "data size : " << data.size() << endl;

    if(!m_drawing) {
        init_drawing();
        m_drawing = true;
    }

    //Make the new VBO active. Repeat here incase changed since initialisation
    glBindBuffer(GL_ARRAY_BUFFER, m_triangleVBO);

    //Establish array contains vertices (not normals, colours, texture coords etc)
    glEnableClientState(GL_VERTEX_ARRAY);

    //Actually draw the triangle, giving the number of vertices provided
//    glDrawArrays(GL_TRIANGLES, 0, sizeof(data) / sizeof(float) / 3);
//    glDrawArrays(GL_TRIANGLES, 36, data.size()*sizeof(Eigen::Vector3d) / sizeof(double) / 3);
    glDrawArrays(GL_TRIANGLES, 0, _cells.size() );


    //Force display to be drawn now
    glFlush();

}



void WorkspaceOccupancyGrid::setRegressedMotions(const std::vector<motion_t>& motions)
{
    m_motions = motions;
}

void WorkspaceOccupancyGrid::computeOccpancy()
{

}

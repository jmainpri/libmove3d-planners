#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <GL/glext.h>

#include "HRICS_WorkspaceOccupancy.hpp"

#include "gridsAPI.hpp"
#include "project.hpp"
#include <sys/time.h>

//#include <PolyVoxCore/SurfaceMesh.h>
//#include <PolyVoxCore/CubicSurfaceExtractorWithNormals.h>
//#include <PolyVoxCore/MarchingCubesSurfaceExtractor.h>
//#include <PolyVoxCore/SurfaceMesh.h>
//#include <PolyVoxCore/SimpleVolume.h>

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace HRICS;

// import most common Eigen types
// USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

HRICS::WorkspaceOccupancyGrid* global_workspaceGrid = NULL;

double glfwGetTime()
{
    timeval tim;
    gettimeofday(&tim, NULL);
    return tim.tv_sec+(tim.tv_usec/1000000.0);
}

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
    API::ThreeDGrid( pace , envSize ), m_drawing(false)
{
    cout << "WorkspaceOccupancyGrid::createAllCells" << endl;
    createAllCells();

    m_human = global_Project->getActiveScene()->getRobotByName("HERAKLES_HUMAN1");
}

WorkspaceOccupancyGrid::~WorkspaceOccupancyGrid()
{
    // global_workspaceGrid = NULL;

    if( m_drawing )
    {
        glDeleteBuffers( 1, &m_trianglebuffer );
        glDeleteBuffers( 1, &m_elementbuffer );
        cout << "Deleteing the gl buffers" << endl;
    }
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


void WorkspaceOccupancyGrid::init_drawing()
{
    //Initialise VBO - do only once, at start of program
    m_grid_cells.resize( 24*_cells.size() );
    m_indices.resize( 36*_cells.size() );

    //    std::vector<Eigen::Vector3d> data;
    vector<Eigen::Vector3d> verticies(8);

    for( int i=0; i<int(_cells.size()); i++)
    {
        dynamic_cast<API::ThreeDCell*>(_cells[i])->getVerticies(verticies);

        // 8*3 = 24; 7*3+2 = 23;

        for( int j=0; j<int(verticies.size()); j++)
        {
            m_grid_cells[24*i+j*3+0] = verticies[j][0];
            m_grid_cells[24*i+j*3+1] = verticies[j][1];
            m_grid_cells[24*i+j*3+2] = verticies[j][2];

            //cout << m_grid_cells[24*i+j*3+0] << "  " <<  m_grid_cells[24*i+j*3+1] << " " << m_grid_cells[24*i+j*3+2] << endl;
        }


        //     V1 -- V0
        //    /      / |     Z  Y
        //   V2 -- V3 V5     |/
        //   |      | /       -- X
        //   V7 -- V4
        //

        // clockwise
        // 7 - 2 - 3 | 3 - 4 - 7 | 4 - 3 - 0 | 0 - 5 - 4
        // 1 - 0 - 3 | 3 - 2 - 1 | 6 - 7 - 4 | 4 - 5 - 6
        // 1 - 2 - 7 | 7 - 6 - 1 | 0 - 1 - 6 | 5 - 0 - 6

        m_indices[36*i+0] = 8*i+7; m_indices[36*i+1] = 8*i+2; m_indices[36*i+2] = 8*i+3;
        m_indices[36*i+3] = 8*i+3; m_indices[36*i+4] = 8*i+4; m_indices[36*i+5] = 8*i+7;
        m_indices[36*i+6] = 8*i+4; m_indices[36*i+7] = 8*i+3; m_indices[36*i+8] = 8*i+0;
        m_indices[36*i+9] = 8*i+0; m_indices[36*i+10] = 8*i+5; m_indices[36*i+11] = 8*i+4;

        m_indices[36*i+12] = 8*i+1; m_indices[36*i+13] = 8*i+0; m_indices[36*i+14] = 8*i+3;
        m_indices[36*i+15] = 8*i+3; m_indices[36*i+16] = 8*i+2; m_indices[36*i+17] = 8*i+1;
        m_indices[36*i+18] = 8*i+6; m_indices[36*i+19] = 8*i+7; m_indices[36*i+20] = 8*i+4;
        m_indices[36*i+21] = 8*i+4; m_indices[36*i+22] = 8*i+5; m_indices[36*i+23] = 8*i+6;

        m_indices[36*i+24] = 8*i+1; m_indices[36*i+25] = 8*i+2; m_indices[36*i+26] = 8*i+7;
        m_indices[36*i+27] = 8*i+7; m_indices[36*i+28] = 8*i+6; m_indices[36*i+29] = 8*i+1;
        m_indices[36*i+30] = 8*i+0; m_indices[36*i+31] = 8*i+1; m_indices[36*i+32] = 8*i+6;
        m_indices[36*i+33] = 8*i+5; m_indices[36*i+34] = 8*i+0; m_indices[36*i+35] = 8*i+6;

        //    cout << "36*i+35 : " << 36*i+35 << endl;
    }

    //    Vertices of a triangle (counter-clockwise winding)
    //    float data[] = {1.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 1.0};
    //    cout << "data size : " << sizeof(data) << endl;

    // Create a new VBO and use the variable id to store the VBO id
    // Make the new VBO active
    // Upload vertex data to the video device
    glGenBuffers( 1, &m_trianglebuffer );
    glBindBuffer( GL_ARRAY_BUFFER, m_trianglebuffer );
    glBufferData( GL_ARRAY_BUFFER, 24*_cells.size()*sizeof(GLfloat), &m_grid_cells[0], GL_STATIC_DRAW );

    // Draw Triangle from VBO - do each time window, view point or data changes
    // Establish its 3 coordinates per vertex with zero stride in this array; necessary here
    glVertexPointer( 3, GL_FLOAT, 0, NULL);

    glGenBuffers( 1, &m_elementbuffer );
    glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_elementbuffer );
    glBufferData( GL_ELEMENT_ARRAY_BUFFER, 36*_cells.size()*sizeof(unsigned int), &m_indices[0], GL_STATIC_DRAW );

}

void WorkspaceOccupancyGrid::transform_cubes()
{
    m_grid_cells.resize( 24*_cells.size() );

    //    std::vector<Eigen::Vector3d> data;
    vector<Eigen::Vector3d> verticies(8);
    Eigen::Transform3d T( m_human->getJoint(1)->getMatrixPos() );
    Eigen::Vector3d vert;

    for( int i=0; i<int(_cells.size()); i++)
    {
        dynamic_cast<API::ThreeDCell*>(_cells[i])->getVerticies(verticies);

        // 8*3 = 24; 7*3+2 = 23;

        for( int j=0; j<int(verticies.size()); j++)
        {
            vert = T*verticies[j];
            m_grid_cells[24*i+j*3+0] = vert[0];
            m_grid_cells[24*i+j*3+1] = vert[1];
            m_grid_cells[24*i+j*3+2] = vert[2];

            //cout << m_grid_cells[24*i+j*3+0] << "  " <<  m_grid_cells[24*i+j*3+1] << " " << m_grid_cells[24*i+j*3+2] << endl;
        }
    }

    //glGenBuffers( 1, &m_trianglebuffer );
    glBindBuffer( GL_ARRAY_BUFFER, m_trianglebuffer );
    glBufferData( GL_ARRAY_BUFFER, 24*_cells.size()*sizeof(GLfloat), &m_grid_cells[0], GL_STATIC_DRAW );
    glVertexPointer( 3, GL_FLOAT, 0, NULL);
}

void WorkspaceOccupancyGrid::draw_voxels( const std::vector<int>& voxels )
{
    cout << "voxel size is " << voxels.size() << endl;
    int start=0; int end=0; int i=0;
    while(i<int(voxels.size()-1))
    {
        start = i;

        while( (voxels[i] == ((voxels[i+1]-1))) && i<int(voxels.size()-1))
        {
            i++;
        }

        end = i-1;
        //cout << " start and end : " << start << " , " << end << endl;
        if( start < end ) {
            glDrawRangeElements(GL_TRIANGLES, 36*start, 36*end, 36*(end-start), GL_UNSIGNED_INT, NULL );
            //cout << "draw : " << start << " and " << end << endl;
        }
        i++;
    }
}

void WorkspaceOccupancyGrid::draw()
{
    if(!m_drawing) {
        cout << "init drawing" << endl;
        init_drawing();
        m_drawing = true;
        m_lasttime = glfwGetTime();
        m_nbframes = 0;
        // Measure speed
        cout << "draw !!!" << endl;
    }

    //transform_cubes();

    // Establish array contains vertices (not normals, colours, texture coords etc)
    glEnableClientState( GL_VERTEX_ARRAY );

    // Make the new VBO active. Repeat here incase changed since initialisation
    // Actually draw the triangle, giving the number of vertices provided
    //  glBindBuffer( GL_ARRAY_BUFFER, m_trianglebuffer );
    //  glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, m_elementbuffer );

    glVertexPointer( 3, GL_FLOAT, 0, NULL );

    // Draw the triangles with m_indices
    //glDrawElements( GL_TRIANGLES, 36*_cells.size(), GL_UNSIGNED_INT, NULL );
    std::vector<int> voxels;
    for(int i=0;i<100;i++) {
        voxels.push_back(i);
    }
    draw_voxels( voxels );

    // Disables array contains vertices
    glDisableClientState( GL_VERTEX_ARRAY );

    // Force display to be drawn now
    glFlush();

    m_nbframes++;
    if ( (glfwGetTime() - m_lasttime) >= 1.0 ){ // If last prinf() was more than 1 sec ago
        printf("%f ms/frame\n", 1000.0/double(m_nbframes));
        m_nbframes = 0;
        m_lasttime += 1.0;
    }
}

void WorkspaceOccupancyGrid::setRegressedMotions(const std::vector<motion_t>& motions)
{
    m_motions = motions;
}

void WorkspaceOccupancyGrid::computeOccpancy()
{

}

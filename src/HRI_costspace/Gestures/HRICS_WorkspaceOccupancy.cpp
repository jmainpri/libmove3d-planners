#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <GL/glext.h>

#include "HRICS_WorkspaceOccupancy.hpp"

#include "gridsAPI.hpp"

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
  double* grid_cells = new double[24*_cells.size()];
  
  //    std::vector<Eigen::Vector3d> data;
  vector<Eigen::Vector3d> verticies(8);
  
  for( int i=0; i<int(_cells.size()); i++)
  {
    dynamic_cast<API::ThreeDCell*>(_cells[i])->getVerticies(verticies);
    
    // 8*3 = 24; 7*3+2 = 23;
    
    for( int j=0; j<int(verticies.size()); j++)
    {
      grid_cells[24*i+j*3+0] = verticies[j][0];
      grid_cells[24*i+j*3+1] = verticies[j][1];
      grid_cells[24*i+j*3+2] = verticies[j][2]; 
    }
  }
  
  //    Vertices of a triangle (counter-clockwise winding)
  //    float data[] = {1.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 1.0};
  //    cout << "data size : " << sizeof(data) << endl;
  
  // Create a new VBO and use the variable id to store the VBO id
  glGenBuffers( 1, &m_triangleVBO );
  
  // Make the new VBO active
  glBindBuffer( GL_ARRAY_BUFFER, m_triangleVBO );
  
  // Upload vertex data to the video device
  glBufferData( GL_ARRAY_BUFFER, 24*_cells.size()*sizeof(double), grid_cells, GL_STATIC_DRAW );
  
  //Draw Triangle from VBO - do each time window, view point or data changes
  //Establish its 3 coordinates per vertex with zero stride in this array; necessary here
  glVertexPointer(3, GL_DOUBLE, 0, NULL);
  
//  glVertexAttribPointer((GLuint)0, 3, GL_FLOAT, GL_FALSE, 0, 0); // Set up our vertex attributes pointer  
//  glEnableVertexAttribArray(0); // Disable our Vertex Array Object  
//  glBindVertexArray(0); // Disable our Vertex Buffer Object  
  
  delete grid_cells;
}

void WorkspaceOccupancyGrid::draw()
{
  if(!m_drawing) {
    cout << "init drawing" << endl;
    init_drawing();
    m_drawing = true;
    cout << "draw !!!" << endl;
  }
  
  // Make the new VBO active. Repeat here incase changed since initialisation
  // glBindBuffer( GL_ARRAY_BUFFER, m_triangleVBO );
  
  // Establish array contains vertices (not normals, colours, texture coords etc)
  glEnableClientState( GL_VERTEX_ARRAY );
  
  // Actually draw the triangle, giving the number of vertices provided
  glDrawArrays( GL_TRIANGLES, 0, 8*_cells.size() );
  
  // Force display to be drawn now
  glFlush();
}

void WorkspaceOccupancyGrid::setRegressedMotions(const std::vector<motion_t>& motions)
{
  m_motions = motions;
}

void WorkspaceOccupancyGrid::computeOccpancy()
{
  
}

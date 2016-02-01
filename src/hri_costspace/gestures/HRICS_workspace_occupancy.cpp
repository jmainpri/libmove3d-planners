/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#include <GL/glext.h>

// This breaks Eigen3
#undef Success

#include "HRICS_workspace_occupancy.hpp"
#include "HRICS_gest_parameters.hpp"

#include "planner/planEnvironment.hpp"

#include "API/Graphic/drawModule.hpp"
#include "API/Grids/gridsAPI.hpp"
#include "API/project.hpp"

#include <libmove3d/p3d/env.hpp>

#include <boost/bind.hpp>
#include <sys/time.h>

//#include <PolyVoxCore/SurfaceMesh.h>
//#include <PolyVoxCore/CubicSurfaceExtractorWithNormals.h>
//#include <PolyVoxCore/MarchingCubesSurfaceExtractor.h>
//#include <PolyVoxCore/SurfaceMesh.h>
//#include <PolyVoxCore/SimpleVolume.h>

using namespace std;
using namespace HRICS;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

HRICS::WorkspaceOccupancyGrid* global_workspaceOccupancy = NULL;

double glfwGetTime() {
  timeval tim;
  gettimeofday(&tim, NULL);
  return tim.tv_sec + (tim.tv_usec / 1000000.0);
}

WorkspaceOccupancyCell::WorkspaceOccupancyCell(int i,
                                               Eigen::Vector3i coord,
                                               Eigen::Vector3d corner,
                                               WorkspaceOccupancyGrid* grid)
    : Move3D::ThreeDCell(i, corner, grid), m_visited(false) {
  m_center = getCenter();
}

WorkspaceOccupancyCell::~WorkspaceOccupancyCell() {}

int WorkspaceOccupancyCell::getNumberOfClassesOccupied() {
  int nb_of_classes_occupied = 0;

  for (size_t i = 0; i < m_occupies_class.size(); i++)
    if (m_occupies_class[i]) nb_of_classes_occupied++;

  return nb_of_classes_occupied;
}

bool WorkspaceOccupancyCell::writeToXml(xmlNodePtr _XmlCellNode_) {
  if (!ThreeDCell::writeToXml(_XmlCellNode_)) {
    return false;
  }

  stringstream ss;
  string str;

  str.clear();
  ss << m_occupies_class.size();
  ss >> str;
  ss.clear();
  xmlNewProp(
      _XmlCellNode_, xmlCharStrdup("NbClasses"), xmlCharStrdup(str.c_str()));

  for (size_t i = 0; i < m_occupies_class.size(); i++) {
    std::stringstream converter;
    converter << i;
    str.clear();
    ss << m_occupies_class[i];
    ss >> str;
    ss.clear();
    xmlNewProp(_XmlCellNode_,
               xmlCharStrdup(("class_" + converter.str()).c_str()),
               xmlCharStrdup(str.c_str()));
  }

  return true;
}

bool WorkspaceOccupancyCell::readCellFromXml(xmlNodePtr cur) {
  if (!ThreeDCell::readCellFromXml(cur)) {
    return false;
  }

  xmlChar* tmp;

  if ((tmp = xmlGetProp(cur, xmlCharStrdup("NbClasses"))) != NULL) {
    int nb_classes;
    sscanf((char*)tmp, "%d", &nb_classes);
    m_occupies_class.resize(nb_classes);
  } else {
    cout << "error reading grid" << endl;
    return false;
  }
  xmlFree(tmp);

  int tmp_bool;

  for (size_t i = 0; i < m_occupies_class.size(); i++) {
    std::stringstream converter;
    converter << i;

    if ((tmp = xmlGetProp(
             cur, xmlCharStrdup(("class_" + converter.str()).c_str()))) !=
        NULL) {
      sscanf((char*)tmp, "%d", &tmp_bool);
      m_occupies_class[i] = tmp_bool;
    } else {
      cout << "error reading grid" << endl;
      return false;
    }
    xmlFree(tmp);
  }

  return true;
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
WorkspaceOccupancyGrid::WorkspaceOccupancyGrid(Robot* human,
                                               double pace,
                                               vector<double> envSize)
    : Move3D::ThreeDGrid(pace, envSize),
      m_drawing(false),
      m_human(human),
      m_id_class_to_draw(0) {
  cout << "WorkspaceOccupancyGrid::createAllCells" << endl;
  createAllCells();
  cout << "_cells.size() : " << _cells.size() << endl;

  if (m_human->getUseLibmove3dStruct()) {
    m_sampler = new BodySurfaceSampler(0.02, 0.05);
    m_sampler->sampleRobotBodiesSurface(m_human);
  }

  m_T_draw = Eigen::Transform3d::Identity();

  //    if( global_DrawModule )
  //    {
  //        global_DrawModule->addDrawFunction( "WorkspaceOccupancyGrid",
  //        boost::bind( &WorkspaceOccupancyGrid::draw, this) );
  //        global_DrawModule->enableDrawFunction( "WorkspaceOccupancyGrid" );
  //    }
}

WorkspaceOccupancyGrid::~WorkspaceOccupancyGrid() {
  // global_workspaceOccupancy = NULL;

  if (m_drawing) {
    glDeleteBuffers(1, &m_trianglebuffer);
    glDeleteBuffers(1, &m_elementbuffer);
    cout << "Deleteing the gl buffers" << endl;
  }
}

//! @brief Virtual function that creates a new cell
//! @param integer index
//! @param integer x position in the grid
//! @param integer y position in the grid
//! @param integer z position in the grid
Move3D::ThreeDCell* WorkspaceOccupancyGrid::createNewCell(unsigned int index,
                                                          unsigned int x,
                                                          unsigned int y,
                                                          unsigned int z) {
  Eigen::Vector3i pos;
  pos[0] = x;
  pos[1] = y;
  pos[2] = z;

  if (index == 0) {
    return new WorkspaceOccupancyCell(0, pos, _originCorner, this);
  }
  return new WorkspaceOccupancyCell(
      index, pos, computeCellCorner(x, y, z), this);
}

//! Creates a list of cells occupied by the human in current conf
//! by processing the sampled points on each body
//! using the cells to store if they belong to the class given as input
//! If so, does not add the cell twice to the list and update the cell class
void WorkspaceOccupancyGrid::get_cells_occupied_by_human(
    std::vector<WorkspaceOccupancyCell*>& occupied_voxels,
    int id_class,
    bool checkclass) {
  Eigen::Transform3d T;  // Each point is transformed to the human config

  for (int i = 0; i < int(m_human->getNumberOfJoints()); i++) {
    p3d_obj* obj = m_human->getJoint(i)->getP3dJointStruct()->o;

    if (obj) {
      T = m_human->getJoint(i)->getMatrixPos();

      PointCloud& cloud = m_sampler->getPointCloud(obj);

      for (int i = 0; i < int(cloud.size()); i++) {
        WorkspaceOccupancyCell* cell = dynamic_cast<WorkspaceOccupancyCell*>(
            getCell(Eigen::Vector3d(T * cloud[i])));

        if (cell == NULL) continue;

        if (!cell->m_visited) {
          cell->m_visited = true;

          if (!checkclass) {
            occupied_voxels.push_back(cell);
          } else {
            if (cell->m_occupies_class.empty()) {
              occupied_voxels.push_back(cell);
            } else {
              if (!cell->m_occupies_class[id_class]) {
                occupied_voxels.push_back(cell);
                cell->m_occupies_class[id_class] = true;
              }
            }
          }
        }
      }
    }
  }

  for (int i = 0; i < int(occupied_voxels.size()); i++) {
    occupied_voxels[i]->m_visited = false;
  }
}

//! Set regessed motion
//! also resets cells occpancy class
//! @param a vector of motions of each class
void WorkspaceOccupancyGrid::setRegressedMotions(
    const std::vector<motion_t>& motions, bool reset_occupied_cells) {
  cout << __PRETTY_FUNCTION__ << endl;
  m_motions.clear();
  m_motions = motions;

  if (reset_occupied_cells) {
    m_occupied_cells.clear();
    m_occupied_cells.resize(m_motions.size());

    for (int i = 0; i < int(_cells.size()); i++) {
      WorkspaceOccupancyCell* cell =
          dynamic_cast<WorkspaceOccupancyCell*>(_cells[i]);

      cell->m_occupies_class.resize(m_motions.size());

      for (int j = 0; j < int(cell->m_occupies_class.size()); j++) {
        cell->m_occupies_class[j] = false;
      }
    }
  }
}

//! Returns true if the cells of blank for a given class
//! @param id of a class
bool WorkspaceOccupancyGrid::are_all_cells_blank(int id) {
  for (int i = 0; i < int(_cells.size()); i++) {
    if (static_cast<WorkspaceOccupancyCell*>(_cells[i])->m_occupies_class[id] ==
        true) {
      return false;
    }

    if (static_cast<WorkspaceOccupancyCell*>(_cells[i])->m_visited == true) {
      return false;
    }
  }

  return true;
}

void WorkspaceOccupancyGrid::computeCurrentOccupancy() {
  for (int i = 0; i < int(m_current_occupied_cells.size()); i++) {
    m_current_occupied_cells[i]->m_currently_occupied = false;
  }

  m_current_occupied_cells.clear();

  get_cells_occupied_by_human(m_current_occupied_cells, -1, false);

  for (int i = 0; i < int(m_current_occupied_cells.size()); i++) {
    m_current_occupied_cells[i]->m_currently_occupied = true;
  }
}

//! Set each cell to belong to a class of motion
//! by computing the voxel occupancy for each point along the path
//! of the corresponding regressed motion
bool WorkspaceOccupancyGrid::computeOccpancy() {
  confPtr_t q = m_human->getCurrentPos();

  cout << __PRETTY_FUNCTION__ << endl;
  cout << "Compute workspace occupancy for " << m_motions.size() << " motions"
       << endl;

  // For each motion class compute the occupancy
  for (int i = 0; i < int(m_motions.size()); i++) {
    // cout << "----------------------------------------" << endl;
    cout << " Motion : " << i << endl;
    // cout << "----------------------------------------" << endl;

    m_occupied_cells[i].clear();

    if (!are_all_cells_blank(i)) {
      cout << "ERROR" << endl;
    }

    // Warning, set to 100 in the normal case
    // const motion_t& swept_volume_motion = m_motions[i];
    const motion_t& swept_volume_motion =
        m_motion_recorder->resample(m_motions[i], 10);

    for (int j = 0; j < int(swept_volume_motion.size()); j++) {
      // Compute occupied workspace for each configuration in the trajectory
      m_human->setAndUpdate(*swept_volume_motion[j].second);

      std::vector<WorkspaceOccupancyCell*> cells;
      get_cells_occupied_by_human(cells, i);
      cout << " Frame : " << j << endl;
      m_occupied_cells[i].insert(
          m_occupied_cells[i].end(), cells.begin(), cells.end());
    }

    for (int j = 0; j < int(m_occupied_cells[i].size()); j++) {
      m_occupied_cells[i][j]->m_visited = false;
    }

    cout << "m_occupied_cells[" << i
         << "].size() : " << m_occupied_cells[i].size() << endl;
  }

  m_human->setAndUpdate(*q);

  set_all_occupied_cells();

  cout << "There are " << m_all_occupied_cells.size() << " occupied cells"
       << endl;
  cout << "End : " << __PRETTY_FUNCTION__ << endl;

  if (m_all_occupied_cells.empty())
    return false;
  else
    return true;
}

//!
void WorkspaceOccupancyGrid::set_all_occupied_cells() {
  m_all_occupied_cells.clear();

  for (int i = 0; i < int(_cells.size()); i++) {
    WorkspaceOccupancyCell* cell =
        static_cast<WorkspaceOccupancyCell*>(_cells[i]);

    for (int j = 0; j < int(cell->m_occupies_class.size()); j++) {
      if (cell->m_occupies_class[j]) {
        m_all_occupied_cells.push_back(cell);
        break;
      }
    }
  }
}

double WorkspaceOccupancyGrid::getOccupancy(
    const Eigen::Vector3d& point) const {
  if (m_all_occupied_cells.empty()) {
    cout << "occupied cells not loaded" << endl;
    cout << "Error : " << __PRETTY_FUNCTION__ << endl;
    return 0.0;
  }

  return double(static_cast<WorkspaceOccupancyCell*>(getCell(point))
                    ->m_occupies_class[m_id_class_to_draw]);
}

double WorkspaceOccupancyGrid::geCurrentOccupancy(
    const Eigen::Vector3d& point) const {
  if (m_current_occupied_cells.empty()) {
    //        cout << "occupied cells not loaded" << endl;
    //        exit(0);
    return 0.0;
  }

  WorkspaceOccupancyCell* cell =
      static_cast<WorkspaceOccupancyCell*>(getCell(point));

  if (cell == NULL) {
    // return 0.0;
    cout << "Null cell" << endl;
    return 0.0;
  }

  return double(cell->m_currently_occupied);
}

double WorkspaceOccupancyGrid::getOccupancyCombination(
    const Eigen::Vector3d& point) const {
  if (m_all_occupied_cells.empty()) {
    cout << "occupied cells not loaded" << endl;
    cout << "Error : " << __PRETTY_FUNCTION__ << endl;
    return 0.0;
  }

  WorkspaceOccupancyCell* cell =
      static_cast<WorkspaceOccupancyCell*>(getCell(point));

  if (cell == NULL) {
    // return 0.0;
    cout << "Null cell" << endl;
    return 0.0;
  }

  double occupancy = 0.0;

  for (int i = 0; i < int(cell->m_occupies_class.size()); i++) {
    occupancy += (m_likelihood[i] * cell->m_occupies_class[i]);
  }

  return occupancy;
}

void WorkspaceOccupancyGrid::setLikelihood(
    const std::vector<double>& likelihood) {
  m_likelihood = likelihood;

  m_min_likelihood =
      *std::min_element(m_likelihood.begin(), m_likelihood.end());

  cout << "log-likelihood ("
       << std::max_element(m_likelihood.begin(), m_likelihood.end()) -
              m_likelihood.begin() << ") : ";
  for (int i = 0; i < int(m_likelihood.size()); i++) {
    cout << m_likelihood[i] << " , ";
  }
  cout << endl;

  //    if( m_min_likelihood<0 )
  //    {
  //        for( int i=0;i<int(m_likelihood.size());i++)
  //        {
  //            m_likelihood[i] += std::abs( m_min_likelihood );
  //        }
  //    }

  //    cout << " likelihood : " ;
  //    for( int i=0;i<int(m_likelihood.size());i++)
  //    {
  //        cout << m_likelihood[i] << " , ";
  //    }
  //    cout << endl;

  for (int i = 0; i < int(m_likelihood.size()); i++) {
    m_likelihood[i] = std::exp(m_likelihood[i]);
  }

  double sum = 0;

  for (int i = 0; i < int(m_likelihood.size()); i++) sum += m_likelihood[i];

  for (int i = 0; i < int(m_likelihood.size()); i++) m_likelihood[i] /= sum;

  //    cout << " likelihood : " ;
  //    for( int i=0;i<int(m_likelihood.size());i++)
  //    {
  //        cout << m_likelihood[i] << " , ";
  //    }
  //    cout << endl;

  m_max_likelihood = 1;
  m_min_likelihood = 0;

  m_id_class_to_draw =
      std::max_element(m_likelihood.begin(), m_likelihood.end()) -
      m_likelihood.begin();

  //    m_max_likelihood = 0.0;

  //    for( int i=0;i<m_likelihood.size();i++)
  //    {
  //        m_max_likelihood += m_likelihood[i];
  //    }

  //    m_max_likelihood = *std::max_element( m_likelihood.begin(),
  //    m_likelihood.end() );
  //    m_min_likelihood = *std::min_element( m_likelihood.begin(),
  //    m_likelihood.end() );

  //    cout << "m_id_class_to_draw : " << m_id_class_to_draw << endl;
  //    cout << " m_min_likelyhood : " << m_min_likelihood << endl;
  //    cout << " m_max_likelyhood : " << m_max_likelihood << endl;
}

//-----------------------------------------------------------------------
//-----------------------------------------------------------------------
//-----------------------------------------------------------------------

void WorkspaceOccupancyGrid::drawSampledPoints() {
  for (unsigned int i = 0; i < m_human->getNumberOfJoints(); i++) {
    Joint* jnt = m_human->getJoint(i);

    p3d_obj* obj = jnt->getP3dJointStruct()->o;

    if (obj) {
      PointCloud& cloud = m_sampler->getPointCloud(obj);
      cloud.drawAllPoints(jnt->getMatrixPos());
    }
  }
}

void WorkspaceOccupancyGrid::setClassToDraw(int id_class) {
  if (id_class < 0 || id_class >= int(m_occupied_cells.size())) return;

  m_id_class_to_draw = id_class;
  //    m_human->setAndUpdate(*m_motions[id_class].back().second);
}

void WorkspaceOccupancyGrid::simple_draw_one_class() {
  std::vector<WorkspaceOccupancyCell*>& occupied_voxels =
      m_occupied_cells[m_id_class_to_draw];

  for (int i = 0; i < int(occupied_voxels.size()); i++) {
    occupied_voxels[i]->drawColorGradient(0.5, 0.0, 1.0);
  }
}

void WorkspaceOccupancyGrid::simple_draw_current_occupancy() {
  for (int i = 0; i < int(m_current_occupied_cells.size()); i++) {
    m_current_occupied_cells[i]->drawColorGradient(1.0, 0.0, 0.0);
  }
}

void WorkspaceOccupancyGrid::simple_draw_combined() {
  //    cout << __PRETTY_FUNCTION__ << endl;

  if (m_all_occupied_cells.empty()) {
    //        cout << "m_all_occupied_cells.empty()" << endl;
    return;
  }

  for (int i = 0; i < int(m_all_occupied_cells.size()); i++) {
    double occupancy = 0.0;
    //        double max = 0.0;

    for (int j = 0; j < int(m_likelihood.size()); j++) {
      occupancy += (m_likelihood[j] *
                    double(m_all_occupied_cells[i]->m_occupies_class[j]));

      //            if( m_all_occupied_cells[i]->m_occupies_class[j] )
      //            {
      //                 if( m_likelihood[j] > max )
      //                     max = m_likelihood[j];
      //            }
    }

    //        if( m_all_occupied_cells[i]->m_occupies_class[m_id_class_to_draw]
    //        )
    //        {
    if (GestEnv->getBool(GestParam::draw_null_cost) ||
        (occupancy - 0.05 > 0.0)) {
      if (true) {
        //                cout << " m_T_draw " << endl << m_T_draw.matrix() <<
        //                endl;
        m_all_occupied_cells[i]->drawColorGradient(
            occupancy, m_min_likelihood, m_max_likelihood, m_T_draw);
      } else
        m_all_occupied_cells[i]->drawColorGradient(
            occupancy, m_min_likelihood, m_max_likelihood);
    }
    //        }
  }
}

void WorkspaceOccupancyGrid::setDrawingTransform(const Eigen::Transform3d& t) {
  m_use_transform_to_draw =
      !(t.matrix() == Eigen::Transform3d::Identity().matrix());
  m_T_draw = t;
}

void WorkspaceOccupancyGrid::init_drawing() {
  // Initialise VBO - do only once, at start of program
  m_grid_cells.resize(24 * _cells.size());
  m_indices.resize(36 * _cells.size());

  //    std::vector<Eigen::Vector3d> data;
  vector<Eigen::Vector3d> verticies(8);

  for (int i = 0; i < int(_cells.size()); i++) {
    dynamic_cast<Move3D::ThreeDCell*>(_cells[i])->getVerticies(verticies);
    //        cout << "index : " <<
    //        dynamic_cast<Move3D::ThreeDCell*>(_cells[i])->getIndex() << endl;

    // 8*3 = 24; 7*3+2 = 23;

    for (int j = 0; j < int(verticies.size()); j++) {
      m_grid_cells[24 * i + j * 3 + 0] = verticies[j][0];
      m_grid_cells[24 * i + j * 3 + 1] = verticies[j][1];
      m_grid_cells[24 * i + j * 3 + 2] = verticies[j][2];

      // cout << m_grid_cells[24*i+j*3+0] << "  " <<  m_grid_cells[24*i+j*3+1]
      // << " " << m_grid_cells[24*i+j*3+2] << endl;
    }

    //     V1 -- V0
    //    /      / |     Z  Y
    //   V2 -- V3 V5     |/
    //   |      | /       -- X
    //   V7 -- V4

    // clockwise
    // 7 - 2 - 3 | 3 - 4 - 7 | 4 - 3 - 0 | 0 - 5 - 4
    // 1 - 0 - 3 | 3 - 2 - 1 | 6 - 7 - 4 | 4 - 5 - 6
    // 1 - 2 - 7 | 7 - 6 - 1 | 0 - 1 - 6 | 5 - 0 - 6

    m_indices[36 * i + 0] = 8 * i + 7;
    m_indices[36 * i + 1] = 8 * i + 2;
    m_indices[36 * i + 2] = 8 * i + 3;
    m_indices[36 * i + 3] = 8 * i + 3;
    m_indices[36 * i + 4] = 8 * i + 4;
    m_indices[36 * i + 5] = 8 * i + 7;
    m_indices[36 * i + 6] = 8 * i + 4;
    m_indices[36 * i + 7] = 8 * i + 3;
    m_indices[36 * i + 8] = 8 * i + 0;
    m_indices[36 * i + 9] = 8 * i + 0;
    m_indices[36 * i + 10] = 8 * i + 5;
    m_indices[36 * i + 11] = 8 * i + 4;

    m_indices[36 * i + 12] = 8 * i + 1;
    m_indices[36 * i + 13] = 8 * i + 0;
    m_indices[36 * i + 14] = 8 * i + 3;
    m_indices[36 * i + 15] = 8 * i + 3;
    m_indices[36 * i + 16] = 8 * i + 2;
    m_indices[36 * i + 17] = 8 * i + 1;
    m_indices[36 * i + 18] = 8 * i + 6;
    m_indices[36 * i + 19] = 8 * i + 7;
    m_indices[36 * i + 20] = 8 * i + 4;
    m_indices[36 * i + 21] = 8 * i + 4;
    m_indices[36 * i + 22] = 8 * i + 5;
    m_indices[36 * i + 23] = 8 * i + 6;

    m_indices[36 * i + 24] = 8 * i + 1;
    m_indices[36 * i + 25] = 8 * i + 2;
    m_indices[36 * i + 26] = 8 * i + 7;
    m_indices[36 * i + 27] = 8 * i + 7;
    m_indices[36 * i + 28] = 8 * i + 6;
    m_indices[36 * i + 29] = 8 * i + 1;
    m_indices[36 * i + 30] = 8 * i + 0;
    m_indices[36 * i + 31] = 8 * i + 1;
    m_indices[36 * i + 32] = 8 * i + 6;
    m_indices[36 * i + 33] = 8 * i + 5;
    m_indices[36 * i + 34] = 8 * i + 0;
    m_indices[36 * i + 35] = 8 * i + 6;

    //    cout << "36*i+35 : " << 36*i+35 << endl;
  }

  //    Vertices of a triangle (counter-clockwise winding)
  //    float data[] = {1.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 1.0};
  //    cout << "data size : " << sizeof(data) << endl;

  // Create a new VBO and use the variable id to store the VBO id
  // Make the new VBO active
  // Upload vertex data to the video device
  glGenBuffers(1, &m_trianglebuffer);
  glBindBuffer(GL_ARRAY_BUFFER, m_trianglebuffer);
  glBufferData(GL_ARRAY_BUFFER,
               24 * _cells.size() * sizeof(GLfloat),
               &m_grid_cells[0],
               GL_STATIC_DRAW);

  // Draw Triangle from VBO - do each time window, view point or data changes
  // Establish its 3 coordinates per vertex with zero stride in this array;
  // necessary here
  glVertexPointer(3, GL_FLOAT, 0, NULL);

  glGenBuffers(1, &m_elementbuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementbuffer);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               36 * _cells.size() * sizeof(unsigned int),
               &m_indices[0],
               GL_STATIC_DRAW);
}

void WorkspaceOccupancyGrid::transform_cubes() {
  m_grid_cells.resize(24 * _cells.size());

  //    std::vector<Eigen::Vector3d> data;
  vector<Eigen::Vector3d> verticies(8);
  Eigen::Transform3d T(m_human->getJoint(1)->getMatrixPos());
  Eigen::Vector3d vert;

  for (int i = 0; i < int(_cells.size()); i++) {
    dynamic_cast<Move3D::ThreeDCell*>(_cells[i])->getVerticies(verticies);

    // 8*3 = 24; 7*3+2 = 23;

    for (int j = 0; j < int(verticies.size()); j++) {
      vert = T * verticies[j];
      m_grid_cells[24 * i + j * 3 + 0] = vert[0];
      m_grid_cells[24 * i + j * 3 + 1] = vert[1];
      m_grid_cells[24 * i + j * 3 + 2] = vert[2];

      // cout << m_grid_cells[24*i+j*3+0] << "  " <<  m_grid_cells[24*i+j*3+1]
      // << " " << m_grid_cells[24*i+j*3+2] << endl;
    }
  }

  // glGenBuffers( 1, &m_trianglebuffer );
  glBindBuffer(GL_ARRAY_BUFFER, m_trianglebuffer);
  glBufferData(GL_ARRAY_BUFFER,
               24 * _cells.size() * sizeof(GLfloat),
               &m_grid_cells[0],
               GL_STATIC_DRAW);
  glVertexPointer(3, GL_FLOAT, 0, NULL);
}

void WorkspaceOccupancyGrid::set_sorted_cell_ids_to_draw(
    const std::vector<WorkspaceOccupancyCell*>& occupied_voxels) {
  m_ids_to_draw.resize(occupied_voxels.size());

  //    cout << " ------------------------------------------------- " <<  endl;
  for (int i = 0; i < int(occupied_voxels.size()); i++) {
    m_ids_to_draw[i] = occupied_voxels[i]->getIndex();
  }

  std::sort(m_ids_to_draw.begin(), m_ids_to_draw.end());

  //    for(int i=0;i<int(m_ids_to_draw.size());i++)
  //    {
  //        cout << " m_ids_to_draw[i] : " << m_ids_to_draw[i] << endl;
  //    }
}

void WorkspaceOccupancyGrid::draw_voxels(
    const std::vector<unsigned int>& voxels) {
  // cout << "voxel size is " << voxels.size() << endl;
  //    unsigned int voxel_count =0;
  unsigned int i = 0;

  while (i < voxels.size()) {
    // TODO also uncoment
    // unsigned int init_id = i;

    while ((voxels[i] == ((voxels[i + 1] - 1))) && i < voxels.size()) {
      i++;
    }

    cout << "Error in draw_voxels function (FIX)" << endl;
    // TODO uncomment region
    /*        GLuint start = sizeof(unsigned int)*36*voxels[init_id];
            GLuint end = sizeof(unsigned int)*36*voxels[i];
            GLsizei count = 36*(voxels[i]-voxels[init_id]+1);
            glDrawRangeElements( GL_TRIANGLES , start, end, count,
       GL_UNSIGNED_INT, (const GLuint*)(start) );
    */

    //        voxel_count += (voxels[i]-voxels[init_id]+1);
    i++;
  }

  //    cout << "voxels.size() : " << voxels.size() << endl;
  //    cout << "voxel_count : " << voxel_count << endl;
}

void WorkspaceOccupancyGrid::draw() {
  //    cout << __PRETTY_FUNCTION__ << endl;

  if (GestEnv->getBool(GestParam::draw_current_occupancy)) {
    //        cout << "draw current occupancy" << endl;
    computeCurrentOccupancy();
    simple_draw_current_occupancy();
    return;
  }

  if (GestEnv->getBool(GestParam::draw_single_class)) {
    // cout << "simple_draw_one_class" << endl;
    simple_draw_one_class();
    return;
  } else {
    // cout << "simple_draw_combined" << endl;
    simple_draw_combined();
    return;
  }

  if (m_motions.empty()) {
    cout << "motion empty" << endl;
    return;
  }

  if (!m_human->getUseLibmove3dStruct()) {
    return;
  }

  if (!m_drawing) {
    cout << "init drawing" << endl;
    init_drawing();
    m_drawing = true;
    m_lasttime = glfwGetTime();
    m_nbframes = 0;
    // Measure speed
    cout << "draw !!!" << endl;
  }

  // transform_cubes();

  // Establish array contains vertices (not normals, colours, texture coords
  // etc)
  glEnableClientState(GL_VERTEX_ARRAY);

  // Make the new VBO active. Repeat here incase changed since initialisation
  // Actually draw the triangle, giving the number of vertices provided
  glBindBuffer(GL_ARRAY_BUFFER, m_trianglebuffer);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementbuffer);

  glVertexPointer(3, GL_FLOAT, 0, NULL);

  // Draw the triangles with m_indices
  //    glDrawElements( GL_TRIANGLES, GLuint(36*_cells.size()), GL_UNSIGNED_INT,
  //    NULL );

  if (m_motions.empty()) {
    std::vector<WorkspaceOccupancyCell*> occupied_voxels;
    get_cells_occupied_by_human(occupied_voxels, 0);
    set_sorted_cell_ids_to_draw(occupied_voxels);
    draw_voxels(m_ids_to_draw);
  }

  else {
    std::vector<WorkspaceOccupancyCell*>& occupied_voxels =
        m_occupied_cells[m_id_class_to_draw];
    set_sorted_cell_ids_to_draw(occupied_voxels);
    draw_voxels(m_ids_to_draw);
  }

  //    GLuint start = sizeof(unsigned int)*36*17000;
  //    GLuint end = sizeof(unsigned int)*36*18000;
  //    GLuint count = 36*1000;
  //    glDrawRangeElements( GL_TRIANGLES, start, end, count, GL_UNSIGNED_INT,
  //    (const GLuint*)(start));

  //    start = sizeof(unsigned int)*36*9000;
  //    end = sizeof(unsigned int)*36*10000;
  //    count = 36*1000;
  //    glDrawRangeElements( GL_TRIANGLES, start, end, count, GL_UNSIGNED_INT,
  //    (const GLuint*)(start));

  //    GLint nb_vertices;
  //    GLint nb_indices;
  //    glGetIntegerv(GL_MAX_ELEMENTS_VERTICES,&nb_vertices);
  //    glGetIntegerv(GL_MAX_ELEMENTS_INDICES,&nb_indices);
  //    cout << int(nb_vertices) << endl;
  //    cout << int(nb_indices) << endl;

  // Disables array contains vertices
  glDisableClientState(GL_VERTEX_ARRAY);

  // Force display to be drawdraw_sampled_pointsn now
  glFlush();

  m_nbframes++;
  if ((glfwGetTime() - m_lasttime) >=
      1.0) {  // If last prinf() was more than 1 sec ago
    // printf("%f ms/frame\n", 1000.0/double(m_nbframes));
    m_nbframes = 0;
    m_lasttime += 1.0;
  }
}

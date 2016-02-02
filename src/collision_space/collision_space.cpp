/*
 *  CollisionSpace.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "collision_space.hpp"
//#include "CollisionSpaceCell.hpp"
#include "body_surface_sampler.hpp"

#include "API/project.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#include <boost/bind.hpp>
#include <Eigen/Core>

using namespace Move3D;
using std::cout;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

CollisionSpace* Move3D::global_collisionSpace = NULL;

CollisionSpaceCell::CollisionSpaceCell(int i,
                                       const Eigen::Vector3d& corner,
                                       Move3D::ThreeDGrid* grid)
    : Move3D::ThreeDCell(i, corner, grid) {
  m_Valid = true;
  m_Visited = false;
  m_Occupied = false;

  m_ClosestPoint.x() = UNINITIALIZED;
  m_ClosestPoint.x() = UNINITIALIZED;
  m_ClosestPoint.x() = UNINITIALIZED;

  _cellSize = grid->getCellSize();
  m_Location = grid->getCellCoord(this);

  closest_negative_point_.x() = m_Location.x();
  closest_negative_point_.y() = m_Location.y();
  closest_negative_point_.z() = m_Location.z();

  // Max radius
  m_DistanceSquare = 10000;
  negative_distance_square_ = 0;
}

void CollisionSpaceCell::draw(int color, int width) {
  Eigen::Vector3d corner = getCorner();

  // cout << "Draw CollisionSpaceCell ( " << color << " )" << endl;
  g3d_draw_simple_box(corner[0],
                      corner[0] + _cellSize[0],
                      corner[1],
                      corner[1] + _cellSize[1],
                      corner[2],
                      corner[2] + _cellSize[2],
                      color,
                      0,
                      width);
}

void CollisionSpaceCell::drawStatic() {
  if (!m_Valid) {
    draw(Red, 1);
  }
}

void CollisionSpaceCell::draw() {
  if (!m_Valid) {
    // drawStatic()
  } else if (m_Occupied) {
    draw(Green, 1);
  }
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------
//-----------------------------------------------------------------

CollisionSpace::CollisionSpace(Robot* rob,
                               double pace,
                               const std::vector<double>& env_size)
    : Move3D::ThreeDGrid(pace, env_size), m_sampler(NULL) {
  this->createAllCells();

  // unvalid cells for each robot except current one ?
  m_size =
      std::max(std::max(_cellSize[0] * _nbCellsX, _cellSize[1] * _nbCellsY),
               _cellSize[2] * _nbCellsZ);

  // Build the meshes env edges
  if (XYZ_ENV->o) {
    if (XYZ_ENV->o[0]->pol[0]->poly->the_edges == NULL) {
      for (int i = 0; i < XYZ_ENV->no; i++) {
        p3d_obj* obj = XYZ_ENV->o[i];

        for (int j = 0; j < obj->np; j++) {
          poly_build_edges(obj->pol[j]->poly);
        }
      }
    }
  }

  // m_Robot = global_Project->getActiveScene()->getActiveRobot();
  m_Robot = rob;

  propagate_negative_ = true;

  initialize();

  cout << "pace : " << pace << endl;
  cout << "nb cells : " << _cells.size() << endl;

  if (global_costSpace != NULL) {
    cout << "add cost functions : "
         << "costCollisionSpace" << endl;
    global_costSpace->addCost("costCollisionSpace",
                              boost::bind(&CollisionSpace::cost, this, _1));
  }
}

CollisionSpace::~CollisionSpace() {
  if (global_costSpace != NULL)
    global_costSpace->deleteCost("costCollisionSpace");

  delete m_sampler;
}

Move3D::ThreeDCell* CollisionSpace::createNewCell(unsigned int index,
                                                  unsigned int x,
                                                  unsigned int y,
                                                  unsigned int z) {
  return new CollisionSpaceCell(index, computeCellCorner(x, y, z), this);
}

void CollisionSpace::initialize() {
  double diagonal = _cellSize.norm();
  // SET CLEARANCE HERE
  double clearance = PlanEnv->getDouble(PlanParam::collison_points_clearance);
  m_sampler = new BodySurfaceSampler(diagonal * 0.25, clearance);
  m_sampler->sampleStaticObjectsSurface();
  m_sampler->sampleAllRobotsBodiesSurface();

  int maxSq =
      (_nbCellsX * _nbCellsX + _nbCellsY * _nbCellsY + _nbCellsZ * _nbCellsZ);

  double resolution = _cellSize[0];

  // create a sqrt table:
  m_SqrtTable.resize(maxSq + 1);
  for (int i = 0; i <= maxSq; ++i) {
    m_SqrtTable[i] = sqrt(double(i)) * resolution;
  }

  cout << "Max m_SqrtTable : " << m_SqrtTable.back() << endl;

  m_invTwiceResolution = 1.0 / (2.0 * resolution);
  initNeighborhoods();
}

void CollisionSpace::resetOccupationCells() {
  for (unsigned int i = 0; i < getNumberOfCells(); i++) {
    dynamic_cast<CollisionSpaceCell*>(_cells[i])->setOccupied(false);
  }
}

//! Updates the cells occupied by the robot
//! All objects composing the robot are processed
//! A list of cells for each body is added
void CollisionSpace::updateRobotOccupationCells(Robot* rob) {
  m_OccupationCells.clear();

  for (int i = 0; i < int(rob->getNumberOfJoints()); i++) {
    p3d_obj* obj =
        static_cast<p3d_jnt*>(rob->getJoint(i)->getP3dJointStruct())->o;

    if (obj) {
      std::vector<CollisionSpaceCell*> objectCell =
          getOccupiedCellsForObject(obj, rob->getJoint(i)->getMatrixPos());

      m_OccupationCells.insert(
          m_OccupationCells.end(), objectCell.begin(), objectCell.end());
    }
  }

  for (int i = 0; i < int(m_OccupationCells.size()); i++) {
    m_OccupationCells[i]->setOccupied(true);
  }
}

//! Return the occupied cells for a given object
//! Each point on the surface of the object leads to one cell
//! the cells allready added are marked as visied
std::vector<CollisionSpaceCell*> CollisionSpace::getOccupiedCellsForObject(
    p3d_obj* obj, const Eigen::Transform3d& T) {
  std::vector<CollisionSpaceCell*> objectCells;

  PointCloud& cloud = m_sampler->getPointCloud(obj);

  for (int i = 0; i < int(cloud.size()); i++) {
    CollisionSpaceCell* cell = dynamic_cast<CollisionSpaceCell*>(
        getCell(Eigen::Vector3d(T * cloud[i])));

    if (cell == NULL) continue;

    if (!cell->isVisited()) {
      cell->setVisited(true);
      objectCells.push_back(cell);
    }
  }

  for (int i = 0; i < int(objectCells.size()); i++)
    objectCells[i]->setVisited(false);

  return objectCells;
}

bool CollisionSpace::areCellsValid(
    const std::vector<CollisionSpaceCell*>& cells) {
  for (unsigned int i = 0; i < cells.size(); i++) {
    if (!cells[i]->isValid()) {
      return false;
    }
  }

  return true;
}

bool CollisionSpace::collisionCheck() {
  updateRobotOccupationCells(m_Robot);

  for (unsigned int i = 0; i < m_OccupationCells.size(); ++i) {
    if (!m_OccupationCells[i]->isValid()) {
      return true;
    }
  }

  return false;
}

void CollisionSpace::initNeighborhoods() {
  // first initialize the direction number mapping:
  m_DirectionNumberToDirection.resize(27);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        int direction_number = getDirectionNumber(dx, dy, dz);
        std::vector<int> n_point(3);
        n_point[0] = dx;
        n_point[1] = dy;
        n_point[2] = dz;
        m_DirectionNumberToDirection[direction_number] = n_point;
      }
    }
  }

  m_Neighborhoods.resize(2);
  for (int n = 0; n < 2; n++) {
    m_Neighborhoods[n].resize(27);
    // source directions
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx = -1; tdx <= 1; ++tdx) {
            for (int tdy = -1; tdy <= 1; ++tdy) {
              for (int tdz = -1; tdz <= 1; ++tdz) {
                if (tdx == 0 && tdy == 0 && tdz == 0) continue;
                if (n >= 1) {
                  if ((abs(tdx) + abs(tdy) + abs(tdz)) != 1) continue;
                  if (dx * tdx < 0 || dy * tdy < 0 || dz * tdz < 0) continue;
                }
                std::vector<int> n_point(3);
                n_point[0] = tdx;
                n_point[1] = tdy;
                n_point[2] = tdz;
                m_Neighborhoods[n][direction_number].push_back(n_point);
              }
            }
          }
        }
      }
    }
  }
}

double CollisionSpace::addPointsToField(
    const std::vector<Eigen::Vector3d>& points) {
  // cout << "Add points to distance field" << endl;
  // cout << "Propagate distance" << endl;

  propagate_negative_ = true;

  double max_distance_sq = getMaxtDistSq();

  cout << "max_distance_sq : " << max_distance_sq << endl;

  // initialize the bucket queue
  std::vector<std::vector<CollisionSpaceCell*> > bucket_queue;
  bucket_queue.resize(max_distance_sq + 1);
  bucket_queue[0].reserve(points.size());

  // negative stack (voxel that will be processed
  std::vector<Eigen::Vector3i> negative_stack;
  negative_stack.reserve(_nbCellsX * _nbCellsY * _nbCellsZ);

  std::vector<std::vector<CollisionSpaceCell*> > negative_bucket_queue;
  negative_bucket_queue.resize(max_distance_sq + 1);
  negative_bucket_queue[0].reserve(points.size());

  // first mark all the points as distance=0, and add them to the queue
  int initial_update_direction = getDirectionNumber(0, 0, 0);
  for (unsigned int i = 0; i < points.size(); ++i) {
    CollisionSpaceCell* voxel = getCellSpaceCell(points[i]);
    if (!voxel) continue;

    Eigen::Vector3i loc = getCellCoord(voxel);
    voxel->setOccupied(true);
    voxel->m_DistanceSquare = 0;
    voxel->m_ClosestPoint = loc;
    voxel->m_UpdateDirection = initial_update_direction;
    bucket_queue[0].push_back(voxel);

    // TODO check if this works
    //    if (propagate_negative_) {
    //      voxel->negative_distance_square_ = max_distance_sq;
    //      voxel->negative_update_direction_ = initial_update_direction;
    //      voxel->closest_negative_point_.x() =
    //      CollisionSpaceCell::UNINITIALIZED;
    //      voxel->closest_negative_point_.y() =
    //      CollisionSpaceCell::UNINITIALIZED;
    //      voxel->closest_negative_point_.z() =
    //      CollisionSpaceCell::UNINITIALIZED;
    //      negative_stack.push_back(loc);
    //    }
  }

  // Add points inside obstacle geometry
  if (m_Robot->getName() != "PR2_ROBOT") {
    for (unsigned int i = 0; i < _cells.size(); i++) {
      CollisionSpaceCell* voxel = dynamic_cast<CollisionSpaceCell*>(_cells[i]);
      Eigen::Vector3d center = voxel->getCenter();
      if (m_sampler->isPointInsidePrimitves(center)) {
        // Get location of the voxel
        Eigen::Vector3i loc = getCellCoord(voxel);

        voxel->setOccupied(true);
        voxel->m_DistanceSquare = 0;
        voxel->m_ClosestPoint = loc;
        voxel->m_UpdateDirection = initial_update_direction;

        if (propagate_negative_) {
          voxel->negative_distance_square_ = max_distance_sq;
          voxel->negative_update_direction_ = initial_update_direction;
          voxel->closest_negative_point_.x() =
              CollisionSpaceCell::UNINITIALIZED;
          voxel->closest_negative_point_.y() =
              CollisionSpaceCell::UNINITIALIZED;
          voxel->closest_negative_point_.z() =
              CollisionSpaceCell::UNINITIALIZED;
          negative_stack.push_back(loc);
        }
      }
    }
  }
  propagatePositive(bucket_queue);

  if (propagate_negative_) {
    while (!negative_stack.empty()) {
      Eigen::Vector3i loc = negative_stack.back();
      negative_stack.pop_back();

      // select the neighborhood list based on the update direction
      std::vector<std::vector<int> >& neighborhood =
          m_DirectionNumberToDirection;

      // Look in the neighbouring cells and update distance
      for (unsigned int n = 0; n < neighborhood.size(); n++) {
        Eigen::Vector3i direction;
        direction[0] = neighborhood[n][0];
        direction[1] = neighborhood[n][1];
        direction[2] = neighborhood[n][2];

        Eigen::Vector3i nloc = loc + direction;
        CollisionSpaceCell* neighbour = getCellSpaceCell(nloc);
        if (!neighbour) continue;

        Eigen::Vector3i& close_point = neighbour->closest_negative_point_;
        CollisionSpaceCell* closest_point_voxel = getCellSpaceCell(close_point);
        if (!closest_point_voxel) {
          closest_point_voxel = neighbour;
        }

        // our closest non-obstacle cell has become an obstacle
        if (closest_point_voxel->negative_distance_square_ != 0) {
          // find all neigbors inside pre-existing obstacles whose
          // closest_negative_point_ is now an obstacle. These must all be
          // set to max_distance_sq_ so they will be re-propogated with a new
          // closest_negative_point_ that is outside the obstacle.
          if (neighbour->negative_distance_square_ != max_distance_sq) {
            neighbour->negative_distance_square_ = max_distance_sq;
            neighbour->closest_negative_point_.x() =
                CollisionSpaceCell::UNINITIALIZED;
            neighbour->closest_negative_point_.y() =
                CollisionSpaceCell::UNINITIALIZED;
            neighbour->closest_negative_point_.z() =
                CollisionSpaceCell::UNINITIALIZED;
            negative_stack.push_back(nloc);
          }
        } else {
          // this cell still has a valid non-obstacle cell,
          // so we need to propogate from it
          neighbour->negative_update_direction_ = initial_update_direction;
          negative_bucket_queue[0].push_back(neighbour);
        }
      }
    }
    propagateNegative(negative_bucket_queue);
  }

  // cout << "All points have been added!!!" << endl;
  return 0.0;
}

void CollisionSpace::propagatePositive(
    std::vector<std::vector<CollisionSpaceCell*> >& bucket_queue) {
  double max_distance_sq = getMaxtDistSq();

  // now process the queue:
  cout << "process the Queue : " << bucket_queue.size() << endl;
  for (unsigned int i = 0; i < bucket_queue.size(); ++i) {
    std::vector<CollisionSpaceCell*>::iterator list_it =
        bucket_queue[i].begin();

    while (list_it != bucket_queue[i].end()) {
      CollisionSpaceCell* vptr = *list_it;

      // TODO: no idea why this happens
      if (vptr == NULL) {
        continue;
      }

      // Get the cell location in grid
      Eigen::Vector3i loc = getCellCoord(vptr);

      int D = i;
      if (D > 1) D = 1;
      // avoid a possible segfault situation:
      if (vptr->m_UpdateDirection < 0 || vptr->m_UpdateDirection > 26) {
        cout << "Invalid update direction detected: " << vptr->m_UpdateDirection
             << endl;

        ++list_it;
        continue;
      }

      // select the neighborhood list based on the update direction:
      std::vector<std::vector<int> >& neighborhood =
          m_Neighborhoods[D][vptr->m_UpdateDirection];

      // Look in the neighbouring cells and update distance
      for (unsigned int n = 0; n < neighborhood.size(); n++) {
        Eigen::Vector3i direction;
        direction.x() = neighborhood[n][0];
        direction.y() = neighborhood[n][1];
        direction.z() = neighborhood[n][2];

        Eigen::Vector3i neigh_loc = loc + direction;
        CollisionSpaceCell* neighbour = getCellSpaceCell(neigh_loc);
        if (!neighbour) continue;

        double new_distance_sq_float =
            (vptr->m_ClosestPoint - neigh_loc).squaredNorm();

        int new_distance_sq = new_distance_sq_float;
        if (new_distance_sq > max_distance_sq) {
          cout << "new_distance_sq : " << new_distance_sq << endl;
          continue;
        }

        if (new_distance_sq < neighbour->m_DistanceSquare) {
          neighbour->m_DistanceSquare = new_distance_sq;
          neighbour->m_ClosestPoint = vptr->m_ClosestPoint;
          neighbour->m_UpdateDirection =
              getDirectionNumber(direction[0], direction[1], direction[2]);

          // and put it in the queue
          bucket_queue[new_distance_sq].push_back(neighbour);
        }
      }
      ++list_it;
    }
    bucket_queue[i].clear();
  }
}

void CollisionSpace::propagateNegative(
    std::vector<std::vector<CollisionSpaceCell*> >& bucket_queue) {
  double max_distance_sq = getMaxtDistSq();

  // now process the queue:
  cout << "process negative Queue : " << bucket_queue.size() << endl;
  for (unsigned int i = 0; i < bucket_queue.size(); ++i) {
    std::vector<CollisionSpaceCell*>::iterator list_it =
        bucket_queue[i].begin();

    while (list_it != bucket_queue[i].end()) {
      CollisionSpaceCell* vptr = *list_it;

      // TODO: no idea why this happens
      if (vptr == NULL) {
        continue;
      }

      // Get the cell location in grid
      Eigen::Vector3i loc = getCellCoord(vptr);

      int D = i;
      if (D > 1) D = 1;
      // avoid a possible segfault situation:
      if (vptr->negative_update_direction_ < 0 ||
          vptr->negative_update_direction_ > 26) {
        cout << "Invalid update direction detected: "
             << vptr->negative_update_direction_ << endl;

        ++list_it;
        continue;
      }

      // select the neighborhood list based on the update direction:
      std::vector<std::vector<int> >& neighborhood =
          m_Neighborhoods[D][vptr->negative_update_direction_];

      // Look in the neighbouring cells and update distance
      for (unsigned int n = 0; n < neighborhood.size(); n++) {
        Eigen::Vector3i direction;
        direction[0] = neighborhood[n][0];
        direction[1] = neighborhood[n][1];
        direction[2] = neighborhood[n][2];

        Eigen::Vector3i neigh_loc = loc + direction;
        CollisionSpaceCell* neighbour = getCellSpaceCell(neigh_loc);
        if (!neighbour) continue;

        double new_distance_sq_float =
            (vptr->closest_negative_point_ - neigh_loc).squaredNorm();

        int new_distance_sq = new_distance_sq_float;
        if (new_distance_sq > max_distance_sq) {
          cout << "new_distance_sq : " << new_distance_sq << endl;
          continue;
        }

        if (new_distance_sq < neighbour->negative_distance_square_) {
          neighbour->negative_distance_square_ = new_distance_sq;
          neighbour->closest_negative_point_ = vptr->closest_negative_point_;
          neighbour->negative_update_direction_ =
              getDirectionNumber(direction[0], direction[1], direction[2]);

          // and put it in the queue:
          bucket_queue[new_distance_sq].push_back(neighbour);
        }
      }
      ++list_it;
    }
    bucket_queue[i].clear();
  }
}

void CollisionSpace::propagateDistance() {
  addPointsToField(m_points_to_add);
  resetPoints();
}

void CollisionSpace::resetPoints() { m_points_to_add.clear(); }

void CollisionSpace::addRobotBody(Joint* jnt) {
  p3d_obj* obj = static_cast<p3d_jnt*>(jnt->getP3dJointStruct())->o;

  if (obj == NULL) return;

  PointCloud& cloud = m_sampler->getPointCloud(obj);

  for (unsigned int j = 0; j < cloud.size(); j++)
    m_points_to_add.push_back(jnt->getMatrixPos() * cloud[j]);
}

void CollisionSpace::addRobot(Robot* rob) {
  for (unsigned int j = 0; j < rob->getNumberOfJoints(); j++) {
    addRobotBody(rob->getJoint(j));
  }
}

void CollisionSpace::addEnvPoints() {
  // Add Static Obstacles
  for (int i = 0; i < XYZ_ENV->no; i++) {
    PointCloud& cloud = m_sampler->getPointCloud(XYZ_ENV->o[i]);

    for (unsigned int j = 0; j < cloud.size(); j++)
      m_points_to_add.push_back(cloud[j]);
  }

  // Add Moving Obstacles
  Scene* sc = global_Project->getActiveScene();

  for (unsigned int i = 0; i < sc->getNumberOfRobots(); i++) {
    Robot* mov_obst = sc->getRobot(i);

    // The robot is not a robot or a human
    if (!((mov_obst->getName().find("ROBOT") != std::string::npos) ||
          (mov_obst->getName().find("HUMAN") != std::string::npos) ||
          mov_obst->getName() == "rob1" || mov_obst->getName() == "rob2" ||
          mov_obst->getName() == "rob3" || mov_obst->getName() == "rob4")) {
      cout << "Adding : " << mov_obst->getName() << endl;

      Joint* jnt = mov_obst->getJoint(1);

      if (static_cast<p3d_jnt*>(jnt->getP3dJointStruct())->o == NULL) continue;

      PointCloud& cloud = m_sampler->getPointCloud(
          static_cast<p3d_jnt*>(jnt->getP3dJointStruct())->o);

      for (int j = 0; j < int(cloud.size()); j++) {
        m_points_to_add.push_back(jnt->getMatrixPos() * cloud[j]);
      }
    }
  }
}

int CollisionSpace::getDirectionNumber(int dx, int dy, int dz) const {
  return (dx + 1) * 9 + (dy + 1) * 3 + dz + 1;
}

double CollisionSpace::getDistance(CollisionSpaceCell* cell) const {
  if (cell->m_DistanceSquare >= (int)m_SqrtTable.size()) {
    return 0;
  }
  return m_SqrtTable[cell->m_DistanceSquare] -
         m_SqrtTable[cell->negative_distance_square_];
}

double CollisionSpace::getDistanceFromCell(int x, int y, int z) const {
  return getDistance(static_cast<CollisionSpaceCell*>(
      _cells[x + y * _nbCellsX + z * _nbCellsX * _nbCellsY]));
}

CollisionSpaceCell* CollisionSpace::getCellSpaceCell(
    const Eigen::Vector3d& point) const {
  ThreeDCell* ptrCell = getCell(point);
  CollisionSpaceCell* cell = NULL;
  if (ptrCell) {
    cell = dynamic_cast<CollisionSpaceCell*>(ptrCell);
  }
  return cell;
}

CollisionSpaceCell* CollisionSpace::getCellSpaceCell(
    const Eigen::Vector3i& loc) const {
  ThreeDCell* ptrCell = getCell(loc);
  CollisionSpaceCell* cell = NULL;
  if (ptrCell) {
    cell = dynamic_cast<CollisionSpaceCell*>(ptrCell);
  }
  return cell;
}

//! Computes the finiate differenting gradient
//!
double CollisionSpace::getDistanceGradient(const Eigen::Vector3d& point,
                                           Eigen::Vector3d& gradient) const {
  CollisionSpaceCell* cell = static_cast<CollisionSpaceCell*>(getCell(point));

  if (!cell) {
    gradient = Eigen::Vector3d::Zero();
    return 0;
  }

  const Eigen::Vector3i& loc = cell->getLocation();

  unsigned int gx = loc(0);
  unsigned int gy = loc(1);
  unsigned int gz = loc(2);

  // if out of bounds, return 0 distance, and 0 gradient
  // we need extra padding of 1 to get gradients
  if (gx < 1 || gy < 1 || gz < 1 || gx >= _nbCellsX - 1 ||
      gy >= _nbCellsY - 1 || gz >= _nbCellsZ - 1) {
    gradient = Eigen::Vector3d::Zero();
    return 0;
  }

  gradient(0) = (getDistanceFromCell(gx + 1, gy, gz) -
                 getDistanceFromCell(gx - 1, gy, gz)) *
                m_invTwiceResolution;
  gradient(1) = (getDistanceFromCell(gx, gy + 1, gz) -
                 getDistanceFromCell(gx, gy - 1, gz)) *
                m_invTwiceResolution;
  gradient(2) = (getDistanceFromCell(gx, gy, gz + 1) -
                 getDistanceFromCell(gx, gy, gz - 1)) *
                m_invTwiceResolution;

  // return getDistanceFromCell(gx,gy,gz);
  return getDistance(cell);
}

//! The distance potential and the gradient is computed for a collision point
//! First the distance d of the collision point (sphere) to the closest
// obstacle
// is computed
//! Then 3 cases apear to compute the potential
//! - 0 if this distance is greater than the coll point clearance
//! -
bool CollisionSpace::getCollisionPointPotentialGradient(
    const CollisionPoint& collision_point,
    const Eigen::Vector3d& collision_point_pos,
    double& field_distance,
    double& potential,
    Eigen::Vector3d& gradient) const {
  Eigen::Vector3d field_gradient;

  // Compute the distance gradient and distance to nearest obstacle
  field_distance =
      this->getDistanceGradient(collision_point_pos, field_gradient);

  double d = field_distance - collision_point.getRadius();

  // three cases below:
  if (d >= collision_point.getClearance()) {
    potential = 0.0;
    gradient.setZero();
  } else if (d >= 0.0) {
    double diff = (d - collision_point.getClearance());
    potential = 0.5 * collision_point.getInvClearance() * diff * diff;
    gradient = diff * collision_point.getInvClearance() * field_gradient;
  } else  // if d < 0.0
  {
    gradient = field_gradient;
    double offset = 0.5 * collision_point.getClearance();
    potential = -d * 10 + offset;
    // potential = -d + 0.5 * collision_point.getClearance();
  }

  //    cout << "field_distance : " << field_distance
  // << ", radius : "  << collision_point.getRadius() << endl;

  // true if point is in collision
  return (field_distance <= collision_point.getRadius());
}

//! Goes through all points and computes whether the robot is colliding
//! Computes the transform of all point from the joint transform
//! Uses the potential gradient function
bool CollisionSpace::isRobotColliding(double& dist, double& pot) const {
  // cout << "Test collision space is robot colliding" << endl;
  bool isRobotColliding = false;

  double potential = 0.0;
  double max_potential = -std::numeric_limits<double>::max();

  double distance = 0.0;
  double min_distance = std::numeric_limits<double>::max();

  Eigen::Vector3d position, gradient;

  for (unsigned int i = 0; i < m_Robot->getNumberOfJoints(); i++) {
    Joint* jnt = m_Robot->getJoint(i);

    std::vector<CollisionPoint>& points = m_sampler->getCollisionPoints(jnt);
    if (points.empty()) continue;

    Eigen::Transform3d T = jnt->getMatrixPos();

    for (unsigned int j = 0; j < points.size(); j++) {
      position = T * points[j].getPosition();

      // CollisionSpaceCell* cell = static_cast<CollisionSpaceCell*>( getCell(
      // position ) );
      // if( ( cell != NULL ? getDistance( cell ) : 0 ) <=
      // points[j].getRadius()
      // )

      if (getCollisionPointPotentialGradient(
              points[j], position, distance, potential, gradient)) {
        points[j].m_is_colliding = true;

        //cout << "point[" << j
         //    << "] in joint is in collision: " << points[j].joint()->getName()
         //    << endl;

        // Hack!!! to exclude certain points
        if (points[j].getSegmentNumber() > 1) {
          // cout << "points[" << j << "].getSegmentNumber()" << endl;
          isRobotColliding = true;
        }
      } else {
        points[j].m_is_colliding = false;
      }

      if (max_potential < potential) max_potential = potential;

      if (min_distance > distance) min_distance = distance;
    }
  }

  pot = max_potential;
  dist = min_distance;
  return isRobotColliding;
}

double CollisionSpace::cost(const Configuration& q) const {
  // bool colliding = false;
  double distance = 0.0;
  double potential = 0.0;

  //    colliding =
  //    isRobotColliding( distance, potential );

  double cost = 0.0;

  Eigen::Vector3d position, gradient;

  for (unsigned int i = 0; i < m_Robot->getNumberOfJoints(); i++) {
    Joint* jnt = m_Robot->getJoint(i);

    std::vector<CollisionPoint>& points = m_sampler->getCollisionPoints(jnt);
    if (points.empty()) continue;

    Eigen::Transform3d T = jnt->getMatrixPos();

    for (unsigned int j = 0; j < points.size(); j++) {
      position = T * points[j].getPosition();
      getCollisionPointPotentialGradient(
          points[j], position, distance, potential, gradient);
      if (potential < EPS6) {
        potential = EPS6;
      }
      cost += potential;
    }
  }

  return 10 * cost;
}

//! Draws a sphere in all voxels
//! the color of the sphere represent the distance
//! to the nearest obstacle
void CollisionSpace::drawStaticVoxels() {
  // Draw all cells
  for (unsigned int i = 0; i < getNumberOfCells(); i++) {
    static_cast<CollisionSpaceCell*>(_cells[i])->drawStatic();
  }
}

void CollisionSpace::drawGradient() {
  //  Eigen::Vector3d unitX(1, 0, 0);
  //  Eigen::Vector3d unitY(0, 1, 0);
  //  Eigen::Vector3d unitZ(0, 0, 1);

  for (unsigned int n = 0; n < getNumberOfCells(); n++) {
    CollisionSpaceCell* cell = static_cast<CollisionSpaceCell*>(_cells[n]);

    if (getDistance(cell) > PlanEnv->getDouble(PlanParam::distMinToDraw))
      continue;

    Eigen::Vector3d point = cell->getCenter();
    Eigen::Vector3d gradient;

    bool isGradientValid = getDistanceGradient(point, gradient);
    if (!isGradientValid) continue;

    if (gradient != Eigen::Vector3d::Zero()) {
      // cout << "gradient = " << endl << gradient << endl;
    }

    /**
 Eigen::Vector3d axis = //gradient.cross(unitX).length() > 0 ?
 gradient.cross(Eigen::Vector3d::UnitX()); // : unitY;
 double angle = -gradient.dot(Eigen::Vector3d::UnitX());

 Eigen::Transform3d t = Eigen::AngleAxisd(angle,axis) *
 Eigen::Translation3d(point);

 p3d_matrix4 mat;
 for (unsigned int i=0; i<4; i++) {
 for (unsigned int j=0;j<4; j++) {
 mat[i][j] =  t(i,j);
 //cout << t.matrix() << endl;
 }
 }
 g3d_draw_frame(mat, 0.10);
 */

    p3d_vector3 p1;
    Eigen::Vector3d p1Eigen = cell->getCenter();

    p1[0] = p1Eigen(0);
    p1[1] = p1Eigen(1);
    p1[2] = p1Eigen(2);

    p3d_vector3 p2;
    Eigen::Vector3d p2Eigen =
        2 * cell->getCellSize().minCoeff() * gradient + p1Eigen;

    p2[0] = p2Eigen(0);
    p2[1] = p2Eigen(1);
    p2[2] = p2Eigen(2);

    g3d_draw_arrow(p1, p2, 0.1, 0.7, 0.0);
  }
}

void CollisionSpace::drawSquaredDist() {
  double distToClosObst;
  CollisionSpaceCell* cell;
  double min_dist = std::numeric_limits<double>::max();
  double draw_min_dist = -0.15;
  //  double draw_min_dist = -1.; // plannar manipulator

  for (unsigned int i = 0; i < getNumberOfCells(); i++) {
    cell = static_cast<CollisionSpaceCell*>(_cells[i]);

    distToClosObst = getDistance(cell);

    if (distToClosObst > PlanEnv->getDouble(PlanParam::distMinToDraw)) {
      continue;
    }

    // Draws the cell at a particular height in the grid
    // * plannar manipulator 10
    // * human trajectories 45
    // if (cell->getLocation().z() != 42) {
    //  continue;
    //}

    if (min_dist > distToClosObst) {
      min_dist = distToClosObst;
    }

    cell->drawColorGradient(distToClosObst,
                            draw_min_dist,
                            PlanEnv->getDouble(PlanParam::distMinToDraw),
                            true);
  }
}

void CollisionSpace::drawCollisionPoints() {
  for (unsigned int i = 0; i < m_Robot->getNumberOfJoints(); i++) {
    Joint* jnt = m_Robot->getJoint(i);
    Eigen::Transform3d T = jnt->getMatrixPos();
    std::vector<CollisionPoint>& points = m_sampler->getCollisionPoints(jnt);

    for (unsigned int j = 0; j < points.size(); j++) {
      // if (points[j].getSegmentNumber() <ENV.getDouble(Env::extensionStep))
      //    continue;

      if (points[j].m_is_colliding) {
        double color[4];

        color[1] = 0.0;  // green
        color[2] = 0.0;  // blue
        color[0] = 1.0;  // red
        color[3] = 0.7;  // transparency

        g3d_set_color(Any, color);
      }

      bool yellow = (!points[j].m_is_colliding);

      points[j].draw(T, yellow);
    }
  }
}

void CollisionSpace::draw() {
  // cout << "Draw CollisionSpace" << endl;
  resetOccupationCells();

  updateRobotOccupationCells(m_Robot);

  // For all robot in the scene
  //  Scene* sc = global_Project->getActiveScene();
  //  for (unsigned int i=0; i< sc->getNumberOfRobots(); i++)
  //  {
  //    updateRobotOccupationCells( sc->getRobot(i) );
  //  }

  //  updateRobotOccupationCells( sc->getRobotByNameContaining("LOTR") );

  // Draw all cells
  for (unsigned int i = 0; i < _cells.size(); i++) {
    static_cast<CollisionSpaceCell*>(_cells[i])->draw();
  }
}

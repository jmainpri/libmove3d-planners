/*
 *  CollisionSpace.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef GRID_COLLISION_CHECKER_H_
#define GRID_COLLISION_CHECKER_H_

#include "API/Grids/gridsAPI.hpp"

#include "collision_space/body_surface_sampler.hpp"
#include "collision_space/collision_point.hpp"

#ifndef _P3D_H
typedef struct obj;
#endif

namespace Move3D {

class CollisionSpaceCell : public Move3D::ThreeDCell {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionSpaceCell(int i,
                     const Eigen::Vector3d& corner,
                     Move3D::ThreeDGrid* grid);

  // setters and getters
  inline bool isValid(void) { return m_Valid; }
  inline void setValid(bool value) { m_Valid = value; }

  inline bool isVisited(void) { return m_Visited; }
  inline void setVisited(bool value) { m_Visited = value; }

  inline bool isOccupied(void) { return m_Occupied; }
  inline void setOccupied(bool value) { m_Occupied = value; }

  const Eigen::Vector3i& getLocation() { return m_Location; }

  void drawStatic();
  void draw(int color, int width);
  void draw(void);

  int m_DistanceSquare;  /**< Squared distance from the closest obstacle */
  int m_UpdateDirection; /**< Direction from which this voxel was updated */
  Eigen::Vector3i m_ClosestPoint; /**< Closes obstacle from this voxel */

 private:
  bool m_Valid;  // There is no static obstacles crossing this cell
  bool m_Visited;
  bool m_Occupied;

  Eigen::Vector3d _cellSize;
  Eigen::Vector3i m_Location; /**< Place in the grid */
};

class CollisionSpace : public Move3D::ThreeDGrid {
 public:
  // constructors and destructors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionSpace(Robot* rob, double pace, const std::vector<double>& env_size);
  virtual ~CollisionSpace();

  // returns the robot associated with the collisionspace
  Robot* getRobot() { return m_Robot; }

  // the sampler on body and obstacles with all points
  BodySurfaceSampler* getBodySampler() { return m_sampler; }

  // functions
  void initialize();

  // ---------------------------------------------------------------
  // point cloud collision checker
  // ---------------------------------------------------------------
  std::vector<CollisionSpaceCell*> getOccupiedCellsForObject(
      obj* obj, const Eigen::Transform3d& T);
  const std::vector<CollisionSpaceCell*>& getOccupiedCells() const {
    return m_OccupationCells;
  }
  std::vector<CollisionSpaceCell*> getFreeCells() const {
    std::vector<CollisionSpaceCell*> free_cells;
    for (int i = 0; i < _cells.size(); i++) {
      if (std::find(m_OccupationCells.begin(),
                    m_OccupationCells.end(),
                    static_cast<CollisionSpaceCell*>(_cells[i])) !=
          m_OccupationCells.end()) {
        free_cells.push_back(static_cast<CollisionSpaceCell*>(_cells[i]));
      }
    }
    return free_cells;
  }
  void resetOccupationCells();
  void updateRobotOccupationCells(Robot* rob);
  bool areCellsValid(const std::vector<CollisionSpaceCell*>& cells);
  bool collisionCheck();

  // ---------------------------------------------------------------
  // Distance field
  // ---------------------------------------------------------------
  void addRobotBody(Joint* jnt);
  void addRobot(Robot* rob);
  void addEnvPoints();
  void propagateDistance();
  void resetPoints();

  void initNeighborhoods();
  int getDirectionNumber(int dx, int dy, int dz) const;
  double getDistanceGradient(const Eigen::Vector3d& point,
                             Eigen::Vector3d& gradient) const;
  double getDistance(CollisionSpaceCell* cell) const;
  bool getCollisionPointPotentialGradient(
      const CollisionPoint& collision_point,
      const Eigen::Vector3d& collision_point_pos,
      double& distance,
      double& potential,
      Eigen::Vector3d& gradient) const;

  bool isRobotColliding(double& distance, double& potential) const;
  double cost(const Configuration& q) const;

  // ---------------------------------------------------------------
  // OpenGl display
  // ---------------------------------------------------------------
  void drawSquaredDist();
  void drawGradient();
  void drawStaticVoxels();
  void drawCollisionPoints();
  void draw();

 protected:
  Move3D::ThreeDCell* createNewCell(unsigned int index,
                                    unsigned int x,
                                    unsigned int y,
                                    unsigned int z);

 private:
  double addPointsToField(const std::vector<Eigen::Vector3d>& points);

  double getDistanceFromCell(int x, int y, int z) const;

  // The position of the origin of the grid regarding th eorigin of the world
  // int m_nbMaxCells; //the number of cell along the longest axis of the
  // environment

  Robot* m_Robot;

  // Call add all point to add this vector
  std::vector<Eigen::Vector3d> m_points_to_add;

  std::vector<CollisionSpaceCell*> m_OccupationCells;

  double m_size;
  double m_invTwiceResolution;

  BodySurfaceSampler* m_sampler;

  double m_MaxClearance;

  // double a__,b__,c__; //,e__,f__,g__;

  // The variables for distance field
  // Implementation
  std::vector<double> m_SqrtTable;
  std::vector<std::vector<std::vector<std::vector<int> > > > m_Neighborhoods;
  std::vector<std::vector<int> > m_DirectionNumberToDirection;
};

extern CollisionSpace* global_collisionSpace;
}

#endif

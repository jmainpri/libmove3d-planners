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
//#include "API/ConfigSpace/localpath.hpp"

#include "planner/Greedy/BodySurfaceSampler.hpp"
#include "planner/Greedy/CollisionPoint.hpp"

//#include <vector>

#ifndef _P3D_H
typedef struct obj;
#endif

class CollisionSpaceCell;

class CollisionSpace : public API::ThreeDGrid
{
public:
  //constructors and destructors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  CollisionSpace();
  virtual ~CollisionSpace();
	
  //setters and getters
  inline int getNbCellsOverX(void){return _nbCellsX;}
  inline int getNbCellsOverY(void){return _nbCellsY;}
  inline int getNbCellsOverZ(void){return _nbCellsZ;}
  
  //the sampler on body and obstacles with all points
  BodySurfaceSampler* getBodySampler() { return m_sampler; }
	
  //functions
  void init(void);
	
  // ---------------------------------------------------------------
  // point cloud collision checker
  // ---------------------------------------------------------------
	std::vector<CollisionSpaceCell*> getCellListForObject(obj* obj, const Eigen::Transform3d& Trans);
	std::vector<CollisionSpaceCell*> getOccupiedCells() { return m_OccupationCells; }
  void resetOccupationCells();
  void updateRobotOccupationCells(Robot* rob);
  
	//std::vector<CollisionSpaceCell*> computeOccupiedCells(LocalPath& path);
	bool areCellsValid(const std::vector<CollisionSpaceCell*>& cells);
	
  void unvalidObjectCells(obj* obj);
  bool collisionCheck();
  
  // ---------------------------------------------------------------
  // Distance field
  // ---------------------------------------------------------------
  void initNeighborhoods();
  
  void addAllPointsToField();
  double addPointsToField(const std::vector<Eigen::Vector3d>& points);

  int getDirectionNumber(int dx, int dy, int dz) const;
  
  double getDistanceGradient(const Eigen::Vector3d& point,Eigen::Vector3d& gradient) const;
  double getDistance(CollisionSpaceCell* cell) const;
  
  bool getCollisionPointPotentialGradient(const CollisionPoint& collision_point, 
                                          const Eigen::Vector3d& collision_point_pos,
                                          double& potential, 
                                          Eigen::Vector3d& gradient) const;
  
  // ---------------------------------------------------------------
  // OpenGl display
  // ---------------------------------------------------------------
  void drawSquaredDist();
  void drawGradient();
  void drawStaticVoxels();
  void draw();
  
protected:
	API::ThreeDCell* createNewCell(unsigned int index,
                                 unsigned  int x,unsigned  int y,unsigned  int z );
	
private:
  
  double getDistanceFromCell(int x, int y, int z) const;
  
  //The position of the origin of the grid regarding th eorigin of the world
  int m_nbMaxCells; //the number of cell along the longest axis of the environment
	
  Robot* m_Robot;	
  
	std::vector<CollisionSpaceCell*>  m_OccupationCells;
  
  double m_size;
  double m_invTwiceResolution;
	
	BodySurfaceSampler* m_sampler;
  
  //double a__,b__,c__; //,e__,f__,g__;
  
  // The variables for distance field 
  // Implementation
  std::vector<double>               m_SqrtTable;
  std::vector<std::vector<std::vector<std::vector<int> > > > m_Neighborhoods;
  std::vector<std::vector<int> > m_DirectionNumberToDirection;
};

extern CollisionSpace* global_CollisionSpace;

#endif

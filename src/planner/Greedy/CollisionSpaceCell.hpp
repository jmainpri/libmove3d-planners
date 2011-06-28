/*
 *  CollisionSpaceCell.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef CELL_COLLISION_CHECKER_H_
#define CELL_COLLISION_CHECKER_H_

#include "API/planningAPI.hpp"
#include "API/Grids/gridsAPI.hpp"

class CollisionSpaceCell : public API::ThreeDCell
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  CollisionSpaceCell(int i, const Eigen::Vector3d& corner, API::ThreeDGrid* grid);
	
  //setters and getters
  inline bool isValid(void){return m_Valid;}
  inline void setValid(bool value){m_Valid = value;}
  
  inline bool isVisited(void){return m_Visited;}
  inline void setVisited(bool value){m_Visited = value;}
  
	inline bool isOccupied(void) {return m_Occupied;}
	inline void setOccupied(bool value) {m_Occupied = value;}
  
  void drawStatic();
  void draw(int color, int width);
  void draw(void);
  
  int                   m_DistanceSquare;         /**< Squared distance from the closest obstacle */
  int                   m_UpdateDirection;        /**< Direction from which this voxel was updated */
  Eigen::Vector3i       m_ClosestPoint;           /**< Closes obstacle from this voxel */
  
private:
  
  std::vector<Edge*> _edges;
  std::vector<Node*> _nodes;
	
  bool m_Valid; //There is no static obstacles crossing this cell
  bool m_Visited;
	bool m_Occupied;
	
  Eigen::Vector3d _cellSize;
};

#endif
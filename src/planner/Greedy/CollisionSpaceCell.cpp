/*
 *  CollisionSpaceCell.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CollisionSpaceCell.hpp"

#include "Graphic-pkg.h"

using namespace std;

CollisionSpaceCell::CollisionSpaceCell(int i, const Eigen::Vector3d& corner, API::ThreeDGrid* grid) : 
API::ThreeDCell(i, corner, grid)
{
	m_Valid = true;
	m_Visited = false;
	m_Occupied = false;
  
  m_ClosestPoint[0] = -1;
  m_ClosestPoint[1] = -1;
  m_ClosestPoint[2] = -1;
  
	_edges.clear();
	_nodes.clear();
	_cellSize = grid->getCellSize();
  
  m_DistanceSquare = 10.0;
}

void CollisionSpaceCell::draw(int color, int width)
{
	Eigen::Vector3d corner = getCorner();
	
	double xmin = corner[0];
	double xmax = corner[0] + _cellSize[0];
	double ymin = corner[1];
	double ymax = corner[1] + _cellSize[1];
	double zmin = corner[2];
	double zmax = corner[2] + _cellSize[2];
	
  //cout << "Draw CollisionSpaceCell ( " << color << " )" << endl;
	g3d_draw_simple_box(xmin, xmax, ymin, ymax, zmin, zmax, color, 0, width);
}

void CollisionSpaceCell::drawStatic()
{
  if( !this->m_Valid )
	{
    draw(Red, 1);
  }
}

void CollisionSpaceCell::draw()
{
	if( !this->m_Valid )
	{
		//drawStatic()
	}
	else if (m_Occupied)
	{
		draw(Green,1);
	}
}

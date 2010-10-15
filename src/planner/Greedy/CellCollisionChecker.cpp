/*
 *  CellCollisionChecker.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CellCollisionChecker.hpp"

#include "Graphic-pkg.h"

using namespace std;

CellCollisionChecker::CellCollisionChecker(int i, const Eigen::Vector3d& corner, API::ThreeDGrid* grid) : 
API::ThreeDCell(i, corner, grid)
{
	m_Valid = true;
	m_Visited = false;
	m_Occupied = false;
	
	_edges.clear();
	_nodes.clear();
	_cellSize = grid->getCellSize();
}

void CellCollisionChecker::draw(int color, int width)
{
	Eigen::Vector3d corner = getCorner();
	
	double xmin = corner[0];
	double xmax = corner[0] + _cellSize[0];
	double ymin = corner[1];
	double ymax = corner[1] + _cellSize[1];
	double zmin = corner[2];
	double zmax = corner[2] + _cellSize[2];
	
	g3d_draw_simple_box(xmin, xmax, ymin, ymax, zmin, zmax, color, 0, width);
}

void CellCollisionChecker::draw()
{
	if(!this->m_Valid)
	{
		draw(Red, 1);
	}
	else if (m_Occupied)
	{
		draw(Green,1);
	}

}
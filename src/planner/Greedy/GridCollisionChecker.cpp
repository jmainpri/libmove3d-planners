/*
 *  GridCollisionChecker.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 07/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "GridCollisionChecker.hpp"
#include "CellCollisionChecker.hpp"
#include "PointsOnBodies.hpp"

#include "P3d-pkg.h"

GridCollisionChecker* global_GridCollisionChecker = NULL;

using namespace std;
using namespace tr1;


GridCollisionChecker::GridCollisionChecker() : m_sampler(NULL)
{
	m_nbMaxCells = ENV.getInt(Env::nbCells);
	double cellSize = (XYZ_ENV->box.x2 - XYZ_ENV->box.x1);
	
	cellSize = MAX(XYZ_ENV->box.y2 - XYZ_ENV->box.y1, cellSize);
	cellSize = MAX(XYZ_ENV->box.z2 - XYZ_ENV->box.z1, cellSize);
	cellSize /= m_nbMaxCells;
	
	_originCorner[0] = XYZ_ENV->box.x1;
	_originCorner[1] = XYZ_ENV->box.y1;
	_originCorner[2] = XYZ_ENV->box.z1;
	
	_nbCellsX = (XYZ_ENV->box.x2 - XYZ_ENV->box.x1)/cellSize;
	_nbCellsY = (XYZ_ENV->box.y2 - XYZ_ENV->box.y1)/cellSize;
	_nbCellsZ = (XYZ_ENV->box.z2 - XYZ_ENV->box.z1)/cellSize;
	
	//_nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;
	_cellSize[0] = _cellSize[1] = _cellSize[2] = cellSize;
	
	this->createAllCells();
	
	cout << "Cell size(0) = " << _cellSize[0] << endl;
	cout << "Cell size(1) = " << _cellSize[1] << endl;
	cout << "Cell size(2) = " << _cellSize[2] << endl;
	
	cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
	
	//Build the meshes env edges
	if(XYZ_ENV->o[0]->pol[0]->poly->the_edges == NULL)
	{
		for(int i = 0; i < XYZ_ENV->no; i++)
		{
			p3d_obj * obj = XYZ_ENV->o[i];
			for(int j = 0; j < obj->np; j++)
			{
				poly_build_edges(obj->pol[j]->poly);
			}
		}
	}
	
	init();
	
	m_Robot = global_Project->getActiveScene()->getActiveRobot();
}

GridCollisionChecker::~GridCollisionChecker()
{
	
}

void GridCollisionChecker::init(void)
{
	m_sampler = new BodySurfaceSampler(_cellSize[0]*0.25);
	
	m_sampler->computeStaticObjectsPointCloud();
	m_sampler->computeAllRobotsBodiesPointCloud();
	
	//unvalid static object Cells
	for(int i = 0; i < XYZ_ENV->no; i++)
	{
		unvalidObjectCells(XYZ_ENV->o[i]);
	}
	//unvalid cells for each robot except current one ?
}

void GridCollisionChecker::updateRobotOccupationCells(Robot* myRobot)
{
	vector<CellCollisionChecker*> robotCell;
	
	for(unsigned int i = 0; i < getNumberOfCells(); i++)
	{
		dynamic_cast<CellCollisionChecker*>(_cells[i])->setOccupied(false);
	}
	
	for(unsigned int i = 0; i < myRobot->getNumberOfJoints(); i++)
	{
		p3d_obj* obj = myRobot->getJoint(i)->getJointStruct()->o;
		
		if(obj)
		{
			vector<CellCollisionChecker*> objectCell = getCellListForObject( 
																obj, 
																myRobot->getJoint(i)->getMatrixPos() );
			
			robotCell.insert(robotCell.end(), objectCell.begin(), objectCell.end());
		}
	}
	
	m_OccupationCells = robotCell;
	
//	MY_FREE(robot->dpgCells, DpgCell*,  robot->nbDpgCells);
//	robot->nbDpgCells = robotCell.size();
//	robot->dpgCells = MY_ALLOC(DpgCell* , robot->nbDpgCells);
	
	for(unsigned int i = 0; i < robotCell.size(); i++)
	{
		m_OccupationCells[i]->setOccupied(true);
	}
}

void GridCollisionChecker::unvalidObjectCells(p3d_obj* obj)
{
	Eigen::Transform3d Trans(Eigen::Matrix4d::Identity());
	
	vector<CellCollisionChecker*> cellTab = getCellListForObject(obj,Trans);
	
	for(unsigned int i = 0; i < cellTab.size(); i++)
	{
		cellTab[i]->setValid(false);
	}
}

vector<CellCollisionChecker*> GridCollisionChecker::getCellListForObject(p3d_obj* obj, const Eigen::Transform3d& Trans)
{
	vector<CellCollisionChecker*> objectCells;
#ifdef DPG
	for(unsigned int i = 0; i < obj->nbPointCloud; i++)
	{
		Eigen::Vector3d WSPoint;
		
		p3d_vector3 vect;
		
		p3d_vectCopy(obj->pointCloud[i],vect);
		
		WSPoint[0] = vect[0];
		WSPoint[1] = vect[1];
		WSPoint[2] = vect[2];
		
		WSPoint = Trans * WSPoint;

		CellCollisionChecker* cell = dynamic_cast<CellCollisionChecker*>(getCell(WSPoint));
		
		if( cell == NULL )
		{
			continue;
		}
		
		if(!cell->isVisited())
		{
			cell->setVisited(true);
			objectCells.push_back(cell);
		}
	}
#endif
	
	for(unsigned int i = 0; i < objectCells.size(); i++)
	{
		objectCells[i]->setVisited(false);
	}
	
	return objectCells;
}

vector<CellCollisionChecker*> GridCollisionChecker::computeOccupiedCells(LocalPath& path)
{
	double ParamMax = path.getParamMax();
	double Step = path.getResolution();
	double u = 0.0;
	
	set<CellCollisionChecker*> setCells;
	
	while( u < ParamMax ) 
	{
		shared_ptr<Configuration> q = path.configAtParam(u);
		
		if( m_Robot->setAndUpdate(*q) )
		{
			updateRobotOccupationCells(m_Robot);
			setCells.insert( m_OccupationCells.begin(), m_OccupationCells.end() );
		}
		u += Step;
	}
	
	
	vector<CellCollisionChecker*> vectCells;
	set<CellCollisionChecker*>::iterator it;
	
	for (it=setCells.begin(); it!=setCells.end(); it++)
	{
		vectCells.push_back(*it);
	}
	
	return vectCells;
}

bool GridCollisionChecker::areCellsValid(vector<CellCollisionChecker*> cells)
{
	for (unsigned int i=0; i<cells.size(); i++) 
	{
		if (!cells[i]->isValid()) 
		{
			return false;
		}
	}
	
	return true;
}

void GridCollisionChecker::draw()
{
	CellCollisionChecker* cell;
	
	updateRobotOccupationCells(m_Robot);
	
	for(unsigned int i=0; i < getNumberOfCells(); i++)
	{
		cell = static_cast<CellCollisionChecker*>(_cells[i]);
		cell->draw();
	}
}

//protected functions
/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param index the cell index
 * \param x The position of the cell over x
 * \param y The position of the cell over y
 * \param z The position of the cell over z
 */
API::ThreeDCell* GridCollisionChecker::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
{
    CellCollisionChecker* newCell = new CellCollisionChecker( index, computeCellCorner(x,y,z) , this );
    return newCell;
}

bool GridCollisionChecker::collisionCheck()
{
  updateRobotOccupationCells(m_Robot);

  for(unsigned int i=0; i<m_OccupationCells.size(); ++i)
  {
    if(!m_OccupationCells[i]->isValid())
    {  return true; }
  }

  return false;
}

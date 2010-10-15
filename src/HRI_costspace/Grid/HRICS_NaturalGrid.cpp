/*
 *  HRICS_NaturalGrid.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_NaturalCell.hpp"
#include "HRICS_NaturalCell.hpp"
#include "HRICS_Natural.hpp"

#include "gridsAPI.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

NaturalGrid::NaturalGrid() :
	API::ThreeDGrid(),
	m_firstDisplay(true)
{
	//setGridOrigin();	
}

NaturalGrid::NaturalGrid(vector<int> size) :
	m_firstDisplay(true)
{
	//setGridOrigin();
}

NaturalGrid::NaturalGrid(double pace, vector<double> envSize, Natural* costSpace) :
	API::ThreeDGrid(pace,envSize),
	m_NaturalCostSpace(costSpace),
	m_firstDisplay(true)
{
	setGridOrigin();
	
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
}

NaturalGrid::NaturalGrid(const NaturalGrid& grid) :
	API::ThreeDGrid(grid),
	m_NaturalCostSpace(grid.m_NaturalCostSpace),
	m_firstDisplay(true)
{
	setGridOrigin();
	
	for (unsigned int i=0; i<grid._cells.size() ; i++) 
	{
		NaturalCell* cell = dynamic_cast<NaturalCell*>( grid._cells[i]);
		
		NaturalCell* newCell = new NaturalCell( *cell );
		newCell->setIsReachable( cell->isReachable() );
		newCell->setIsReachableWithLA( cell->isReachableWithLA() );
		newCell->setIsReachableWithRA( cell->isReachableWithRA() );
		newCell->setGrid( this );
		
		_cells[i] = newCell;
	}
}


void NaturalGrid::setGridOrigin()
{
	m_RobotOriginPos = getNaturalCostSpace()->getGridOriginMatrix();
	cout << "m_RobotOriginPos = " << endl << m_RobotOriginPos.matrix() << endl;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
API::ThreeDCell* NaturalGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
{
    Vector3i pos;
	
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
	
    //cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
	
    if (index == 0)
    {
        return new NaturalCell( 0, pos ,_originCorner , this );
    }
    return new NaturalCell( index, pos , computeCellCorner(x,y,z) , this );
}

/*!
 * \brief Reset Grid Cost
 */
void NaturalGrid::resetCellCost()
{
    unsigned int nbCells = this->getNumberOfCells();
	
    for(unsigned int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( _cells[i] )->setBlankCost();
    }
}

/*!
 * \brief Reset Grid Reachability
 */
void NaturalGrid::resetReachability()
{
    unsigned int nbCells = this->getNumberOfCells();
	
    for(unsigned int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( _cells[i] )->resetReachable();
    }
}

/*!
 * \brief Compute Grid Cost
 */
void NaturalGrid::computeAllCellCost()
{
    int nbCells = this->getNumberOfCells();
    shared_ptr<Configuration> q = getRobot()->getCurrentPos();
	
    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( _cells[i] )->getCost();
    }
	
    getRobot()->setAndUpdate(*q);
    API_activeGrid = this;
}

/*!
 * Compute Grid Accecibility
 */
#ifdef HRI_GENERALIZED_IK
void NaturalGrid::computeReachability(bool LeftArm)
{
	int nbCells = this->getNumberOfCells();
    shared_ptr<Configuration> robotConf = getRobot()->getInitialPosition();
	
    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->computeReachability(LeftArm);
		getRobot()->setAndUpdate(*robotConf);
		cout << "Computing Reachability of Cell : " << i << endl;
    }
	
    API_activeGrid = this;
}
#endif

/*!
 * @breif Init Reach
 */
void NaturalGrid::initReachable()
{
    unsigned int nbCells = this->getNumberOfCells();
	
    for(unsigned int i=0; i<nbCells; i++)
    {
        NaturalCell* cell = dynamic_cast<NaturalCell*>( _cells[i] );
		double Cost = cell->getCost();
		if (Cost > 0) 
		{
			cell->setIsReachable(true);
			
			if (Cost == 100) 
			{
				cell->setIsReachableWithLA(true);
				//cell->setIsReachableWithRA(true);
			}
		}
    }
}

/*!
 * @breif Fusion Grid
 */
NaturalGrid* NaturalGrid::mergeWith(NaturalGrid* otherGrid)
{
	if (otherGrid->getNumberOfCells() != getNumberOfCells() ) 
	{
		cout << "Error in NaturalGrid::fusionWith" << endl;
		return NULL;
	}
	
	NaturalGrid* grid = new NaturalGrid(*this);
	
	for (unsigned int x=0; x<_nbCellsX; x++) 
	{
		for (unsigned int y=0; y<_nbCellsY; y++) 
		{
			for (unsigned int z=0; z<_nbCellsZ; z++) 
			{	
				NaturalCell* cell = dynamic_cast<NaturalCell*>( grid->getCell(x,y,z) );
				NaturalCell* otherCell = dynamic_cast<NaturalCell*>( otherGrid->getCell(x,y,z) );
				
				if ( otherCell->isReachableWithRA() ) 
				{
					cell->setIsReachable(true);
					cell->setIsReachableWithRA(true);
				}
				
				cell->setCost( 0.0 );
			}
		}
	}
	
	cout << "Merge Done" << endl;
	return grid;
}

/*!
 * @breif Reachable Cells
 */
vector<NaturalCell*> NaturalGrid::getAllReachableCells()
{
	vector<NaturalCell*> ReachableCells;
	
	ReachableCells.clear();
	
	unsigned int nbCells = this->getNumberOfCells();
	
    for(unsigned int i=0; i<nbCells; i++)
    {
        NaturalCell* cell = dynamic_cast<NaturalCell*>( _cells[i] );
		
		if ( cell->isReachable() ) 
		{
			ReachableCells.push_back( cell );
		}
    }
	
	return ReachableCells;
}

/*!
 * Get all reachable
 * bellow some threshold cost
 */
vector<NaturalCell*> NaturalGrid::getAllReachableCells(double CostThreshold)
{
	vector<NaturalCell*> ReachableCells;
	
	ReachableCells.clear();
	
	unsigned int nbCells = this->getNumberOfCells();
	
    for(unsigned int i=0; i<nbCells; i++)
    {
        NaturalCell* cell = dynamic_cast<NaturalCell*>( _cells[i] );
		
		if ( cell->isReachable() && (cell->getCost() < CostThreshold ) )
		{
			ReachableCells.push_back( cell );
		}
    }
	cout << "Number of Reachable bellow " << CostThreshold << " is :  " << ReachableCells.size() <<  endl;
	return ReachableCells;
}

/*!
 * @breif get Config
 */
int NaturalGrid::robotConfigInCell(int i)
{
	return dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->setRobotToStoredConfig();
}

/*!
 * @breif Get the Transform Matrix 
 * between the robot and the grid point
 */
Eigen::Transform3d NaturalGrid::getTransformFromRobotPos()
{
	shared_ptr<Configuration> q_actual = getRobot()->getCurrentPos();
	
	Transform3d actual(Transform3d::Identity());
	
	Vector3d trans;
	
	trans[0] = (*q_actual)[6];
	trans[1] = (*q_actual)[7];
	trans[2] = (*q_actual)[8];
	
	actual.translation() = trans;
	
	Matrix3d rot;
	
	rot =	Eigen::AngleAxisd((*q_actual)[9],  Vector3d::UnitX())
		*	Eigen::AngleAxisd((*q_actual)[10], Vector3d::UnitY())
		*	Eigen::AngleAxisd((*q_actual)[11], Vector3d::UnitZ());
	
	actual.linear() = rot;
	
	Transform3d t2( actual * getRobotOrigin().inverse() );
	return t2;
}

/*!
 * Transform the point to the robot frame
 */
Vector3d NaturalGrid::getTranformedToRobotFrame(const Vector3d& WSPoint)
{
	Transform3d t( getTransformFromRobotPos().inverse() );
	Vector3d inGridPoint( t*WSPoint );
	return inGridPoint;
}

/*!
 *
 */
bool NaturalGrid::isInReachableGrid(const Eigen::Vector3d& WSPoint)
{	
	Vector3d gridSize;
	gridSize[0] = _nbCellsX*_cellSize[0];
	gridSize[1] = _nbCellsY*_cellSize[1];
	gridSize[2] = _nbCellsZ*_cellSize[2];
	
	// Hack
	Vector3d topCorner = _originCorner+gridSize;
	
	for (unsigned int i=0; i<3; i++) 
	{
		if( (WSPoint[i] > topCorner[i]) || (WSPoint[i] < _originCorner[i]))
		{
		   return false;
		}
	}
	
	return true;
}

/*!
 * Returns wether a point
 * is reachable in the natural grid
 */
bool NaturalGrid::isReachable(const Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	
	if (isInReachableGrid(inGridPoint)) 
	{
		if( dynamic_cast<NaturalCell*>(getCell(inGridPoint))->isReachable() )
		{
			return true;
		}
	}
	return false;
}

bool NaturalGrid::isReachableWithRA(const Eigen::Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	
	if (isInReachableGrid(inGridPoint)) 
	{
		if( dynamic_cast<NaturalCell*>(getCell(inGridPoint))->isReachableWithRA() )
		{
			return true;
		}
	}
	return false;
}

bool NaturalGrid::isReachableWithLA(const Eigen::Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	
	if (isInReachableGrid(inGridPoint)) 
	{
		if( dynamic_cast<NaturalCell*>(getCell(inGridPoint))->isReachableWithLA() )
		{
			return true;
		}
	}
	return false;
}

/*!
 * Get the cell containing WSPoint and 
 * returns the cost
 */
double NaturalGrid::getCellCostAt(const Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	return dynamic_cast<NaturalCell*>(getCell(inGridPoint))->getCost();
}

Robot* NaturalGrid::getRobot()
{ 
	return this->getNaturalCostSpace()->getRobot(); 
}

vector<Vector3d> NaturalGrid::getBox()
{
	Vector3d gridSize;
	gridSize[0] = _nbCellsX*_cellSize[0];
	gridSize[1] = _nbCellsY*_cellSize[1];
	gridSize[2] = _nbCellsZ*_cellSize[2];
	
	Vector3d topCorner = _originCorner+gridSize/*+_cellSize*/;
	
	Vector3d v6 = topCorner;
	Vector3d v8 = topCorner;		v8[2] = _originCorner[2];
	Vector3d v5 = topCorner;		v5[1] = _originCorner[1];
	Vector3d v2 = topCorner;		v2[0] = _originCorner[0];
	Vector3d v3 = _originCorner;
	Vector3d v1 = _originCorner;	v1[2] = topCorner[2];
	Vector3d v4 = _originCorner;	v4[1] = topCorner[1];
	Vector3d v7 = _originCorner;	v7[0] = topCorner[0];
	
	vector<Vector3d> box;
	box.push_back(getTransformFromRobotPos()*v1);
	box.push_back(getTransformFromRobotPos()*v2);
	box.push_back(getTransformFromRobotPos()*v3);
	box.push_back(getTransformFromRobotPos()*v4);
	box.push_back(getTransformFromRobotPos()*v5);
	box.push_back(getTransformFromRobotPos()*v6);
	box.push_back(getTransformFromRobotPos()*v7);
	box.push_back(getTransformFromRobotPos()*v8);
	
	return box;
}

void NaturalGrid::draw()
{
	m_ActualConfig = getRobot()->getCurrentPos();
	
	unsigned int nbCells = this->getNumberOfCells();
	
	/*if (m_firstDisplay) 
	{
		cout << "First Draw of natural grid" << endl;
		for(unsigned int i=0; i<nbCells; i++)
		{
			//cout << BaseGrid::getCell(i) << endl;
			dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->createDisplaylist();
		}
		
		m_firstDisplay = false;
	}*/
	
    for(unsigned int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->draw();
    }
	
	//getRobot()->setAndUpdate(*m_ActualConfig);
}

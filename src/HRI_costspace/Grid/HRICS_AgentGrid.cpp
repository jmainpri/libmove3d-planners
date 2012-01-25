/*
 *  HRICS_AgentGrid.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_AgentGrid.hpp"

#include "HRICS_Distance.hpp"
#include "HRICS_Visibility.hpp"
#include "HRICS_Natural.hpp"

#include "HRICS_costspace.hpp"

#include "gridsAPI.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

AgentCell::AgentCell() :
m_Open(false),
m_Closed(false),
m_IsCostComputed(false),
m_Cost(-1),
m_IsReachable(false),
m_IsReachWithLeftArm(false),
m_IsReachWithRightArm(false),
m_NbDirections(1.0),
m_list(NULL)
{
	
}

AgentCell::AgentCell(int i, Vector3i coord , Vector3d corner, AgentGrid* grid) :
API::ThreeDCell(i,corner,grid),
m_Open(false),
m_Closed(false),
m_IsCostComputed(false),
m_Cost(-1),
m_IsReachable(false),
m_IsReachWithLeftArm(false),
m_IsReachWithRightArm(false),
m_NbDirections(1.0)
{
  m_Coord = coord;
  m_v0 = new double[3]; m_v1 = new double[3]; m_v2 = new double[3]; m_v3 = new double[3];
  m_v4 = new double[3]; m_v5 = new double[3]; m_v6 = new double[3]; m_v7 = new double[3];
	
  m_v0[0] = _corner[0] + _grid->getCellSize()[0];
  m_v0[1] = _corner[1] + _grid->getCellSize()[1];
  m_v0[2] = _corner[2] + _grid->getCellSize()[2];
	
  m_v1[0] = _corner[0] ;
  m_v1[1] = _corner[1] + _grid->getCellSize()[1];
  m_v1[2] = _corner[2] + _grid->getCellSize()[2];
	
  m_v2[0] = _corner[0] ;
  m_v2[1] = _corner[1] ;
  m_v2[2] = _corner[2] + _grid->getCellSize()[2];
	
  m_v3[0] = _corner[0] + _grid->getCellSize()[0];
  m_v3[1] = _corner[1] ;
  m_v3[2] = _corner[2] + _grid->getCellSize()[2];
	
  m_v4[0] = _corner[0] + _grid->getCellSize()[0];
  m_v4[1] = _corner[1] ;
  m_v4[2] = _corner[2] ;
	
  m_v5[0] = _corner[0] + _grid->getCellSize()[0];
  m_v5[1] = _corner[1] + _grid->getCellSize()[1];
  m_v5[2] = _corner[2] ;
	
  m_v6[0] = _corner[0] ;
  m_v6[1] = _corner[1] + _grid->getCellSize()[1];
  m_v6[2] = _corner[2] ;
	
  m_v7[0] = _corner[0] ;
  m_v7[1] = _corner[1] ;
  m_v7[2] = _corner[2] ;
}

/*
 AgentCell::AgentCell(const AgentCell& cell) :
 m_Open(false),
 m_Closed(false),
 m_IsCostComputed(cell.m_IsCostComputed),
 m_Cost(cell.m_Cost),
 m_IsReachable(cell.m_IsReachable),
 m_IsReachWithLeftArm(cell.m_IsReachWithLeftArm),
 m_IsReachWithRightArm(cell.m_IsReachWithRightArm),
 m_NbDirections(1.0),
 m_list(NULL)
 {
 
 }
 */

AgentCell::~AgentCell()
{
  delete m_v0; delete m_v1; delete m_v2; delete m_v3;
  delete m_v4; delete m_v5; delete m_v6; delete m_v7;
}

void AgentCell::setBlankCost()
{ 
	m_IsCostComputed = false;
	m_Cost = 0.0;
	//this->resetExplorationStatus(); 
}

void AgentCell::resetReachable()
{
	m_IsReachable = false;
	m_IsReachWithLeftArm = false;
	m_IsReachWithRightArm = false;
}


/*!
 * Get the Workspace Point transformed 
 * by the freeflyer of the human
 */
Vector3d AgentCell::getWorkspacePoint()
{
  //	shared_ptr<Configuration> q_actual = dynamic_cast<AgentGrid*>(_grid)->getRobot()->getCurrentPos();
  //	
  //	Transform3d actual(Transform3d::Identity());
  //	
  //	Vector3d trans;
  //	
  //	trans[0] = (*q_actual)[6];
  //	trans[1] = (*q_actual)[7];
  //	trans[2] = (*q_actual)[8];
  //	
  //	actual.translation() = trans;
  //	
  //	Matrix3d rot;
  //	
  //	rot =	Eigen::AngleAxisd((*q_actual)[9],  Vector3d::UnitX())
  //		*	Eigen::AngleAxisd((*q_actual)[10], Vector3d::UnitY())
  //		*	Eigen::AngleAxisd((*q_actual)[11], Vector3d::UnitZ());
  //	
  //	actual.linear() = rot;
	
	//Eigen::Transform3d origin() ;
	
	return (dynamic_cast<AgentGrid*>(_grid)->getTransformFromRobotPos() *  getCenter());
}


void AgentCell::createDisplaylist()
{
	Vector3d center = getCenter();
	
	//	cout << "createDisplaylist()" << endl;
	//	cout << "center " << endl;
	//	cout << center << endl;
	
	m_list=glGenLists(1);
	glNewList(m_list, GL_COMPILE);
	double diagonal = getCellSize().minCoeff();
	g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/3, 10);
	//g3d_draw_solid_sphere(center[0], center[1], center[2], ENV.getDouble(Env::CellSize)/2, 20);
	//g3d_drawSphere(center[0], center[1], center[2], ENV.getDouble(Env::CellSize)/2 );
	glEndList();
}

int AgentCell::setRobotToStoredConfig()
{
	AgentGrid* grid = dynamic_cast<AgentGrid*>(_grid);
	
	if(this->getCost() != 0.0)
	{
		//m_QStored->print();
		return grid->getRobot()->setAndUpdateMultiSol(*m_QStored);
	}
	
	return -1;
}

//! Compute the from the cell to the human
void AgentCell::computeDistance()
{
  Distance* CostSpace = dynamic_cast<AgentGrid*>(_grid)->getDistance();
  
  if ( CostSpace == NULL )
    return;
  
  m_Distance = CostSpace->getWorkspaceCost( getWorkspacePoint() );
}

//! Compute the visibility of the cell
void AgentCell::computeVisibility()
{
  Visibility* CostSpace = dynamic_cast<AgentGrid*>(_grid)->getVisibility();
  
  if ( CostSpace == NULL )
    return;
  
  m_Visiblity = CostSpace->getWorkspaceCost( getWorkspacePoint() );
}

//! Compute the cell reachbility
void AgentCell::computeReachability()
{
	Natural* NatSpace = dynamic_cast<AgentGrid*> (_grid)->getNatural();
  
  if ( NatSpace == NULL )
    return;
  
	resetReachable();
  
//	if ( NatSpace->computeIsReachableOnly(getWorkspacePoint(),true) )
//	{
//		m_IsReachable = true;
//		m_IsReachWithLeftArm = true;
//	}
//  
//	if ( NatSpace->computeIsReachableOnly(getWorkspacePoint(),false) )
//	{
//		m_IsReachable = true;
//		m_IsReachWithRightArm = true;
//	}
}

void AgentCell::computeCombined()
{
  m_Combined = m_Distance + m_Visiblity / 2;
}


//!
//!compute cost depending on right/left hand
//!
double AgentCell::getCost()
{
//  cout << "m_Distance : " << m_Distance << endl;
//  cout << "m_Visiblity : " << m_Visiblity << endl;
//  cout << "m_Reachability : " << m_Reachability << endl;
  
  return m_Combined;
  
//	if (!m_IsCostComputed) 
//	{
//		m_Cost = 0.0;
//		
//		if(m_IsReachable)
//		{
//			double rightArmPref = 1.0;
//			double leftArmPref = 1.0;
//			//cout << "Computing cost of cell number ( " << _index << " )" << endl;
//			//Vector3d center = getCenter();
//      
//			double pref = ENV.getDouble(Env::coeffArmPr);
//			if (pref >= 0.0)
//			{
//				rightArmPref -= pref;
//			}
//			else
//			{
//				leftArmPref += pref;
//			}
//      
//			if (m_IsReachWithLeftArm && !m_IsReachWithRightArm)
//			{
//				m_Cost = getCost(true) * leftArmPref;
//			}
//			else if (!m_IsReachWithLeftArm && m_IsReachWithRightArm)
//			{
//				m_Cost = getCost(false) * rightArmPref;
//			}
//			else if (m_IsReachWithLeftArm && m_IsReachWithRightArm)
//			{
//				double left_cost = getCost(true) * leftArmPref;
//				double right_cost = getCost(false) * rightArmPref;
//        
//				if (left_cost<right_cost)
//				{
//					m_IsReachWithRightArm = false;
//					m_Cost = left_cost;
//				}
//				else
//				{
//					m_IsReachWithLeftArm = false;
//					m_Cost = right_cost;
//				}
//			}
//      
//		}
//		
//		m_IsCostComputed = true;
//	}
//	
//	return m_Cost;
  return NULL;
}

/*
 shouldn't be called alone : see upside.
 */
double AgentCell::getCost(bool leftArm)
{
  
	// Get the cost of the Workspace point associated to the cell
	Natural* NatSpace = dynamic_cast<AgentGrid*>(_grid)->getNatural();
	shared_ptr<Configuration> q_actual = NatSpace->getRobot()->getCurrentPos();
  
	HRI_GIK_TASK_TYPE task;
	task = GIK_RATREACH;
	if ( leftArm )
	{
		task = GIK_LATREACH;
	}
  
  p3d_vector3 Tcoord;
  Vector3d center = getWorkspacePoint();
  Tcoord[0] = center[0];
  Tcoord[1] = center[1];
  Tcoord[2] = center[2];
  
  configPt q;
  HRI_AGENTS * agents = hri_create_agents();
  
	q = p3d_get_robot_config(agents->humans[0]->robotPt);
  
	bool IKSucceded;
	double distance_tolerance = 0.02;
	IKSucceded = hri_agent_single_task_manip_move(agents->humans[0], task, &Tcoord, distance_tolerance, &q);
  
	shared_ptr<Configuration> ptrQ(new Configuration(NatSpace->getRobot(),q));
  
	if (IKSucceded)
	{
		if( !ptrQ->isInCollision())
		{
			NatSpace->getRobot()->setAndUpdate(*ptrQ);
			m_Cost = NatSpace->getCost(getWorkspacePoint(),leftArm);
			if (m_Cost < 0.0)
			{
				resetReachable();
			}
		}
		else
		{
			resetReachable();
		}
	}
	else
	{
		resetReachable();
	}
  
	NatSpace->getRobot()->setAndUpdate(*q_actual);
  
	return m_Cost;
}

bool  AgentCell::writeToXml(xmlNodePtr cur)
{
	stringstream ss;
	string str; 
	
	str.clear(); ss << getCost(); ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("Cost"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << ((int)m_IsReachable); ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("Reach"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << ((int)m_IsReachWithLeftArm); ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("RWLA"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << ((int)m_IsReachWithRightArm); ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("RWRA"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << _corner[0] ; ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("CornerX"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << _corner[1]; ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("CornerY"), xmlCharStrdup(str.c_str()));
	
	str.clear(); ss << _corner[2]; ss >> str; ss.clear();
	xmlNewProp (cur, xmlCharStrdup("CornerZ"), xmlCharStrdup(str.c_str()));
	
	return true;
}


bool AgentCell::readCellFromXml(xmlNodePtr cur)
{	
	xmlChar* tmp;
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("Cost"))) != NULL)
	{
		float cost;
		sscanf((char *) tmp, "%f", &(cost));
		m_IsCostComputed = true ;
		m_Cost = cost;
	}
	else 
	{
		cout << "Document error in reading Cost"<< endl;
		return false;
	}
	xmlFree(tmp);
	
	//m_IsCostComputed = false;
	m_IsCostComputed = true;
	
	int Reach=0;
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("Reach"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &Reach );
		m_IsReachable = Reach; 
	}
	else 
	{
		cout << "Document error in reading Reach"<< endl;
		return false;
	}
	xmlFree(tmp);
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("RWLA"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &Reach );
		m_IsReachWithLeftArm = Reach; 
	}
	else 
	{
		cout << "Document error in reading Reach"<< endl;
		return false;
	}
	xmlFree(tmp);
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("RWRA"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &Reach );
		m_IsReachWithRightArm = Reach; 
	}
	else 
	{
		cout << "Document error in reading Reach"<< endl;
		return false;
	}
	xmlFree(tmp);
	
  
  // Read the corner of the cell
  // (X,Y,Z)
	float Corner[3];
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("CornerX"))) != NULL)
	{
		sscanf((char *) tmp, "%f", Corner + 0 );
	}
	else 
	{
		cout << "Document error in reading CornerX"<< endl;
		return false;
	}
	xmlFree(tmp);
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("CornerY"))) != NULL)
	{
		sscanf((char *) tmp, "%f", Corner + 1 );
	}
	else 
	{
		cout << "Document error in reading CornerY"<< endl;
		return false;
	}
	xmlFree(tmp);
	
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("CornerZ"))) != NULL)
	{
		sscanf((char *) tmp, "%f", Corner + 2 );
	}
	else 
	{
		cout << "Document error in reading CornerZ"<< endl;
		return false;
	}
	xmlFree(tmp);
	
	_corner[0] = Corner[0];
	_corner[1] = Corner[1];
	_corner[2] = Corner[2];
	
	return true;
}

void AgentCell::draw(bool transform)
{
  double Cost = 0.0;
  double diagonal = 0.07;
  double colorvector[4];
	
  colorvector[0] = 0.0;       //red
  colorvector[1] = 0.0;       //green
  colorvector[2] = 0.0;       //blue
  colorvector[3] = 0.01;       //transparency
	
  if (transform) 
  {
    m_Center = getWorkspacePoint();
  }
  
  
  //	if (!m_IsReachable)
  //	{
  //		if (ENV.getBool(Env::drawEntireGrid))
  //		{
  //			Vector3d center = getWorkspacePoint();
  //			double colorvector[4];
  //      
  //			colorvector[0] = 0.5;       //red
  //			colorvector[1] = 0.5;       //green
  //			colorvector[2] = 0.5;       //blue
  //			colorvector[3] = 0.01;       //transparency
  //			double diagonal = getCellSize().minCoeff();
  //			g3d_set_color(Any,colorvector);
  //			g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/6, 10);
  //		}
  //    
  //		return;
  //	}
  
  if( ENV.getInt(Env::hriCostType) == HRICS_Distance )
  {
    Cost = m_Distance;
    GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
    //colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*Cost; //+0.01;
  }
  
  if( ENV.getInt(Env::hriCostType) == HRICS_Visibility )
  {
    Cost = m_Visiblity;
    GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
    //colorvector[3] = ENV.getDouble(Env::colorThreshold2)*1/Cost; 
  }
  
  if( ENV.getInt(Env::hriCostType) == HRICS_Combine )
  {
    Cost = m_Combined;
    GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
    //colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*Cost; //+0.01;
  }
  
  if ( ENV.getInt(Env::hriCostType) == HRICS_Reachability ) 
  {
    Cost = m_Reachability;
    
    if ( Cost != 0.0 )
    {
      GroundColorMixGreenToRed(colorvector,Cost);
      //glCallList(m_list);
      diagonal = getCellSize().minCoeff();    
    }
    
    if ( (Cost == 0.0) && m_IsReachable )
    {
      //glCallList(m_list);
      diagonal = getCellSize().minCoeff();
    }
  }
  
  g3d_set_color(Any,colorvector);
  g3d_draw_solid_sphere(m_Center[0], m_Center[1], m_Center[2], diagonal, 10);
}


//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
//--------------------------------------------------------------------------

AgentGrid::AgentGrid() :
API::ThreeDGrid(),
m_firstDisplay(true)
{

}

AgentGrid::AgentGrid(vector<int> size) :
m_firstDisplay(true)
{

}

//! @brief Creates a grid for a agent given as input
//! @param pace, the size of the cells
//! @param envSize, the size of the cube in which the grid is created
//! @param Distance, the functions used to compute distance cost
//! @param Visibility, the functions used to compute the visibility cost
//! @param Natural, the functions used to compute the comfort of the agen in a reaching posture
AgentGrid::AgentGrid(double pace, vector<double> envSize, 
                     Robot* robot, Distance* distCostSpace,Visibility* VisiCostSpace, Natural* NatuCostSpace) :
API::ThreeDGrid(pace,envSize),
m_Robot(robot),
m_DistanceCostSpace(distCostSpace),
m_VisibilityCostSpace(VisiCostSpace),
m_NaturalCostSpace(NatuCostSpace),
m_firstDisplay(true)
{  
  cout << "AgentGrid::createAllCells" << endl;
  createAllCells();
  
  cout << "AgentGrid::computeRadius" << endl;
  computeRadius();
}

//! @brief Creates a copy of a given grid
//! @param the agent grid to be copied
AgentGrid::AgentGrid(const AgentGrid& grid) :
API::ThreeDGrid(grid),
m_NaturalCostSpace(grid.m_NaturalCostSpace),
m_firstDisplay(true)
{	
	for (unsigned int i=0; i<grid._cells.size() ; i++) 
	{
		AgentCell* cell = dynamic_cast<AgentCell*>( grid._cells[i]);
		
		AgentCell* newCell = new AgentCell( *cell );
		newCell->setIsReachable( cell->isReachable() );
		newCell->setIsReachableWithLA( cell->isReachableWithLA() );
		newCell->setIsReachableWithRA( cell->isReachableWithRA() );
		newCell->setGrid( this );
		
		_cells[i] = newCell;
	}
  
  computeRadius();
}

//! @brief Virtual function that creates a new cell
//! @param integer index
//! @param integer x position in the grid
//! @param integer y position in the grid
//! @param integer z position in the grid
API::ThreeDCell* AgentGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
{
  Vector3i pos;
  pos[0] = x; pos[1] = y; pos[2] = z;
	
  if (index == 0)
  {
    return new AgentCell( 0, pos ,_originCorner , this );
  }
  return new AgentCell( index, pos , computeCellCorner(x,y,z) , this );
}

AgentGrid::~AgentGrid()
{
  
}

Robot* AgentGrid::getRobot()
{ 
	return m_Robot; 
}

Distance* AgentGrid::getDistance()
{ 
	return m_DistanceCostSpace; 
}

Visibility* AgentGrid::getVisibility()
{ 
	return m_VisibilityCostSpace; 
}

Natural* AgentGrid::getNatural()
{ 
	return m_NaturalCostSpace; 
}

void AgentGrid::computeRadius()
{  
  m_Radius = _nbCellsX*_cellSize[0]/2;
  cout << "Radius of grid : " << m_Radius << endl;
}

//! @brief Get the transform matrix between the origin and the robot current position
//! All grid points are stored relative to the agent first joint
//! To get points in the global frame points must be transformed by this matrix
Eigen::Transform3d AgentGrid::getTransformFromRobotPos()
{	 
	return ( m_Robot->getJoint(1)->getMatrixPos() );
}

//! @brief Get the global frame points in the robot frame
//! @param A point in the global frame
//! All grid points are stored relative to the agent first joint
//! To get global frame points in the robot frame the points are
//! transformed, using this function which uses the invers of the first joint matrix
Vector3d AgentGrid::getTranformedToRobotFrame(const Vector3d& WSPoint)
{
  Eigen::Transform3d t( getTransformFromRobotPos().inverse() );
	return ( t*WSPoint );
}

//! @brief Returns the bounding box of the human grid
//! The initial box is tranformed to the current agent position
//! A vector of 3D dimensional vertex in the global frame is returned
vector<Vector3d> AgentGrid::getBox()
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

//! @brief Returns wether a point is reachable in the natural grid
//! @param A point in the global frame
bool AgentGrid::isReachable(const Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	
	if (isInReachableGrid(inGridPoint)) 
	{
		if( dynamic_cast<AgentCell*>(getCell(inGridPoint))->isReachable() )
		{
			return true;
		}
	}
	return false;
}

bool AgentGrid::isReachableWithRA(const Eigen::Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	
	if (isInReachableGrid(inGridPoint)) 
	{
		if( dynamic_cast<AgentCell*>(getCell(inGridPoint))->isReachableWithRA() )
		{
			return true;
		}
	}
	return false;
}

bool AgentGrid::isReachableWithLA(const Eigen::Vector3d& WSPoint)
{
	Vector3d inGridPoint( getTranformedToRobotFrame(WSPoint) );
	
	if (isInReachableGrid(inGridPoint)) 
	{
		if( dynamic_cast<AgentCell*>(getCell(inGridPoint))->isReachableWithLA() )
		{
			return true;
		}
	}
	return false;
}

//! @brief Get the cell containing WSPoint and returns the cost
double AgentGrid::getCellCostAt(const Vector3d& WSPoint)
{
  Vector3d pointInGrid = getTranformedToRobotFrame(WSPoint);
  
  AgentCell* cell = dynamic_cast<AgentCell*>(getCell(pointInGrid));
  
  if(cell != NULL)
  {
    return cell->getCost();
  }
  else
  {
    // cout << "No cell at : " << endl << pointInGrid << endl;
    return NULL;
  }
}


//! @brief Reset Grid Cost
void AgentGrid::resetCellCost()
{
  unsigned int nbCells = this->getNumberOfCells();
	
  for(unsigned int i=0; i<nbCells; i++)
  {
    dynamic_cast<AgentCell*>( _cells[i] )->setBlankCost();
  }
}

//! @brief Reset Grid Reachability
void AgentGrid::resetReachability()
{
  unsigned int nbCells = this->getNumberOfCells();
	
  for(unsigned int i=0; i<nbCells; i++)
  {
    dynamic_cast<AgentCell*>( _cells[i] )->resetReachable();
  }
}

//! @brief Compute Grid Accecibility whith right and left hand !
void AgentGrid::computeReachability()
{
	int nbCells = this->getNumberOfCells();
  //	m_NaturalCostSpace->setRobotToConfortPosture();
	shared_ptr<Configuration> robotConf = m_Robot->getInitialPosition();
	
	for(int i=0; i<nbCells; i++)
  {
    cout <<  "Computing Reachability of Cell : " << i << endl;
    dynamic_cast<AgentCell*>( BaseGrid::getCell(i) )->computeReachability();
    m_Robot->setAndUpdate(*robotConf);
    //        m_NaturalCostSpace->setRobotToConfortPosture();
  }
  
  API_activeGrid = this;
}

//! @brief Init Reach
void AgentGrid::initReachable()
{
  int nbCells = this->getNumberOfCells();
	
  for( int i=0; i<nbCells; i++)
  {
    AgentCell* cell = dynamic_cast<AgentCell*>( _cells[i] );
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

//! @brief Fusion Grid
AgentGrid* AgentGrid::mergeWith(AgentGrid* otherGrid)
{
	if (otherGrid->getNumberOfCells() != getNumberOfCells() ) 
	{
		cout << "Error in AgentGrid::fusionWith" << endl;
		return NULL;
	}
	
	AgentGrid* grid = new AgentGrid(*this);
	
	for ( int x=0; x<int(_nbCellsX); x++) 
	{
		for ( int y=0; y<int(_nbCellsY); y++) 
		{
			for ( int z=0; z<int(_nbCellsZ); z++) 
			{	
				AgentCell* cell = dynamic_cast<AgentCell*>( grid->getCell(x,y,z) );
				AgentCell* otherCell = dynamic_cast<AgentCell*>( otherGrid->getCell(x,y,z) );
				
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

//! @brief Reachable Cells
vector<AgentCell*> AgentGrid::getAllReachableCells()
{
	vector<AgentCell*> ReachableCells;
	
	ReachableCells.clear();
	
	int nbCells = this->getNumberOfCells();
	
  for(int i=0; i<nbCells; i++)
  {
    AgentCell* cell = dynamic_cast<AgentCell*>( _cells[i] );
		
		if ( cell->isReachable() ) 
		{
			ReachableCells.push_back( cell );
		}
  }
  
	return ReachableCells;
}

//! @brief Natural cell comparator
class AgentCellComp
{
public:
  
	bool operator()(pair<double,AgentCell*> first, pair<double,AgentCell*> second)
	{
		return ( first.first < second.first );
	}
  
} AgentCellCompObj;


//! @brief Reachable cells sorted
//! Returns the reachable cells sorted by cost
vector<pair<double,AgentCell*> > AgentGrid::getAllReachableCellsSorted()
{
	vector<pair<double,AgentCell*> > ReachableCells;
  
	ReachableCells.clear();
  
	int nbCells = this->getNumberOfCells();
  
  for(int i=0; i<nbCells; i++)
  {
    AgentCell* cell = dynamic_cast<AgentCell*>( _cells[i] );
    
		if ( cell->isReachable() )
		{
			pair<double, AgentCell*> costCell;
			costCell.second = cell;
			costCell.first = cell->getCost();
			ReachableCells.push_back( costCell );
		}
	}
	sort(ReachableCells.begin(),ReachableCells.end(),AgentCellCompObj);
  
	return ReachableCells;
}

//! @brief Get all reachable bellow some threshold cost
vector<AgentCell*> AgentGrid::getAllReachableCells(double CostThreshold)
{
	vector<AgentCell*> ReachableCells;
	
	ReachableCells.clear();
	
	int nbCells = this->getNumberOfCells();
	
  for(int i=0; i<nbCells; i++)
  {
    AgentCell* cell = dynamic_cast<AgentCell*>( _cells[i] );
		
		if ( cell->isReachable() && (cell->getCost() < CostThreshold ) )
		{
			ReachableCells.push_back( cell );
		}
  }
	cout << "Number of Reachable bellow " << CostThreshold << " is :  " << ReachableCells.size() <<  endl;
	return ReachableCells;
}

//! @brief get config
int AgentGrid::robotConfigInCell(int i)
{
	return dynamic_cast<AgentCell*>( _cells[i] )->setRobotToStoredConfig();
}

//! @brief Return true if the point is in the grid
//! @param 3D point in the eucledan space
bool AgentGrid::isInReachableGrid(const Eigen::Vector3d& WSPoint)
{	
	Vector3d gridSize;
	gridSize[0] = _nbCellsX*_cellSize[0];
	gridSize[1] = _nbCellsY*_cellSize[1];
	gridSize[2] = _nbCellsZ*_cellSize[2];
	
	// Hack
	Vector3d topCorner = _originCorner+gridSize;
	
	for (int i=0; i<3; i++)
	{
		if( (WSPoint[i] > topCorner[i]) || (WSPoint[i] < _originCorner[i]))
		{
      return false;
		}
	}
	
	return true;
}

//! @brief Compute Grid Cost
void AgentGrid::computeAllCellCost()
{
	cout << "AgentGrid::computeAllCellCost" << endl;
  //	vector<HRICS::AgentCell*> cells = getAllReachableCells();
  
	shared_ptr<Configuration> q = m_Robot->getCurrentPos();
  
  m_DangerCells.clear();
  m_VisibilityCells.clear();
  m_ReachableCells.clear();
  m_CombinedCells.clear();
  
  for (unsigned int i=0; i < _cells.size() ; i++) 
  {
    AgentCell* cell = static_cast<AgentCell*>(_cells[i]);
    
    cell->computeDistance();
    //if ( cell->getDistance() > 0.2  ) 
      m_DangerCells.push_back( cell );
    
    cell->computeVisibility();
    //if ( cell->getVisibility() > 0.2 ) 
      m_VisibilityCells.push_back( cell );
    
    cell->computeReachability();
    //if ( cell->getReachability() > 0 ) 
      m_ReachableCells.push_back( cell );
    
    cell->computeCombined();
    //if ( cell->getCombined() > 0.2 ) 
      m_CombinedCells.push_back( cell );
  }
  m_Robot->setAndUpdate(*q);
  API_activeGrid = this;
}


bool AgentGrid::writeToXmlFile(string docname)
{
  stringstream ss;
  string str;
  
  //Creating the file Variable version 1.0
  xmlDocPtr doc = xmlNewDoc(xmlCharStrdup("1.0"));
  
  //Writing the root node
  xmlNodePtr root = xmlNewNode (NULL, xmlCharStrdup("Grid"));
  
  xmlNewProp (root, xmlCharStrdup("Type"), xmlCharStrdup("3D"));
  
	//Writing the first Node
	xmlNodePtr cur = xmlNewChild (root, NULL, xmlCharStrdup("OriginCorner"), NULL);
  
  str.clear(); ss << _originCorner[0]; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("X"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _originCorner[1]; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("Y"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _originCorner[2]; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("Z"), xmlCharStrdup(str.c_str()));
  
	//Writing cell size
	cur = xmlNewChild (cur, NULL, xmlCharStrdup("CellSize"), NULL);
  
  str.clear(); ss << _cellSize[0]; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("X"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _cellSize[1]; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("Y"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _cellSize[2]; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("Z"), xmlCharStrdup(str.c_str()));
  
  //Writing the cells
  cur = xmlNewChild (cur, NULL, xmlCharStrdup("Cells"), NULL);
  
  str.clear(); ss << getNumberOfCells(); ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("NbOfCells"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _nbCellsX; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("NbOnX"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _nbCellsY; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("NbOnY"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << _nbCellsZ; ss >> str; ss.clear();
  xmlNewProp (cur, xmlCharStrdup("NbOnZ"), xmlCharStrdup(str.c_str()));
  
  for (unsigned int i=0; i<getNumberOfCells(); i++)
  {
    xmlNodePtr _XmlCellNode_ = xmlNewChild(cur,
                                           NULL,
                                           xmlCharStrdup("Cell"), NULL);
    
    _cells[i]->writeToXml(_XmlCellNode_);
  }
  
  ////Writing the second Node (coef prop)
  xmlNodePtr coef = xmlNewChild (root, NULL, xmlCharStrdup("EnvCoeff"), NULL);
  
  str.clear(); ss << ENV.getDouble(Env::coeffJoint);; ss >> str; ss.clear();
  xmlNewProp (coef, xmlCharStrdup("coeffJoint"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << ENV.getDouble(Env::coeffEnerg);; ss >> str; ss.clear();
  xmlNewProp (coef, xmlCharStrdup("coeffEnerg"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << ENV.getDouble(Env::coeffConfo);; ss >> str; ss.clear();
  xmlNewProp (coef, xmlCharStrdup("coeffConfo"), xmlCharStrdup(str.c_str()));
  
  str.clear(); ss << ENV.getDouble(Env::coeffArmPr);; ss >> str; ss.clear();
  xmlNewProp (coef, xmlCharStrdup("coeffArmPr"), xmlCharStrdup(str.c_str()));
  
  xmlDocSetRootElement(doc, root);
  //	writeRootNode(graph, root);
  //	writeSpeGraph(graph, file, root);
  
  //Writing the file on HD
  xmlSaveFormatFile (docname.c_str(), doc, 1);
  xmlFreeDoc(doc);
  
  cout << "Writing Grid to : " << docname << endl;
  return true;
}

/*!
 * \brief Reads the grid
 * from an xml file
 */
bool AgentGrid::loadFromXmlFile(string docname)
{
  //Creating the file Variable version 1.0
  xmlDocPtr doc;
  xmlNodePtr cur;
  xmlNodePtr root;
  
  doc = xmlParseFile(docname.c_str());
  
  if(doc==NULL)
  {
    cout << "Document not parsed successfully (doc==NULL)" << endl;
    return false;
  }
  
  root = xmlDocGetRootElement(doc);
  
  if (root == NULL)
  {
    cout << "Document not parsed successfully" << endl;
    xmlFreeDoc(doc);
    return false;
  }
  
  
	if (xmlStrcmp(root->name, xmlCharStrdup("Grid")))
	{
		cout << "Document of the wrong type root node not Grid" << endl;
		xmlFreeDoc(doc);
		return false;
	}
  
  
	xmlChar* tmp;
  
	tmp = xmlGetProp(root, xmlCharStrdup("Type"));
	if (xmlStrcmp(tmp, xmlCharStrdup("3D")))
	{
		cout << "Doccument not a 3D grid"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	/***************************************/
	// NODE OriginCorner
  
	cur = root->xmlChildrenNode->next;
	root = cur;
  
	float originCorner[3];
  
	if (xmlStrcmp(cur->name, xmlCharStrdup("OriginCorner")))
	{
		cout << "Document second node is not OriginCorner ( " << cur->name << " )"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("X"))) != NULL)
	{
		sscanf((char *) tmp, "%f", originCorner+0 );
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error origin X"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("Y"))) != NULL)
	{
		sscanf((char *) tmp, "%f", originCorner+1 );
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error origin Y"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("Z"))) != NULL)
	{
		sscanf((char *) tmp, "%f", originCorner+2 );
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error NbOnZ"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	_originCorner[0] = originCorner[0];
	_originCorner[1] = originCorner[1];
	_originCorner[2] = originCorner[2];
  
	cout << "_originCorner = " << endl <<_originCorner << endl;
  
	/***************************************/
	// NODE CellSize
  
	cur = cur->xmlChildrenNode->next;
  
	float cellSize[3];
  
	if (xmlStrcmp(cur->name, xmlCharStrdup("CellSize")))
	{
		cout << "Document second node is not CellSize ( " << cur->name << " )"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("X"))) != NULL)
	{
		sscanf((char *) tmp, "%f", cellSize+0);
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error origin X"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("Y"))) != NULL)
	{
		sscanf((char *) tmp, "%f", cellSize+1);
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error origin Y"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("Z"))) != NULL)
	{
		sscanf((char *) tmp, "%f", cellSize+2);
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error NbOnZ"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	_cellSize[0] = cellSize[0];
	_cellSize[1] = cellSize[1];
	_cellSize[2] = cellSize[2];
  
	cout << "_cellSize = " << endl <<_cellSize << endl;
  
	/***************************************/
	// NODE Cells
  
	cur = cur->xmlChildrenNode->next;
  
	if (xmlStrcmp(cur->name, xmlCharStrdup("Cells")))
	{
		cout << "Document second node is not Cells ( " << cur->name << " )"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
  
	unsigned int NbOfCells;
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("NbOfCells"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &(NbOfCells));
	}
	else
	{
		xmlFree(tmp);
		cout << "Document not a 3D grid"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("NbOnX"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &(_nbCellsX));
	}
	else
	{
		xmlFree(tmp);
		cout << "Document error NbOnX"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("NbOnY"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &(_nbCellsY));
	}
	else
	{
		xmlFree(tmp);
		cout << "Doccument error NbOnY"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if ((tmp = xmlGetProp(cur, xmlCharStrdup("NbOnZ"))) != NULL)
	{
		sscanf((char *) tmp, "%d", &(_nbCellsZ));
	}
	else
	{
		xmlFree(tmp);
		cout << "Doccument error NbOnZ"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
  
	if( _nbCellsX*_nbCellsY*_nbCellsZ != NbOfCells )
	{
		cout << "Doccument error _nbCellsX*_nbCellsY*_nbCellsZ != NbOfCells"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
  
	/***************************************/
	//  Reads the Cells
  
	_cells.resize(NbOfCells);
  
	cur = cur->xmlChildrenNode;
  
	for (unsigned int i=0; i<NbOfCells; i++)
	{
    
		cur = cur->next;
    
		if (cur == NULL)
		{
			cout << "Document error on the number of Cell" << endl;
			break;
		}
    
		_cells[i] = createNewCell(i,0,0,0);
    
		if (xmlStrcmp(cur->name, xmlCharStrdup("Cell")))
		{
			cout << "Document node is not Cell ( " << cur->name << " )"<< endl;
			xmlFreeDoc(doc);
			return false;
		}
    
		if ( ! _cells[i]->readCellFromXml(cur) )
		{
			cout << "Document error while reading cell"<< endl;
			xmlFreeDoc(doc);
			return false;
		}
    
		cur = cur->next;
	}
  
	/***************************************/
	//  Reads the EnvCoeff
	cur = root->next->next;
	if (cur)
	{
		double result = 0;
		std::stringstream ss;
		if (!xmlStrcmp(cur->name, xmlCharStrdup("EnvCoeff")))
		{
			if ((tmp = xmlGetProp(cur, xmlCharStrdup("coeffJoint"))) == NULL)
			{
				xmlFree(tmp);
				cout << "Document error coeffJoint"<< endl;
				xmlFreeDoc(doc);
				return false;
			}
			else
			{
        
				ss << (char *) tmp;
				ss >> result;
				ENV.setDouble(Env::coeffJoint,result);
				xmlFree(tmp);
				result = 0;
			}
      
			if ((tmp = xmlGetProp(cur, xmlCharStrdup("coeffEnerg"))) == NULL)
			{
				xmlFree(tmp);
				cout << "Document error coeffEnerg"<< endl;
				xmlFreeDoc(doc);
				return false;
			}
			else
			{
				ss.clear();
				ss << (char *) tmp;
				ss >> result;
				ENV.setDouble(Env::coeffEnerg,result);
				xmlFree(tmp);
				result = 0;
			}
      
			if ((tmp = xmlGetProp(cur, xmlCharStrdup("coeffConfo"))) == NULL)
			{
				xmlFree(tmp);
				cout << "Document error coeffConfo"<< endl;
				xmlFreeDoc(doc);
				return false;
			}
			else
			{
				ss.clear();
				ss << (char *) tmp;
				ss >> result;
				ENV.setDouble(Env::coeffConfo,result);
				xmlFree(tmp);
				result = 0;
			}
      
			if ((tmp = xmlGetProp(cur, xmlCharStrdup("coeffArmPr"))) == NULL)
			{
				xmlFree(tmp);
				cout << "Document error coeffArmPr"<< endl;
				xmlFreeDoc(doc);
				return false;
			}
			else
			{
				ss.clear();
				ss << (char *) tmp;
				ss >> result;
				ENV.setDouble(Env::coeffArmPr,result);
				xmlFree(tmp);
			}
		}
	}
  
  cout << "Reading Grid : " << docname << endl;
  xmlFreeDoc(doc);
  return true;
}

void AgentGrid::draw()
{
  //cout << "AgentGrid::draw()" << endl;
	m_ActualConfig = m_Robot->getCurrentPos();
	
	/*if (m_firstDisplay) 
   {
   cout << "First Draw of natural grid" << endl;
   for(unsigned int i=0; i<nbCells; i++)
   {
   //cout << BaseGrid::getCell(i) << endl;
   dynamic_cast<AgentCell*>( BaseGrid::getCell(i) )->createDisplaylist();
   }
   
   m_firstDisplay = false;
   }*/
  
  int size = 0;
  int type = ENV.getInt(Env::hriCostType);
  
  if( type == HRICS_Distance )
  {
    size = m_DangerCells.size();
  }
  else if( type == HRICS_Visibility )
  {
    size = m_VisibilityCells.size();
  }
  else if ( type == HRICS_Reachability ) 
  {
    size = m_ReachableCells.size();
  }
  else if ( type == HRICS_Combine ) 
  {
    size = m_CombinedCells.size();
  }
  else {
    cout << "No cells to draw" << endl;
    return;
  }
	
  //  cout << "drawCells.size() = " << size << endl;
  
  bool configChanged = true;
  
  //  if(!m_firstDisplay)
  //  {
  //    configChanged = !m_LastConfig->equal(*m_ActualConfig);
  //  }
  //  else
  //  {
  //    m_firstDisplay = false;
  //  }
  
  Transform3d T = getTransformFromRobotPos();
  
  int d = ENV.getInt(Env::lineToShow);
  int l = ENV.getInt(Env::hriShownGridLine);
  
  // Draws all cells
  for( int i=0; i<size; i++)
  {    
    AgentCell* cell = NULL;
    
    if( type == HRICS_Distance )
    {
      cell = m_DangerCells[i];
    }
    else if( type == HRICS_Visibility )
    {
      cell = m_VisibilityCells[i];
    }
    else if ( type == HRICS_Reachability ) 
    {
      cell = m_ReachableCells[i];
    }
    else if ( type == HRICS_Combine ) 
    {
      cell = m_CombinedCells[i];
    }
    else {
      cout << "No cells to draw" << endl;
      return;
    }
    
    // Only draws cells inside the ball
    Vector3d center = cell->getCenter();
    
    if( center.norm() > m_Radius )
      continue;
    
    // Only draws that are over the floor
     Vector3d WSPoint =  T*center;
    
    if( WSPoint[2] < 0 )
      continue;
    
    // Case only one line
    if (ENV.getBool(Env::drawOnlyOneLine)) 
		{
      if( getCellCoord(cell)[d] != l )
        continue;
    }
    
    // Draw the cell
    cell->draw(configChanged);
  }
  
  m_LastConfig = m_ActualConfig;
  
  // Get the center of the sphere
  Vector3d c(Vector3d::Zero());
  c = T*c;
  
  GLdouble color_vect[4];
  color_vect[0] = 0.1;
  color_vect[1] = 0.1;
  color_vect[2] = 0.1;
  color_vect[3] = 0.1;
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  g3d_set_color(Any,color_vect);
  g3d_draw_solid_sphere(c[0],c[1],c[2],m_Radius,20);
  
  glDisable(GL_BLEND);
  
	//getRobot()->setAndUpdate(*m_ActualConfig);
}

/*
 *  HRICS_NaturalCell.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "Grid/HRICS_NaturalCell.hpp"

#include "HRICS_Natural.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

NaturalCell::NaturalCell() :
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

NaturalCell::NaturalCell(int i, Vector3i coord , Vector3d corner, NaturalGrid* grid) :
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
NaturalCell::NaturalCell(const NaturalCell& cell) :
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

void NaturalCell::setBlankCost()
{ 
	m_IsCostComputed = false;
	m_Cost = 0.0;
	//this->resetExplorationStatus(); 
}

void NaturalCell::resetReachable()
{
	m_IsReachable = false;
	m_IsReachWithLeftArm = false;
	m_IsReachWithRightArm = false;
}


/*!
 * Get the Workspace Point transformed 
 * by the freeflyer of the human
 */
Vector3d NaturalCell::getWorkspacePoint()
{
//	shared_ptr<Configuration> q_actual = dynamic_cast<NaturalGrid*>(_grid)->getRobot()->getCurrentPos();
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
	
	return (dynamic_cast<NaturalGrid*>(_grid)->getTransformFromRobotPos() *  getCenter());
}


void NaturalCell::createDisplaylist()
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

void NaturalCell::draw()
{
	if (!m_IsReachable) 
	{
		return;
	}
	
    double colorvector[4];
	
    colorvector[0] = 0.0;       //red
    colorvector[1] = 0.0;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.01;       //transparency
	
	double Cost = this->getCost();
	//double Cost = 10.0;
	//Cost = m_Cost;
	
	//if ( m_IsReachWithLeftArm && m_IsReachWithRightArm )
//	{
//		Cost = 20.0;
//	}
	
	Vector3d center = getWorkspacePoint();
	
	if ( Cost != 0.0 ) 
	{
		GroundColorMixGreenToRed(colorvector,Cost);
		g3d_set_color(Any,colorvector);
		//glCallList(m_list);
		double diagonal = getCellSize().minCoeff();
		g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/6, 10);
		//cout << "Robot : " << dynamic_cast<NaturalGrid*>(_grid)->getRobot()->getName() << "Draw Sphere, Cost = " << Cost << endl;
	}
	
	if ( (Cost == 0.0) && m_IsReachable )
	{
		g3d_set_color(Any,colorvector);
		//glCallList(m_list);
		double diagonal = getCellSize().minCoeff();
		g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/3, 10);
	}
}

int NaturalCell::setRobotToStoredConfig()
{
	NaturalGrid* grid = dynamic_cast<NaturalGrid*>(_grid);
	
	if(this->getCost() != 0.0)
	{
		//m_QStored->print();
		return grid->getRobot()->setAndUpdateMultiSol(*m_QStored);
	}
	
	return -1;
}

double NaturalCell::getCost()
{
	if (!m_IsCostComputed) 
	{
		m_Cost = 0.0;
		
		if(m_IsReachable)
		{
			//cout << "Computing cost of cell number ( " << _index << " )" << endl;
			//Vector3d center = getCenter();

			bool useLeftVsRight=false;
			
			if ( m_IsReachWithLeftArm ) 
			{
				useLeftVsRight = true;
			}
			if ( m_IsReachWithRightArm ) // Right prefered
			{
				useLeftVsRight = false;
			}
			
			// Get the cost of the Workspace point associated to the cell
			Natural* NatSpace = dynamic_cast<NaturalGrid*>(_grid)->getNaturalCostSpace();
			
			shared_ptr<Configuration> q_actual = NatSpace->getRobot()->getCurrentPos();
			
			m_Cost = NatSpace->getCost(getWorkspacePoint(),useLeftVsRight);
			
			NatSpace->getRobot()->setAndUpdate(*q_actual);
		}
		
		m_IsCostComputed = true;
	}
	
	return m_Cost;
}

#ifdef HRI_PLANNER
void NaturalCell::computeReachability(bool leftArm)
{
	Natural* NatSpace = dynamic_cast<NaturalGrid*> (_grid)->getNaturalCostSpace();
	
	shared_ptr<Configuration> q_actual = NatSpace->getRobot()->getCurrentPos();
	
	if ( NatSpace->computeIsReachable(getCenter(),leftArm) )
	{
		m_IsReachable = true;
		
		if (leftArm) 
		{
			m_IsReachWithLeftArm = true;
		}
		else 
		{
			m_IsReachWithRightArm = true;
		}

	}
	else 
	{
		m_IsReachable = false;
		
		if (leftArm) 
		{
			m_IsReachWithLeftArm = false;
		}
		else 
		{
			m_IsReachWithRightArm = false;
		}
	}
	
	NatSpace->getRobot()->setAndUpdate(*q_actual);

}
#endif

bool  NaturalCell::writeToXml(xmlNodePtr cur)
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


bool NaturalCell::readCellFromXml(xmlNodePtr cur)
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
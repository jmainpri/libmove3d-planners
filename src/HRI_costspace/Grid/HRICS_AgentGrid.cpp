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

double AgentCell::getDistance() 
{ 
    //cout << "distance : " << m_Distance <<  endl;
    return m_Distance;
}

double AgentCell::getVisibility() 
{ 
    //cout << "visibility : " << m_Visiblity <<  endl;
    return m_Visiblity;
}

double AgentCell::getReachability() 
{ 
    //cout << "reachability : " << m_Reachability <<  endl;
    return m_Reachability;
}

double AgentCell::getCombined() 
{ 
    //cout << "combined : " << m_Combined <<  endl;
    return m_Combined;
}

//!
//! compute cost depending on right/left hand
//!
double AgentCell::getCost()
{
    return m_Combined;
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
    //	confPtr_t q_actual = dynamic_cast<AgentGrid*>(_grid)->getRobot()->getCurrentPos();
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

    m_Distance = ENV.getDouble(Env::Kdistance)*CostSpace->getWorkspaceCost( getWorkspacePoint() );
}

//! Compute the visibility of the cell
void AgentCell::computeVisibility()
{
    Visibility* CostSpace = dynamic_cast<AgentGrid*>(_grid)->getVisibility();

    if ( CostSpace == NULL )
        return;

    m_Visiblity = ENV.getDouble(Env::Kvisibility)*CostSpace->getWorkspaceCost( getWorkspacePoint() );
}

//! Compute the cell reachbility
void AgentCell::computeReachability()
{
    Natural* CostSpace = dynamic_cast<AgentGrid*> (_grid)->getNatural();

    if ( CostSpace == NULL )
        return;

    m_IsReachable = CostSpace->getWorkspaceIsReachable( getWorkspacePoint() );

    if( m_IsReachable ) {
        m_Reachability = CostSpace->getWorkspaceCost( getWorkspacePoint() );
    }
    else {
        //m_Reachability = numeric_limits<double>::max();
        m_Reachability = 90;
    }
}

void AgentCell::computeCombined()
{
    m_Combined = 0.0;

    m_Combined += m_Distance;
    m_Combined += m_Visiblity;

    //  if( m_IsReachable )
    //  {
    //    m_Combined += m_Reachability*ENV.getDouble(Env::Kreachable);
    //  }
    //  else {
    //    m_Combined += (2.0)*ENV.getDouble(Env::Kreachable);
    //  }
    //  m_Combined /= (ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility)+ENV.getDouble(Env::Kreachable));

    m_Combined /= 2;
}

bool AgentCell::writeToXml(xmlNodePtr cur)
{
    stringstream ss;
    string str;

    str.clear(); ss << m_Combined; ss >> str; ss.clear();
    xmlNewProp (cur, xmlCharStrdup("Combine"), xmlCharStrdup(str.c_str()));

    str.clear(); ss << m_Distance; ss >> str; ss.clear();
    xmlNewProp (cur, xmlCharStrdup("Distance"), xmlCharStrdup(str.c_str()));

    str.clear(); ss << m_Visiblity; ss >> str; ss.clear();
    xmlNewProp (cur, xmlCharStrdup("Visibility"), xmlCharStrdup(str.c_str()));

    str.clear(); ss << m_Reachability; ss >> str; ss.clear();
    xmlNewProp (cur, xmlCharStrdup("Rechability"), xmlCharStrdup(str.c_str()));

    str.clear(); ss << ((int)m_IsReachable); ss >> str; ss.clear();
    xmlNewProp (cur, xmlCharStrdup("is_Reachable"), xmlCharStrdup(str.c_str()));

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

    if ((tmp = xmlGetProp(cur, xmlCharStrdup("Combine"))) != NULL)
    {
        float cost;
        sscanf((char *) tmp, "%f", &(cost));
        m_Combined = cost;
    }
    else
    {
        cout << "Document error in reading Cost"<< endl;
        return false;
    }
    xmlFree(tmp);

    if ((tmp = xmlGetProp(cur, xmlCharStrdup("Distance"))) != NULL)
    {
        float cost;
        sscanf((char *) tmp, "%f", &(cost));
        m_Distance = cost;
    }
    else
    {
        cout << "Document error in reading Cost"<< endl;
        return false;
    }
    xmlFree(tmp);

    if ((tmp = xmlGetProp(cur, xmlCharStrdup("Visibility"))) != NULL)
    {
        float cost;
        sscanf((char *) tmp, "%f", &(cost));
        m_Visiblity = cost;
    }
    else
    {
        cout << "Document error in reading Cost"<< endl;
        return false;
    }
    xmlFree(tmp);

    if ((tmp = xmlGetProp(cur, xmlCharStrdup("Rechability"))) != NULL)
    {
        float cost;
        sscanf((char *) tmp, "%f", &(cost));
        m_Reachability = cost;
    }
    else
    {
        cout << "Document error in reading Cost"<< endl;
        return false;
    }
    xmlFree(tmp);

    if ((tmp = xmlGetProp(cur, xmlCharStrdup("is_Reachable"))) != NULL)
    {
        int Reach=0;
        sscanf((char *) tmp, "%d", &Reach );
        m_IsReachable = Reach;
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

void AgentCell::drawOnePoint( bool withTransform )
{
    double _corner[3];
    double colorvector[4];

    colorvector[0] = 0.0;   // red
    colorvector[1] = 0.0;   // green
    colorvector[2] = 0.0;   // blue
    colorvector[3] = 0.01;  // transparency

    if (withTransform)
    {
        m_Center = getWorkspacePoint();
    }

    _corner[0] = m_Center[0];
    _corner[1] = m_Center[1];
    _corner[2] = m_Center[2];

    double Cost;

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

    if ( ENV.getInt(Env::hriCostType) == HRICS_Reachability )
    {
        if ( (Cost == numeric_limits<double>::max()) && !m_IsReachable )
        {
            return;
        }

        Cost = m_Reachability;
        GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
    }

    if( ENV.getInt(Env::hriCostType) == HRICS_Combine )
    {
        Cost = m_Combined;
        GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
        //colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*Cost; //+0.01;
    }

    Eigen::Vector3d m_CubeSize;

    m_CubeSize[0] = 0.05;
    m_CubeSize[1] = 0.05;
    m_CubeSize[2] = 0.05;

    double _v0[3]; double _v1[3]; double _v2[3]; double _v3[3];
    double _v4[3]; double _v5[3]; double _v6[3]; double _v7[3];

    g3d_set_color(Any,colorvector);

    //  _v0 = new double[3]; _v1 = new double[3]; _v2 = new double[3]; _v3 = new double[3];
    //  _v4 = new double[3]; _v5 = new double[3]; _v6 = new double[3]; _v7 = new double[3];

    _v0[0] = _corner[0] + m_CubeSize[0];
    _v0[1] = _corner[1] + m_CubeSize[1];
    _v0[2] = _corner[2] + m_CubeSize[2];

    _v1[0] = _corner[0] ;
    _v1[1] = _corner[1] + m_CubeSize[1];
    _v1[2] = _corner[2] + m_CubeSize[2];

    _v2[0] = _corner[0] ;
    _v2[1] = _corner[1] ;
    _v2[2] = _corner[2] + m_CubeSize[2];

    _v3[0] = _corner[0] + m_CubeSize[0];
    _v3[1] = _corner[1] ;
    _v3[2] = _corner[2] + m_CubeSize[2];

    _v4[0] = _corner[0] + m_CubeSize[0];
    _v4[1] = _corner[1] ;
    _v4[2] = _corner[2] ;

    _v5[0] = _corner[0] + m_CubeSize[0];
    _v5[1] = _corner[1] + m_CubeSize[1];
    _v5[2] = _corner[2] ;

    _v6[0] = _corner[0] ;
    _v6[1] = _corner[1] + m_CubeSize[1];
    _v6[2] = _corner[2] ;

    _v7[0] = _corner[0] ;
    _v7[1] = _corner[1] ;
    _v7[2] = _corner[2] ;

    glNormal3f(0,0,1);
    glVertex3dv(_v0);    // front face
    glVertex3dv(_v1);
    glVertex3dv(_v2);
    glVertex3dv(_v3);

    glNormal3f(1,0,0);
    glVertex3dv(_v0);    // right face
    glVertex3dv(_v3);
    glVertex3dv(_v4);
    glVertex3dv(_v5);

    glNormal3f(0,1,0);
    glVertex3dv(_v0);    // up face
    glVertex3dv(_v5);
    glVertex3dv(_v6);
    glVertex3dv(_v1);

    glNormal3f(-1,0,0);
    glVertex3dv(_v1);
    glVertex3dv(_v6);
    glVertex3dv(_v7);
    glVertex3dv(_v2);

    glNormal3f(0,0,-1);
    glVertex3dv(_v4);
    glVertex3dv(_v7);
    glVertex3dv(_v6);
    glVertex3dv(_v5);

    glNormal3f(0,-1,0);
    glVertex3dv(_v7);
    glVertex3dv(_v4);
    glVertex3dv(_v3);
    glVertex3dv(_v2);
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

    if ( ENV.getInt(Env::hriCostType) == HRICS_Reachability )
    {
        if ( (Cost == numeric_limits<double>::max()) && !m_IsReachable )
        {
            return;
        }

        Cost = m_Reachability;
        GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
    }

    if( ENV.getInt(Env::hriCostType) == HRICS_Combine )
    {
        Cost = m_Combined;
        GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*Cost);
        //colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*Cost; //+0.01;
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

AgentGrid::AgentGrid( Robot* robot, Distance* distCostSpace, Visibility* VisiCostSpace, Natural* NatuCostSpace ) : 
    API::ThreeDGrid(),
    m_Robot(robot),
    m_DistanceCostSpace(distCostSpace),
    m_VisibilityCostSpace(VisiCostSpace),
    m_NaturalCostSpace(NatuCostSpace),
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

//! @brief Get the cell containing WSPoint and returns the cost
double AgentGrid::getCompleteCellCostAt(const Vector3d& WSPoint, vector<double>& costs)
{
    Vector3d pointInGrid = getTranformedToRobotFrame(WSPoint);

    AgentCell* cell = dynamic_cast<AgentCell*>(getCell(pointInGrid));

    if(cell != NULL)
    {
        costs[0] = cell->getDistance();
        costs[1] = cell->getVisibility();
        costs[2] = cell->getReachability();
        costs[3] = cell->getCombined();

        cout << "distance : " << costs[0];
        cout << " , visibility : " << costs[1];
        cout << " , reachability : " << costs[2];
        cout << " , combined : " << costs[3] << endl;
        //double cost = costs[3]
        return 0.0;
    }
    else {
        // cout << "No cell at : " << endl << pointInGrid << endl;
        return 0.0;
    }
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
    else {
        // cout << "No cell at : " << endl << pointInGrid << endl;
        return 0.0;
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

//! @brief Natural cell comparator
class AgentCellComp
{
public:

    bool operator()(pair<double,AgentCell*> first, pair<double,AgentCell*> second)
    {
        return ( first.first < second.first );
    }

} AgentCellCompObj;

//! @brief get config
int AgentGrid::robotConfigInCell(int i)
{
    return dynamic_cast<AgentCell*>( _cells[i] )->setRobotToStoredConfig();
}

void AgentGrid::computeRadius()
{  
    m_Radius = _nbCellsX*_cellSize[0]/2;
    cout << "AgentGrid::computeRadius: " << m_Radius << endl;
}

void AgentGrid::computeCostCombination()
{
    confPtr_t q = m_Robot->getCurrentPos();

    for (unsigned int i=0; i<_cells.size(); i++)
    {
        static_cast<AgentCell*>(_cells[i])->computeCombined();
    }
    m_Robot->setAndUpdate(*q);
}

//! @brief Compute Grid Cost
void AgentGrid::computeCellVectors()
{
    confPtr_t q = m_Robot->getCurrentPos();

    m_DangerCells.clear();
    m_VisibilityCells.clear();
    m_ReachableCells.clear();
    m_CombinedCells.clear();

    for (unsigned int i=0; i<_cells.size(); i++)
    {
        AgentCell* cell = static_cast<AgentCell*>(_cells[i]);

        m_DangerCells.push_back( cell );
        m_VisibilityCells.push_back( cell );

        if( cell->isReachable() )
            m_ReachableCells.push_back( cell );

        m_CombinedCells.push_back( cell );
    }
    m_Robot->setAndUpdate(*q);
}

//! @brief Compute Grid Cost
void AgentGrid::computeAllCellCost()
{
    cout << "AgentGrid::computeAllCellCost" << endl;
    //	vector<HRICS::AgentCell*> cells = getAllReachableCells();

    confPtr_t q = m_Robot->getCurrentPos();

    m_DangerCells.clear();
    m_VisibilityCells.clear();
    m_ReachableCells.clear();
    m_CombinedCells.clear();

    for ( int i=0; i<int(_cells.size()) ; i++)
    {
        AgentCell* cell = static_cast<AgentCell*>(_cells[i]);

        cell->computeDistance();
        //if ( cell->getDistance() > 0.2  )
        m_DangerCells.push_back( cell );

        cell->computeVisibility();
        //if ( cell->getVisibility() > 0.2 )
        m_VisibilityCells.push_back( cell );

        cell->computeReachability();

        if( cell->isReachable() )
            m_ReachableCells.push_back( cell );

        cell->computeCombined();
        m_CombinedCells.push_back( cell );
    }
    m_Robot->setAndUpdate(*q);
    API_activeGrid = this;
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
        //cell->drawOnePoint(configChanged);
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


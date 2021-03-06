/*
 *  HRICS_NaturalGrid.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "HRICS_natural_cell.hpp"
#include "HRICS_natural_grid.hpp"
#include "HRICS_natural.hpp"

#include "gridsAPI.hpp"

using namespace std;
using namespace HRICS;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

NaturalGrid::NaturalGrid() :
	Move3D::ThreeDGrid(),
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
	Move3D::ThreeDGrid(pace,envSize),
	m_NaturalCostSpace(costSpace),
	m_firstDisplay(true)
{
	setGridOrigin();
	
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
}

NaturalGrid::NaturalGrid(const NaturalGrid& grid) :
	Move3D::ThreeDGrid(grid),
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
Move3D::ThreeDCell* NaturalGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y,unsigned  int z )
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
	cout << "NaturalGrid::computeAllCellCost" << endl;
	vector<HRICS::NaturalCell*> cells = getAllReachableCells();

	confPtr_t q = getRobot()->getCurrentPos();

	unsigned int nbCells = cells.size();

	for (unsigned int i=0; i < nbCells; i++)
	{
		double cost = cells[i]->getCost();

		if (cost <= 0.0 )
		{
			cells[i]->setIsReachable(false);
		}
		else
		{
			cout << "cell " << i << " cost is : " << cost << endl;
		}
	}
	cout << nbCells - getAllReachableCells().size() << " cells lost!!!" << endl;

  
//  int nbCells = this->getNumberOfCells();

	
//  m_NaturalCostSpace->setRobotToConfortPosture();
  
//  for(int i=0; i<nbCells; i++)
//  {
//    cout << "cell " << i << "cost is : " << dynamic_cast<NaturalCell*>( _cells[i] )->getCost() << endl;
//  }
	
  getRobot()->setAndUpdate(*q);
  API_activeGrid = this;
}

/*!
 * \brief Compute Grid Accecibility whith right and left hand !
 */
#ifdef HRI_PLANNER
void NaturalGrid::computeReachability()
{

	int nbCells = this->getNumberOfCells();
//	m_NaturalCostSpace->setRobotToConfortPosture();
	confPtr_t robotConf = getRobot()->getInitPos();
	
	for(int i=0; i<nbCells; i++)
    {
        cout <<  "Computing Reachability of Cell : " << i << endl;
        dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->computeReachability();
        getRobot()->setAndUpdate(*robotConf);
//        m_NaturalCostSpace->setRobotToConfortPosture();

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

//! @breif Reachable Cells
//! return all reachable cells by the human
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

//! @breif Reachable Cells
//! return all reachable by one arm
std::vector<NaturalCell*> NaturalGrid::getAllReachableCellsOneArm(bool right_arm)
{
  vector<NaturalCell*> ReachableCells;
	ReachableCells.clear();
	unsigned int nbCells = this->getNumberOfCells();
	
  for(unsigned int i=0; i<nbCells; i++)
  {
    NaturalCell* cell = dynamic_cast<NaturalCell*>( _cells[i] );
		
		if ( right_arm ) {
      if(cell->isReachableWithRA() ) 
        ReachableCells.push_back( cell );
    }
    else {
      if(cell->isReachableWithLA() ) 
        ReachableCells.push_back( cell );
    }
  }
  
	return ReachableCells;
}

/*!
 * Natural cell comparator
 */
class NaturalCellComp
{
public:

	bool operator()(pair<double,NaturalCell*> first, pair<double,NaturalCell*> second)
	{
		return ( first.first < second.first );
	}

} NaturalCellCompObj;


/*!
 * @breif Reachable Cells sorted
 */
vector<pair<double,NaturalCell*> > NaturalGrid::getAllReachableCellsSorted()
{
	vector<pair<double,NaturalCell*> > ReachableCells;

	ReachableCells.clear();

	unsigned int nbCells = this->getNumberOfCells();

    for(unsigned int i=0; i<nbCells; i++)
    {
        NaturalCell* cell = dynamic_cast<NaturalCell*>( _cells[i] );

		if ( cell->isReachable() )
		{
			pair<double, NaturalCell*> costCell;
			costCell.second = cell;
			costCell.first = cell->getCost();
			ReachableCells.push_back( costCell );
		}
	}
	sort(ReachableCells.begin(),ReachableCells.end(),NaturalCellCompObj);

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
// confPtr_t q_actual = getRobot()->getCurrentPos();
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
//		 *	Eigen::AngleAxisd((*q_actual)[10], Vector3d::UnitY())
//		 *	Eigen::AngleAxisd((*q_actual)[11], Vector3d::UnitZ());
//	
//	actual.linear() = rot;
//
//	Transform3d t2( actual * getRobotOrigin().inverse() );
  
  Transform3d t2( getRobot()->getJoint(1)->getMatrixPos() * getRobotOrigin().inverse() );  
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

void NaturalGrid::drawVector( const std::vector< std::pair<double,NaturalCell*> >& cells )
{
  NaturalCell* cell=NULL;
  
  for( int i=0; i<int(cells.size()); i++ )
  {
    cell = cells[i].second; 
    cell->setUseExternalCost(true);
    cell->setExternalCost( cells[i].first );
    cell->draw();
    cell->setUseExternalCost(false);
  }
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

bool NaturalGrid::writeToXmlFile(string docname)
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
bool NaturalGrid::loadFromXmlFile(string docname)
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



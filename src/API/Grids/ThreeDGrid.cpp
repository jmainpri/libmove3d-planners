#include "ThreeDGrid.hpp"
#include "Graphic-pkg.h"

#include <iostream>
#include <libxml/parser.h>

using namespace std;
using namespace API;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

/*!
 * \brief Constructor
 *
 * \param X number of cells
 * \param Y number of cells
 * \param Z number of cells
 */
ThreeDGrid::ThreeDGrid()
{
	
}

/*!
 * \brief Destructor
 */
ThreeDGrid::~ThreeDGrid()
{
	
}

/*!
 * \brief Initializes the grid with a pace
 *
 * \param vector int size (number of cells in X, Y, Z)
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
ThreeDGrid::ThreeDGrid( Vector3i size, vector<double> envSize )

{
    _nbCellsX = size[0];
    _nbCellsY = size[1];
    _nbCellsZ = size[2];
	
    _cellSize[0] = (envSize.at(1) - envSize.at(0)) / _nbCellsX ;
    _cellSize[1] = (envSize.at(3) - envSize.at(2)) / _nbCellsY ;
    _cellSize[2] = (envSize.at(5) - envSize.at(4)) / _nbCellsZ ;
	
    _originCorner[0] = envSize.at(0);
    _originCorner[0] = envSize.at(2);
    _originCorner[0] = envSize.at(4);
	
    //    cout << "_originCorner[0] = " << _originCorner.at(0) <<  endl;
    //    cout << "_originCorner[1] = " << _originCorner.at(1) <<  endl;
    //    cout << "_originCorner[2] = " << _originCorner.at(2) <<  endl;
}


/*!
 * \brief Initializes the grid with a certain pace
 *
 * \param double pace : sizes of the squared cells IMPORTANT Cells are squared
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
ThreeDGrid::ThreeDGrid( double samplingRate, vector<double> envSize )
{
    for(unsigned int i= 0; i< envSize.size() ; i++)
    {
        cout << envSize.at(i) << " ";
    }
    cout << endl;
	
    if(((int)samplingRate) != 0 )
    {
        if( ( ((int) (envSize.at(1) - envSize.at(0))) % (int)samplingRate ) != 0 )
        {
            cout << "ThreeDGrid Warning : not good X disctretization " << endl;
        }
		
        if( ( ((int) (envSize.at(3) - envSize.at(2))) % (int)samplingRate ) != 0 )
        {
            cout << "ThreeDGrid Warning : not good Y disctretization " << endl;
        }
		
        if( ( ((int) (envSize.at(5) - envSize.at(4))) % (int)samplingRate ) != 0 )
        {
            cout << "ThreeDGrid Warning : not good Z disctretization " << endl;
        }
    }
	
    //    _cellSize.push_back( (envSize.at(1) - envSize.at(0))/pace );
    //    _cellSize.push_back( (envSize.at(3) - envSize.at(2))/pace );
    //    _cellSize.push_back( (envSize.at(5) - envSize.at(4))/pace );
	
    _cellSize[0] =  samplingRate ;
    _cellSize[1] =  samplingRate ;
    _cellSize[2] =  samplingRate ;
	
    _nbCellsX =  (envSize.at(1) - envSize.at(0)) / samplingRate ;
    _nbCellsY =  (envSize.at(3) - envSize.at(2)) / samplingRate ;
    _nbCellsZ =  (envSize.at(5) - envSize.at(4)) / samplingRate ;
	
    cout << " _nbCellsX = " << _nbCellsX << endl;
    cout << " _nbCellsY = " << _nbCellsY << endl;
    cout << " _nbCellsZ = " << _nbCellsZ << endl;
	
    _originCorner[0] = envSize.at(0);
    _originCorner[1] = envSize.at(2);
    _originCorner[2] = envSize.at(4);
	
    cout << "_originCorner[0] = " << _originCorner[0] <<  endl;
    cout << "_originCorner[1] = " << _originCorner[1] <<  endl;
    cout << "_originCorner[2] = " << _originCorner[2] <<  endl;
	
}

/*!
 * \brief Copy
 */
ThreeDGrid::ThreeDGrid( const ThreeDGrid& grid ) : 
	BaseGrid( grid ),
    _originCorner( grid._originCorner),
    _cellSize( grid._cellSize),
	_nbCellsX(grid._nbCellsX ),
	_nbCellsY(grid._nbCellsY ),
	_nbCellsZ(grid._nbCellsZ )
{
	_cells.resize( grid._cells.size() );
	
	/*
	for (unsigned int i=0; i<grid._cells.size() ; i++) 
	{
		_cells[i] = new ThreeDCell( *dynamic_cast<ThreeDCell*>(grid._cells[i]) );
		dynamic_cast<ThreeDCell*>( _cells[i] )->setGrid( this );
	}*/
}

/*!
 * \brief Creates All Cells
 *
 * \param vector envSize XMin Xmax YMin YMax ZMin ZMax
 */
void ThreeDGrid::createAllCells()
{
	
    unsigned int _nbCells = _nbCellsX * _nbCellsY * _nbCellsZ;
	
    _cells.resize(_nbCells);
	
    unsigned int x=0;
    unsigned int y=0;
    unsigned int z=0;
	
    for(unsigned int i = 0; i < _nbCells; i++)
    {

		//cout << "("<< x << "," << y << "," << z << ")" << endl;
        ThreeDCell* ptrCell = createNewCell(i,x,y,z);
        _cells[i] = ptrCell;
		
        x++;
        if( x >= _nbCellsX )
        {
            y++;
            x=0;
            if( y >= _nbCellsY )
            {
                z++;
                y=0;
                if( z > _nbCellsZ )
                {
                    cout << "ThreeDGrid : Error Size of ThreeDGrid " << endl;
                    return;
                }
            }
        }
    }
    //    cout << "Finished"<< endl;
}

/*!
 * \brief Retruns the Cell at (x,y,z)
 *
 * \param integers x, y, z
 */
ThreeDCell* ThreeDGrid::getCell(unsigned int x, unsigned int y, unsigned int z) const
{
    if(x<0 || x >= _nbCellsX)
    {
        //cout << "ThreeDGrid Error : out of bands"<< endl;
        return NULL;
    }
    if(y<0 || y >= _nbCellsY)
    {
        //cout << "ThreeDGrid Error : out of bands"<< endl;
        return NULL;
    }
    if(z<0 || z >= _nbCellsZ)
    {
        //cout << "ThreeDGrid Error : out of bands"<< endl;
        return NULL;
    }
	
    return static_cast<ThreeDCell*>(_cells[ x + y*_nbCellsX + z*_nbCellsX*_nbCellsY ]);
}

/*!
 * \brief Get Cell
 *
 * \param index
 */
ThreeDCell* ThreeDGrid::getCell(Vector3i cell) const
{
    return getCell(cell[0],cell[1],cell[2]);
}

/*!
 * \brief Get Cell in 3D ThreeDGrid
 *
 * \param index
 */
const bool DebugCell = false;
ThreeDCell* ThreeDGrid::getCell(const Vector3d& point) const
{
	if( (point[0]<_originCorner[0]) || (point[0]>(_originCorner[0]+_nbCellsX*_cellSize[0]) ))
	{
    if (DebugCell)
      cout << "Error point not in grid in " << __func__ << endl;
    return 0x00;
	}
	
	if( (point[1]<_originCorner[1]) || (point[1]>(_originCorner[1]+_nbCellsY*_cellSize[1]) ))
	{
    if (DebugCell)
      cout << "Error point not in grid in " << __func__ << endl;
    return 0x00;
	}
	
	if( (point[2]<_originCorner[2]) || (point[2]>(_originCorner[2]+_nbCellsZ*_cellSize[2]) ))
	{
    if (DebugCell)
      cout << "Error point not in grid in " << __func__ << endl;
    return 0x00;
	}
	
  unsigned int x = (unsigned int)floor((point[0]-_originCorner[0])/_cellSize[0]);
  unsigned int y = (unsigned int)floor((point[1]-_originCorner[1])/_cellSize[1]);
  unsigned int z = (unsigned int)floor((point[2]-_originCorner[2])/_cellSize[2]);
	
  //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
  return getCell(x,y,z);
}

/*!
 * \brief Get Cell in 3D ThreeDGrid
 *
 * \param index
 */
ThreeDCell* ThreeDGrid::getCell(double* pos) const
{
  Vector3d vect(pos[0],pos[1],pos[2]);
  return getCell(vect);
}

/*!
 * \brief Get place in grid
 *
 * \param index
 */
Vector3i ThreeDGrid::getCellCoord(ThreeDCell* ptrCell) const
{
    Vector3i coord;
//	
    int index = ptrCell->getIndex();
	
//    coord[0] = (i/1) % 3 - 1 ; // x
//    coord[1] = (i/3) % 3 - 1 ; // y
//    coord[2] = (i/9) % 3 - 1 ; // z
	
    int sizeXY = _nbCellsX * _nbCellsY;
    coord[2] = floor(index / sizeXY);
    coord[1] = floor((index - coord[2]*sizeXY) / _nbCellsX);
    coord[0] = floor((index - coord[2]*sizeXY) - coord[1] * _nbCellsX);
	
    return coord;
}


/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
ThreeDCell* ThreeDGrid::createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z )
{
	//cout << " ThreeDCell " << endl;
    if (index == 0)
    {
        return new ThreeDCell( 0, _originCorner , this );
    }
    ThreeDCell* newCell = new ThreeDCell( index, computeCellCorner(x,y,z) , this );
    Vector3d corner = newCell->getCorner();
    //cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;
    return newCell;
}

/*!
 * \brief Computes the corner of a cell
 *
 * \param integer index
 */
Vector3d ThreeDGrid::computeCellCorner(unsigned int x, unsigned int y, unsigned int z)
{
    Vector3d corner;
	
    corner[0] = _originCorner[0] + x*_cellSize[0];
    corner[1] = _originCorner[1] + y*_cellSize[1];
    corner[2] = _originCorner[2] + z*_cellSize[2];
	
    //    cout << " = (" << x <<"," << y << "," << z << ")" << endl;
    //    cout << " = (" << corner[0] <<"," << corner[1] << "," << corner[2] << ")" << endl;
	
    return corner;
}


/*!
 * \brief Get Neighboor Cell
 */
ThreeDCell* ThreeDGrid::getNeighbour( const Vector3i& pos, unsigned int i) const
{
  if( i<0 || i>26 )
  {
    return 0x0;
  }
  else
  {
    if(i>=13) i++;
		
    unsigned int dx =  (i/1) % 3 - 1 ;
    unsigned int dy =  (i/3) % 3 - 1 ;
    unsigned int dz =  (i/9) % 3 - 1 ;
		
    //        cout << "( "<<dx<<" , "<<dy<<" , "<<dz<<" ) "<< endl;
		
    unsigned int x = pos[0] + dx ;
    unsigned int y = pos[1] + dy ;
    unsigned int z = pos[2] + dz ;
		
    if( x>=_nbCellsX ||  y>=_nbCellsY || z>=_nbCellsZ || x<0 || y<0 || z<0 )
    {
      //            cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
      //            cout << "OutBands" << endl;
      return 0x0;
    }
    else
    {
      //            cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
      return getCell(x,y,z);
    }
  }
}

/**
 * Retrive the X Y Z coordinate of the cell from its index
 */
Vector3d ThreeDGrid::getCoordinates(ThreeDCell* cell) const
{
    Vector3d coordinates;
    int index = cell->getIndex();
    int sizeXY = _nbCellsX * _nbCellsY;
    coordinates[2] = floor(index / sizeXY);
    coordinates[1] = floor((index - coordinates[2]*sizeXY) / _nbCellsX);
    coordinates[0] = floor(index - coordinates[2]*sizeXY - coordinates[1] * _nbCellsX);
    return coordinates;
}

unsigned int ThreeDGrid::getXlineOfCell(unsigned int ith)
{	
	 //x + y*_nbCellsX + z*_nbCellsX*_nbCellsY
    return ((ith/1) % _nbCellsX - 1); // x

}

unsigned int ThreeDGrid::getYlineOfCell(unsigned int ith)
{
	return ((ith/_nbCellsX) % _nbCellsY - 1); // y
}

unsigned int ThreeDGrid::getZlineOfCell(unsigned int ith)
{
    return ((ith/(_nbCellsX*_nbCellsY)) % _nbCellsZ - 1 ); // z
}

/*!
 * \brief Draw a openGl view of the grid
 */
void ThreeDGrid::draw()
{
    double colorvector[4];
	
    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency
	
	
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
	
    glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);
	
    int nbCells = this->getNumberOfCells();
	
    for(int i=0; i<nbCells; i++)
    {
        ThreeDCell* cell = dynamic_cast<ThreeDCell*>( BaseGrid::getCell(i) );
        glColor4dv(colorvector);
        cell->draw();
    }
	
    glEnd();
	
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
	
    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
}

/*!
 * \brief Writes the grid 
 * to en xml file
 */
bool ThreeDGrid::writeToXmlFile(string docname)
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
bool ThreeDGrid::loadFromXmlFile(string docname)
{
    //Creating the file Variable version 1.0
    xmlDocPtr doc;
    xmlNodePtr cur;
	
    doc = xmlParseFile(docname.c_str());
	
    if(doc==NULL)
    {
        cout << "Document not parsed successfully (doc==NULL)" << endl;
        return false;
    }
	
    cur = xmlDocGetRootElement(doc);
	
    if (cur == NULL)
    {
        cout << "Document not parsed successfully" << endl;
        xmlFreeDoc(doc);
        return false;
	}
	
	
	if (xmlStrcmp(cur->name, xmlCharStrdup("Grid"))) 
	{
		cout << "Document of the wrong type root node not Grid" << endl;
		xmlFreeDoc(doc);
		return false;
	}
	
	
	xmlChar* tmp;
	
	tmp = xmlGetProp(cur, xmlCharStrdup("Type"));
	if (xmlStrcmp(tmp, xmlCharStrdup("3D")))
	{
		cout << "Doccument not a 3D grid"<< endl;
		xmlFreeDoc(doc);
		return false;
	}
	xmlFree(tmp);
	
	/***************************************/
	// NODE OriginCorner
	
	cur = cur->xmlChildrenNode->next;
	
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
	
    cout << "Reading Grid : " << docname << endl;
    xmlFreeDoc(doc);
    return true;
}

#include "HRICS_Grid.hpp"
#include "HRICS_Cell.hpp"

#include "HRICS_costspace.hpp"

#include "P3d-pkg.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

Grid::Grid()
{
}

Grid::Grid(vector<int> size)
{

}

Grid::Grid(double pace, vector<double> envSize) :
        API::ThreeDGrid(pace,envSize)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
}



/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
API::ThreeDCell* Grid::createNewCell(unsigned int index, unsigned int x, unsigned int y, unsigned int z )
{
    Vector3i pos;

    pos[0] = x;
    pos[1] = y;
    pos[2] = z;

    //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;

    if (index == 0)
    {
        return new Cell( 0, pos ,_originCorner , this );
    }
    return new Cell( index, pos , computeCellCorner(x,y,z) , this );
}

/*!
 * \brief Compute Grid Cost
 */
void Grid::computeAllCellCost()
{
	cout << "Grid::computeAllCellCost()" << endl;
	
    int nbCells = this->getNumberOfCells();

    shared_ptr<Configuration> robotConf = _Robot->getCurrentPos();
    for(int i=0; i<nbCells; i++)
    {
//        dynamic_cast<Cell*>( BaseGrid::getCell(i) )->getHRICostSpace();
        dynamic_cast<Cell*>( BaseGrid::getCell(i) )->getCost();
    }
    _Robot->setAndUpdate(*robotConf);
    API_activeGrid = this;
}

/*!
 * \brief Reset Grid Cost
 */
void Grid::resetCellCost()
{
    int nbCells = this->getNumberOfCells();

    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<Cell*>( BaseGrid::getCell(i) )->setBlankCost();
    }
}

/*!
 * \brief Draw Grid Cells
 */
void Grid::drawSpheres()
{
    double colorvector[4];
	
    colorvector[0] = 1.0;       //red
    colorvector[1] = 0.5;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.05;       //transparency
	
	int nbCells = this->getNumberOfCells();
	
	for(int i=0; i<nbCells; i++)
    {
        Cell* cell = dynamic_cast<Cell*>( BaseGrid::getCell(i) );
        
		if ((!cell->getIsCostComputed())) 
		{
			cell->createDisplaylist();
		}
	}
	
//    cout << "Drawing HRICS::Grid::draw()"  << endl;
	
    for(int i=0; i<nbCells; i++)
    {
		if (ENV.getBool(Env::drawOnlyOneLine)) 
		{
			//cout << "Drawing one line" << endl;
			int line=0;
			
			int xyz = ENV.getInt(Env::lineToShow);
			
			if(xyz == 0) line = getXlineOfCell(i);
			if(xyz == 1) line = getYlineOfCell(i);
			if(xyz == 2) line = getZlineOfCell(i);
			
			if (ENV.getInt(Env::hriShownGridLine) <= line) 
			{
				continue;
			}
		}
		
        Cell* cell = dynamic_cast<Cell*>( BaseGrid::getCell(i) );
        
//		if ((cell->getCenter()).norm()>1) {
//			continue;
//		}
		
		double alpha = cell->getCost();
//		cout << "Alpha : " << alpha << endl;
		
		if(ENV.getInt(Env::hriCostType) == HRICS_Distance )
        {
			if(alpha < 0.1 )
				continue;
			
//			cout << "Drawing cell " << i << endl;
//			cout << "Alpha : " << alpha << endl;
			
			// Alpha goes from 0 to 1 + Epsilon
			//GroundColorMix(colorvector,360-360*alpha+100,0,1);
			GroundColorMixGreenToRed(colorvector,alpha);
            //colorvector[1] = 0.5*(1-ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            //colorvector[3] = 0.3*ENV.getDouble(Env::colorThreshold1)*alpha; //+0.01;
        }
		
        if( ENV.getInt(Env::hriCostType) == HRICS_Visibility )
        {
//			alpha *= 14;
//			if(alpha > 0.8)
//				continue;
			
			GroundColorMixGreenToRed(colorvector,alpha);
			
            //colorvector[1] = 0.5*(1-10*ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            //colorvector[3] = 0.1*(0.7-ENV.getDouble(Env::colorThreshold1)*alpha)+0.01;
        }
        glColor4dv(colorvector);
        //        g3d_set_color_mat(Any,colorvector);
		
        //cell->draw();
		glCallList(cell->getDisplayList());
    }
}

/*!
 * \brief Draw Grid Cells
 */
void Grid::draw()
{
	if (ENV.getBool(Env::drawVectorField)) 
	{
    cout << "drawVectorFeild" << endl;
		return drawVectorFeild();
	}
	//cout << "Drawing spheres" << endl;
	return drawSpheres();
	
    double colorvector[4];

    colorvector[0] = 0.2;       //red
    colorvector[1] = 1.0;       //green
    colorvector[2] = 0.2;       //blue
    colorvector[3] = 0.05;       //transparency

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

 //   glEnable(GL_CULL_FACE);
    glBegin(GL_QUADS);
	
	const int nbCells = getNumberOfCells();

    //cout << "Drawing HRICS::Grid::draw()"  << endl;

    for(int i=0; i<nbCells; i++)
    {
        Cell* cell = dynamic_cast<Cell*>( _cells[i] );
        
		double alpha = cell->getCost();
		
		GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*alpha);
		
		if( ENV.getInt(Env::hriCostType) == HRICS_Distance )
        {
//			if(alpha < ENV.getDouble(Env::colorThreshold1))
//				continue;
			
			// Alpha goes from 0 to 1 + Epsilon
			//GroundColorMix(colorvector,360-360*alpha+100,0,1);
            //colorvector[1] = 0.5*(1-ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*alpha; //+0.01;
        }
		
        if( ENV.getInt(Env::hriCostType) == HRICS_Visibility )
		{
			colorvector[3] = ENV.getDouble(Env::colorThreshold2)*1/alpha; 
		}
			
        if ( ENV.getInt(Env::hriCostType) == HRICS_Combine )
        {
			alpha /= (ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility));
			
			//GroundColorMixGreenToRed(colorvector,ENV.getDouble(Env::colorThreshold1)*alpha);
			
            //colorvector[1] = 0.5*(1-10*ENV.getDouble(Env::colorThreshold2)*alpha)+0.5;
            //colorvector[3] = 0.20*(1-ENV.getDouble(Env::colorThreshold2)*alpha);
          colorvector[3] = 0.20*ENV.getDouble(Env::colorThreshold2)*alpha; 
        }
        glColor4dv(colorvector);
        //        g3d_set_color_mat(Any,colorvector);

        cell->draw();
    }

    glEnd();

//    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}


bool Grid::isVirtualObjectPathValid(Cell* fromCell,Cell* toCell)
{
//    shared_ptr<Configuration> configFrom(new Configuration(_Robot));
//
//    vector<double> cellCenter = fromCell->getCenter();
//
//    configFrom->print();

//    configFrom->getConfigStruct()[ENV.getInt(Env::akinJntId)] = cellCenter[0];
//    configFrom->getConfigStruct()[7] = cellCenter[1];
//    configFrom->getConfigStruct()[8] = cellCenter[2];
//    configFrom->getConfigStruct()[9] =    0;
//    configFrom->getConfigStruct()[10] =   0;
//    configFrom->getConfigStruct()[11] =   0;

    return true;
}

/*!
 * Draws the Vector Field
 */
void Grid::drawVectorFeild()
{
	//cout << "Drawing Vector Field" << endl;
	
    const int nbCells = getNumberOfCells();
	
    for(int i=0; i<nbCells; i++)
    {
        Cell* cell = dynamic_cast<Cell*>( _cells[i] );
		
		if( cell->getCost() < 0.01 )
			continue;
		
		p3d_vector3 p1;
		Vector3d p1Eigen = cell->getCenter();
		
		p1[0] = p1Eigen(0);
		p1[1] = p1Eigen(1);
		p1[2] = p1Eigen(2);
		
		p3d_vector3 p2;
		Vector3d p2Eigen = 2*cell->getCellSize().minCoeff()*cell->getGradient() + p1Eigen;
		
		p2[0] = p2Eigen(0);
		p2[1] = p2Eigen(1);
		p2[2] = p2Eigen(2);
		
		g3d_draw_arrow(p1,p2,0.1,0.7,0.0);
    }
}

/*!
 * Computes the Vector Field
 */
void Grid::computeVectorField()
{
	const unsigned int nbCells = getNumberOfCells();
	
	for (unsigned int i=0; i<nbCells; i++) 
	{
		// For Each cell in the Grid
		Cell* thisCell = dynamic_cast<Cell*>(_cells[i]);
		Vector3i CellCoord = getCellCoord( thisCell );
		vector< pair<double,Cell*> > neigh;
		
		// Compute the best from 25 Neighboors
		for (unsigned int j=0; j<26; j++) 
		{
			Cell* neighbourCell = dynamic_cast<Cell*>( getNeighbour( CellCoord , j ));
			
			if ( neighbourCell != NULL ) 
			{
				pair<double,Cell*> CellCostPair;
				CellCostPair.first	= neighbourCell->getCost();
				CellCostPair.second = neighbourCell;
				neigh.push_back( CellCostPair );
			}
		}
		
		sort(neigh.begin(),neigh.end());
		
//		cout << "neigh.size(() = " << neigh.size() << endl;
		
		// Sets the gradient
		Vector3d Gradient = neigh[0].second->getCenter() - thisCell->getCenter();
		thisCell->setGradient( Gradient );
	}
}


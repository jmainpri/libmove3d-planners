#include "HRICS_TwoDGrid.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef LIGHT_PLANNER
#include "../../lightPlanner/proto/lightPlannerApi.h"
#endif

//---------------------------------------------------------------------------
// Grid
//---------------------------------------------------------------------------
PlanGrid::PlanGrid() :
        API::TwoDGrid(),
        mRobot(0x00)
{
}

PlanGrid::PlanGrid(double pace, vector<double> envSize) :
        API::TwoDGrid(pace,envSize),
        mRobot(0x00)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
API::TwoDCell* PlanGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    Vector2i coord;

    coord[0] = x;
    coord[1] = y;

    if (index == 0)
    {
        return new PlanCell( 0, coord, _originCorner , this );
    }
    API::TwoDCell* newCell = new PlanCell( index, coord, computeCellCorner(x,y) , this );
    //    Vector2d corner = newCell->getCorner();
    //    cout << " = (" << corner[0] <<"," << corner[1] << ")" << endl;
    return newCell;
}

void PlanGrid::draw()
{
    if( mRobot == 0x00 )
    {
        std::cout << "Error : PlanGrid::draw() => No Robot "  << std::endl;
    }

#ifdef LIGHT_PLANNER
    deactivateCcCntrts(mRobot->getRobotStruct(),-1);
#else
	cout << "Warning: Lihght Planner not compiled" << endl;
#endif
	
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

    double depth = -0.60;

    //    cout << "Drawing 2D Grid"  << endl;

    double color[3];

    //    double nbCells = static_cast<double>(getNumberOfCells());

    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {

            PlanCell* Cell = dynamic_cast<PlanCell*>(getCell(x,y));

            double colorRation = ENV.getDouble(Env::colorThreshold1)-(Cell->getCost()/(ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility)));

            Vector2d center = Cell->getCenter();

            //            double colorRation = (((double)x*(double)_nbCellsY)+(double)y)/(nbCells);
            //            cout << " X = "  << _nbCellsX << " , Y = "  << _nbCellsY << endl;
            //            cout << "ColorRation[" << x*_nbCellsY+y << "]  =  "  << colorRation << endl;

            GroundColorMix(color,colorRation*ENV.getDouble(Env::colorThreshold2)*1000,0,1);
            glColor3d(color[0],color[1],color[2]);

            glVertex3d( (double)(center[0] - _cellSize[0]/2) , (double)(center[1] - _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] + _cellSize[0]/2) , (double)(center[1] - _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] + _cellSize[0]/2) , (double)(center[1] + _cellSize[1]/2), depth );
            glVertex3d( (double)(center[0] - _cellSize[0]/2) , (double)(center[1] + _cellSize[1]/2), depth );
        }
    }


    //    for(int i=0; i<nbCells; i++)
    //    {
    //        TwoDCell* cell = static_cast<TwoDCell*>(getCell(i));
    //        glColor4dv(colorvector);
    //        cell->draw();
    //    }

    glEnd();

    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);

    //    glEnable(GL_LIGHTING);
    //    glEnable(GL_LIGHT0);
}

/**
 * Write a Cost Tab to
 * an ObPlane format, this output has
 * to go through a script to be used as
 * a Cost Map
 */
void PlanGrid::writeToOBPlane() {

    FILE* fd = fopen("OB_Plane.macro", "w");

    fprintf(fd, "p3d_beg_desc P3D_OBSTACLE\n\n\n");
    fprintf(fd, "\tp3d_add_desc_poly polyhedre1\n");

    const double xMin=-25;
    const double xMax=25;

    const double yMin=-25;
    const double yMax=25;

    for (unsigned int a=0; a<_nbCellsX; a++) {
        for (unsigned int b=0; b<_nbCellsY; b++) {
            PlanCell* Cell = dynamic_cast<PlanCell*>(this->getCell(a,b));
            fprintf(fd, "\t\tp3d_add_desc_vert %.9f %.9f %.9f\n",
                    (xMax-xMin)*(((double)a)/((double)(_nbCellsX-1)))+xMin,
                    (yMax-yMin)*(((double)b)/((double)(_nbCellsY-1)))+yMin,
                    Cell->getCost());
        }
    }

    fprintf(fd, "\n");

    for (unsigned int a=0; a<_nbCellsX-1; a++) {
        for (unsigned int b=0; b<_nbCellsY-1; b++) {
            fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
                    b*_nbCellsX + a +1,
                    (b+1)*_nbCellsX + a +1,
                    b*_nbCellsX + (a+1)+1);

            fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
                    (b+1)*_nbCellsX + a+1,
                    (b+1)*_nbCellsX + (a+1)+1,
                    b*_nbCellsX + (a+1)+1);
        }
    }

    fprintf(fd, "    p3d_end_desc_poly\n");
    fprintf(fd,
            "    p3d_set_prim_pos_by_mat polyhedre1 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n");
    fprintf(fd, "p3d_end_desc\n\n");
    fprintf(fd, "p3d_set_obst_poly_color 1 Any 0.8 0.8 0.8\n");

    fclose(fd);

    cout << "Save cost tab to ObPlane format"<<endl;
}


//---------------------------------------------------------------------------
// Cell
//---------------------------------------------------------------------------
PlanCell::PlanCell() :
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0)
{

}

PlanCell::PlanCell(int i, Vector2i coord, Vector2d corner, PlanGrid* grid) :
        API::TwoDCell(i,corner,grid),
        _Coord(coord),
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0)
{
}

double PlanCell::getCost()
{
    if(mCostIsComputed && (!ENV.getBool(Env::RecomputeCellCost)))
    {
        //        return 0.0;
        return mCost;
    }

    Robot* rob = dynamic_cast<PlanGrid*>(_grid)->getRobot();

    //    double cost(0.0);
    mCost=0;
    const int nbRandShoot=10;

    for(int i=0;i<nbRandShoot;i++)
    {
        shared_ptr<Configuration> q = rob->shoot();
        q->getConfigStruct()[6] = this->getCenter()[0];
        q->getConfigStruct()[7] = this->getCenter()[1];
        //    q->print();
        mCost += q->cost();
    }
    mCost /= (double)nbRandShoot;
    mCostIsComputed = true;
    //    cout << "cost  = "  << cost << endl;
    return mCost;
}

//---------------------------------------------------------------------------
// State
//---------------------------------------------------------------------------
PlanState::PlanState( Vector2i cell , PlanGrid* grid) :
        _Grid(grid)
{
    _Cell = dynamic_cast<PlanCell*>(grid->getCell(cell));
}

PlanState::PlanState( PlanCell* cell , PlanGrid* grid) :
        _Grid(grid),
        _Cell(cell)
{

}


vector<API::State*> PlanState::getSuccessors()
{
    vector<API::State*> newStates;
    //    newStates.reserve(26);

    //    cout << "--------------------" << endl;
    for(int i=0;i<8;i++)
    {
        PlanCell* neigh = dynamic_cast<PlanCell*>(_Grid->getNeighbour( _Cell->getCoord(), i));
        if( neigh != NULL )
        {
            //            _Grid->isVirtualObjectPathValid(dynamic_cast<PlanCell*>(_Cell),neigh);
            newStates.push_back( new PlanState(neigh,_Grid));
        }
    }

    return newStates;
}

bool PlanState::isLeaf()
{
    return false;
}

bool PlanState::equal(API::State* other)
{
//    bool equal(false);
    PlanState* state = dynamic_cast<PlanState*>(other);
    Vector2i pos = _Cell->getCoord();
    for(int i=0;i<2;i++)
    {
        if( pos[i] != state->_Cell->getCoord()[i] )
        {
            //            cout << "PlanState::equal false" << endl;
            return false;
        }
    }


    //    cout << "State::equal true" << endl;
    return true;
}

void PlanState::setClosed(std::vector<PlanState*>& closedStates,std::vector<PlanState*>& openStates)
{
    //    cout << "State :: set Closed" <<endl;
    _Cell->setClosed();
}

bool PlanState::isColsed(std::vector<PlanState*>& closedStates)
{
    //    cout << "State :: get Closed" <<endl;
    return _Cell->getClosed();
}

void PlanState::setOpen(std::vector<PlanState*>& openStates)
{
    //     cout << "State :: set open" <<endl;
    _Cell->setOpen();
}


bool PlanState::isOpen(std::vector<PlanState*>& openStates)
{
    //    cout << "State :: get open" <<endl;
    return _Cell->getOpen();
}

void PlanState::reset()
{
    _Cell->resetExplorationStatus();
}

void PlanState::print()
{

}

double PlanState::computeLength(API::State *parent)
{
    PlanState* preced = dynamic_cast<PlanState*>(parent);

    Vector2d pos1 = _Cell->getCenter();
    Vector2d pos2 = preced->_Cell->getCenter();

    double dist = ( pos1 - pos2 ).norm();

    //    double cost1 = preced->_Cell->getCost();
    double cost2 = _Cell->getCost();
    double g = preced->g() + /*cost1 +*/ cost2 * dist;

    //    cout << "dist = " << dist << endl;
    //    cout << "g = " << g << endl;
    return g;
}

double PlanState::computeHeuristic( API::State *parent, API::State* goal )
{
    PlanState* state = dynamic_cast<PlanState*>(goal);

    Vector2d posGoal = state->_Cell->getCenter();
    Vector2d posThis = _Cell->getCenter();

    double dist=0;

//    dist += ( posGoal - posThis ).norm();

    return dist;
}

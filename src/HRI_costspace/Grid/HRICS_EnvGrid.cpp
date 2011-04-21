#include "HRICS_EnvGrid.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

//---------------------------------------------------------------------------
// Grid
//---------------------------------------------------------------------------
EnvGrid::EnvGrid() :
        API::TwoDGrid(),
        mRobot(0x00)
{
}

EnvGrid::EnvGrid(double pace, vector<double> envSize) :
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
API::TwoDCell* EnvGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    Vector2i coord;

    coord[0] = x;
    coord[1] = y;

    if (index == 0)
    {
        return new EnvCell( 0, coord, _originCorner , this );
    }
    API::TwoDCell* newCell = new EnvCell( index, coord, computeCellCorner(x,y) , this );
    //    Vector2d corner = newCell->getCorner();
    //    cout << " = (" << corner[0] <<"," << corner[1] << ")" << endl;
    return newCell;
}

void EnvGrid::draw()
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

    double depth = 0.0;

    //    cout << "Drawing 2D Grid"  << endl;

    double color[3];

    //    double nbCells = static_cast<double>(getNumberOfCells());

    for (unsigned int x =0;x<_nbCellsX;++x)
    {
        for (unsigned int y =0;y<_nbCellsY;++y)
        {

            EnvCell* Cell = dynamic_cast<EnvCell*>(getCell(x,y));

//            double colorRation = ENV.getDouble(Env::colorThreshold1)-(Cell->getCost()/(ENV.getDouble(Env::Kdistance)+ENV.getDouble(Env::Kvisibility)));

            double colorRation = Cell->getCost()*360;
            Vector2d center = Cell->getCenter();

            //            double colorRation = (((double)x*(double)_nbCellsY)+(double)y)/(nbCells);
            //            cout << " X = "  << _nbCellsX << " , Y = "  << _nbCellsY << endl;
            //            cout << "ColorRation[" << x*_nbCellsY+y << "]  =  "  << colorRation << endl;

//            colorRation = colorRation*ENV.getDouble(Env::colorThreshold2)*1000;
            GroundColorMix(color,colorRation,0,1);

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
 * call setBlankCost() in each cell
 */
void EnvGrid::setCellsToblankCost()
{
    for (int i = 0; i < getNumberOfCells(); i++)
    {
        dynamic_cast<EnvCell*>(this->getCell(i))->setBlankCost();
    }

}

///**
// * Write a Cost Tab to
// * an ObPlane format, this output has
// * to go through a script to be used as
// * a Cost Map
// */
//void EnvGrid::writeToOBPlane() {

//    FILE* fd = fopen("OB_Plane.macro", "w");

//    fprintf(fd, "p3d_beg_desc P3D_OBSTACLE\n\n\n");
//    fprintf(fd, "\tp3d_add_desc_poly polyhedre1\n");

//    const double xMin=-25;
//    const double xMax=25;

//    const double yMin=-25;
//    const double yMax=25;

//    for (unsigned int a=0; a<_nbCellsX; a++) {
//        for (unsigned int b=0; b<_nbCellsY; b++) {
//            EnvCell* Cell = dynamic_cast<EnvCell*>(this->getCell(a,b));
//            fprintf(fd, "\t\tp3d_add_desc_vert %.9f %.9f %.9f\n",
//                    (xMax-xMin)*(((double)a)/((double)(_nbCellsX-1)))+xMin,
//                    (yMax-yMin)*(((double)b)/((double)(_nbCellsY-1)))+yMin,
//                    Cell->getCost());
//        }
//    }

//    fprintf(fd, "\n");

//    for (unsigned int a=0; a<_nbCellsX-1; a++) {
//        for (unsigned int b=0; b<_nbCellsY-1; b++) {
//            fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
//                    b*_nbCellsX + a +1,
//                    (b+1)*_nbCellsX + a +1,
//                    b*_nbCellsX + (a+1)+1);

//            fprintf(fd, "\t\tp3d_add_desc_face %d %d %d \n",
//                    (b+1)*_nbCellsX + a+1,
//                    (b+1)*_nbCellsX + (a+1)+1,
//                    b*_nbCellsX + (a+1)+1);
//        }
//    }

//    fprintf(fd, "    p3d_end_desc_poly\n");
//    fprintf(fd,
//            "    p3d_set_prim_pos_by_mat polyhedre1 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n");
//    fprintf(fd, "p3d_end_desc\n\n");
//    fprintf(fd, "p3d_set_obst_poly_color 1 Any 0.8 0.8 0.8\n");

//    fclose(fd);

//    cout << "Save cost tab to ObPlane format"<<endl;
//}


//---------------------------------------------------------------------------
// Cell
//---------------------------------------------------------------------------
EnvCell::EnvCell() :
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0)
{

}

EnvCell::EnvCell(int i, Vector2i coord, Vector2d corner, EnvGrid* grid) :
        API::TwoDCell(i,corner,grid),
        _Coord(coord),
        _Open(false),
        _Closed(false),
        mCostIsComputed(false),
        mCost(0.0)
{
}

double EnvCell::getCost()
{
    if(mCostIsComputed && (!ENV.getBool(Env::RecomputeCellCost)))
    {
        return mCost;
    }

    Robot* rob = dynamic_cast<EnvGrid*>(_grid)->getRobot();
    mCost=0;
    shared_ptr<Configuration> q_cur = rob->getCurrentPos();

    shared_ptr<Configuration> q_tmp = rob->getCurrentPos();
    int firstIndexOfDof = dynamic_cast<p3d_jnt*>(rob->getRobotStruct()->baseJnt)->user_dof_equiv_nbr;

    (*q_tmp)[firstIndexOfDof + 0] = this->getCenter()[0];
    (*q_tmp)[firstIndexOfDof + 1] = this->getCenter()[1];

    rob->setAndUpdate(*q_tmp);

    mCost=0.1;
    if (rob->isInCollision())
    {
        mCost=1.0;
    }

    rob->setAndUpdate(*q_cur);
    mCostIsComputed = true;

    return mCost;
}



//---------------------------------------------------------------------------
// State
//---------------------------------------------------------------------------
EnvState::EnvState( Vector2i cell , EnvGrid* grid) :
        _Grid(grid)
{
    _Cell = dynamic_cast<EnvCell*>(grid->getCell(cell));
}

EnvState::EnvState( EnvCell* cell , EnvGrid* grid) :
        _Grid(grid),
        _Cell(cell)
{

}


vector<API::State*> EnvState::getSuccessors()
{
    vector<API::State*> newStates;
    //    newStates.reserve(26);

    //    cout << "--------------------" << endl;
    for(int i=0;i<8;i++)
    {
        EnvCell* neigh = dynamic_cast<EnvCell*>(_Grid->getNeighbour( _Cell->getCoord(), i));
        if( neigh != NULL )
        {
            //            _Grid->isVirtualObjectPathValid(dynamic_cast<EnvCell*>(_Cell),neigh);
            newStates.push_back( new EnvState(neigh,_Grid));
        }
    }

    return newStates;
}

bool EnvState::isLeaf()
{
    return false;
}

bool EnvState::equal(API::State* other)
{
//    bool equal(false);
    EnvState* state = dynamic_cast<EnvState*>(other);
    Vector2i pos = _Cell->getCoord();
    for(int i=0;i<2;i++)
    {
        if( pos[i] != state->_Cell->getCoord()[i] )
        {
            //            cout << "EnvState::equal false" << endl;
            return false;
        }
    }


    //    cout << "State::equal true" << endl;
    return true;
}

void EnvState::setClosed(std::vector<EnvState*>& closedStates,std::vector<EnvState*>& openStates)
{
    //    cout << "State :: set Closed" <<endl;
    _Cell->setClosed();
}

bool EnvState::isColsed(std::vector<EnvState*>& closedStates)
{
    //    cout << "State :: get Closed" <<endl;
    return _Cell->getClosed();
}

void EnvState::setOpen(std::vector<EnvState*>& openStates)
{
    //     cout << "State :: set open" <<endl;
    _Cell->setOpen();
}


bool EnvState::isOpen(std::vector<EnvState*>& openStates)
{
    //    cout << "State :: get open" <<endl;
    return _Cell->getOpen();
}

void EnvState::reset()
{
    _Cell->resetExplorationStatus();
}

void EnvState::print()
{

}

double EnvState::computeLength(API::State *parent)
{
    EnvState* preced = dynamic_cast<EnvState*>(parent);

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

double EnvState::computeHeuristic( API::State *parent, API::State* goal )
{
    EnvState* state = dynamic_cast<EnvState*>(goal);

    Vector2d posGoal = state->_Cell->getCenter();
    Vector2d posThis = _Cell->getCenter();

    double dist=0;

//    dist += ( posGoal - posThis ).norm();

    return dist;
}

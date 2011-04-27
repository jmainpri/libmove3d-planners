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
        mRobot(0x00),
        mHuman(0x00),
        _isHumanCentered(false)
{
}

EnvGrid::EnvGrid(double pace, vector<double> envSize, bool isHumanCentered) :
        API::TwoDGrid(pace,envSize),
        mRobot(0x00),
        mHuman(0x00),
        _isHumanCentered(isHumanCentered)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
    ENV.setDouble(Env::optimalDistFactor,0.5);
    ENV.setDouble(Env::robotMaximalDistFactor,0);
    ENV.setDouble(Env::gazeAngleFactor,1.0);
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

            double colorRation = Cell->getCost();//*360;
            Vector2d center = Cell->getCenter();

            //            double colorRation = (((double)x*(double)_nbCellsY)+(double)y)/(nbCells);
            //            cout << " X = "  << _nbCellsX << " , Y = "  << _nbCellsY << endl;
            //            cout << "ColorRation[" << x*_nbCellsY+y << "]  =  "  << colorRation << endl;

//            colorRation = colorRation*ENV.getDouble(Env::colorThreshold2)*1000;
            GroundColorMixGreenToRed(color,colorRation);

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

class CellComp
{
public:

	bool operator()(EnvCell* first, EnvCell* second)
	{
		return ( first->getCost() < second->getCost() );
	}

} CellCompObject;

vector<EnvCell*> EnvGrid::getSortedCells()
{
    vector<EnvCell*> cells;
    for (unsigned int i=0; i < _cells.size(); i++)
    {
        cells.push_back(dynamic_cast<EnvCell*>(_cells.at(i)));
    }

    sort(cells.begin(), cells.end(),CellCompObject);
    return cells;
}

void EnvGrid::recomputeCostRobotOnly()
{
    ENV.setDouble(Env::optimalDistFactor,0);
    ENV.setDouble(Env::robotMaximalDistFactor,1);
    ENV.setDouble(Env::gazeAngleFactor,0);
    ENV.setDouble(Env::robotMaximalDist,6);

    setCellsToblankCost();



}

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
    if(mCostIsComputed)
    {
        return mCost;
    }

    if (!dynamic_cast<EnvGrid*>(_grid)->isHumanCentered())
    {

        Robot* human = dynamic_cast<EnvGrid*>(_grid)->getHuman();
        shared_ptr<Configuration> q_human_cur = human->getCurrentPos();
        shared_ptr<Configuration> q_human = human->getCurrentPos();
        (*q_human)[6] = 0;
        (*q_human)[7] = 0;
        human->setAndUpdate(*q_human);


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

        human->setAndUpdate(*q_human_cur);
        return mCost;
    }
    else
    {
        double optimalDist = ENV.getDouble(Env::optimalDist);

        double robotDistanceMax = ENV.getDouble(Env::robotMaximalDist);

        Robot* r =dynamic_cast<EnvGrid*>(_grid)->getHuman();
        if (r->getName().find( "ACHILE") != string::npos)
        {
            int mIndexOfDoF = r->getJoint("Pelvis")->getIndexOfFirstDof();
            double x = (*r->getCurrentPos())[mIndexOfDoF + 0];
            double y = (*r->getCurrentPos())[mIndexOfDoF + 1];

            //the distance
            double distance = std::sqrt(std::pow(x - getCenter()[0],2) + std::pow((y - getCenter()[1]),2));

            double distanceParam = 0.0;
            if (distance < optimalDist)
            {
                distanceParam = 1 - std::exp(-(std::pow((distance - optimalDist)*2,2)));
            }
            else
            {
                distanceParam = (distance - optimalDist)/8.0;
            }
            if (distanceParam > 1){ distanceParam = 1.; }
            else if (distanceParam < 0){ distanceParam = 0.; }

            // get the rotation of the human
            double rot = (*r->getCurrentPos())[mIndexOfDoF + 5];
            double angle = atan2((getCenter()[1] - y ), getCenter()[0] - x);

            double fieldOfVision = 1.0;

            // the opening angle of the field of vision
            double cAngle = ENV.getDouble(Env::gazeAngle)*M_PI/180.0 ;
            if ((angle < cAngle + rot && angle > 0 + rot) || (cAngle + rot > M_PI && angle < cAngle + rot - 2*M_PI))
            {
                fieldOfVision = tan(angle - rot) / tan(cAngle);
            }
            else if ((angle > -cAngle + rot && angle < 0 + rot) || (-cAngle + rot < -M_PI && angle > -cAngle + rot + 2*M_PI))
            {
                fieldOfVision = -tan(angle - rot ) / tan(cAngle);
            }

            Robot* r2 = dynamic_cast<EnvGrid*>(_grid)->getRobot();

            double RobotPreference = 1.0;
            if (r2->getName().find( "JIDO") != string::npos ||r2->getName().find( "PR2") != string::npos)
            {
                // distance between robot (jido) and the point
                double distance2 = std::sqrt(std::pow((*r2->getCurrentPos())[mIndexOfDoF + 0] - getCenter()[0],2) + std::pow((*r2->getCurrentPos())[mIndexOfDoF + 1] - getCenter()[1],2));
                RobotPreference = distance2/robotDistanceMax;
                if (RobotPreference > 1.0)
                {
                    RobotPreference = 1.0;
                }
            }


              int type =  ENV.getInt(Env::typeRobotBaseGrid);
              if (type == 0)
              {
                  double a = ENV.getDouble(Env::optimalDistFactor);
                  double b = ENV.getDouble(Env::robotMaximalDistFactor);
                  double c = ENV.getDouble(Env::gazeAngleFactor);
                  double d = a + b + c;
                  a = a / d;
                  b = b / d;
                  c = c / d;

                  mCost = 1 - (a * ( 1 - distanceParam) + b * (1 - RobotPreference) + c * (1 - fieldOfVision));
                  // 1 - (weighted sum of : human ditance distanceParam , robot distance and field of vision)
              }

              else if (type == 1){ mCost = distanceParam; }
              else if (type == 2){ mCost = RobotPreference; }
              else if (type == 3){ mCost = fieldOfVision; }

        }

        mCostIsComputed = true;


        return mCost;
    }
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

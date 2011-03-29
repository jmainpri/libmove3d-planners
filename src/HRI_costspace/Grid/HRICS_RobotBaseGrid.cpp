#include "HRICS_RobotBaseGrid.hpp"

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;


extern string global_ActiveRobotName;

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

//---------------------------------------------------------------------------
// Grid
//---------------------------------------------------------------------------
RobotBaseGrid::RobotBaseGrid() :
        API::TwoDGrid()
{
}


RobotBaseGrid::RobotBaseGrid(double pace, vector<double> envSize, Natural* costSpace) :
        API::TwoDGrid(pace,envSize),
        m_NaturalCostSpace(costSpace)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY << endl;
    m_RobotOriginPos = getNaturalCostSpace()->getGridOriginMatrix();
}


/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
API::TwoDCell* RobotBaseGrid::createNewCell(unsigned int index,unsigned  int x,unsigned  int y )
{
    Vector2i coord;

    coord[0] = x;
    coord[1] = y;

    if (index == 0)
    {
        return new RobotBaseCell( 0, coord, _originCorner , this );
    }
    API::TwoDCell* newCell = new RobotBaseCell( index, coord, computeCellCorner(x,y) , this );
    return newCell;
}

void RobotBaseGrid::draw()
{
    unsigned int nbCells = this->getNumberOfCells();

    for(unsigned int i=0; i<nbCells; i++)
    {
        dynamic_cast<RobotBaseCell*>( TwoDGrid::getCell(i) )->draw();
    }
}

Eigen::Transform3d RobotBaseGrid::getTransformFromRobotPos()
{
    Transform3d t2( getRobot()->getJoint(1)->getMatrixPos() * getRobotOrigin().inverse() );
    return t2;
}

Robot* RobotBaseGrid::getRobot()
{
    return this->getNaturalCostSpace()->getRobot();
}

void RobotBaseGrid::recomputeAllCosts()
{
    unsigned int nbCells = this->getNumberOfCells();

    for(unsigned int i=0; i<nbCells; i++)
    {
        dynamic_cast<RobotBaseCell*>( TwoDGrid::getCell(i) )->setBlankCost();
        dynamic_cast<RobotBaseCell*>( TwoDGrid::getCell(i) )->getCost();
    }
}

//---------------------------------------------------------------------------
// Cell
//---------------------------------------------------------------------------
RobotBaseCell::RobotBaseCell() :
        mCostIsComputed(false),
        mCost(0.0)
{

}

RobotBaseCell::RobotBaseCell(int i, Vector2i coord, Vector2d corner, RobotBaseGrid* grid) :
        API::TwoDCell(i,corner,grid),
        _Coord(coord),
        mCostIsComputed(false),
        mCost(1.0)
{
}

Vector3d RobotBaseCell::getWorkspacePoint()
{
    RobotBaseGrid* g = dynamic_cast<RobotBaseGrid*>(_grid);
    Vector3d vec3(getCenter()[0], getCenter()[1], mCost);
    vec3 =  g->getTransformFromRobotPos() * vec3;
    return vec3;
}

double RobotBaseCell::getCost()
{
    double optimalDist = ENV.getDouble(Env::optimalDist);

    double robotDistanceMax = ENV.getDouble(Env::robotMaximalDist);


    if (!mCostIsComputed)
    {
        Robot* r =dynamic_cast<RobotBaseGrid*>(_grid)->getNaturalCostSpace()->getRobot();
//        cout << r->getName() << endl;
//        cout << _index << endl;
        if (r->getName() == "HUMAN_ACHILE")
        {
//            shared_ptr<Configuration> q = r->getCurrentPos();
            int mIndexOfDoF = 6;
            double x = (*r->getCurrentPos())[mIndexOfDoF + 0];
            double y = (*r->getCurrentPos())[mIndexOfDoF + 1];

            //the distance with a distordance parameter
            double distance = std::sqrt(std::pow(x - getWorkspacePoint()[0],2) + std::pow((y - getWorkspacePoint()[1]),2));

            // get the rotation of the human
            double rot = (*r->getCurrentPos())[mIndexOfDoF + 5];
            double angle = atan2((getWorkspacePoint()[1] - y ), getWorkspacePoint()[0] - x);

            double fieldOfVision = 1.0;

            // the opening angle of the field of vision
            double cAngle = ENV.getDouble(Env::gazeAngle)*M_PI/180.0 ;
            if (angle < cAngle + rot && angle > 0 + rot || (cAngle + rot > M_PI && angle < cAngle + rot - 2*M_PI))
            {
                fieldOfVision = tan(angle - rot) / tan(cAngle);
            }
            else if (angle > -cAngle + rot && angle < 0 + rot || (-cAngle + rot < -M_PI && angle > -cAngle + rot + 2*M_PI))
            {
                fieldOfVision = -tan(angle - rot ) / tan(cAngle);
            }

            Scene* environnement = global_Project->getActiveScene();

            Robot* r2 = environnement->getRobotByNameContaining(global_ActiveRobotName);

            double RobotPreference = 1.0;
            if (r2->getName() == "JIDOKUKA_ROBOT")
            {
                // distance between robot (jido) and the point
                double distance2 = std::sqrt(std::pow((*r2->getCurrentPos())[mIndexOfDoF + 0] - getWorkspacePoint()[0],2) + std::pow((*r2->getCurrentPos())[mIndexOfDoF + 1] - getWorkspacePoint()[1],2));
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

                  mCost = 1 - (a * std::exp(-(std::pow((distance - optimalDist)*2,2))) + b * (1 - RobotPreference) + c * (1 - fieldOfVision));
                  // 1 - (weighted sum of : human ditance (e^(-(2*(x-optimalDist)^2))) ), robot distance and field of vision)
              }

              else if (type == 1){ mCost = 1 - std::exp(-(std::pow((distance - optimalDist)*2,2))); }
              else if (type == 2){ mCost = RobotPreference; }
              else if (type == 3){ mCost = fieldOfVision; }

        }

        mCostIsComputed = true;
    }

    return mCost;
}


void RobotBaseCell::draw()
{
	if (!mCostIsComputed)
	{
		getCost();
	}

//	if (mCost != 0.0)
//	{
		Vector3d center = getWorkspacePoint();


		double colorvector[4];

		colorvector[0] = 0.5;       //red
		colorvector[1] = 0.5;       //green
		colorvector[2] = 0.5;       //blue
		colorvector[3] = 0.01;       //transparency

		GroundColorMixGreenToRed(colorvector,mCost);

		double diagonal = getCellSize().minCoeff();
		g3d_set_color(Any,colorvector);
		g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/6, 10);

//	}
}

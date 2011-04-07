#include "HRICS_Cell.hpp"

#ifdef HRI_PLANNER
#include <hri/hri.h>
#endif

#include "../HRICS_costspace.hpp"

#include "Graphic-pkg.h"

//HRICS::MainPlanner* HRICS_MOPL = NULL;

using namespace std;
using namespace tr1;
using namespace HRICS;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

Cell::Cell() :
        _Open(false),
        _Closed(false),
		_CostIsComputed(false)
{

}

Cell::Cell(int i, Vector3i coord , Vector3d corner, Grid* grid) :
        API::ThreeDCell(i,corner,grid),
        _Open(false),
        _Closed(false),
        _CostIsComputed(false)
{
    _Coord = coord;
    _v0 = new double[3]; _v1 = new double[3]; _v2 = new double[3]; _v3 = new double[3];
    _v4 = new double[3]; _v5 = new double[3]; _v6 = new double[3]; _v7 = new double[3];

    _v0[0] = _corner[0] + _grid->getCellSize()[0];
    _v0[1] = _corner[1] + _grid->getCellSize()[1];
    _v0[2] = _corner[2] + _grid->getCellSize()[2];

    _v1[0] = _corner[0] ;
    _v1[1] = _corner[1] + _grid->getCellSize()[1];
    _v1[2] = _corner[2] + _grid->getCellSize()[2];

    _v2[0] = _corner[0] ;
    _v2[1] = _corner[1] ;
    _v2[2] = _corner[2] + _grid->getCellSize()[2];

    _v3[0] = _corner[0] + _grid->getCellSize()[0];
    _v3[1] = _corner[1] ;
    _v3[2] = _corner[2] + _grid->getCellSize()[2];

    _v4[0] = _corner[0] + _grid->getCellSize()[0];
    _v4[1] = _corner[1] ;
    _v4[2] = _corner[2] ;

    _v5[0] = _corner[0] + _grid->getCellSize()[0];
    _v5[1] = _corner[1] + _grid->getCellSize()[1];
    _v5[2] = _corner[2] ;

    _v6[0] = _corner[0] ;
    _v6[1] = _corner[1] + _grid->getCellSize()[1];
    _v6[2] = _corner[2] ;

    _v7[0] = _corner[0] ;
    _v7[1] = _corner[1] ;
    _v7[2] = _corner[2] ;

}

double Cell::getCost()
{
    if(_CostIsComputed && (!ENV.getBool(Env::RecomputeCellCost)))
    {
        return _Cost;
    }

//	cout << "HRICS::Cell::getCost()" << endl;
	
	Vector3d cellCenter = this->getCenter();
	
	if (ENV.getBool(Env::HRINoRobot)) 
	{
		p3d_vector3 robot;
		p3d_vector3 human;
		
		switch ( ENV.getInt(Env::hriCostType))
        {
			case HRICS_Distance :
			{
				_Cost = HRICS_MotionPL->getDistance()->getWorkspaceCost(cellCenter);
				cout << "Cost = " << _Cost << endl;
			}
				break;
			case HRICS_Visibility :
				//_Cost = HRICS_MotionPL->getVisibility()->getCost(cellCenter);
				_Cost = HRICS_MotionPL->getVisibility()->akinVisibilityCost(cellCenter);
				break;
//			case 2 :
//				break;
//			case 3 :
//				break;
			case HRICS_Combine :
				_Cost = ENV.getDouble(Env::Kdistance)*(HRICS_MotionPL->getDistance()->computeBoundingBalls(cellCenter,robot,human));
				_Cost += ENV.getDouble(Env::Kvisibility)*(HRICS_MotionPL->getVisibility()->getCost(cellCenter));
				break;
			default:
				cout << "Type of Cost undefine in Grid "  << endl;
        }
		
		_CostIsComputed = true;
		return _Cost;
	}

    Robot* rob = dynamic_cast<Grid*>(this->_grid)->getRobot();

    shared_ptr<Configuration> configStored = rob->getCurrentPos();
	shared_ptr<Configuration> config = rob->getCurrentPos();

#if LIGHT_PLANNER
    (*config)[rob->getObjectDof()+0] = cellCenter[0];
		(*config)[rob->getObjectDof()+1] = cellCenter[1];
    (*config)[rob->getObjectDof()+2] = cellCenter[2];
#endif
	
    rob->setAndUpdate(*config);

    if(ENV.getBool(Env::HRIPlannerWS))
    {
        switch ( ENV.getInt(Env::hriCostType))
        {
        case HRICS_Distance :
            _Cost = HRICS_MotionPL->getDistance()->getDistToZones()[0];
//			cout << "Cost = " << _Cost << endl;
            break;
        case HRICS_Visibility :
			_Cost = HRICS_MotionPL->getVisibility()->getCost(cellCenter);
            break;
		case 2 :
			break;
		case 3 :
			break;
        case HRICS_Combine :
            _Cost = ENV.getDouble(Env::Kdistance)*(HRICS_MotionPL->getDistance()->getDistToZones()[0]);
            _Cost += ENV.getDouble(Env::Kvisibility)*(HRICS_MotionPL->getVisibility()->getCost(cellCenter));
            break;
        default:
            cout << "Type of Cost undefine in Grid "  << endl;
        }
    }

    if(ENV.getBool(Env::HRIPlannerCS))
    {
        switch ( ENV.getInt(Env::hriCostType))
        {
        case 0 :
            _Cost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPL)->getDistanceCost();
            break;
        case 1 :
            _Cost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPL)->getVisibilityCost(cellCenter);
            break;
        case 2 :
            _Cost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPL)->getConfigCost();
            break;
        default:
            cout << "Type of Cost undefine in Grid "  << endl;
        }
    }

    _CostIsComputed = true;
    rob->setAndUpdate(*configStored);
//	cout << "Cost = " << _Cost << endl;
    return _Cost;
}

void Cell::resetExplorationStatus()
{
    //    cout << "Reseting Cell " << this << endl;
    _Open = false;
    _Closed =false;
}

void Cell::createDisplaylist()
{
	Vector3d center = getCenter();
	
	m_list=glGenLists(1);
	glNewList(m_list, GL_COMPILE);
	double diagonal = getCellSize().minCoeff();
	g3d_draw_solid_sphere(center[0], center[1], center[2], diagonal/6, 10);
	//g3d_drawSphere(center[0], center[1], center[2], ENV.getDouble(Env::CellSize)/2 );
	glEndList();
}

void Cell::draw()
{
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

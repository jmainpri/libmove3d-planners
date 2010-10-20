//
// C++ Implementation: robot
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "API/Device/joint.hpp"
#include "API/Device/robot.hpp"
#include "API/Grids/gridsAPI.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Collision-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

Robot* API_activeRobot = NULL;

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

//constructor and destructor
//Robot::Robot(p3d_rob* R,Graph* G)
//{
//	_Robot = R;
//	_nbCreatedGraph = 0;
//
//	if(!G)
//	{
//		G = this->newGraph();
//	}
//	else
//	{
//		this->insertGraph(G);
//		setActivGraph(nbGraph() - 1);
//	}
//
//	_Robot->GRAPH = G->getGraphStruct();
//	_Name = _Robot->name;
//}

//Robot::Robot(p3d_rob* R)
//{
//    _Robot = R;
//    string name(R->name);
//    _Name = name;
//}

Robot::Robot(p3d_rob* robotPt, bool copy )
{
	_copy = copy;
	
	if (_copy) {
		_Robot = copyRobStructure(robotPt);
	}
	else {
		_Robot = robotPt;
	}


    string name(robotPt->name);
    _Name = name;
	
	m_Joints.clear();
	
	for (int i=0; i<=_Robot->njoints; i++) 
	{
		m_Joints.push_back( new Joint( this , _Robot->joints[i] , _copy ) );
	}
}

Robot::~Robot()
{
	if(_copy)
	{
		deleteRobStructure(_Robot);
	}
}

string Robot::getName()
{
    return _Name;
}

//Accessors
p3d_rob* Robot::getRobotStruct()
{
    return _Robot;
}

/**
 * Gets traj
 * @return pointer to structure p3d_traj
 */
p3d_traj* Robot::getTrajStruct()
{
	return _Robot->tcur;
}

API::Trajectory Robot::getCurrentTraj()
{
	API::Trajectory traj(this,_Robot->tcur);
	return traj;
}

unsigned int Robot::getNumberOfJoints()
{
    return m_Joints.size();
}

Joint* Robot::getJoint(unsigned int i)
{
	return m_Joints[i];
}

vector<Vector3d> Robot::getObjectBox()
{
//	if (m_ObjectBox.empty()) {
//		cout << "Warning : " << __func__ << " : no Object Box" << endl;
//	}
	
	vector<Vector3d> box(8);
	
	box[0][0] = 1;		box[4][0] = -1;
	box[0][1] = -1;		box[4][1] = -1;
	box[0][2] = 1;		box[4][2] = 1;
	
	box[1][0] = 1;		box[5][0] = -1;
	box[1][1] = 1;		box[5][1] = -1;
	box[1][2] = 1;		box[5][2] = 1;
	
	box[2][0] = 1;		box[6][0] = -1;
	box[2][1] = -1;		box[6][1] = -1;
	box[2][2] = -1;		box[6][2] = -1;
	
	box[3][0] = 1;		box[7][0] = -1;
	box[3][1] = 1;		box[7][1] = 1;
	box[3][2] = -1;		box[7][2] = -1;
	
/*	vector<Vector3d> box;
	box.clear();
	
	Transform3d T = getJoint(1)->getMatrixPos();
	
	for (unsigned int i=0; i<8; i++) 
	{
		box.push_back(m_ObjectBox[i]);
	}*/
	
	Matrix3d A = Matrix3d::Zero();
	
    A(0,0) = m_ObjectBoxDimentions[0]/2;
    A(1,1) = m_ObjectBoxDimentions[1]/2;
    A(2,2) = m_ObjectBoxDimentions[2]/2;
	
	for (unsigned int i=0; i<8; i++) 
	{
		box[i] = A*box[i];
		box[i] = m_ObjectBoxCenter+box[i];
	}
	
	return box;
}

void Robot::initObjectBox()
{
	
}

#ifdef LIGHT_PLANNER

bool Robot::isActiveCcConstraint()
{
	for(int i = 0; i < _Robot->nbCcCntrts; i++)
	{
		if(_Robot->ccCntrts[i]->active)
		{
			return true;
		}
	}
	
	return false;
}

/**
 * Activate Constraint
 */
void Robot::activateCcConstraint()
{
	activateCcCntrts(_Robot,-1,false);
	desactivateTwoJointsFixCntrt(_Robot,_Robot->curObjectJnt,
															 _Robot->ccCntrts[0]->pasjnts[ _Robot->ccCntrts[0]->npasjnts-1 ]
															 );
}

/**
 * Deactivate Constraint
 */
void Robot::deactivateCcConstraint()
{
	deactivateCcCntrts(_Robot,-1);
	setAndActivateTwoJointsFixCntrt(_Robot,_Robot->curObjectJnt,
																	_Robot->ccCntrts[0]->pasjnts[ _Robot->ccCntrts[0]->npasjnts-1 ]
																	);
}

/**
 * Returns the Virtual object dof
 */
int Robot::getObjectDof() 
{
	if (_Robot->curObjectJnt) 
	{
		return _Robot->curObjectJnt->index_dof;
	}
	else 
	{
		return 0;
		std::cout << "Robot::getObjectDof()::Warning" << std::endl;
	}
}


/**
 * Returns the base Joint
 */
p3d_jnt* Robot::getBaseJnt()
{
	return _Robot->baseJnt;
}

/**
 * Shoots the base Joint of the robot
 */
shared_ptr<Configuration> Robot::shootBaseWithoutCC()
{
	shared_ptr<Configuration> q = getCurrentPos();
	m_Joints[_Robot->baseJnt->num]->shoot(*q);
	cout << "Base Num = " << _Robot->baseJnt->num << endl;
	deactivateCcConstraint();
	setAndUpdate(*q);
	activateCcConstraint();
	return getCurrentPos();
}

/**
 * Shoots all exept base
 */
shared_ptr<Configuration> Robot::shootAllExceptBase()
{
	shared_ptr<Configuration> q( new Configuration(this));
	
	for (unsigned int i=0; i<m_Joints.size(); i++) 
	{
		if (_Robot->baseJnt->num == ((int)i)) 
		//if (i<2)
		{
			m_Joints[i]->setConfigFromDofValues(*q);
		}
		else
		{
			m_Joints[i]->shoot(*q);
		}
	}
	
	return q;
}

bool Robot::setAndUpdateAllExceptBase(Configuration& q)
{
	for(int i=0; i<(int)m_Joints.size(); i++) 
	{
		p3d_jnt* jntPt = m_Joints[i]->getJointStruct();
		
		//if (_Robot->baseJnt->num == ((int)i)) 
		if ( i <= 1)
		{ continue; }
		
		for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			int k = jntPt->index_dof+j;
			
			m_Joints[i]->setJointDof(j,q[k]); 
		}
	}
	
	return p3d_update_this_robot_pos(_Robot);
}

shared_ptr<Configuration> Robot::shootFreeFlyer(double* box)
{
	//    cout << "box  = ( " << box[0] << " , " ;
	//    cout << box[1] << " , " ;
	//    cout << box[2] << " , " ;
	//    cout << box[3] << " , " ;
	//    cout << box[4] << " , " ;
	//    cout << box[5] << " )" << endl;
	shared_ptr<Configuration> q(new Configuration(this));
	p3d_FreeFlyerShoot(_Robot, q->getConfigStruct(), box);
	return q;
}


void Robot::shootObjectJoint(Configuration& Conf)
{
	/*shared_ptr<Configuration> q(new Configuration(Conf));
	
	vector<Vector3d> FFBox = getObjectBox();
	
	Vector3d center(Vector3d::Zero());
	
	double box[8][3];
	
	for (int i=0; i<8; i++) 
	{
		for (int j=0; j<3; j++) 
		{
			box[i][j] = FFBox[i][j];
		}
	}*/
	
	shared_ptr<Configuration> q(new Configuration(Conf));
	deactivateCcConstraint();
	
	p3d_shoot_justin_left_arm(_Robot, q->getConfigStruct(), false);
	setAndUpdate(*q);
	q = getCurrentPos();
	
	activateCcConstraint();
	
	int ObjectDof = getObjectDof();
	
	Conf[ObjectDof+0] = (*q)[ObjectDof+0];
	Conf[ObjectDof+1] = (*q)[ObjectDof+1];
	Conf[ObjectDof+2] = (*q)[ObjectDof+2];
	Conf[ObjectDof+3] = (*q)[ObjectDof+3];
	Conf[ObjectDof+4] = (*q)[ObjectDof+4];
	Conf[ObjectDof+5] = (*q)[ObjectDof+5];
	
	//p3d_JointFreeFlyerShoot(_Robot,_Robot->curObjectJnt,q->getConfigStruct(),box);
}

/**
 * Shoots a random direction
 */
shared_ptr<Configuration> Robot::shootRandomDirection()
{
	shared_ptr<Configuration> q( new Configuration(this) );
	p3d_RandDirShoot( _Robot, q->getConfigStruct() , false );
	return q;
}

/**
 * Shoots the base Joint of the robot
 */
shared_ptr<Configuration> Robot::shootBase()
{
	shared_ptr<Configuration> q( new Configuration(this));
	m_Joints[_Robot->baseJnt->num]->shoot(*q);
	return q;
}

#endif

shared_ptr<Configuration> Robot::shoot(bool samplePassive)
{
	shared_ptr<Configuration> q(new Configuration(this));
#ifdef LIGHT_PLANNER
	if(ENV.getBool(Env::FKShoot))
	{
		deactivateCcCntrts(_Robot,-1);
		p3d_shoot(_Robot, q->getConfigStruct(), false);
		setAndUpdate(*q);
		q = getCurrentPos();
		activateCcCntrts(_Robot,-1,true);
		
		if (ENV.getBool(Env::drawPoints)) {
			PointsToDraw->push_back(q->getTaskPos());
			//g3d_draw_allwin_active();
		}
		
		return q;
	}
	else
	{
		p3d_shoot(_Robot, q->getConfigStruct(), samplePassive);
		return q;
	}
#else
	p3d_shoot(_Robot, q->getConfigStruct(), samplePassive);
	return q;
#endif
}

shared_ptr<Configuration> Robot::shootDir(bool samplePassive)
{
	shared_ptr<Configuration> q(new Configuration(this));
	//	p3d_RandDirShoot(_Robot, q->getConfigStruct(), samplePassive);
	p3d_RandNShpereDirShoot(_Robot, q->getConfigStruct(), samplePassive);
	return q;
}

/**
 * Returns false if does not respect the 
 * constraints
 */
int Robot::setAndUpdate(Configuration& q)
{
    p3d_set_robot_config(_Robot, q.getConfigStruct());
    return p3d_update_this_robot_pos(_Robot);
}

bool Robot::setAndUpdateMultiSol(Configuration& q)
{
	int *ikSol = NULL;
	p3d_set_robot_config(_Robot, q.getConfigStruct());
	return p3d_update_this_robot_pos_multisol(_Robot, NULL, 0, ikSol);	
}

void Robot::setAndUpdateWithoutConstraints(Configuration& q)
{
    p3d_set_robot_config(_Robot, q.getConfigStruct());
    p3d_update_this_robot_pos_without_cntrt(_Robot);
}

bool Robot::setAndUpdateHumanArms(Configuration& q)
{
	for(int i=2; i<=21; i++) // Just Arms
	{
		p3d_jnt* jntPt = _Robot->joints[i];
		for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			p3d_jnt_set_dof(jntPt, j, q[jntPt->index_dof+j]); 
		}
	}
	
	return p3d_update_this_robot_pos(_Robot);
}

bool Robot::isInCollision()
{
	bool ncol = false;
	
	/* collision checking */
	if( g3d_get_KCD_CHOICE_IS_ACTIVE() )
	{
		if(G3D_ACTIVE_CC)
		{
			ncol = p3d_col_test_choice();
		}
	}
	else
	{
		if(G3D_ACTIVE_CC)
		{
			//cout << "p3d_col_test_all()" << endl;
			ncol = p3d_col_test_all();
		}
	}
	
	return true;
	//cout << "Collision = " << ncol << endl;
}

shared_ptr<Configuration> Robot::getInitialPosition()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         _Robot->ROBOT_POS)));
}

void Robot::setInitialPosition(Configuration& conf)
{
	configPt q = _Robot->ROBOT_POS;
	
	p3d_copy_config_into(_Robot,
						 conf.getConfigStruct(),
						 &q);
}

shared_ptr<Configuration> Robot::getGoTo()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         _Robot->ROBOT_GOTO)));
}

void Robot::setGoTo(Configuration& conf)
{
	configPt q = _Robot->ROBOT_GOTO;
	
	p3d_copy_config_into(_Robot,
						 conf.getConfigStruct(),
						 &q);
}

shared_ptr<Configuration> Robot::getCurrentPos()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         p3d_get_robot_config(_Robot))));
}

shared_ptr<Configuration> Robot::getNewConfig()
{
    return (shared_ptr<Configuration> (new Configuration(this,
                                                         p3d_alloc_config(_Robot),true)));
}

Matrix4d Robot::getJointAbsPos(int id)
{
	Matrix4d absPos;
	
	//p3d_jnt* jntPt= _Robot->joints[id];
	
//	absPos[0][0] = jntPt->abs_pos[0][0];
//    absPos[1][1] = jntPt->abs_pos[1][0];
//    absPos[2][2] = jntPt->abs_pos[2][0];
//	absPos[3][2] = jntPt->abs_pos[3][0];
//	
//	absPos[0][0] = jntPt->abs_pos[0][1];
//    absPos[1][1] = jntPt->abs_pos[1][1];
//    absPos[2][2] = jntPt->abs_pos[2][1];
//	absPos[3][2] = jntPt->abs_pos[3][1];
//	
//	absPos[0][0] = jntPt->abs_pos[0][2];
//    absPos[1][1] = jntPt->abs_pos[1][2];
//    absPos[2][2] = jntPt->abs_pos[2][2];
//	absPos[3][2] = jntPt->abs_pos[3][2];
//	
//	absPos[0][0] = jntPt->abs_pos[0][4];
//    absPos[1][1] = jntPt->abs_pos[1][4];
//    absPos[2][2] = jntPt->abs_pos[2][4];
//	absPos[3][2] = jntPt->abs_pos[3][4];
	
	return absPos;
}

Vector3d Robot::getJointPos(int id)
{
    Vector3d vect;

    p3d_jnt* jntPt= _Robot->joints[id];

    vect[0] = jntPt->abs_pos[0][3];
    vect[1] = jntPt->abs_pos[1][3];
    vect[2] = jntPt->abs_pos[2][3];

//    cout << "vect = " << endl << vect << endl;

    return vect;
}

/**
 * Returns the number of DoF active in the planning phase
 * @return Number of Active DoFs
 */
unsigned int Robot::getNumberOfActiveDoF()
{
	unsigned int nbDoF(0);
	
	for(unsigned int i=0;i<m_Joints.size();i++)
	{
		p3d_jnt* jntPt = m_Joints[i]->getJointStruct();
		
		for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			int k = jntPt->index_dof + j;
			
			if (
					(p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
					(_Robot->cntrt_manager->in_cntrt[k] != 2) ) 
			{
				nbDoF++;
			}
		}
	}
	
	return nbDoF;
}

/*
 * Gets the joint of the ith active DoF
 */
Joint* Robot::getIthActiveDoFJoint(unsigned int ithActiveDoF , unsigned int& ithDofOnJoint )
{
	unsigned int activeDof = 0;
	
	for(unsigned int i=0;i<m_Joints.size();i++)
	{
		p3d_jnt* jntPt = m_Joints[i]->getJointStruct();
		
		for(int j=0; j<jntPt->dof_equiv_nbr; j++) 
		{
			int k = jntPt->index_dof + j;
			
			if (
					(p3d_jnt_get_dof_is_user(jntPt, j) && p3d_jnt_get_dof_is_active_for_planner(jntPt,j)) &&
					(_Robot->cntrt_manager->in_cntrt[k] != 2) ) 
			{
				if( activeDof == ithActiveDoF )
				{
					ithDofOnJoint = j;
					return m_Joints[i];
				}
				
				activeDof++;
			}
		}
	}
	
	return NULL;
}

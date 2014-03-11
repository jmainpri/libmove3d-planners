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
#include "move3d-gui.h"
#include "move3d-headless.h"
#include "Collision-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace std;
using namespace Move3D;
using namespace Eigen;

MOVE3D_USING_SHARED_PTR_NAMESPACE
//using MOVE3D_PTR_NAMESPACE::shared_ptr;


Move3D::Robot* Move3D::API_activeRobot = NULL;

Move3D::Robot::Robot(p3d_rob* robotPt, bool copy )
{
    _copy = copy;

    if (_copy) {
        _Robot = copyRobStructure(robotPt);
    }
    else {
        _Robot = robotPt;
    }

    _Name = robotPt->name;

    m_Joints.clear();

    for (int i=0; i<=_Robot->njoints; i++)
    {
        m_Joints.push_back( new Joint( this , _Robot->joints[i] , i , _copy ) );
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

Move3D::Trajectory Robot::getCurrentTraj()
{
    Move3D::Trajectory traj(this,_Robot->tcur);
    return traj;
}

unsigned int Robot::getNumberOfJoints()
{
    return m_Joints.size();
}

Joint* Robot::getJoint( unsigned int i )
{
    if (m_Joints.empty() || i > m_Joints.size()-1 ) {
        return NULL;
    }

    return m_Joints[i];
}

Joint* Robot::getJoint(std::string name) const
{
    for (unsigned int i=0; i<m_Joints.size(); i++)
    {
        if( m_Joints[i]->getName() == name )
        {
            return m_Joints[i];
        }
    }

    return NULL;
}

const std::vector<Joint*>& Robot::getAllJoints()
{
    return m_Joints;
}

vector<Vector3d> Robot::getObjectBox()
{
    //	if (m_ObjectBox.empty()) {
    //    cout << "Warning : " << __PRETTY_FUNCTION__ << " : no Object Box" << endl;
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
    int pas_jnt_index = _Robot->ccCntrts[0]->npasjnts-1;
    p3d_jnt* manip_jnt = (*_Robot->armManipulationData)[0].getManipulationJnt();
    activateCcCntrts(_Robot,-1,false);
    desactivateTwoJointsFixCntrt(_Robot,manip_jnt,_Robot->ccCntrts[0]->pasjnts[ pas_jnt_index ]);
}

/**
 * Deactivate Constraint
 */
void Robot::deactivateCcConstraint()
{
    int pas_jnt_index = _Robot->ccCntrts[0]->npasjnts-1;
    p3d_jnt* manip_jnt = (*_Robot->armManipulationData)[0].getManipulationJnt();
    deactivateCcCntrts(_Robot,-1);
    setAndActivateTwoJointsFixCntrt(_Robot,manip_jnt,_Robot->ccCntrts[0]->pasjnts[ pas_jnt_index ]);
}

/**
 * Returns the Virtual object dof
 */
int Robot::getObjectDof() 
{
    p3d_jnt* jnt = (*_Robot->armManipulationData)[0].getManipulationJnt();

    if ( jnt )
    {
        return jnt->index_dof;
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
confPtr_t Robot::shootBaseWithoutCC()
{
    confPtr_t q = getCurrentPos();
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
confPtr_t Robot::shootAllExceptBase()
{
    confPtr_t q( new Configuration(this));

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

confPtr_t Robot::shootFreeFlyer(double* box)
{
    //    cout << "box  = ( " << box[0] << " , " ;
    //    cout << box[1] << " , " ;
    //    cout << box[2] << " , " ;
    //    cout << box[3] << " , " ;
    //    cout << box[4] << " , " ;
    //    cout << box[5] << " )" << endl;
    confPtr_t q(new Configuration(this));
    p3d_FreeFlyerShoot(_Robot, q->getConfigStruct(), box);
    return q;
}


void Robot::shootObjectJoint(Configuration& Conf)
{
    /*confPtr_t q(new Configuration(Conf));

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

    confPtr_t q(new Configuration(Conf));
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
confPtr_t Robot::shootRandomDirection()
{
    confPtr_t q( new Configuration(this) );
    p3d_RandDirShoot( _Robot, q->getConfigStruct() , false );
    return q;
}

/**
 * Shoots the base Joint of the robot
 */
confPtr_t Robot::shootBase()
{
    confPtr_t q( new Configuration(this));
    m_Joints[_Robot->baseJnt->num]->shoot(*q);
    return q;
}

#endif

confPtr_t Robot::shoot(bool samplePassive)
{
    confPtr_t q(new Configuration(this));

#ifdef LIGHT_PLANNER
    if(ENV.getBool(Env::FKShoot))
    {
        this->deactivateCcConstraint();
        p3d_shoot(_Robot, q->getConfigStruct(), false);
        setAndUpdate(*q);
        q = getCurrentPos();
        this->activateCcConstraint();
        //    cout << "FKShoot" <<endl;
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

confPtr_t Robot::shootDir(bool samplePassive)
{
    confPtr_t q(new Configuration(this));
    //	p3d_RandDirShoot(_Robot, q->getConfigStruct(), samplePassive);
    p3d_RandNShpereDirShoot(_Robot, q->getConfigStruct(), samplePassive);
    return q;
}

int Robot::setAndUpdateFreeFlyer(const Eigen::Vector3d& pos)
{
    confPtr_t q = this->getCurrentPos();
    (*q)[6] = pos[0];
    (*q)[7] = pos[1];
    (*q)[8] = pos[2];
    return this->setAndUpdate(*q);
}

/**
 * Returns false if does not respect the
 * constraints, does not change configuration
 */
int Robot::setAndUpdate( const Configuration& q, bool withoutFreeFlyers )
{
    if(q.getConfigStructConst() == NULL)
        return false;
    
    if (!withoutFreeFlyers) {
        p3d_set_robot_config(_Robot, q.getConfigStructConst());
    }else {
        p3d_set_robot_config_without_free_flyers(_Robot, q.getConfigStructConst());
    }
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
        //		if(G3D_ACTIVE_CC)
        {
            ncol = p3d_col_test_choice();
        }
    }
    else
    {
        //		if(G3D_ACTIVE_CC)
        {
            //cout << "p3d_col_test_all()" << endl;
            ncol = p3d_col_test_all();
        }
    }

    return ncol;
    //cout << "Collision = " << ncol << endl;
}

bool Robot::isInCollisionWithOthersAndEnv()
{
    return pqp_robot_all_no_self_collision_test(_Robot);
}

double Robot::distanceToEnviroment()
{
    p3d_vector3 closest_point_rob;
    p3d_vector3 closest_point_obst;
    return pqp_robot_environment_distance( _Robot, closest_point_rob, closest_point_obst);
}

double Robot::distanceToRobot(Robot *robot)
{
    p3d_vector3 closest_point_rob1;
    p3d_vector3 closest_point_rob2;
    return pqp_robot_robot_distance( _Robot, robot->_Robot, closest_point_rob1, closest_point_rob2 );
}

confPtr_t Robot::getInitPos()
{
    return (confPtr_t (new Configuration(this, _Robot->ROBOT_POS)));
}

void Robot::setInitPos(Configuration& conf)
{
    configPt q = _Robot->ROBOT_POS;
    p3d_copy_config_into(_Robot, conf.getConfigStruct(), &q);
}

confPtr_t Robot::getGoalPos()
{
    return (confPtr_t (new Configuration(this,_Robot->ROBOT_GOTO)));
}

void Robot::setGoalPos(Configuration& conf)
{
    configPt q = _Robot->ROBOT_GOTO;
    p3d_copy_config_into(_Robot,conf.getConfigStruct(),&q);
}

confPtr_t Robot::getCurrentPos()
{
    // This creates a memory leak
    return (confPtr_t (new Configuration(this,p3d_get_robot_config(_Robot),true)));
}

confPtr_t Robot::getNewConfig()
{
    return (confPtr_t (new Configuration(this,p3d_alloc_config(_Robot),true)));
}

std::vector<int> Robot::getActiveDoFsFromJoints( const std::vector<int>& joint_ids )
{
    std::vector<int> dof_ids;

    for (size_t i=0; i<joint_ids.size(); i++)
    {
        Joint* move3d_joint = getJoint( joint_ids[i] );

        for (size_t j=0; j<move3d_joint->getNumberOfDof(); j++)
        {
            // cout << "Joint(" << j << "), Dof : " << move3d_joint->getIndexOfFirstDof() + j << ", " << move3d_joint->getName() << "" << endl;

            if( !move3d_joint->isJointDofUser(j) )
                continue;

            double min,max;
            // robot_->getJoint( active_joints[i] )->getDofBounds(j,min,max);
            move3d_joint->getDofRandBounds(j,min,max);

            if (min == max)
                continue;

            dof_ids.push_back( move3d_joint->getIndexOfFirstDof() + j );
        }
    }

    return dof_ids;
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

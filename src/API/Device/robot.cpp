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

#include <boost/function.hpp>

using namespace std;
using namespace Move3D;
using namespace Eigen;

MOVE3D_USING_SHARED_PTR_NAMESPACE

Robot* Move3D::API_activeRobot = NULL;

// ****************************************************************************************************
// API FUNCTIONS
// ****************************************************************************************************

static boost::function<void( Robot*, void*, bool, std::string&, std::vector<Joint*>& )> Move3DRobotConstructor;
static boost::function<Move3D::Trajectory( Robot* )> Move3DRobotGetCurrentTrajectory;
static boost::function<confPtr_t( confPtr_t, bool )> Move3DRobotShoot;
static boost::function<confPtr_t( confPtr_t, bool )> Move3DRobotShootDir;
static boost::function<bool( Robot*, const Configuration& q, bool )> Move3DRobotSetAndUpdate;
static boost::function<bool( Robot*, const Configuration& q )> Move3DRobotSetAndUpdateMultiSol;
static boost::function<void( Robot*, const Configuration& q )> Move3DRobotSetAndUpdateWithoutConstraints;
static boost::function<bool( Robot* )> Move3DRobotIsInCollision;
static boost::function<bool( Robot* )> Move3DRobotIsInCollisionWithOthersAndEnv;
static boost::function<double( Robot* )> Move3DRobotDistanceEnv;
static boost::function<double( Robot*, Robot* )> Move3DRobotDistanceToRobot;
static boost::function<confPtr_t( Robot* )> Move3DRobotGetInitPos;
static boost::function<void( Robot*, const Configuration& )> Move3DRobotSetInitPos;
static boost::function<confPtr_t( Robot* )> Move3DRobotGetGoalPos;
static boost::function<void( Robot*, const Configuration& )> Move3DRobotSetGoalPos;
static boost::function<confPtr_t( Robot* )> Move3DRobotGetCurrentPos;
static boost::function<confPtr_t( Robot* )> Move3DRobotGetNewPos;
static boost::function<unsigned int( Robot* )> Move3DRobotGetNumberOfActiveDofs;
static boost::function<Joint*( Robot*, unsigned int, unsigned int& )> Move3DRobotGetIthActiveDofJoint;

// ****************************************************************************************************
// SETTERS
// ****************************************************************************************************

void move3d_set_fct_robot_constructor( boost::function<void( Robot*, void*, bool, std::string&, std::vector<Joint*>& )> fct ) {  Move3DRobotConstructor = fct; }
void move3d_set_fct_robot_get_current_trajectory( boost::function<Move3D::Trajectory( Robot* )> fct ) {  Move3DRobotGetCurrentTrajectory = fct; }
void move3d_set_fct_robot_shoot( boost::function<confPtr_t( confPtr_t, bool )> fct ) {  Move3DRobotShoot = fct; }
void move3d_set_fct_robot_shoot_dir( boost::function<confPtr_t( confPtr_t, bool )> fct ) {  Move3DRobotShootDir = fct; }
void move3d_set_fct_robot_set_and_update( boost::function<bool( Robot*, const Configuration& q, bool )> fct ) {  Move3DRobotSetAndUpdate = fct; }
void move3d_set_fct_robot_set_and_update_multi_sol( boost::function<bool( Robot*, const Configuration& q )> fct ) {  Move3DRobotSetAndUpdateMultiSol = fct; }
void move3d_set_fct_robot_without_constraints( boost::function<void( Robot*, const Configuration& q )> fct ) {  Move3DRobotSetAndUpdateWithoutConstraints = fct; }
void move3d_set_fct_robot_is_in_collision( boost::function<bool( Robot* )> fct ) {  Move3DRobotIsInCollision = fct; }
void move3d_set_fct_robot_is_in_collision_with_others_and_env( boost::function<bool( Robot* )> fct ) {  Move3DRobotIsInCollisionWithOthersAndEnv = fct; }
void move3d_set_fct_robot_distance_to_env( boost::function<double( Robot* )> fct ) {  Move3DRobotDistanceEnv = fct; }
void move3d_set_fct_robot_distance_to_robot( boost::function<double( Robot*, Robot* )> fct ) {  Move3DRobotDistanceToRobot = fct; }
void move3d_set_fct_robot_get_init_pos( boost::function<confPtr_t( Robot* )> fct ) {  Move3DRobotGetInitPos = fct; }
void move3d_set_fct_robot_set_init_pos( boost::function<void( Robot*, const Configuration& )> fct ) {  Move3DRobotSetInitPos = fct; }
void move3d_set_fct_robot_get_goal_pos( boost::function<confPtr_t( Robot* )> fct ) {  Move3DRobotGetGoalPos = fct; }
void move3d_set_fct_robot_set_goal_pos( boost::function<void( Robot*, const Configuration& )> fct ) {  Move3DRobotSetGoalPos = fct; }
void move3d_set_fct_robot_get_current_pos( boost::function<confPtr_t( Robot* )> fct ) {  Move3DRobotGetCurrentPos = fct; }
void move3d_set_fct_robot_get_new_pos( boost::function<confPtr_t( Robot* )> fct ) {  Move3DRobotGetNewPos = fct; }
void move3d_set_fct_robot_get_number_of_active_dofs( boost::function<unsigned int( Robot* )> fct ) {  Move3DRobotGetNumberOfActiveDofs = fct; }
void move3d_set_fct_robot_get_ith_active_dof_joint( boost::function<Joint*( Robot*, unsigned int, unsigned int& )> fct ) {  Move3DRobotGetIthActiveDofJoint = fct; }

// ****************************************************************************************************

Robot::Robot(void* robotPt, bool copy )
{
    _copy = copy;
    _Robot = static_cast<p3d_rob*>(robotPt);
    Move3DRobotConstructor( this, _Robot, copy, _Name, m_Joints );
}

Robot::~Robot()
{
    if(_copy)
    {
        deleteRobStructure(static_cast<p3d_rob*>(_Robot));
    }
}

string Robot::getName()
{
    return _Name;
}

//Accessors
rob* Robot::getP3dRobotStruct()
{
    return static_cast<p3d_rob*>(_Robot);
}

/**
 * Gets traj
 * @return pointer to structure p3d_traj
 */
void* Robot::getTrajStruct()
{
    return static_cast<p3d_rob*>(_Robot)->tcur;
}

Move3D::Trajectory Robot::getCurrentTraj()
{
    return Move3DRobotGetCurrentTrajectory(this);
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
    for(int i = 0; i < static_cast<p3d_rob*>(_Robot)->nbCcCntrts; i++)
    {
        if(static_cast<p3d_rob*>(_Robot)->ccCntrts[i]->active)
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
    int pas_jnt_index = static_cast<p3d_rob*>(_Robot)->ccCntrts[0]->npasjnts-1;
    p3d_jnt* manip_jnt = (*static_cast<p3d_rob*>(_Robot)->armManipulationData)[0].getManipulationJnt();
    activateCcCntrts(static_cast<p3d_rob*>(_Robot),-1,false);
    desactivateTwoJointsFixCntrt(static_cast<p3d_rob*>(_Robot),manip_jnt,static_cast<p3d_rob*>(_Robot)->ccCntrts[0]->pasjnts[ pas_jnt_index ]);
}

/**
 * Deactivate Constraint
 */
void Robot::deactivateCcConstraint()
{
    int pas_jnt_index = static_cast<p3d_rob*>(_Robot)->ccCntrts[0]->npasjnts-1;
    p3d_jnt* manip_jnt = (*static_cast<p3d_rob*>(_Robot)->armManipulationData)[0].getManipulationJnt();
    deactivateCcCntrts(static_cast<p3d_rob*>(_Robot),-1);
    setAndActivateTwoJointsFixCntrt(static_cast<p3d_rob*>(_Robot),manip_jnt,static_cast<p3d_rob*>(_Robot)->ccCntrts[0]->pasjnts[ pas_jnt_index ]);
}

/**
 * Returns the Virtual object dof
 */
int Robot::getObjectDof() 
{
    p3d_jnt* jnt = (*static_cast<p3d_rob*>(_Robot)->armManipulationData)[0].getManipulationJnt();

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
void* Robot::getBaseJnt()
{
    return static_cast<p3d_rob*>(_Robot)->baseJnt;
}

/**
 * Shoots the base Joint of the robot
 */
confPtr_t Robot::shootBaseWithoutCC()
{
    confPtr_t q = getCurrentPos();
    m_Joints[static_cast<p3d_rob*>(_Robot)->baseJnt->num]->shoot(*q);
    cout << "Base Num = " << static_cast<p3d_rob*>(_Robot)->baseJnt->num << endl;
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
        if (static_cast<p3d_rob*>(_Robot)->baseJnt->num == ((int)i))
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
        p3d_jnt* jntPt = static_cast<p3d_jnt*>( m_Joints[i]->getP3dJointStruct() );

        //if (static_cast<p3d_rob*>(_Robot)->baseJnt->num == ((int)i))
        if ( i <= 1)
        { continue; }

        for(int j=0; j<jntPt->dof_equiv_nbr; j++)
        {
            int k = jntPt->index_dof+j;

            m_Joints[i]->setJointDof(j,q[k]);
        }
    }

    return p3d_update_this_robot_pos(static_cast<p3d_rob*>(_Robot));
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
    p3d_FreeFlyerShoot(static_cast<p3d_rob*>(_Robot), q->getConfigStruct(), box);
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

    p3d_shoot_justin_left_arm(static_cast<p3d_rob*>(_Robot), q->getConfigStruct(), false);
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

    //p3d_JointFreeFlyerShoot(static_cast<p3d_rob*>(_Robot),static_cast<p3d_rob*>(_Robot)->curObjectJnt,q->getConfigStruct(),box);
}

/**
 * Shoots a random direction
 */
confPtr_t Robot::shootRandomDirection()
{
    confPtr_t q( new Configuration(this) );
    p3d_RandDirShoot( static_cast<p3d_rob*>(_Robot), q->getConfigStruct() , false );
    return q;
}

/**
 * Shoots the base Joint of the robot
 */
confPtr_t Robot::shootBase()
{
    confPtr_t q( new Configuration(this));
    m_Joints[static_cast<p3d_rob*>(_Robot)->baseJnt->num]->shoot(*q);
    return q;
}

#endif

confPtr_t Robot::shoot(bool samplePassive)
{
    confPtr_t q(new Configuration(this));
    return Move3DRobotShoot( q, samplePassive );
}

confPtr_t Robot::shootDir(bool samplePassive)
{
    confPtr_t q(new Configuration(this));
    return Move3DRobotShootDir( q, samplePassive );
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

    return Move3DRobotSetAndUpdate( this, q, withoutFreeFlyers );
}

bool Robot::setAndUpdateMultiSol( const Configuration& q)
{
    if(q.getConfigStructConst() == NULL)
        return false;

    return Move3DRobotSetAndUpdateMultiSol( this, q );
}

void Robot::setAndUpdateWithoutConstraints( const Configuration& q)
{
    if(q.getConfigStructConst() == NULL)
        return;

    Move3DRobotSetAndUpdateWithoutConstraints( this, q );
}

bool Robot::isInCollision()
{
    return Move3DRobotIsInCollision( this );
    //cout << "Collision = " << ncol << endl;
}

bool Robot::isInCollisionWithOthersAndEnv()
{
    return Move3DRobotIsInCollisionWithOthersAndEnv( this );
}

double Robot::distanceToEnviroment()
{
    return Move3DRobotDistanceEnv( this );
}

double Robot::distanceToRobot(Robot* robot)
{
    return Move3DRobotDistanceToRobot( this, robot );
}

confPtr_t Robot::getInitPos()
{
    return Move3DRobotGetInitPos( this );
}

void Robot::setInitPos(Configuration& conf)
{
    Move3DRobotSetInitPos( this, conf );
}

confPtr_t Robot::getGoalPos()
{
    return Move3DRobotGetGoalPos( this );
}

void Robot::setGoalPos(Configuration& conf)
{
    Move3DRobotSetGoalPos( this, conf );
}

confPtr_t Robot::getCurrentPos()
{
    // This creates a memory leak
    return Move3DRobotGetCurrentPos( this );
}

confPtr_t Robot::getNewConfig()
{
    return Move3DRobotGetNewPos( this );
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
    return Move3DRobotGetNumberOfActiveDofs( this );
}

/*
 * Gets the joint of the ith active DoF
 */
Joint* Robot::getIthActiveDoFJoint(unsigned int ithActiveDoF , unsigned int& ithDofOnJoint )
{
    return Move3DRobotGetIthActiveDofJoint( this, ithActiveDoF, ithDofOnJoint );
}

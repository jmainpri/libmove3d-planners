//
// C++ Implementation: configuration
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "API/ConfigSpace/configuration.hpp"
#include "API/Device/robot.hpp"

#include "P3d-pkg.h"
#include "Collision-pkg.h"
//#include "Localpath-pkg.h"

#include "planner/cost_space.hpp"
#include "collision_space/collision_space.hpp"

#include <boost/function.hpp>

using namespace std;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

// ****************************************************************************************************
// API FUNCTIONS
// ****************************************************************************************************

static boost::function<double*(Robot*)> Move3DConfigurationConstructorRobot;
static boost::function<double*(Robot*, double*)> Move3DConfigurationConstructorConfigStruct;
static boost::function<void(const Configuration& q_s, Configuration& q_t)> Move3DConfigurationAssignment;
static boost::function<void(Robot*,double*)> Move3DConfigurationClear;
static boost::function<void(Robot*,double*)> Move3DConfigurationConvertToRadians;
static boost::function<confPtr_t(Robot*,double*)> Move3DConfigurationConvertToDegrees;
static boost::function<double*(Robot*,double*)> Move3DConfigurationGetStructCopy;
static boost::function<double(Robot*, double*, double*, bool)> Move3DConfigurationDist;
static boost::function<double(Robot*, const Configuration&, const Configuration&, int)> Move3DConfigurationDistChoice;
static boost::function<bool(Robot* R)> Move3DConfigurationInCollision;
static boost::function<bool(Robot*, double*, bool)> Move3DConfigurationIsOutOfBounds;
static boost::function<void(Robot*, double*)> Move3DConfigurationAdaptCircularJointLimits;
static boost::function<double(Robot*)> Move3DConfigurationDistEnv;
static boost::function<bool(Robot*, double*, double*, bool)> Move3DConfigurationEqual;
static boost::function<void(Robot*, double*)> Move3DConfigurationCopy;
static boost::function<void(Robot*, double*, double*)> Move3DConfigurationCopyPassive;
static boost::function<void(Robot* R, double*, double*, double*)> Move3DConfigurationAdd;
static boost::function<void(Robot* R, double*, double*, double*)> Move3DConfigurationSub;
static boost::function<void(Robot* R, double*, double*, double)> Move3DConfigurationMult;
static boost::function<void(Robot* R, double*, bool)> Move3DConfigurationPrint;

// ****************************************************************************************************
// SETTERS
// ****************************************************************************************************

void move3d_set_fct_configuration_constructor_robot( boost::function<double*(Robot*)> fct ) {  Move3DConfigurationConstructorRobot = fct; }
void move3d_set_fct_configuration_constructor_config_struct( boost::function<double*(Robot*, double*)> fct ) { Move3DConfigurationConstructorConfigStruct = fct; }
void move3d_set_fct_configuration_assignment( boost::function<void(const Configuration& q_s, Configuration& q_t)> fct ) { Move3DConfigurationAssignment = fct; }
void move3d_set_fct_configuration_clear( boost::function<void(Robot*,double*)> fct ) { Move3DConfigurationClear = fct; }
void move3d_set_fct_configuration_convert_to_radians( boost::function<void(Robot*,double*)> fct ) { Move3DConfigurationConvertToRadians = fct; }
void move3d_set_fct_configuration_convert_to_degrees( boost::function<confPtr_t(Robot*,double*)> fct ) { Move3DConfigurationConvertToDegrees = fct; }
void move3d_set_fct_configuration_get_struct_copy( boost::function<double*(Robot*,double*)> fct ) { Move3DConfigurationGetStructCopy = fct; }
void move3d_set_fct_configuration_dist( boost::function<double(Robot*, double*, double*, bool)> fct ) { Move3DConfigurationDist = fct; }
void move3d_set_fct_configuration_dist_choice( boost::function<double(Robot*, const Configuration&, const Configuration&, int)> fct ) { Move3DConfigurationDistChoice = fct; }
void move3d_set_fct_configuration_in_collision( boost::function<bool(Robot* R)> fct ) { Move3DConfigurationInCollision = fct; }
void move3d_set_fct_configuration_is_out_of_bounds( boost::function<bool(Robot*,double*,bool)> fct ) { Move3DConfigurationIsOutOfBounds = fct; }
void move3d_set_fct_configuration_adapt_circular_joint_limits( boost::function<void(Robot*,double*)> fct ) { Move3DConfigurationAdaptCircularJointLimits = fct; }
void move3d_set_fct_configuration_dist_env( boost::function<double(Robot*)> fct ) { Move3DConfigurationDistEnv = fct; }
void move3d_set_fct_configuration_equal( boost::function<bool(Robot*, double*, double*, bool)> fct ) { Move3DConfigurationEqual = fct; }
void move3d_set_fct_configuration_copy( boost::function<void(Robot*, double*)> fct ) { Move3DConfigurationCopy = fct; }
void move3d_set_fct_configuration_copy_passive( boost::function<void(Robot*, double*, double*)> fct ) { Move3DConfigurationCopyPassive = fct; }
void move3d_set_fct_configuration_add( boost::function<void(Robot* R, double*, double*, double*)> fct ) { Move3DConfigurationAdd = fct; }
void move3d_set_fct_configuration_sub( boost::function<void(Robot* R, double*, double*, double*)> fct ) { Move3DConfigurationSub = fct; }
void move3d_set_fct_configuration_mult( boost::function<void(Robot* R, double*, double*, double)> fct ) { Move3DConfigurationMult = fct; }
void move3d_set_fct_configuration_print( boost::function<void(Robot* R, double*, bool)> fct ) { Move3DConfigurationPrint = fct; }

// ****************************************************************************************************
// ****************************************************************************************************

//constructor and destructor
Configuration::Configuration(Robot* robot) :
    _flagInitQuaternions(false),
    _CollisionTested(false),
    _InCollision(true),
    _CostTested(false),
    _Cost(0.0),
    _phiEvaluated(false),
    _Robot( robot )
{
    if(robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        _Configuration = Move3DConfigurationConstructorRobot( robot );
    }
}

Configuration::Configuration(Robot* R, double* C, bool noCopy) :
    _flagInitQuaternions(false),
    _CollisionTested(false),
    _InCollision(true),
    _CostTested(false),
    _Cost(0.0),
    _phiEvaluated(false),
    _Robot(R)
{
    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
         _Configuration = noCopy ? C : Move3DConfigurationConstructorConfigStruct( R, C );
//        _Configuration = Move3DConfigurationConstructorConfigStruct( R, C );
    }
}

Configuration::Configuration(const Configuration& conf) :
    _flagInitQuaternions(conf._flagInitQuaternions),
    _CollisionTested(conf._CollisionTested),
    _InCollision(conf._InCollision),
    _CostTested(conf._CostTested),
    _Cost(conf._Cost),
    _phiEvaluated(conf._phiEvaluated),
    _phi(conf._phi),
    _Robot(conf._Robot)
{
    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        _Configuration = Move3DConfigurationConstructorConfigStruct( _Robot, conf._Configuration );
    }
}

Configuration& Configuration::operator= (const Configuration& source)
{
    _flagInitQuaternions = source._flagInitQuaternions;
    _CollisionTested = source._CollisionTested;
    _InCollision = source._InCollision;
    _CostTested = source._CostTested;
    _Cost = source._Cost;
    _Robot =  source._Robot;
    _phiEvaluated = source._phiEvaluated;
    _phi = source._phi;

    if(_Robot==NULL)
    {
        _Configuration = NULL;
    }
    else
    {
        Move3DConfigurationAssignment( source, *this );
    }

    return *this;
}

Configuration::~Configuration()
{
    this->Clear();
}

void Configuration::Clear()
{
    if( _Configuration != NULL )
    {
        Move3DConfigurationClear( _Robot, _Configuration );
    }
}


void Configuration::convertToRadian()
{
    Move3DConfigurationConvertToRadians( _Robot, _Configuration );
}

confPtr_t Configuration::getConfigInDegree()
{
    return Move3DConfigurationConvertToDegrees( _Robot, _Configuration );
}

//Accessors
Robot* Configuration::getRobot()
{
    return _Robot;
}

double* Configuration::getConfigStructConst() const
{
    return _Configuration;
}

double* Configuration::getConfigStructCopy()
{
    return Move3DConfigurationGetStructCopy(_Robot, _Configuration);
}

void Configuration::setConfiguration(double* C)
{
    _Configuration = C;
}

void Configuration::setConfiguration(Configuration& C)
{
    _Configuration = C.getConfigStruct();
}

void Configuration::setConfigurationCopy(Configuration& C)
{
    _Configuration = C.getConfigStructCopy();
}

/**
 * Computes the distance
 *  between configurations
 */
double Configuration::dist(const Configuration& Conf, bool print) const
{
    return Move3DConfigurationDist( _Robot, _Configuration, Conf._Configuration, print );
}

double Configuration::dist(const Configuration& q, int distChoice) const
{
    return Move3DConfigurationDistChoice( _Robot, *this, q, distChoice );
}

bool Configuration::isInCollision()
{
    if(!_CollisionTested)
    {
        _CollisionTested = true;

        if( !_Robot->setAndUpdate(*this) )
        {
            _InCollision = true;
            return _InCollision;
        }
        _InCollision = Move3DConfigurationInCollision( _Robot );
    }

    return _InCollision;
}

bool Configuration::isOutOfBounds(bool print)
{
    return Move3DConfigurationIsOutOfBounds( _Robot, _Configuration, print );
}

void Configuration::adaptCircularJointsLimits()
{
    Move3DConfigurationAdaptCircularJointLimits( _Robot, _Configuration );
}

void Configuration::setAsNotTested()
{
    _CollisionTested = false;
}

double Configuration::distEnv()
{
    _Robot->setAndUpdate(*this);
    return Move3DConfigurationDistEnv( _Robot );
}

bool Configuration::equal(Configuration& Conf, bool print)
{
    if(_Configuration==Conf.getConfigStruct())
    {
        if(_Configuration==NULL)
        {
            return false;
        }

        return true;
    }
    else
    {
        if(_Configuration==NULL || Conf.getConfigStruct()==NULL)
        {
            return false;
        }
    }

    return Move3DConfigurationEqual( _Robot, _Configuration, Conf.getConfigStruct(), print );
}

//copie la Configuration courante dans une nouvelle Configuration
confPtr_t Configuration::copy()
{
    return (confPtr_t(new Configuration(*this)));
}

void Configuration::copyPassive(Configuration& q)
{
    Move3DConfigurationCopyPassive( _Robot, _Configuration, q.getConfigStruct() );
}

confPtr_t Configuration::add(Configuration& C)
{
    confPtr_t ptrQ(new Configuration(_Robot));

    Move3DConfigurationAdd( _Robot, _Configuration, C.getConfigStruct(), ptrQ->getConfigStruct() );

    return ptrQ;
}

confPtr_t Configuration::sub(Configuration& C)
{
    confPtr_t ptrQ(new Configuration(_Robot));

    Move3DConfigurationSub( _Robot, _Configuration, C.getConfigStruct(), ptrQ->getConfigStruct() );

    return ptrQ;
}

confPtr_t Configuration::mult(double coeff)
{
    confPtr_t ptrQ(new Configuration(_Robot));

    Move3DConfigurationMult( _Robot, _Configuration, ptrQ->getConfigStruct(), coeff );

    return ptrQ;
}

Eigen::VectorXd Configuration::getEigenVector()  const
{
    unsigned int nbDof= 0;
    unsigned int njnt = _Robot->getNumberOfJoints();

    // Get  number of Dofs of all joints
    for (unsigned int i = 0; i < njnt; i++)
        nbDof += _Robot->getJoint(i)->getNumberOfDof();

    VectorXd q(nbDof);

    // Get the values of the dofs
    for (unsigned int i = 0; i < nbDof; i++)
    {
        q(i) = _Configuration[i];
    }

    return q;
}

Eigen::VectorXd Configuration::getEigenVector( int startIndex, int endIndex ) const
{	
    VectorXd q( endIndex - startIndex + 1);

    // Get the values of the dofs
    for ( int i=startIndex; i <= endIndex; i++)
    {
        q(i-startIndex) = _Configuration[i];
    }

    return q;
}

Eigen::VectorXd Configuration::getEigenVector(const std::vector<int>& dof_indices) const
{
    VectorXd q( dof_indices.size() );

    // Get the values of the dofs
    for ( int i=0; i<int(dof_indices.size()); i++)
    {
        q(i) = _Configuration[ dof_indices[i] ];
    }

    return q;
}

void Configuration::setFromEigenVector(const Eigen::VectorXd& conf)
{	
    // Get the values of the dofs
    for (int i = 0; i < conf.size(); i++)
    {
        _Configuration[i] = conf(i);
    }
}

void Configuration::setFromEigenVector(const Eigen::VectorXd& conf, int startIndex, int endIndex)
{
    // Get the values of the dofs
    for (int i = startIndex; i <= endIndex; i++)
    {
        _Configuration[i] = conf(i);
    }
}

void Configuration::setFromEigenVector(const Eigen::VectorXd& conf, const std::vector<int>& dof_indices)
{
    // Get the values of the dofs
    for ( size_t i=0; i<dof_indices.size(); i++)
    {
        _Configuration[ dof_indices[i] ] = conf(i);
    }
}

bool Configuration::setConstraintsWithSideEffect()
{
    bool respect = _Robot->setAndUpdate( *this );

    if(respect)
    {
        this->Clear();
        _Configuration = _Robot->getCurrentPos()->getConfigStructCopy();
    }

    return respect;
}

bool Configuration::setConstraints()
{
    confPtr_t q( _Robot->getCurrentPos() );

    bool respect = setConstraintsWithSideEffect();

    _Robot->setAndUpdate(*q);

    return respect;
}

#ifdef LIGHT_PLANNER
Vector3d Configuration::getTaskPos()
{
    // TODO: fix the problem of having a function
    // that modifies the configuration in cost computation
    //    if(!ENV.getBool(Env::isInverseKinematics))
    //    {
    //        this->setConstraints();
    //    }
    Vector3d taskPos;
    int objDof = _Robot->getObjectDof();
    //cout << "Robot object dof = " << objDof << endl;
    taskPos[0] = _Configuration[objDof+0];
    taskPos[1] = _Configuration[objDof+1];
    taskPos[2] = _Configuration[objDof+2];
    return taskPos;
}
#endif

double Configuration::cost()
{
    if(!_CostTested)
    {
        if( global_costSpace )
        {
            _Robot->setAndUpdate( *this );
            _Cost = global_costSpace->cost( *this );
        }

        _CostTested = true;
        //				cout << "Cost = " << _Cost << endl;
    }

    return _Cost;
}

void Configuration::setCostAsNotTested()
{
    _CostTested = false;
}

void Configuration::print(bool withPassive) const
{
    Move3DConfigurationPrint( _Robot, _Configuration, withPassive );
}

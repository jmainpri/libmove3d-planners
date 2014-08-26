// C++ Implementation: localpath
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "localpath.hpp"

#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Planner-pkg.h"

#include "Graphic-pkg.h"

using namespace std;
using namespace Move3D;
using namespace Eigen;

MOVE3D_USING_SHARED_PTR_NAMESPACE

// ****************************************************************************************************
// API FUNCTIONS
// ****************************************************************************************************

static boost::function<void*(const LocalPath&, Robot*)> Move3DLocalPathCopy;
static boost::function<void*(const LocalPath&, Robot*)> Move3DLocalPathCopyP3d;
static boost::function<void(LocalPath&)> Move3DLocalPathDestructor;
static boost::function<void*(LocalPath&, void*, confPtr_t&, confPtr_t& )> Move3DLocalPathCopyFromStruct;
static boost::function<void*(LocalPath&, bool, int&)> Move3DLocalPathGetStruct;
static boost::function<bool(LocalPath&, confPtr_t, int&, double&)> Move3DLocalPathClassicTest;
static boost::function<bool(LocalPath&, int&)> Move3DLocalPathIsValid;
static boost::function<double(LocalPath&)> Move3DLocalPathGetLength;
static boost::function<double(LocalPath&)> Move3DLocalPathGetParamMax;
static boost::function<confPtr_t(LocalPath&,double)> Move3DLocalPathConfigAtDist;
static boost::function<void(LocalPath&,double,confPtr_t&)> Move3DLocalPathConfigAtParam;
static boost::function<double(LocalPath&,double,bool,double&)> Move3DLocalPathStayWithinDist;

// ****************************************************************************************************
// SETTERS
// ****************************************************************************************************

void move3d_set_fct_localpath_copy( boost::function<void*(const LocalPath&, Robot*)> fct ) {  Move3DLocalPathCopy = fct; }
void move3d_set_fct_localpath_copy_p3d( boost::function<void*(const LocalPath&, Robot*)> fct ) {  Move3DLocalPathCopyP3d = fct; }
void move3d_set_fct_localpath_path_destructor( boost::function<void(LocalPath&)> fct ) {  Move3DLocalPathDestructor = fct; }
void move3d_set_fct_localpath_copy_from_struct( boost::function<void*(LocalPath&, void*, confPtr_t&, confPtr_t& )> fct ) {  Move3DLocalPathCopyFromStruct = fct; }
void move3d_set_fct_localpath_get_struct( boost::function<void*(LocalPath&, bool, int&)> fct ) {  Move3DLocalPathGetStruct = fct; }
void move3d_set_fct_localpath_classic_test( boost::function<bool(LocalPath&, confPtr_t, int&, double&)> fct ) {  Move3DLocalPathClassicTest = fct; }
void move3d_set_fct_localpath_is_valid( boost::function<bool(LocalPath&, int&)> fct ) {  Move3DLocalPathIsValid = fct; }
void move3d_set_fct_localpath_get_length( boost::function<double(LocalPath&)> fct ) {  Move3DLocalPathGetLength = fct; }
void move3d_set_fct_localpath_get_param_max( boost::function<double(LocalPath&)> fct ) {  Move3DLocalPathGetParamMax = fct; }
void move3d_set_fct_localpath_config_at_dist( boost::function<confPtr_t(LocalPath&,double)> fct ) {  Move3DLocalPathConfigAtDist = fct; }
void move3d_set_fct_localpath_config_at_param( boost::function<void(LocalPath&,double,confPtr_t&)> fct ) {  Move3DLocalPathConfigAtParam = fct; }
void move3d_set_fct_localpath_stay_within_dist( boost::function<double(LocalPath&,double,bool,double&)> fct ) {  Move3DLocalPathStayWithinDist = fct; }

// ****************************************************************************************************
// ****************************************************************************************************

LocalPath::LocalPath(confPtr_t B, confPtr_t E) :
    _Robot(B->getRobot()),
    _Begin(B),
    _End(E),
    _LocalPath(NULL),
    _Valid(false),
    _Evaluated(false),
    _lastValidParam(0.0),
    _lastValidEvaluated(false),
    _NbColTest(0),
    _costEvaluated(false), _Cost(0.0),
    _ResolEvaluated(false), _Resolution(0.0),
    _Type(LINEAR),
    _phiEvaluated(false)
{
    if ( _Begin->getRobot() != _End->getRobot() )
    {
        cout << "_Begin->getRobot() != _End->getRobot() in LocalPath" << endl;
    }
}

/**
 * Construct a smaller localpath from the extend method
 */
LocalPath::LocalPath(LocalPath& path, double& p , bool lastValidConfig) :
    _Robot(path.getRobot()),
    _Begin(path._Begin),
    _LocalPath(NULL),
    _phiEvaluated(false)
{
    // For extend (Construct a smaller local path)
    // This function is used in Base Expansion
    if(!lastValidConfig)
    {
        _End = path.configAtParam( p * path.getParamMax() );
        _Valid =false;
        _Evaluated = false;
        _lastValidParam = 0.0;
        _lastValidEvaluated = false;
        _NbColTest = 0;
        _costEvaluated = false;
        _Cost = 0.0;
        _ResolEvaluated = false;
        _Resolution = 0.0;
        _Type = LINEAR;
    }
    else
    {
        // The new path represents the valid part
        // of the path given to the constructor.
        // If the parameter p is > 0,
        // the given path is at least partially valid
        // and consequently the new path is valid.
        _End = path.getLastValidConfig(p);
        _Valid = (p > 0);
        _Evaluated = (path._Evaluated), _lastValidParam = (p > 0 ? 1.0 : 0.0);
        _lastValidEvaluated = true;
        _NbColTest = path._NbColTest;
        _costEvaluated = false ;
        _Cost = path._Cost;
        _ResolEvaluated = false;
        _Resolution = 0.0;
        _Type = path._Type;
    }
}


LocalPath::LocalPath(const LocalPath& path) :
    _Robot(path._Robot),
    _Begin(path._Begin),
    _End(path._End),
    _Valid(path._Valid),
    _Evaluated(path._Evaluated),
    _lastValidParam(0.0),
    _lastValidConfig(path._lastValidConfig),
    _lastValidEvaluated(false),
    _NbColTest(path._NbColTest),
    _costEvaluated(path._costEvaluated),
    _Cost(path._Cost),
    _ResolEvaluated(path._ResolEvaluated),
    _Resolution(path._Resolution),
    _Type(path._Type),
    _phiEvaluated(path._phiEvaluated),
    _phi(path._phi)
{
    _LocalPath = Move3DLocalPathCopy( path, _Robot );
}

LocalPath::LocalPath( Robot* R, p3d_localpath* lpPtr ) :
    _Robot(R),
    _LocalPath(NULL),
    _Valid(false),
    _Evaluated(false),
    _lastValidParam(0.0),
    _lastValidEvaluated(false),
    _NbColTest(0),
    _costEvaluated(false),
    _Cost(0.0),
    _ResolEvaluated(false),
    _Resolution(0.0),
    _phiEvaluated(false)
{

    _LocalPath = Move3DLocalPathCopyFromStruct( *this, lpPtr, _Begin, _End );
}


LocalPath::~LocalPath()
{
    Move3DLocalPathDestructor( *this );
}

//Accessors
p3d_localpath* LocalPath::getP3dLocalpathStruct(bool multi_sol)
{
    return static_cast<p3d_localpath*>( Move3DLocalPathGetStruct( *this, multi_sol, _Type ) );
}

confPtr_t LocalPath::getBegin()
{
    return _Begin;
}

confPtr_t LocalPath::getEnd()
{
    return _End;
}

Robot* LocalPath::getRobot()
{
    return _Robot;
}

bool LocalPath::getEvaluated()
{
    return _Evaluated;
}

int LocalPath::getType()
{
    if (getP3dLocalpathStruct())
    {
        return _Type;
    }
    else
    {
        return (int) (NULL);
    }
}

confPtr_t LocalPath::getLastValidConfig(double& p)
{
    _lastValidConfig = _Robot->getNewConfig();
    this->classicTest();
    p = _lastValidParam;
    return (_lastValidConfig);
}

bool LocalPath::classicTest()
{
    //printf("Classic test\n");

    if (!_lastValidEvaluated)
    {
        if (_lastValidConfig == NULL)
        {
            _lastValidConfig = _Robot->getNewConfig();
        }

        _Valid = Move3DLocalPathClassicTest( *this, _lastValidConfig, _NbColTest, _lastValidParam );
        _Evaluated = true;
        _lastValidEvaluated = true;
    }
    return(_Valid);
}

bool LocalPath::isValid()
{
    if( !_Evaluated )
    {
        // Classic changes the configurations
        // So they might be changed from the call from
        // another local path
        // _Begin->adaptCircularJointsLimits();
        // _End->adaptCircularJointsLimits();

        if ( _End->isOutOfBounds() || _Begin->isOutOfBounds() )
        {
            _Valid = false;

            cout << "Start or goal out of bounds" << endl;

            cout << "begin : " << endl;
            _Begin->isOutOfBounds( true );
            cout << "end : " << endl;
            _End->isOutOfBounds( true );

        }
        else if ( _End->isInCollision() || _Begin->isInCollision() )
        {
            _Valid = false;
        }
        else
        {
            if (*_Begin != *_End)
            {
                _Valid = Move3DLocalPathIsValid( *this, _NbColTest );
            }
        }
        _NbColTest++;
        //    cout << "_NbColTest : " << _NbColTest << endl;
        _Evaluated = true;
    }
    return _Valid;
}

int LocalPath::getNbColTest()
{
    if (_Evaluated)
    {
        return _NbColTest;
    }
    else
    {
        cout << "LocalPath::Warning => is not evaluated in getNbColTest" << endl;
    }
    return 0;
}

int LocalPath::getNbCostTest()
{
    if (_costEvaluated)
    {
        return _NbCostTest;
    }
    else
    {
        cout << "LocalPath::Warning => is not evaluated in getNbCostTest" << endl;
    }
    return 0;
}

double LocalPath::length()
{
    return Move3DLocalPathGetLength( *this );
}

double LocalPath::getParamMax()
{
    return Move3DLocalPathGetParamMax( *this );
}

confPtr_t LocalPath::configAtDist(double dist)
{
    return Move3DLocalPathConfigAtDist( *this, dist );
}

confPtr_t LocalPath::configAtParam(double param)
{
    confPtr_t q(new Configuration(_Robot));
    q.reset();
    Move3DLocalPathConfigAtParam( *this, param, q );
    return q;
}

double LocalPath::stayWithInDistance(double u, bool goForward, double* distance)
{
    return Move3DLocalPathStayWithinDist( *this, u, goForward, *distance );
}

//bool LocalPath::unvalidLocalpathTest(Robot* R, int* ntest)
//{
//    return p3d_unvalid_localpath_test( (p3d_rob*)R->getP3dRobotStruct(), getP3dLocalpathStruct(), ntest );
//}

/*!
 * Returns the closest Resolution To Step
 */
double LocalPath::getResolution(double step)
{
    if (!_ResolEvaluated)
    {
        double length = getParamMax();

        if (step == 0.0)
        {
            step = PlanEnv->getDouble(PlanParam::costResolution)*ENV.getDouble(Env::dmax);
        }

        _Resolution = length/ceil(length/(step));
        _ResolEvaluated = true;
    }

    return _Resolution;
}

/*!
 * Returns the number of cost segment
 * in the chosen resolution
 */
unsigned int LocalPath::getNumberOfCostSegments()
{
    return round(getParamMax()/getResolution());
}


/*!
 * Returns the cost profile
 */
vector< pair<double,double> > LocalPath::getCostProfile()
{
    vector< pair<double,double> > vectOfCost;

    // When not cost space
    if (!ENV.getBool(Env::isCostSpace))
    {
        return vectOfCost;
    }

    // Cost along path
    double currentCost;

    // Param along path
    double currentParam = 0.0;
    const double DeltaStep = getResolution();
    const unsigned int nStep = floor( ( getParamMax() / DeltaStep) + 0.5);

    cout <<  "nStep : " << nStep <<  endl;

    for (unsigned int i = 0; i < nStep; i++)
    {
        currentCost = configAtParam( currentParam )->cost();
        vectOfCost.push_back( make_pair( currentParam , currentCost ) );
        currentParam += DeltaStep;

        //		cout << "vectOfCost[" << i << "].first = " << vectOfCost[i].first << endl;
        //		cout << "vectOfCost[" << i << "].second = " << vectOfCost[i].second << endl;
    }

    return vectOfCost;
}


double LocalPath::whenCostIntegralPasses(double thresh)
{
    // When not cost space
    if (!ENV.getBool(Env::isCostSpace))
    {
        if (thresh > getParamMax())
        {
            return getParamMax();
        }

        return thresh;
    }

    // Cost along path
    double costSoFar = 0.0;
    double currentCost, prevCost;

    // Param along path
    double currentParam = 0.0;
    const double DeltaStep = getResolution();
    const unsigned int nStep = floor( ( getParamMax() / DeltaStep) + 0.5) ;

    confPtr_t confPtr;

    prevCost = getBegin()->cost();

    for (unsigned int i = 0; i < nStep; i++)
    {
        currentParam += DeltaStep;

        confPtr = configAtParam( currentParam );

        currentCost = global_costSpace->cost( *confPtr );
        costSoFar +=	global_costSpace->deltaStepCost( prevCost, currentCost, DeltaStep);

        if ( costSoFar > thresh )
        {
            return (currentParam-DeltaStep);
        }

        prevCost = currentCost;
    }

    return getParamMax();
}


double LocalPath::cost()
{
    if (!_costEvaluated)
    {
        _Cost = global_costSpace->cost( *this, _NbCostTest );
        _costEvaluated = true;
        // cout << "local path cost : " << _Cost << endl;
    }
//    else {
//        cout << "already eval" << endl;
//    }

    return _Cost;
}

void LocalPath::print()
{
    cout << "------------ Localpath Description----------------" << endl;
    cout << "mBegin =>" << endl;
    _Begin->print();
    cout << "mEnd   =>" << endl;
    _End->print();
    cout << "range_param = " << this->getParamMax() << endl;
    cout << "length = " << length() << endl;
    cout << "--------------- End  Description ------------------" << endl;
}

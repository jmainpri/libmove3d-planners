//
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
#include "planner/cost_space.hpp"
#include "planner/planEnvironment.hpp"

#include "API/planningAPI.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "Planner-pkg.h"

#include "Graphic-pkg.h"

using namespace std;
using namespace tr1;


// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

LocalPath::LocalPath(confPtr_t B, confPtr_t E) :
  _Robot(B->getRobot()),
  _Begin(B), _End(E),
  _LocalPath(NULL),
  //_Graph(_Robot->getActivGraph()),
  _Valid(false),
  _Evaluated(false),
  _lastValidParam(0.0),
  _lastValidEvaluated(false),
  _NbColTest(0),
  _costEvaluated(false), _Cost(0.0),
  _ResolEvaluated(false), _Resolution(0.0),
  _Type(LINEAR)
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
_LocalPath(NULL)
//_Graph(_Robot->getActivGraph()),

{
	// For extend (Construct a smaller local path)
	// This function is used in Base Expansion
	if(!lastValidConfig)
	{
		_End = path.configAtParam(p * path.getParamMax());
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
		_Evaluated = (path._Evaluated), 
		_lastValidParam = (p > 0 ? 1.0 : 0.0);
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
  _LocalPath(path._LocalPath ?
	     path._LocalPath->copy(path._Robot->getRobotStruct(), path._LocalPath) :
	     NULL),
  //	_Graph(path._Graph),
  _Valid(path._Valid),
  _Evaluated(path._Evaluated),
  _lastValidParam(0.0),
  _lastValidConfig(path._lastValidConfig),  
  _lastValidEvaluated(false),
  _NbColTest(path._NbColTest),
  _costEvaluated(path._costEvaluated), _Cost(path._Cost),
  _ResolEvaluated(path._ResolEvaluated), _Resolution(path._Resolution),
  _Type(path._Type)
{
}


LocalPath::LocalPath(Robot* R, p3d_localpath* lpPtr) :
  _Robot(R),
  _LocalPath(NULL),
  _Valid(false),
  _Evaluated(false),
  _lastValidParam(0.0),
  _lastValidEvaluated(false),
  _NbColTest(0),
  _costEvaluated(false), _Cost(0.0),
  _ResolEvaluated(false), _Resolution(0.0)
{
  // TODO : check which copy are/are not necessary.
  if (lpPtr)
  {
    _LocalPath = lpPtr->copy(_Robot->getRobotStruct(), lpPtr);
    
    _Begin = confPtr_t (
      new Configuration(_Robot,
					getLocalpathStruct()->config_at_param(
					  _Robot->getRobotStruct(), 
						getLocalpathStruct(),
					  0),true));
    
    _Begin->setConstraints();
    
    _End = confPtr_t (
      new Configuration(_Robot,
						getLocalpathStruct()->config_at_param(
					  _Robot->getRobotStruct(), 
						getLocalpathStruct(),
					  getLocalpathStruct()->range_param),true));
    
    _End->setConstraints();
    
    _Type = lpPtr->type_lp;
  }
  else
  {
    cout << "Warning : creating Localpath from uninitialized p3d_localpath"
	 << endl;
  }
}


LocalPath::~LocalPath()
{
  if (_LocalPath)
  {
    getLocalpathStruct()->destroy(_Robot->getRobotStruct(), getLocalpathStruct());
  }
}

//Accessors
p3d_localpath* LocalPath::getLocalpathStruct(bool multi_sol)
{
	if (!_LocalPath)
	{
    if( !multi_sol )
    {
      _LocalPath = p3d_local_planner(_Robot->getRobotStruct(),_Begin->getConfigStruct(), _End->getConfigStruct());
    }
    else
    {
      _LocalPath = p3d_local_planner_multisol(_Robot->getRobotStruct(), _Begin->getConfigStruct(), _End->getConfigStruct(), _ikSol);
    }

		if (_LocalPath)
		{
			_Type = _LocalPath->type_lp;
		}
	}
	return _LocalPath;
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

p3d_localpath_type LocalPath::getType()
{
	if (getLocalpathStruct())
	{
		return _Type;
	}
	else
	{
		return (p3d_localpath_type) (NULL);
	}
}

confPtr_t LocalPath::getLastValidConfig(double& p)
{
	_lastValidConfig = confPtr_t(new Configuration(_Robot));
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
			_lastValidConfig = confPtr_t (new Configuration(_Robot));
		}
		configPt q = _lastValidConfig->getConfigStruct();
		configPt *q_atKpath = &q;
		
		_Valid = !p3d_unvalid_localpath_classic_test(_Robot->getRobotStruct(),
													 this->getLocalpathStruct(),
													 /*&(_Graph->getGraphStruct()->nb_test_coll)*/&_NbColTest,
													 &_lastValidParam, q_atKpath);
		
		_Evaluated = true;
		_lastValidEvaluated = true;
	}
	return(_Valid);
}

bool LocalPath::isValid()
{
	if (!_Evaluated)
	{
		if (_End->isInCollision())
		{
			_Valid = false;
		}
		else
		{
			if (*_Begin != *_End)
			{
				_Valid = !p3d_unvalid_localpath_test(_Robot->getRobotStruct(), this->getLocalpathStruct(), &_NbColTest);
//        if( !_Valid ) 
//        {
//          cout << "_NbColTest : " << _NbColTest << endl;
//          g3d_draw_allwin_active();
//        }
			}
		}
		_NbColTest++;
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

double LocalPath::length()
{
	if (this->getLocalpathStruct())
	{
		return (this->getLocalpathStruct()->length_lp);
	}
	else
	{
		if (_Begin->equal(*_End))
			return 0;
		else
		{
		  std::cout << "ERROR : in Localpath::length() : this->getLocalpathStruct() is NULL" << std::endl;
		  return(-1);
		}
	}
}

double LocalPath::getParamMax()
{
	if (*_Begin == *_End)
	{
		return 0;
	}

	return this->getLocalpathStruct()->range_param;
}

confPtr_t LocalPath::configAtDist(double dist)
{
	//fonction variable en fonction du type de local path
  configPt q(NULL);
	switch (getType())
	{
	case HILFLAT://hilare
		q = p3d_hilflat_config_at_distance(_Robot->getRobotStruct(),getLocalpathStruct(), dist);
		break;
	case LINEAR://linear
		q = p3d_lin_config_at_distance(_Robot->getRobotStruct(),getLocalpathStruct(), dist);
		break;
	case MANHATTAN://manhatan
		q = p3d_manh_config_at_distance(_Robot->getRobotStruct(),getLocalpathStruct(), dist);
		break;
	case REEDS_SHEPP://R&S
		q = p3d_rs_config_at_distance(_Robot->getRobotStruct(),getLocalpathStruct(), dist);
		break;
	case TRAILER:
		q = p3d_trailer_config_at_distance(_Robot->getRobotStruct(),getLocalpathStruct(), dist);
		break;
	default:
	  // TODO : implement those methods !
	  std::cout << "ERROR : LocalPath::configAtDist : the TRAILER_FORWARD, HILFLAT_FORWARD, and DUBINS localpath types are not implemented." << std::endl;
	}
	return confPtr_t(new Configuration(_Robot, q));
}

confPtr_t LocalPath::configAtParam(double param)
{
	//fonction variable en fonction du type de local path
	configPt q;

	if (param > getParamMax())
	{
		return _End;
	}
	if (param < 0)
	{
		return _Begin;
	}

	q = getLocalpathStruct()->config_at_param(_Robot->getRobotStruct(),
			getLocalpathStruct(), param);

	if ( q == NULL ) 
	{
		throw string("Could not find configuration along path");
	}
	
	confPtr_t ptrQ(new Configuration( _Robot, q, true ));
	ptrQ->setConstraints();
	return ptrQ;
}

double LocalPath::stayWithInDistance(double u, bool goForward, double* distance)
{
	int way;
	
	if (goForward) {
		way = 1;
	}
	else {
		way = -1;
	}

	double du = getLocalpathStruct()->stay_within_dist(_Robot->getRobotStruct() , getLocalpathStruct(), 
											  u,
											  way, 
											  distance);
	
	return du;
}

bool LocalPath::unvalidLocalpathTest(Robot* R, int* ntest)
{
	return p3d_unvalid_localpath_test(R->getRobotStruct(),
			getLocalpathStruct(), ntest);
}

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
			step = PlanEnv->getDouble(PlanParam::costResolution)*p3d_get_env_dmax();
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
		_Cost = global_costSpace->cost(*this);
		_costEvaluated = true;
	}
	
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

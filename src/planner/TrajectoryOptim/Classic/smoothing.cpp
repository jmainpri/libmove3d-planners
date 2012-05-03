/*
 * Smoothing.cpp
 *
 *  Created on: Jun 26, 2009
 *      Author: jmainpri
 */
#include "smoothing.hpp"

#include "API/Device/robot.hpp"
#include "../p3d/env.hpp"
#include "planEnvironment.hpp"

#include <fstream>
#include <algorithm>

#include "Util-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

using namespace API;

#define DEBUG_STATUS 1

Smoothing::Smoothing() :
m_ContextName("Context"),
m_nbBiased(0), 
m_nbReallyBiased(0), 
m_step(0.0),
m_ShortCutBiased(true)
{
	m_Selected.resize(0);
}

Smoothing::Smoothing(const Trajectory& T) :
Trajectory(T),
m_nbBiased(0), 
m_nbReallyBiased(0), 
m_step(0.0),
m_ShortCutBiased(true)
{
  if( PlanEnv->getBool(PlanParam::trajBiasOptim) ) {
    setSortedIndex();
  }
	m_Selected.resize(0);
}

Smoothing::Smoothing(Robot* R, p3d_traj* t) :
Trajectory(R, t),
m_nbBiased(0), 
m_nbReallyBiased(0),
m_step(0.0),
m_ShortCutBiased(true)
{
  if( PlanEnv->getBool(PlanParam::trajBiasOptim) ) {
    setSortedIndex();
  }
	m_Selected.resize(0);
}

Smoothing::~Smoothing()
{
	//	cout << "Delete Smoothing" << endl;	
}

void Smoothing::setStep( double step )
{
  m_useAutoStep = false;
  m_step = step;
}

void Smoothing::resetStep()
{
  m_useAutoStep = true;
}

/*!
 * One loop of the short cut method
 */
bool Smoothing::oneLoopShortCut( )
{
	bool isOptimSuccess(false);
	
	double lFirst(0.0);
	double lSecond(0.0);
	
	if (!(getNbOfPaths() > 1))
		return false;
	
	vector<confPtr_t> vectConf = get2RandomConf( m_step, lFirst, lSecond );
	
	LocalPath* newPath = new LocalPath( vectConf[0], vectConf[1] );
	
	bool supposedValid = true;
  
	if ( !ENV.getBool(Env::costBeforeColl) && PlanEnv->getBool(PlanParam::trajComputeCollision) )
	{
		supposedValid = newPath->isValid();
	}
  
  // If the path is valid, check for cost
	if ( supposedValid )
	{
		if (false /*ENV.getBool(Env::debugCostOptim)*/)
		{
			if( getIdOfPathAt(lFirst)==getHighestCostId() || getIdOfPathAt(lSecond)==getHighestCostId() )
      {
				debugShowTraj(lFirst,lSecond);
				m_nbReallyBiased++;
			}
		}
		
		// If the new path is of lower cost, replace in trajectory
		if(ENV.getBool(Env::costBeforeColl) && PlanEnv->getBool(PlanParam::trajComputeCollision) )
		{
			supposedValid = newPath->isValid();
		}
		
		vector<LocalPath*> portion; 
    portion.push_back( newPath );
		
		if ( supposedValid && isLowerCostLargePortion( lFirst, lSecond, portion ) )
		{
			/**
			 * Replace
			 */
			vector<LocalPath*> vect_path;
			vect_path.push_back(newPath);
			
			replacePortion( lFirst, lSecond, vect_path );
      
      if( PlanEnv->getBool(PlanParam::trajBiasOptim) ) {
        setSortedIndex();
      }
			
			if ( !getBegin()->equal( *configAtParam(0) ) ) {
				cout << "ERROR" << endl;
			}
			
			if (!getEnd()->equal(*configAtParam(getRangeMax()))) {
				cout << "ERROR" << endl;
			}
			
			isOptimSuccess = true;
		}
	}
	
	if (isOptimSuccess == false)
	{
		delete newPath;
	}
	
	return isOptimSuccess;
}

/*!
 * Replaces by a shortcut if of lower cost
 * but recomputes the entire trajectory
 */
bool Smoothing::oneLoopShortCutRecompute()
{
	bool isOptimSuccess(false);
	
	double lFirst(0.0);
	double lSecond(0.0);
	
	if (!(getNbOfPaths() > 1))
		return false;
	
	vector<confPtr_t> vectConf;
	
	vectConf = get2RandomConf(0.0,lFirst,lSecond);
	
	confPtr_t qFirstPt = vectConf.at(0);
	confPtr_t qSecondPt = vectConf.at(1);
	
	// Take a configuration in a certain direction
	LocalPath* newPath = new LocalPath(qFirstPt, qSecondPt);
  
  bool newPathIsValid = true;
  
  if (PlanEnv->getBool(PlanParam::trajComputeCollision))
  {
    newPathIsValid = newPath->isValid();
  }
	
	// If the new path is free in CFree
	if ( newPathIsValid )
	{
    Trajectory newTraj(*this);
    
		vector<LocalPath*> paths;
		paths.push_back(newPath);
		
		newTraj.replacePortion( lFirst, lSecond, paths );
    
    double CurrentCost = this->costRecomputed();
		
		// If the new path is of lower cost, replace in current trajectory
		if (newTraj.costRecomputed() < CurrentCost )
		{
			vector<LocalPath*> vect_path;
			vect_path.push_back(new LocalPath(*newPath));
			
			replacePortion( lFirst, lSecond, vect_path );
			
			setSortedIndex();
			
			if (!getBegin()->equal(*configAtParam(0)))
			{
				cout << "ERROR" << endl;
			}
			
			if (!getEnd()->equal(*configAtParam(getRangeMax())))
			{
				cout << "ERROR" << endl;
			}
			isOptimSuccess = true;
		}
	}
	
	return isOptimSuccess;
}

/*!
 * One loop of the short cut method
 */
bool Smoothing::partialShortcut()
{
	bool isOptimSuccess(false);
	
	double lFirst(0.0);
	double lSecond(0.0);
	
	shared_ptr<Configuration> qFirstPt;
	shared_ptr<Configuration> qSecondPt;
	
	if (!(getNbOfPaths() > 1))
		return false;
	
	//cout << "ithActiveDoF = " << ithActiveDoF << endl;
	
	vector<shared_ptr<Configuration> > vectConf;
	
	// Get Two Configurations
	vectConf = get2RandomConf( m_step, lFirst, lSecond );
	qFirstPt = vectConf.at(0);
	qSecondPt = vectConf.at(1);
	
	try 
	{
		vector<shared_ptr<Configuration> > vectConf;
		
		// Select a DoF
		unsigned int ithActiveDoF = p3d_random(0,m_Robot->getNumberOfActiveDoF());
		
		vectConf.clear();
		vectConf = getConfAtStepAlongTraj( m_step/5 , lFirst , lSecond );
		
		// Too little step between lFirst and lSecond
		if (vectConf.size() < 3)
		{
			return false;
		}
		
		if (!qFirstPt->equal(*vectConf[0])) 
		{
			cout << "Warning : partialShortcut start" << endl;
		}
		if (!qSecondPt->equal(*vectConf.back())) 
		{
			cout << "Warning : partialShortcut end" << endl;
		}
		
		double interpolationStep = 1/(double)vectConf.size();
		
		// Get the value of the configuration i and i+1
		double init = vectConf[0]->getActiveDoF(ithActiveDoF);
		double end = vectConf.back()->getActiveDoF(ithActiveDoF);
		
		if ( vectConf.size() >= 3) 
		{
			double alpha = interpolationStep;
			
			for (unsigned int i=1; i<vectConf.size()-1; i++) 
			{
				// Interpolate on one Dof
				double value = interpolateOneDoF( ithActiveDoF , init , end , alpha );
				
				// Change the value of a configuration at the ith active dof
				changeIthActiveDofValueOnConf( *vectConf[i] , ithActiveDoF , value );
				
				// Go to next Conf
				alpha += interpolationStep;
			}
		}
		
		Trajectory traj( vectConf );
		
		vector<LocalPath*> newPaths = traj.getCourbe();
		
		if ( traj.isValid() && isLowerCostLargePortion( lFirst , lSecond , newPaths ) ) 
		{
			copyPaths( newPaths );
			
			if( replacePortion( lFirst, lSecond , newPaths ) )
			{
				isOptimSuccess = true;
			}
			//			else 
			//			{
			//				cout << "Could not replace portion in " << __func__ << " " << __FILE__ << endl;
			//			}
		}
	}
	catch (string str) 
	{
		cout << "Exeption : " << str << endl;
	}
	catch (...) 
	{
		cout << "Second part" << endl;
	}
	
	return isOptimSuccess;
}

/**
 * Compare the cost of two portions
 */
bool Smoothing::isLowerCostLargePortion( double lFirst, double lSecond , vector<LocalPath*>& newPaths)
{
	vector<LocalPath*> paths;
	unsigned int first,last;
	
	vector<confPtr_t> confs = getTowConfigurationAtParam( lFirst, lSecond, first, last );
	
	for(unsigned int i=first;i<=last;i++) {
		paths.push_back( getLocalPathPtrAt(i) );
	}
	
	double costOfPortion = computeSubPortionCost(paths);
	
	//	if( !(*vectConf[0] == *qFirstPt) || !(*vectConf[1] == *qSecondPt) )
	//	{
	//		cout << "Error in oneLoopShortCut" << endl;
	//	}
	
	paths.clear();
	paths = newPaths;
	
	LocalPath* LP1 = new LocalPath( getLocalPathPtrAt(first)->getBegin(), confs[0] );
	if(LP1->getParamMax()>0) {
		paths.insert(paths.begin(),LP1);
	}
	
	LocalPath* LP2 = new LocalPath( confs.back(), getLocalPathPtrAt(last)->getEnd() );
	if(LP2->getParamMax()>0) {
		paths.push_back(LP2);
	}
	
	// Compute the cost of the portion to be replaced
	double sumOfCost = computeSubPortionCost(paths);
	
	delete LP1;
	delete LP2;
	
	return ( sumOfCost < costOfPortion );
}

/**
 * Interpolates the ith active DoF of the robot
 */
double Smoothing::interpolateOneDoF( unsigned int ithActiveDoF , 
																		double init , 
																		double end , 
																		double alpha )
{
	unsigned int ithDofOnJoint; // Ith Dof on the joint
	double vmin,vmax; // Min and Max values of the dof
	
	Joint* jntPt = m_Robot->getIthActiveDoFJoint( ithActiveDoF, ithDofOnJoint );
	
	// alpha should be between 0 and 1
  alpha = MAX(0.,MIN(1.,alpha));
	
  if (p3d_jnt_is_dof_circular(jntPt->getJointStruct(), ithDofOnJoint)) 
	{
		// Get the min and max values of the DoF
    p3d_jnt_get_dof_bounds_deg(jntPt->getJointStruct(), 
															 ithDofOnJoint, 
															 &vmin, &vmax);
		
    if(vmin < 0.0) 
		{
      return angle_limit_PI(init + alpha *
                            diff_angle(init, end));
    }
		else 
		{
      return angle_limit_2PI(init + alpha *
                             diff_angle(init, end));
    }
  }
	
	// Linear interpolation in case of translation DoF
  return (init + alpha * (end - init));
}

/**
 * returns a configuration with the interpoated DoF
 */
void Smoothing::changeIthActiveDofValueOnConf( Configuration& q,
																							unsigned int ithActiveDoF, 
																							double value )
{
	unsigned int activeDof = 0;
	
	// All Joints
	for(unsigned int i=0;i<m_Robot->getNumberOfJoints();i++)
	{
		Joint* jntPt = m_Robot->getJoint(i);
		
		// All Dofs on Joint
		for(unsigned int j=0; j<jntPt->getNumberOfDof(); j++) 
		{
			int k = jntPt->getIndexOfFirstDof() + j;
			
			// All Active DoFs
			if (
					(p3d_jnt_get_dof_is_user(jntPt->getJointStruct(), j) && p3d_jnt_get_dof_is_active_for_planner(jntPt->getJointStruct(),j)) &&
					(m_Robot->getRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) 
			{
				// Change the corresponding value of the configuration
				// Passed as an argument of the active DoF
				if( activeDof == ithActiveDoF )
				{
					q[k] = value; return;
				}
				
				activeDof++;
			}
		}
	}
}

void Smoothing::removeRedundantNodes()
{
	if(!isValid())
	{
		cout << "Trajectory Not Valid" << endl;
		return;
	}
	else
	{
		cout << "Trajectory Valid" << endl;
	}
	
	//	this->cutTrajInSmallLP(20);
	//	this->cost();
	
	uint NbNodes = getNbOfPaths();
	uint initNbNodes = NbNodes;
	uint nbRemoved(0);
	
	for (uint i = 0; i < NbNodes-2; i++)
	{
		for (uint j = i + 2; j < NbNodes; j++)
		{
			confPtr_t start = getLocalPathPtrAt(i)->getBegin();
			confPtr_t end   = getLocalPathPtrAt(j)->getBegin();
			
			LocalPath* pathPtr = new LocalPath(start, end);
			
			if(start->equal(*end))
			{
				cout << "Error start equal goal" << endl;
			}
			
			if (pathPtr->isValid())
			{
				double costOfPortion = 0;
				
				for (uint k = i; k < j; k++)
				{
					costOfPortion += getLocalPathPtrAt(k)->cost();
				}
				
				if (pathPtr->cost() < costOfPortion)
				{
					vector<LocalPath*> path;
					path.push_back(pathPtr);
					this->replacePortionOfLocalPaths(i, j, path);
					NbNodes = getNbOfPaths();
					nbRemoved++;
					if(!isValid())
					{
						cout << "Trajectory Not Valid" << endl;
					}
				}
				else
				{
					delete pathPtr;
				}
			}
			else
			{
				delete pathPtr;
			}
		}
	}
	
	setSortedIndex();
	
	cout << nbRemoved << " nodes were removed out of " << initNbNodes << endl;
}

void Smoothing::debugShowTraj(double lPrev,double lNext)
{
	vector<shared_ptr<Configuration> > vectConf(2);
	vectConf.at(0) = configAtParam(lPrev);
	vectConf.at(1) = configAtParam(lNext);
	
	trajToDraw.resize(4);
	
	trajToDraw.at(0) = extractSubTrajectory(0,lPrev);
	trajToDraw.at(1) = extractSubTrajectory(lPrev,lNext);
	trajToDraw.at(2) = *new Trajectory(vectConf);
	trajToDraw.at(3) = extractSubTrajectory(lNext,getRangeMax());
	
	trajToDraw.at(0).setColor(0);
	trajToDraw.at(1).setColor(2);
	trajToDraw.at(2).setColor(1);
	trajToDraw.at(3).setColor(0);
	
	//			basicTraj.print();
	//			triangleTraj.print();
	g3d_draw_allwin_active();
	
	vector<LocalPath*> pathsTmp(1);
	pathsTmp.at(0) = (new LocalPath(vectConf.at(0), vectConf.at(1)));
	
	Trajectory tmpT(*this);
	tmpT.replacePortion(lPrev, lNext, pathsTmp);
	double newCost = tmpT.cost();
	
	//	if( mincost > newCost ){
	//		mincost = newCost;
	//
	//	}
	
	double oldCost = cost();
	double sumOfCost = pathsTmp.at(0)->cost();
	double costOfPortion = this->costOfPortion(lPrev, lNext);
	
	cout << "Difference on the portion : " << (costOfPortion - sumOfCost)
	<< endl;
	cout << "Difference on the trajectory: " << (oldCost - newCost) << endl;
	cout << "---------------------------------------------------------" << endl;
	
	// double diff = fabs((costOfPortion - sumOfCost) - (oldCost - newCost));
	
	//	if(diff > 0.001 ){
	//		Errors.push_back(diff);
	//		nbErrors++;
	//	}
	
}

double Smoothing::closestResolutionToStep(double length,double step)
{
	if ( length < step )
	{
		return length;
	}
	
	if ( floor(length/step) == length/step ) 
	{
		return step;
	}
	
	double n = floor(length/step); // minimal number of segment
	return length / n;
}

vector<shared_ptr<Configuration> > Smoothing::getConfAtStepAlongTraj( double step , double firstDist, double secondDist )
{
	double length = secondDist-firstDist;
	
	if (length < 0.0 ) 
	{
		throw string("length is negative");
	}
	
	double newStep = closestResolutionToStep(length,step); 
	
	vector<shared_ptr<Configuration> > vectConf;
	
	if ( length == newStep ) 
	{
		vectConf.push_back( configAtParam( firstDist ) );
		vectConf.push_back( configAtParam( secondDist ) );
		return vectConf;
	}
	
	unsigned int N = floor(length/newStep);
	if (N == 1) 
	{
		throw string("weird case");
	}
	
	length = firstDist;
	
	for (unsigned int i=0; i<N ; i++) 
	{
		vectConf.push_back( configAtParam( length ) ); length += newStep;
		if( i == N-2 ) // last configuration
		{
			length = secondDist;
		}
	}
	
	//	cout << "number of configuration : " << vectConf.size() << endl;
	
	return vectConf;
}

vector<shared_ptr<Configuration> > Smoothing::get2RandomConf(double step, double& firstDist, double& secondDist)
{	
	unsigned int id1 = 0;
	unsigned int id2 = 0;
	
	if (getNbOfPaths() == 1)
	{
		vector<confPtr_t> vect(0);
		return vect;
	}
	
	vector<confPtr_t> vectConf(2);
	
  // While two configuration have not been selected on different localpath
  // keep on sampling parameters along the trajectory
	while (id1 == id2)
	{
		double param;
		
		if (PlanEnv->getBool(PlanParam::trajBiasOptim) && !PlanEnv->getBool(PlanParam::trajPartialShortcut) )
		{
			param = getBiasedParamOnTraj();
      
			if ( step > 0.0 ) 
			{
				firstDist		= param - step/2;
				secondDist	= param + step/2;
				
				if ( firstDist < 0.0 ) {
					firstDist = 0.0;
				}
				if ( secondDist > getRangeMax() ) {
					secondDist = getRangeMax();
				}
			}
			else 
			{
				// If param is closer to the end of the traj
        // then choose the first dist between the start and the random param
        // else choose the second dist between tne end and the random param
				if (fabs(param - getRangeMax()) < fabs(param))
				{
					secondDist = param;
					firstDist = p3d_random( 0 , secondDist );
				}
				else
				{
					firstDist = param;
					secondDist = p3d_random( firstDist , getRangeMax()) ;
				}
			}
		}
		else
		{
			firstDist = p3d_random( 0, getRangeMax() );
			secondDist = p3d_random( firstDist , getRangeMax() );
		}
		
		id1 = getIdOfPathAt(firstDist);
		id2 = getIdOfPathAt(secondDist);
		
		if (id1 != id2)
		{
			vectConf.at(0) = configAtParam(firstDist);
			vectConf.at(1) = configAtParam(secondDist);
		}
	}
	
	return vectConf;
}

class myComparator
{
	
public:
	Smoothing* ptrOptim;
	
	bool operator()(uint i, uint j)
	{
		
		return (ptrOptim->getLocalPathPtrAt(i)->cost()
						> ptrOptim->getLocalPathPtrAt(j)->cost());
	}
	
} myCompObject;

void Smoothing::setSortedIndex()
{
	myCompObject.ptrOptim = this;
	
	m_IdSorted.resize(getNbOfPaths());
  
  if (getNbOfPaths() == 0 ) {
    return;
  }
	
	for (int i=0; i<getNbOfPaths(); i++) {
		m_IdSorted.at(i) = i;
	}
	
	sort(m_IdSorted.begin(), m_IdSorted.end(), myCompObject);
	
	isHighestCostIdSet = true;
	HighestCostId = m_IdSorted[0];
	
	if (ENV.getBool(Env::debugCostOptim))
	{
		/*for(uint i=0;i<mIdSorted.size();i++){
		 cout<<"mIdSorted["<<i<<"]"<<getLocalPathPtrAt(mIdSorted[i])->cost()<<endl;
		 }*/
	}
}

//!
//!
double Smoothing::getBiasedParamOnTraj()
{
	double x = (double) (pow(p3d_random(0, 1), 3));
	uint id = (uint) (x * (getNbOfPaths() - 1));
  
	double randDist = 0;
	
  
	for (uint i=0; i<m_IdSorted[id]; i++) {
		randDist += getLocalPathPtrAt(i)->getParamMax();
	}
	
	randDist += p3d_random(0, getLocalPathPtrAt(m_IdSorted[id])->getParamMax());
	
	if (getIdOfPathAt(randDist) == HighestCostId) {
		m_nbBiased++;
	}
	
	//	double percent = (double)id/(double)(mIdSorted.size());
	if (ENV.getBool(Env::debugCostOptim)) {
		m_Selected.push_back(id);
	}
	
	return randDist;
}

confPtr_t Smoothing::getRandConfAlongTraj(double& randDist, bool use_bias)
{
	// Computes the distances along the 
	// trajectories from which to select the random configuration
	if ( use_bias ) 
  {
		randDist = getBiasedParamOnTraj();
	}
	else {
		randDist = p3d_random(0, getRangeMax());
	}
	
	return configAtParam(randDist);
}

/*!
 * Checks out if the plannification
 * problem has reach its goals
 */
bool Smoothing::checkStopConditions(unsigned int iteration)
{	
	if ( Env_stopUser() )
	{
		PlanEnv->setBool(PlanParam::stopPlanner,true);
	}
	
	if ( PlanEnv->getBool(PlanParam::stopPlanner) )
	{
		cout << "Smoothin cancelled by user" << endl;
		return true;
	}
	
	if ( PlanEnv->getBool(PlanParam::withMaxIteration) ) 
	{
		if ( iteration > m_MaxNumberOfIterations ) 
		{
			cout << "Smoothing has reached max number of iteration ( " << m_MaxNumberOfIterations << " ) " << endl;
			return true;
		}
	}
	
	if( PlanEnv->getBool(PlanParam::trajWithTimeLimit) )
	{
		if ( m_time > PlanEnv->getDouble(PlanParam::timeLimitSmoothing) ) 
		{
			cout << "Smoothin has reached time limit ( " << m_time << " ) " << endl;
			return true;
		}
	}
	
	if ( PlanEnv->getBool(PlanParam::withGainLimit) ) 
	{
		const int n = 30;
		if ( gainOfLastIterations( n ) < /*0.005*/ 0.01 )
		{
			cout << "Smooting has reached maximal gain ( " << gainOfLastIterations( n ) << " ) " << endl;
			return true;
		}
	}
	
	return false;
}

/*!
 * This function computes the gain that
 * have made the last iteration of the smoothing process
 */
double Smoothing::gainOfLastIterations(unsigned int n)
{
	if ( m_GainOfIterations.size() < n ) 
	{
    // if the gain history is infierior to n in size 
    // return an avearage gain of 1.0, to continue the optimization process
		return 1.0;
	}
	else 
	{    
    //		int start=0;
    //    double gain(0.0);
    
    //		start = m_GainOfIterations.size() - n;
    //		
    //		for ( int i=start; i<int(m_GainOfIterations.size()); i++ ) 
    //		{
    //			gain += m_GainOfIterations[i];
    //		}
    //		
    //		gain /= double(n);
    //    return gain;
		return *max_element(m_GainOfIterations.end()-20,m_GainOfIterations.end());
	}
}

void Smoothing::storeCostAndGain( double NewCost, double CurCost )
{
  if ( NewCost > CurCost )
  {
    cout << "Smoothing::runDeformation : NewCost > CurrentCost"  << endl;
  }
  m_OptimCost.push_back( NewCost );
  m_GainCost.push_back( ( CurCost - NewCost ) / CurCost );
}

/*!
 * This function saves the 
 * trajectory cost to a file along the smoothing process
 */
void Smoothing::saveOptimToFile(string fileName)
{
	std::ostringstream oss;
	oss << "statFiles/"<< fileName << ".csv";
	
	const char *res = oss.str().c_str();
	
	std::ofstream s;
	s.open(res);
	
	cout << "Opening save file : " << res << endl;
	
	s << "Cost" << ";";
	s << "Gain" << ";" ;
	
	for (int i=0; i<int(m_OptimCost.size()); i++)
	{
		s << m_OptimCost[i] << ";";
		s << m_GainCost[i] << ";" ;
		s << endl;
	}
	
	cout << "Closing save file" << endl;
	
	s.close();
}

/*!
 * This is the main function iterating the smoothing methods
 */
void Smoothing::runShortCut( int nbIteration, int idRun )
{
  //  this->costNoRecompute();
  //#ifdef DEBUG_STATUS
  //  cout << "Before Short Cut : Traj cost = " << this->costNoRecompute() << endl;
  //#endif
	double costBeforeDeformation = this->cost();
	
  m_OptimCost.clear();
  m_GainCost.clear();
	m_GainOfIterations.clear();
	
  m_MaxNumberOfIterations = nbIteration;
	
	// Fix the step
  if( m_useAutoStep )
  {
    m_step = getRangeMax() / PlanEnv->getDouble(PlanParam::MaxFactor) ;
  }
	
	double ts(0.0); m_time = 0.0; ChronoOn();
	
	for (int i=0; !checkStopConditions(i); i++)
	{
		if ( this->getCourbe().size() == 1 ) 
		{
#ifdef DEBUG_STATUS
      cout << "Smooting has stoped, only one localpath in traj" << endl;
#endif
			break;
		}
		
		double CurCost = cost();
		
		try 
		{
			if ( PlanEnv->getBool( PlanParam::trajPartialShortcut ) ) 
			{
				partialShortcut();
			}
			else 
			{
        oneLoopShortCut();
			}
		}
		catch (string str)
		{
			cout << "Exeption in shortcut loop : " << str << endl;
			break;
		}
		//		catch (...) 
		//		{
		//			cout << "Exeption in shortcut loop" << endl;
		//			break;
		//		}
		
		double NewCost = cost();
		
		m_IterationSucceded = ( CurCost > NewCost );
		
		if(PlanEnv->getBool(PlanParam::trajSaveCost))
		{
			storeCostAndGain( NewCost, CurCost );
		}
		
    // Push back the gain of iteration that have succeded
		if ( m_IterationSucceded  ) 
		{
			double Gain = (( CurCost - NewCost ) / CurCost) ;
      
      if( PlanEnv->getBool( PlanParam::trajPrintGain ) ) {
        cout << "Gain = " << 100*Gain <<  " %" << endl;
      }
			m_GainOfIterations.push_back( Gain );
		}
		
		ChronoTimes( &m_time , &ts );
	}
	
	ChronoOff();
	
  if( PlanEnv->getBool(PlanParam::trajComputeCollision ) )
  {
    if ( isValid() ) {
#ifdef DEBUG_STATUS
      cout << "Trajectory valid" << endl;
#endif
    }
    else { 
      cout << "Trajectory not valid" << endl;
    }
  }
  
  if(PlanEnv->getBool(PlanParam::trajSaveCost))
  {
    ostringstream oss;
    oss << "ShortCutOptim_"<< m_ContextName << "_" << idRun << "_" ;
    this->saveOptimToFile( oss.str() );
  }
#ifdef DEBUG_STATUS
  cout << "Before : cost = " << costBeforeDeformation << endl;
  cout << "After : cost = " << this->costNoRecompute() << endl;
#endif
}

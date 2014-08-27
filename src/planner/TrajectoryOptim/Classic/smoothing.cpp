/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "smoothing.hpp"

#include "API/Device/robot.hpp"
#include "../p3d/env.hpp"
#include "planEnvironment.hpp"

#include "Util-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"

#include <iomanip>
#include <fstream>
#include <algorithm>
#include <sys/time.h>

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

using namespace Move3D;

std::vector< std::pair<double,double> > traj_convergence_with_time;

//#define DEBUG_STATUS_SHORTCUT 0

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
		return isOptimSuccess;
	
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
			if( getLocalPathId(lFirst)==getHighestCostId() || getLocalPathId(lSecond)==getHighestCostId() )
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
			/// Replace
			vector<LocalPath*> vect_path;
			vect_path.push_back(newPath);
			
			replacePortion( lFirst, lSecond, vect_path );
      
      if( PlanEnv->getBool(PlanParam::trajBiasOptim) ) {
        setSortedIndex();
      }
			
			if ( !getBegin()->equal( *configAtParam(0) ) ) {
                cout << "ERROR in " << __PRETTY_FUNCTION__ << endl;
			}
			
			if (!getEnd()->equal(*configAtParam(getParamMax()))) {
                cout << "ERROR in " << __PRETTY_FUNCTION__<< endl;
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
    
    if( newTraj.isValid() )
    {
      double CurrentCost = 0.0;
      double NewTrajCost = 0.0;
      
      if( PlanEnv->getBool( PlanParam::trajNPoints ) )
      {
        CurrentCost = m_currentCost; 
        NewTrajCost = newTraj.costNPoints( PlanEnv->getInt(PlanParam::nb_pointsOnTraj) );
      }
      else
      {
        CurrentCost = this->costRecomputed();
        NewTrajCost = newTraj.costRecomputed(); 
      }
      
      // If the new path is of lower cost, replace in current trajectory
      if ( NewTrajCost < CurrentCost )
      {
        vector<LocalPath*> vect_path;
        vect_path.push_back(new LocalPath(*newPath));
        
        replacePortion( lFirst, lSecond, vect_path );
        
        setSortedIndex();
        
        if( PlanEnv->getBool( PlanParam::trajNPoints ) )
        {
          m_currentCost = NewTrajCost;
        }
        
        if (!getBegin()->equal(*configAtParam(0)))
        {
          cout << "ERROR" << endl;
        }
        
        if (!getEnd()->equal(*configAtParam(getParamMax())))
        {
          cout << "ERROR" << endl;
        }
        isOptimSuccess = true;
      }
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
        cout << "ERROR in : " << __PRETTY_FUNCTION__ << endl;
        // double init = vectConf[0]->getActiveDoF(ithActiveDoF);
        // double end = vectConf.back()->getActiveDoF(ithActiveDoF);

        double init = 0;
        double end = 0;
		
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
            //				cout << "Could not replace portion in " << __PRETTY_FUNCTION__ << " " << __FILE__ << endl;
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
		paths.push_back( getLocalPath(i) );
	}
	
	double costOfPortion = computeSubPortionCost(paths);
	
	//	if( !(*vectConf[0] == *qFirstPt) || !(*vectConf[1] == *qSecondPt) )
	//	{
	//		cout << "Error in oneLoopShortCut" << endl;
	//	}
	
	paths.clear();
	paths = newPaths;
	
	LocalPath* LP1 = new LocalPath( getLocalPath(first)->getBegin(), confs[0] );
	if(LP1->getParamMax()>0) {
		paths.insert(paths.begin(),LP1);
	}
	
	LocalPath* LP2 = new LocalPath( confs.back(), getLocalPath(last)->getEnd() );
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
	
  if (p3d_jnt_is_dof_circular( static_cast<p3d_jnt*>(jntPt->getP3dJointStruct()), ithDofOnJoint))
	{
		// Get the min and max values of the DoF
    p3d_jnt_get_dof_bounds_deg( static_cast<p3d_jnt*>(jntPt->getP3dJointStruct()), ithDofOnJoint, &vmin, &vmax);
		
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
                    (p3d_jnt_get_dof_is_user( static_cast<p3d_jnt*>(jntPt->getP3dJointStruct()), j) && p3d_jnt_get_dof_is_active_for_planner( static_cast<p3d_jnt*>(jntPt->getP3dJointStruct()),j)) &&
					(m_Robot->getP3dRobotStruct()->cntrt_manager->in_cntrt[k] != 2) ) 
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
			confPtr_t start = getLocalPath(i)->getBegin();
			confPtr_t end   = getLocalPath(j)->getBegin();
			
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
					costOfPortion += getLocalPath(k)->cost();
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
	
	global_trajToDraw.resize(4);
	
	global_trajToDraw.at(0) = extractSubTrajectory(0,lPrev);
	global_trajToDraw.at(1) = extractSubTrajectory(lPrev,lNext);
	global_trajToDraw.at(2) = *new Trajectory(vectConf);
	global_trajToDraw.at(3) = extractSubTrajectory(lNext,getParamMax());
	
	global_trajToDraw.at(0).setColor(0);
	global_trajToDraw.at(1).setColor(2);
	global_trajToDraw.at(2).setColor(1);
	global_trajToDraw.at(3).setColor(0);
	
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
				if ( secondDist > getParamMax() ) {
					secondDist = getParamMax();
				}
			}
			else 
			{
				// If param is closer to the end of the traj
        // then choose the first dist between the start and the random param
        // else choose the second dist between tne end and the random param
				if (fabs(param - getParamMax()) < fabs(param))
				{
					secondDist = param;
					firstDist = p3d_random( 0 , secondDist );
				}
				else
				{
					firstDist = param;
					secondDist = p3d_random( firstDist , getParamMax()) ;
				}
			}
		}
		else
		{
			firstDist = p3d_random( 0, getParamMax() );
			secondDist = p3d_random( firstDist , getParamMax() );
		}
		
		id1 = getLocalPathId(firstDist);
		id2 = getLocalPathId(secondDist);
		
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
		
		return (ptrOptim->getLocalPath(i)->cost()
						> ptrOptim->getLocalPath(j)->cost());
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
	
    m_isHighestCostIdSet = true;
    m_HighestCostId = m_IdSorted[0];
	
	if (ENV.getBool(Env::debugCostOptim))
	{
		/*for(uint i=0;i<mIdSorted.size();i++){
		 cout<<"mIdSorted["<<i<<"]"<<getLocalPath(mIdSorted[i])->cost()<<endl;
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
		randDist += getLocalPath(i)->getParamMax();
	}
	
	randDist += p3d_random(0, getLocalPath(m_IdSorted[id])->getParamMax());
	
    if (getLocalPathId(randDist) == m_HighestCostId) {
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
		randDist = p3d_random(0, getParamMax());
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
//#ifdef DEBUG_STATUS_SHORTCUT
			cout << "Smoothing has reached max number of iteration ( " << m_MaxNumberOfIterations << " ) " << endl;
//#endif
			return true;
		}
	}
	
	if( PlanEnv->getBool(PlanParam::trajWithTimeLimit) )
	{
		if ( m_time > PlanEnv->getDouble(PlanParam::timeLimitSmoothing) ) 
		{
//#ifdef DEBUG_STATUS_SHORTCUT
			cout << "Smoothin has reached time limit ( " << m_time << " ) " << endl;
//#endif
			return true;
		}
	}
	
#ifdef DEBUG_STATUS_SHORTCUT
	if ( PlanEnv->getBool(PlanParam::withGainLimit) ) 
	{
		const int n = 30;
		if ( gainOfLastIterations( n ) < /*0.005*/ 0.01 )
		{
			cout << "Smooting has reached maximal gain ( " << gainOfLastIterations( n ) << " ) " << endl;
			return true;
		}
	}
#endif
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

void Smoothing::computeStats()
{
  m_convergence_rate.clear();
  
  for (int i=0; i<int(m_convergence_trajs.size()); i++) 
  {
    double time = m_convergence_trajs[i].first;
    Move3D::Trajectory traj( m_convergence_trajs[i].second );
    
    TrajectoryStatistics stat;
    traj.costStatistics( stat );
    m_convergence_rate.push_back( make_pair(time,stat) );
  }
}

void Smoothing::storeCostAndGain( double NewCost, double CurrCost )
{
  double Gain=0.0;
  
  if(NewCost != CurrCost )
    Gain = ( CurrCost - NewCost ) / CurrCost;
  
  m_OptimCost.push_back( NewCost );
  m_GainCost.push_back( Gain );
  
  if( PlanEnv->getBool( PlanParam::trajPrintGain ) ) 
    cout << "Gain = " << 100*Gain <<  " %" << endl;
  
  vector<confPtr_t> traj;
  for (int i=0; i<getNbOfViaPoints(); i++) 
  {
    traj.push_back( (*this)[i]->copy() );
  }
  m_convergence_trajs.push_back( make_pair(m_time,traj) );
}

/*!
 * This function saves the 
 * trajectory cost to a file along the smoothing process
 */
void Smoothing::saveOptimToFile(string fileName)
{
  std::ostringstream oss;
  std::ofstream s;
  
    oss << getenv("HOME_MOVE3D") << "/statFiles/convergence_traj_smooth_" << std::setfill('0') << std::setw(4) << m_runId << ".csv";
  
	const char *res = oss.str().c_str();
	
	s.open(res);
	cout << "Opening save file : " << res << endl;
	
  s << "TIME" << ";";
  s << "LENGTH" << ";";
  s << "MAX" << ";";
  s << "AVERAGE" << ";";
  s << "INTEGRAL" << ";";
  s << "MECHA WORK" << ";";
  s << "SUM" << ";";
  s << endl;
  
	for (int i=0; i<int(m_convergence_rate.size()); i++)
	{    
    s << m_convergence_rate[i].first << ";";
    s << m_convergence_rate[i].second.length << ";";
    s << m_convergence_rate[i].second.max << ";";
    s << m_convergence_rate[i].second.average << ";";
    s << m_convergence_rate[i].second.integral << ";";
    s << m_convergence_rate[i].second.mecha_work << ";";
    s << m_convergence_rate[i].second.sum << ";";
    s << endl;
	}
  
  s.close();
  cout << "Closing save file" << endl;
}

/*!
 * This is the main function iterating the smoothing methods
 */
void Smoothing::runShortCut( int nbIteration, int idRun )
{
  m_runId = idRun;
  m_MaxNumberOfIterations = nbIteration;
  
#ifdef DEBUG_STATUS_SHORTCUT
	double costBeforeDeformation = cost();
#endif
	
  m_OptimCost.clear();
  m_GainCost.clear();
	m_GainOfIterations.clear();
	m_convergence_rate.clear();
	m_convergence_trajs.clear();
  
	// Fix the step
  if( m_useAutoStep )
  {
    m_step = getParamMax() / PlanEnv->getDouble(PlanParam::MaxFactor) ;
  }
	
  timeval tim;
	gettimeofday(&tim, NULL);
  double ts = tim.tv_sec+(tim.tv_usec/1000000.0);
  m_time = 0.0; 
  
  if(PlanEnv->getBool(PlanParam::trajSaveCost))
    storeCostAndGain( cost(), cost() );
  
	for (int m_Iteration=0; !checkStopConditions(m_Iteration); m_Iteration++)
	{
		if ( this->getCourbe().size() == 1 ) 
		{
#ifdef DEBUG_STATUS_SHORTCUT
      cout << "Smooting has stoped, only one localpath in traj" << endl;
#endif
			break;
		}
		
		double CurrCost = cost();
	
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
		
		double NewCost = cost();
		
		m_IterationSucceded = ( CurrCost > NewCost );
		
		if( m_IterationSucceded && PlanEnv->getBool(PlanParam::trajSaveCost))
		{
      gettimeofday(&tim, NULL);
      m_time = tim.tv_sec+(tim.tv_usec/1000000.0) - ts;
      
			storeCostAndGain( NewCost, CurrCost );
		}
    
    gettimeofday(&tim, NULL);
    m_time = tim.tv_sec+(tim.tv_usec/1000000.0) - ts;
	}
	
#ifdef DEBUG_STATUS_SHORTCUT
  if( PlanEnv->getBool(PlanParam::trajComputeCollision ) )
  {
    if ( isValid() )
      cout << "Trajectory valid" << endl;
    else 
      cout << "Trajectory not valid" << endl;
  }
#endif
  
  if(PlanEnv->getBool(PlanParam::trajSaveCost))
  {
    ostringstream oss;
    oss << "ShortCutOptim_"<< m_ContextName << "_" << idRun << "_" ;
    computeStats();
    saveOptimToFile( oss.str() );
  }
#ifdef DEBUG_STATUS_SHORTCUT
  cout << "Before : cost = " << costBeforeDeformation << endl;
  cout << "After : cost = " << this->costNoRecompute() << endl;
#endif
}

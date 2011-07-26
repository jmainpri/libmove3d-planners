/*
 * CostOptimization.cpp
 *
 *  Created on: Jun 25, 2009
 *      Author: jmainpri
 */

#include "costOptimization.hpp"

#include "planEnvironment.hpp"

#include <algorithm>

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"

#include "API/Grids/gridsAPI.hpp"
#include "API/scene.hpp"

const bool show_debug_terminal = false;


extern void* GroundCostObj;

using namespace std;
using namespace tr1;

using namespace API;

CostOptimization::CostOptimization() :
m_cheat(false),
m_mincost(numeric_limits<double>::max()),
m_nbErrors(0),
m_DeformBiased(true)
{
}

CostOptimization::CostOptimization(const Trajectory& T) :
Smoothing(T),
m_cheat(false),
m_mincost(numeric_limits<double>::max()),
m_nbErrors(0),
m_DeformBiased(true)
{
}

CostOptimization::CostOptimization(Robot* R, p3d_traj* t) :
Smoothing(R, t),
m_cheat(false),
m_mincost(numeric_limits<double>::max()),
m_nbErrors(0),
m_DeformBiased(true)
{
}

CostOptimization::~CostOptimization()
{
	
	//	cout << "Delete CostOptimization" << endl;
	
}

bool CostOptimization::oneLoopDeform()
{
	double lPrev = 0., lCurrent = 0., lNext = 0.;
	
	bool isOptimSuccess(false);
	m_inCollision=false;
	
	double TrajCost = this->cost();
	//	cout << "nloc = " << getNbOfPaths() << endl;
	//	cout << "TrajCost = " << TrajCost << endl;
	
	//Get 3 configurations at random along the trajectory
	vector<shared_ptr<Configuration> >  vectConf = 
	get3RandSuccesConfAlongTraj(lPrev,lCurrent,lNext,m_step);
	
	//	vectConf = getClosestConfOnTraj(lPrev, lCurrent, lNext, qRandPt, step);
	
	shared_ptr<Configuration> qPrevPt = vectConf.at(0);
	shared_ptr<Configuration> qCurrPt = vectConf.at(1);
	shared_ptr<Configuration> qNextPt = vectConf.at(2);
	
	// Take a configuration in a certain direction
	//	 LocalPath path(qCurrentPt,getRobot()->shoot());
	
	shared_ptr<Configuration> qRandPt = getRobot()->shoot();
	
	//	Eigen::VectorXd dir = qRandPt->getEigenVector();
	//	Eigen::VectorXd cur = qCurrPt->getEigenVector();
	//	
	//	qRandPt->setFromEigenVector( (step/2)*( dir.normalized() + cur.normalized() ) );
	
	// Direction path for perturbation
	LocalPath path( qCurrPt, qRandPt );
	
	/*qNewPt = path.configAtParam( step/4  );
	 
	 if( !qNewPt->setConstraintsWithSideEffect() )
	 {
	 return isOptimSuccess;	
	 }*/
	shared_ptr<Configuration> qNewPt = perturbCurrent(qCurrPt,qRandPt);
	
	if ( qNewPt->getConfigStruct() == NULL )
	{
		return false;
	}
	
	//	cout << "Param max : "	<< path.getParamMax() << endl;
	//	cout << "Dist max : "		<< qCurrPt->dist( *qRandPt ) << endl;
	//	cout << "-_-_-_-_-_-_-_-_-_-_-_-_" << endl;
	//	cout << "old Step : "		<< step << endl;
	//	cout << "new Step : "		<< divFactor*step << endl;
	//	cout << "Dist : "				<< qCurrPt->dist( *qNewPt ) << endl;
	
	//	qCurrentPt->print();
	//	qNewPt->print();
	
	// Set the middle config in the configuration vector
	vectConf[1] = qNewPt;
	
	// If qNew is free then
	// Check the triangle localPath
	if ( !qNewPt->isInCollision() )
	{
		LocalPath* FirstHalf	= new LocalPath(qPrevPt, qNewPt);
		LocalPath* SecondHalf = new LocalPath(qNewPt,  qNextPt);
		
		bool supposedValid = true;
		// If the path is valid
		// Check for cost
		if ( !ENV.getBool(Env::costBeforeColl) )
		{
			supposedValid = FirstHalf->isValid() && SecondHalf->isValid();
		}
		
		if ( supposedValid )
		{
			vector<LocalPath*> paths;
			
			//			double sumOfCost = FirstHalf->cost() + SecondHalf->cost();
			//			double costOfPortion = this->extractCostPortion(lPrev,lNext);
			
			/**
			 * Computes the subportion of the CURRENT trajectory
			 */
			unsigned int first,last;
			
			vector< shared_ptr<Configuration> >  confs =
			getTowConfigurationAtParam(lPrev,lNext,first,last);
			
			for(unsigned int i=first;i<=last;i++)
			{
				paths.push_back(getLocalPathPtrAt(i));
			}
			
			//			cout << "costOfPortion =>"<< endl;
			double costOfPortion = computeSubPortionCost(paths);
			
			if( !(*confs[0] == *qPrevPt) || !(*confs[1] == *qNextPt) )
			{
				cout << "Error in oneLoopDeform" << endl;
			}
			
			paths.clear();
			
			/**
			 * Computes the NEW subportion cost
			 */
			paths.push_back(FirstHalf);
			paths.push_back(SecondHalf);
			
			LocalPath* LP1 = new LocalPath(getLocalPathPtrAt(first)->getBegin(),qPrevPt);
			if(LP1->getParamMax()>0)
			{
				paths.insert(paths.begin(),LP1);
			}
			
			
			LocalPath* LP2 = new LocalPath(qNextPt,getLocalPathPtrAt(last)->getEnd());
			if(LP2->getParamMax()>0)
			{
				paths.push_back(LP2);
			}
			
			double sumOfCost = computeSubPortionCost(paths);
			
			delete LP1;
			delete LP2;
			
			/**
			 * Replace
			 */
			bool lowerCost = ( sumOfCost < costOfPortion );
			
			if(ENV.getBool(Env::costBeforeColl))
			{
				supposedValid = ( FirstHalf->isValid() && SecondHalf->isValid() );
			}
			if ( lowerCost && supposedValid )
			{
				vector<LocalPath*> newPortion;
				newPortion.push_back(FirstHalf);
				newPortion.push_back(SecondHalf);
				replacePortion(lPrev, lNext, newPortion);
				
				if (ENV.getBool(Env::debugCostOptim))
				{
					/**
					 * False optim
					 */
					double newTrajCost = cost();
					
					double delta1 = costOfPortion - sumOfCost;
					double delta2 = TrajCost - newTrajCost;
					
					//	cout << "nloc = " << getNbOfPaths() << endl;
					//	cout << "newTrajCost = " << newTrajCost << endl;
					
					if( TrajCost < newTrajCost )
					{
						cout << "Delta = delta2 - delta1 = " << delta2 - delta1 << endl;
						cout << "Error (TrajCost < newTrajCost)" << endl;
					}
				}
				
				setSortedIndex();
				
				/**
				 * Begin and End validity
				 */
				if (! (*getBegin() == *configAtParam(0)) )
				{
					cout << "------------------------------------------" << endl;
					cout << "Error in oneLoopDeform : !getBegin()->equal(*configAtParam(0))" << endl;
				}
				
				if (! (*getEnd() == *configAtParam(getRangeMax())) )
				{
					cout << "------------------------------------------" << endl;
					cout << "Error in oneLoopDeform : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
					getEnd()->print();
					configAtParam(getRangeMax())->print();
				}
				
				isOptimSuccess = true;
			}
			if (ENV.getBool(Env::debugCostOptim))
			{
				if (isOptimSuccess)
				{
					debugShowTraj(lPrev, lNext, qNewPt, 1);
					// ENV.setVector( Env::costAlongTraj, getCostAlongTrajectory(100) );
				}
				else
				{
					debugShowTraj(lPrev, lNext, qNewPt, 2);
				}
			}
			
		}
		else
		{
			m_inCollision=true;
			if (ENV.getBool(Env::debugCostOptim))
			{
				debugShowTraj(lPrev, lNext, qNewPt, 3);
			}
		}
		
		if (!isOptimSuccess)
		{
			delete FirstHalf;
			delete SecondHalf;
		}
	}
	else
	{
		m_inCollision=true;
	}
	
	return isOptimSuccess;
}

bool CostOptimization::oneLoopDeformRecompute(double step)
{
	double lPrev = 0., lCurrent = 0., lNext = 0.;
	
	step *= p3d_get_env_dmax();
	
	bool isOptimSuccess(false);
	m_inCollision=false;
	
	// double TrajCost = cost();
	//	cout << "nloc = " << getNbOfPaths() << endl;
	//	cout << "TrajCost = " << TrajCost << endl;
	
	//Get 3 configurations at random along the trajectory
	vector<shared_ptr<Configuration> > vectConf;
	shared_ptr<Configuration> qRandPtr = getRobot()->shootDir();
	//	shared_ptr<Configuration> qRandPtr = getRobot()->shoot();
	
	
	vectConf = get3RandSuccesConfAlongTraj(lPrev,lCurrent,lNext,step);
	//	vectConf = getClosestConfOnTraj(lPrev, lCurrent, lNext, qRandPtr, step);
	
	shared_ptr<Configuration> qPrevPt = vectConf.at(0);
	shared_ptr<Configuration> qCurrentPt = vectConf.at(1);
	shared_ptr<Configuration> qNextPt = vectConf.at(2);
	
	// Take a configuration in a certain direction
	//	 LocalPath path(qCurrentPt,getRobot()->shoot());
	//	qRandPtr->print();
	qRandPtr = *qRandPtr + *qCurrentPt;
	
	//	qRandPtr = cheat();
	
	if(m_cheat)
	{
		cout << "qRandPtr = cheat() " << endl;
		qRandPtr = cheat();
		m_cheat = false;
	}
	
	LocalPath path(qCurrentPt, qRandPtr);
	
	shared_ptr<Configuration> qNewPt;
	int maxDiv=0;
	do{
		qNewPt = path.configAtParam( ((double)ENV.getInt(Env::heightFactor))*step/4  );
		step /= 2;
		maxDiv++;
	}
	while( qNewPt->isOutOfBounds() && maxDiv<10);
	
	if(maxDiv==10)
	{
		cout << "Error in oneLoopDeform: Max Division" << endl;
		return false;
	}
	
	vectConf.at(1) = qNewPt;
	
	// If qNew is free then
	// Check the triangle localPath
	if (!qNewPt->isInCollision())
	{
		LocalPath* FirstHalf = new LocalPath(qPrevPt, qNewPt);
		LocalPath* SecondHalf = new LocalPath(qNewPt, qNextPt);
		
		// If the path is valid
		// Check for cost
		if (FirstHalf->isValid() && SecondHalf->isValid())
		{
			Trajectory newTraj(*this);
			
			vector<LocalPath*> newPortion;
			
			newPortion.push_back(FirstHalf);
			newPortion.push_back(SecondHalf);
			
			newTraj.replacePortion(lPrev, lNext, newPortion);
			
			if ( newTraj.cost() < this->cost() )
			{
				newPortion.clear();
				newPortion.push_back(new LocalPath(*FirstHalf));
				newPortion.push_back(new LocalPath(*SecondHalf));
				
				this->replacePortion(lPrev, lNext, newPortion);
				
				setSortedIndex();
				
				if (! (*getBegin() == *configAtParam(0)) )
				{
					cout << "Error in oneLoopDeform : !getBegin()->equal(*configAtParam(0))" << endl;
				}
				
				if (! (*getEnd() == *configAtParam(getRangeMax())) )
				{
					cout << "------------------------------------------" << endl;
					cout << "Error in oneLoopDeform : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
					getEnd()->print();
					configAtParam(getRangeMax())->print();
				}
				
				isOptimSuccess = true;
				
			}
			if (ENV.getBool(Env::debugCostOptim))
			{
				if (isOptimSuccess)
				{
					debugShowTraj(lPrev, lNext, qNewPt, 1);
				}
				else
				{
					debugShowTraj(lPrev, lNext, qNewPt, 2);
				}
			}
			
		}
		else
		{
			m_inCollision=true;
			if (ENV.getBool(Env::debugCostOptim))
			{
				debugShowTraj(lPrev, lNext, qNewPt, 3);
			}
		}
	}
	else
	{
		m_inCollision=true;
	}
	
	return isOptimSuccess;
}

/*!
 * Perturb the current configuration
 */
shared_ptr<Configuration> CostOptimization::perturbCurrent(shared_ptr<Configuration> qCurrPt,
																													 shared_ptr<Configuration> qRandPt)
{
	LocalPath path(qCurrPt,qRandPt);
	
	shared_ptr<Configuration> qNewPt;
	
	// Dividing the step n times to stay in the bounds
	const unsigned int	max_div = 4;
	unsigned int				ith_div = 0;
	double							divFactor = 1/PlanEnv->getDouble(PlanParam::MinStep); 
	bool								QIsOutOfBounds = true;
	
	for (ith_div=0; ith_div<max_div && QIsOutOfBounds; ith_div++) 
	{
		if( !m_descent ) 
		{
			qNewPt = path.configAtParam( divFactor*m_step  );
			QIsOutOfBounds = qNewPt->isOutOfBounds();
      divFactor /= 2;
		}
		else 
		{
			qNewPt = path.configAtParam( getLastDescendingConfParam(path) );
			QIsOutOfBounds = false;
		}
	}
  
//  cout << "path.getParamMax() = " << path.getParamMax() << endl;
//  cout << "divFactor = " << divFactor << endl;
//  cout << "m_step = " << m_step << endl;
	
	//qNewPt->print();
	
	return qNewPt;
	
//	if ( ith_div == max_div ) 
//	{
//		return false;
//	}
//	
//	return true;
}

/*!
 * Compute descent on cost map
 */
double CostOptimization::getLastDescendingConfParam(LocalPath& directionPath)
{
	bool failed(false);
	bool upHill(false);
	
	shared_ptr<Configuration> fromConfig = directionPath.getBegin();
	
	double praramAlongDirection( m_Robot->getActiveScene()->getDMax() );
	
	double extensionCost(0.);
	double prevCost(0.);
	
	// Perform extension toward directionConfig
	// Additional nodes creation in the nExtend case, but without 
	// checking for expansion control
	while ((!failed) && ( praramAlongDirection < directionPath.getParamMax() ) )
	{
		// Takes one configuration along the path
		// Tests if the new small portion is valid
		shared_ptr<Configuration> toConfig = directionPath.configAtParam( praramAlongDirection );
		LocalPath extensionLP( fromConfig , toConfig );
		failed = (!extensionLP.isValid()); 
		//nbOfExtend++;
		
		// Get the cost of the extremal configuration
		prevCost						= fromConfig->cost();
		extensionCost				= toConfig->cost();
		
		// Cost Test Control
		if ((!failed) && ( prevCost < extensionCost ))
		{
			upHill = true;
			failed = true;
		}
		
		// Check if it's going down hill for next iterations
		if ( !upHill ) 
		{
			praramAlongDirection += extensionLP.getParamMax();
			fromConfig = toConfig;
		}
	}
	
	return praramAlongDirection;
}

/*!
 * Bias to one specific configuration
 */
shared_ptr<Configuration> CostOptimization::cheat()
{
	double q[26];
	
	q[0] = 0.000000;
	q[1] = 0.000000;
	q[2] = 0.000000;
	q[3] = 0.000000;
	q[4] = 0.000000;
	q[5] = 0.000000;
	q[6] = 96.863319;
	q[7] = -13.028515;
	q[8] = 13.028515;
	q[9] = 0.000000;
	q[10] = 9.292039;
	q[11] = 9.655849;
	q[12] = -15.000000;
	q[13] = -46.000000;
	q[14] = -8.000000;
	q[15] = 119.000000;
	q[16] = 138.000000;
	q[17] = 62.000000;
	q[18] = 29.000000;
	q[19] = -1.671586;
	q[20] = -90.855453;
	q[21] = -56.833824;
	q[22] = 116.814163;
	q[23] = 30.000000;
	q[24] = 14.000000;
	q[25] = 0.000000;
	
	shared_ptr<Configuration> ptrConfig(new Configuration(this->getRobot(),q));
	ptrConfig->convertToRadian();
	return ptrConfig;
}
void CostOptimization::debugShowTraj(double lPrev, double lNext, shared_ptr<
                                     Configuration> qNew, int color)
{
	
	vector< shared_ptr<Configuration> > vectConf(3);
	vectConf.at(0) = configAtParam(lPrev);
	vectConf.at(1) = qNew;
	vectConf.at(2) = configAtParam(lNext);
	
	trajToDraw.resize(4);
	
	trajToDraw.at(0) = extractSubTrajectory(0, lPrev);
	trajToDraw.at(1) = extractSubTrajectory(lPrev, lNext);
	trajToDraw.at(2) = *new Trajectory(vectConf);
	trajToDraw.at(3) = extractSubTrajectory(lNext, getRangeMax());
	
	int color_base_traj;
	
	if(GroundCostObj == NULL)
	{
		color_base_traj = 0;
	}
	else
	{
		color_base_traj = 3;
	}
	
	trajToDraw.at(0).setColor(color_base_traj);
	trajToDraw.at(1).setColor(2);
	trajToDraw.at(2).setColor(color);
	trajToDraw.at(3).setColor(color_base_traj);
	
	//			basicTraj.print();
	//			triangleTraj.print();
	g3d_draw_allwin_active();
	usleep(200000);
	
	vector<LocalPath*> pathsTmp(2);
	pathsTmp.at(0) = (new LocalPath(vectConf.at(0), vectConf.at(1)));
	pathsTmp.at(1) = (new LocalPath(vectConf.at(1), vectConf.at(2)));
	
	Trajectory tmpT(*this);
	tmpT.replacePortion(lPrev, lNext, pathsTmp);
	double newCost = tmpT.cost();
	
	if (m_mincost > newCost)
	{
		m_mincost = newCost;
		
	}
	
	double oldCost = cost();
	double sumOfCost = pathsTmp.at(0)->cost() + pathsTmp.at(1)->cost();
	double costOfPortion1 = this->costOfPortion(lPrev, lNext);
	double costOfPortion2 = this->extractCostPortion(lPrev,lNext);
	
	if( show_debug_terminal )
	{
		cout << "Difference between costOfPortion and extractCostPortion : "
		<< (costOfPortion1 - costOfPortion2) << endl;
		
		cout << "Difference on the portion : " << (costOfPortion2 - sumOfCost) << endl;
		cout << "Difference on the trajectory: " << (oldCost - newCost) << endl;
		cout << "---------------------------------------------------------" << endl;
	}
	
	double diff = fabs((costOfPortion2 - sumOfCost) - (oldCost - newCost));
	
	if (diff > 0.001)
	{
		m_Errors.push_back(diff);
		m_nbErrors++;
	}
	
}

void CostOptimization::printDebugInfo()
{
	cout << "Errors : " << endl;
	for (uint i = 0; i < m_Errors.size(); i++)
	{
		cout << "Errors[" << i << "] = " << m_Errors.at(i) << endl;
	}
	
	if (m_Errors.size() > 2)
	{
		double max = *max_element(m_Errors.begin(), m_Errors.end());
		double min = *min_element(m_Errors.begin(), m_Errors.end());
		cout << "Error Max. =  " << max << endl;
		cout << "Error Min. =  " << min << endl;
	}
	
	cout << "Selected : " << endl;
	
	for (uint i = 0; i < m_Selected.size(); i++)
	{
		cout << "mSelected[" << i << "] = " << m_Selected.at(i) << endl;
	}
	//	cout << "meanSelected =" <<  meanSelected / mSelected.size() << endl;
	
	cout << "nbBiased = " << m_nbBiased << endl;
	cout << "nbReallyBiased = " << m_nbReallyBiased << endl;
}

int nb_runs = 0;


vector<shared_ptr<Configuration> > CostOptimization::getClosestConfOnTraj(
																																					double& prevDistPt, double& randDistPt, double& nextDistPt, shared_ptr<
																																					Configuration> ptrConf, double step)
{
	const int N = 30;
	double delta = this->getRangeMax() / (double)N;
	
	// Gets 30 configurations at a step delta
	vector<shared_ptr<Configuration> > vectConf = getNConfAtParam(delta);
	vector<shared_ptr<Configuration> > threeConfVect;
	
	// Returns directly the vector if the path is smaller
	// than 3 deltas
	if (vectConf.size() < 2 )
	{
		if (vectConf.size() == 1)
		{
			threeConfVect.push_back(vectConf.at(0));
		}
		else if (vectConf.size() != 0 ) 
		{
			throw string("Error in CostOptimization::getClosestConfOnTraj");
		}
		return threeConfVect;
	}
	
	// Compute closest configuration on the trajectory
	double minDist = ptrConf->dist(*vectConf.at(0), GENERAL_CSPACE_DIST);
	unsigned int id = 0;
	
	for (unsigned int i=1; i < vectConf.size(); i++)
	{
		double dist = ptrConf->dist(*vectConf.at(i), GENERAL_CSPACE_DIST);
		
		if (dist < minDist)
		{
			minDist = dist;
			id = i;
		}
	}
	
	// Watch out for extreme values on the trajectory
	threeConfVect.resize(3);
	
	if (id == 0)
	{
		id = id + 1;
	}
	if (id == vectConf.size() - 1)
	{
		id = id - 1;
	}
	
	// Compute the id of the 3 configurations in the vector
	randDistPt = delta * (id);
	
	int nbOfDelta = (int) ((step / delta) + 0.5);
	
	int idPrev = id - nbOfDelta;
	if (idPrev < 0)
		idPrev = 0;
	
	int idNext = id + nbOfDelta;
	if (idNext > (int) vectConf.size() - 1)
		idNext = vectConf.size() - 1;
	
	step = ((double) nbOfDelta) * delta;
	
	prevDistPt = randDistPt - step;
	if (prevDistPt < 0)
		prevDistPt = 0;
	
	nextDistPt = randDistPt + step;
	if (nextDistPt > getRangeMax())
		nextDistPt = getRangeMax();
	
	if (ENV.getBool(Env::debugCostOptim))
	{
		//put the parameter of the 3 configurations on the
		cout << "nbOfDelta = " << nbOfDelta << endl;
		cout << "step = " << step << endl;
		cout << "getRangeMax() = " << getRangeMax() << endl;
		cout << "prevDistPt = " << prevDistPt << endl;
		cout << "randDistPt = " << randDistPt << endl;
		cout << "nextDistPt = " << nextDistPt << endl;
	}
	
	threeConfVect.at(0) = vectConf.at(idPrev);
	threeConfVect.at(1) = vectConf.at(id);
	threeConfVect.at(2) = vectConf.at(idNext);
	
	return threeConfVect;
}

vector<shared_ptr<Configuration> > CostOptimization::get3RandSuccesConfAlongTraj(
																																								 double& prevDist, double& randDist, double& nextDist, double step)
{
	vector< shared_ptr<Configuration> > vectConf(3);
	
	// Computes the distances along the 
	// trajectories from which to select the configurations
	if (m_DeformBiased)
	{
		randDist = getBiasedParamOnTraj();
	}
	else
	{
		randDist = p3d_random(0, getRangeMax());
	}
	
	prevDist = MAX(0, randDist - step/2 );
	nextDist = MIN(getRangeMax(), randDist + step/2);
	
	//	cout << "step : " << step << endl;
	//	cout << "nextDist - prevDist : " << nextDist - prevDist << endl;
	//	cout << "getRangeMax() : " << getRangeMax() << endl;
	
	// Gets the 3 configurations along the trajectory
	vectConf.at(0) = configAtParam(prevDist);
	vectConf.at(1) = configAtParam(randDist);
	vectConf.at(2) = configAtParam(nextDist);
	
	if (prevDist > nextDist)
	{
		cerr << "Error in get3RandSuccesConfAlongTraj" << endl;
	}
	
	return vectConf;
}

void CostOptimization::runDeformation(int nbIteration, int idRun )
{
	//cout << "Before Deform : Traj cost = " << this->cost() << endl;
	double costBeforeDeformation = this->cost();
	double initalRange = getRangeMax();
	
	m_GainOfIterations.clear();
	m_MaxNumberOfIterations = nbIteration;
	
	if (PointsToDraw != 0) {
		delete PointsToDraw;
	}
	
	PointsToDraw = new PointCloud;
	
	if (PlanEnv->getBool(PlanParam::withDescent)) 
	{
		m_descent = true;
	}
	else
	{
		m_descent = false;
	}
	
	double ts(0.0); m_time = 0.0; ChronoOn();
	
	int ith_deformation;
	for ( ith_deformation = 0; 
			 !checkStopConditions(ith_deformation); ith_deformation++)
	{
    m_step = initalRange/PlanEnv->getDouble(PlanParam::MaxFactor);
//    cout << "m_step = " << m_step << endl;
    double CurCost = cost();
		
		oneLoopDeform();
		
		double NewCost = cost();
		
		m_IterationSucceded = ( CurCost > NewCost );
		
		if(PlanEnv->getBool(PlanParam::saveTrajCost))
		{
			NewCost = cost();
			
			if ( NewCost > CurCost )
			{
				cout << "CostOptimization::runDeformation : NewCost > CurrentCost"  << endl;
			}
			m_OptimCost.push_back( NewCost );
		}
		
		if ( m_IterationSucceded ) 
		{
			double Gain = (( CurCost - NewCost ) / CurCost) ;
			cout << "Gain = " << 100*Gain << " %" << endl;
			m_GainOfIterations.push_back( Gain );
		}
		
		ChronoTimes( &m_time , &ts );
	}
	
	ChronoOff();
	
	if ( isValid() )
	{
		cout << "Trajectory valid" << endl;
	}
	else
	{
		cout << "Trajectory not valid" << endl;
	}
	
	if(PlanEnv->getBool(PlanParam::saveTrajCost))
	{
		ostringstream oss;
		oss << "CostOptim_"<< idRun << "_" ;
		this->saveOptimToFile(oss.str());
	}
	cout << "Before : Traj cost = " << costBeforeDeformation << endl;
	this->resetCostComputed();
	cout << "After (" << ith_deformation << ") Deform : Traj cost = " << this->cost() << endl;
}

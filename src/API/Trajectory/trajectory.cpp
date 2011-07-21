/*
 * trajectory.cpp
 *
 *  Created on: Jun 17, 2009
 *      Author: jmainpri
 */

#include "API/Trajectory/trajectory.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Localpath-pkg.h"
#include "GroundHeight-pkg.h"
#include "Planner-pkg.h"

#include "cost_space.hpp"

#if defined( HRI_COSTSPACE ) && defined ( HRI_PLANNER )
#include "HRI_costspace/HRICS_HAMP.hpp"
#endif

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

using namespace API;

std::vector<Trajectory> trajToDraw;

Trajectory::Trajectory() :
m_Robot(NULL),
HighestCostId(0),
isHighestCostIdSet(false),
name(""),
file(""),
nloc(0),
mColor(0),
range_param(0),
m_Source(new Configuration(m_Robot,NULL))
{
  
}

Trajectory::Trajectory(Robot* R) :
m_Robot(R),
HighestCostId(0),
isHighestCostIdSet(false),
name(""),
file(""),
nloc(0),
mColor(0),
range_param(0),
m_Source(new Configuration(m_Robot,NULL))
{
  
}

Trajectory::Trajectory(const Trajectory& T) :
m_Robot(T.m_Robot), 
HighestCostId(T.HighestCostId), 
isHighestCostIdSet(T.isHighestCostIdSet),
name(T.name), 
file(T.file),
nloc(T.nloc), 
mColor(T.mColor),
range_param(T.range_param), 
m_Source(T.m_Source), 
m_Target(T.m_Target)
{
	
	for (uint i = 0; i < nloc; i++)
	{
		m_Courbe.push_back(new LocalPath(*(T.m_Courbe.at(i))));
	}
	//	cout << "Copy Trajectory" << endl;
}

Trajectory& Trajectory::operator=(const Trajectory& t)
{
	
	if (nloc > 0)
	{
		for (uint i = 0; i < nloc; i++)
		{
			delete m_Courbe.at(i);
		}
	}
	
	m_Courbe.clear();
	
	// TODO Name of file and robot
	name = t.name;
	file = t.file;
	
	m_Robot = t.m_Robot;
	
	/* Number of localpath */
	nloc = t.nloc;
	range_param = t.range_param;
	
	for (uint i = 0; i < nloc; i++)
	{
		m_Courbe.push_back(new LocalPath(*(t.m_Courbe.at(i))));
	}
	
	//	cout << "Copy in operator= Trajtecory" << endl;
	
	m_Source = t.m_Source;
	m_Target = t.m_Target;
	mColor = t.mColor;
	HighestCostId = t.HighestCostId;
	isHighestCostIdSet = t.isHighestCostIdSet;
	
	return *this;
}

Trajectory::Trajectory( vector< shared_ptr<Configuration> >& configs) :
HighestCostId(0), isHighestCostIdSet(false)
{
	if ( !configs.empty() )
	{
		name = "";
		file = "";
		
		nloc = configs.size()-1; /* Number of localpath */
		m_Robot	= configs[0]->getRobot();
		
		m_Source = configs[0];
		m_Target = configs[nloc];
		
		range_param = 0;
		m_Courbe.clear();
		
		for (unsigned int i = 0; i < nloc; i++)
		{
			if ( !configs[i]->equal( *configs[i+1]) ) 
			{
				LocalPath* path = new LocalPath( configs[i], configs[i+1] );
				range_param += path->getParamMax();
				m_Courbe.push_back(path);
			}
			else 
			{
				cout << "two configuration are the same in traj constructor" << endl;
			}
		}
		
		nloc = m_Courbe.size();
	}
	else
	{
		cout
		<< "Warning: Constructing a class out of empty vector of configuration"
		<< endl;
	}
	mColor = 1;
}

Trajectory::Trajectory(Robot* R, p3d_traj* t)
{
	if ((t == NULL) || (t->courbePt==NULL))
	{
		return;
	}
	
	// TODO Name and file (string based)
	m_Robot = R;
	
	nloc = t->nlp;
	
	p3d_localpath* localpathPt = t->courbePt;
	
	while (localpathPt != NULL)
	{
		LocalPath* path = new LocalPath(m_Robot, localpathPt);
		//			path->getBegin()->print();
		//			path->getEnd()->print();
		m_Courbe.push_back(path);
		localpathPt = localpathPt->next_lp;
	}
	
#ifdef P3D_PLANNER
	range_param = p3d_compute_traj_rangeparam(t);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
	m_Source = shared_ptr<Configuration> (new Configuration(m_Robot,
																													p3d_config_at_param_along_traj(t, 0)));
	
	m_Source->setConstraints();
	
	//	cout << "m_Source:" << endl;
	//	m_Source->print();
	
	m_Target = shared_ptr<Configuration> (new Configuration(m_Robot,
																													p3d_config_at_param_along_traj(t, range_param)));
	
	m_Target->setConstraints();
	//	cout << "m_Target:" << endl;
	//	m_Target->print();
	
	updateRange();
	//	cout << "range_param = " << range_param << endl;
	mColor = 0;
	
	if (!getBegin()->equal(*configAtParam(0)))
	{
		cout << "Error in constructor : !getBegin()->equal(*configAtParam(0))" << endl;
	}
	
	if (!getEnd()->equal(*configAtParam(getRangeMax())))
	{
		cout << "------------------------------------------" << endl;
		cout << "Error in constructor : !getEnd()->equal(*configAtParam(getRangeMax()))" << endl;
		getEnd()->print();
		configAtParam(getRangeMax())->print();
	}
}

Trajectory::~Trajectory()
{
	
	//	cout << "Delete Trajectory ("<<nloc<<")" << endl;
	
	for (uint i = 0; i < nloc; i++)
	{
		//		cout<<"delete "<<i<<endl;
		delete m_Courbe.at(i);
	}
}

void Trajectory::replaceP3dTraj()
{
  //cout << "Robot name : " << m_Robot->getRobotStruct()->name << endl;
	replaceP3dTraj(p3d_get_robot_by_name(m_Robot->getRobotStruct()->name)->tcur);
}

p3d_traj* Trajectory::replaceP3dTraj(p3d_traj* trajPt)
{
	//	print();
	
	if(trajPt!=NULL)
	{
		if(strcmp(trajPt->rob->name,m_Robot->getRobotStruct()->name) != 0 )
		{
			cout << " Warning : Robot not the same as the robot in traj "  << endl;
			return NULL;
		}
		destroy_list_localpath(m_Robot->getRobotStruct(), trajPt->courbePt);
	}
	else
	{
		trajPt = p3d_create_empty_trajectory(m_Robot->getRobotStruct());
	}
	
	//	trajPt->name       = strdup(name);
	//	trajPt->file       = NULL;  // Modification Fabien
	trajPt->num = 0; //m_Robot->getRobotStruct()->nt;
	//    trajPt->rob = m_Robot->getRobotStruct();
	
	//	cout << m_Robot->getRobotStruct() << endl;
	
	nloc = 0;
	
	p3d_localpath *localpathPt = NULL;
	p3d_localpath *localprevPt = NULL;
  
  bool first = true;
	
	for (unsigned int i = 0; i < m_Courbe.size(); i++)
	{
//    if ( *m_Courbe[i]->getBegin() ==  *m_Courbe[i]->getEnd() )
//    {
//      cout << "null LocalPath in replaceP3dTraj" << endl;
//      continue;
//    }
    
		localprevPt = localpathPt;
		localpathPt = m_Courbe[i]->getLocalpathStruct()->copy( m_Robot->getRobotStruct(), 
                                                           m_Courbe[i]->getLocalpathStruct() );

    if( localprevPt )
    {
      localprevPt->next_lp = localpathPt;
    }
    
		localpathPt->prev_lp = localprevPt;
    
    if ( first ) 
    {
      trajPt->courbePt = localpathPt;
      first = false;
    }
    
    nloc++;
	}
	
  if (nloc != 0) 
  {
    localpathPt->next_lp = NULL;
  }
	else 
  {
    cout << "replaceP3dTraj with empty trajectory" << endl;
  }

	trajPt->nlp = nloc;
	
#ifdef P3D_PLANNER
	trajPt->range_param = p3d_compute_traj_rangeparam(trajPt);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
  
  return trajPt;
		//	print()	
}

void Trajectory::copyPaths( vector<LocalPath*>& vect )
{
	vector<LocalPath*>::iterator it;
	
	for ( it = vect.begin() ; it != vect.end() ; it++ ) 
	{
		*it = new LocalPath(**it);
	}
}

shared_ptr<Configuration> Trajectory::configAtParam(double param)
{
	double soFar(0.0);
	double prevSoFar(0.0);
	
	for (uint i = 0; i < nloc; i++)
	{
		
		soFar = soFar + m_Courbe.at(i)->getParamMax();
		
		// Parameter lies in the previous local path
		// return configuration inside previous LP
		if (param < soFar)
		{
			
			if (param < prevSoFar)
			{
				cout
				<< "Error: getting Configuration at parameter in trajectory"
				<< endl;
			}
			
			return m_Courbe.at(i)->configAtParam(param - prevSoFar);
		}
		prevSoFar = soFar;
	}
	return m_Courbe.back()->configAtParam(param);
}

vector< shared_ptr<Configuration> > Trajectory::getVectorOfConfiguration()
{
	vector< shared_ptr<Configuration> > vect;
	
	if (m_Courbe.empty()) {
		return vect;
	}
	
	vect.push_back( m_Courbe[0]->getBegin() );
	
	for (unsigned int i=0; i<m_Courbe.size(); i++) 
		vect.push_back( m_Courbe[i]->getEnd() );
	
	return vect;
}
vector<shared_ptr<Configuration> > Trajectory::getNConfAtParam(double delta)
{
	vector<shared_ptr<Configuration> > tmpVector(0);
	
	double param = 0;
	
	double soFar(0.0);
	double prevSoFar(0.0);
	
	for (uint i = 0; i < nloc; i++)
	{
		
		soFar = soFar + m_Courbe.at(i)->getParamMax();
		
		// Parameter lies in the previous local path
		// return configuration inside previous LP
		while (param < soFar)
		{
			
			if (param < prevSoFar)
			{
				cout
				<< "Error: getting Configuration at parameter in trajectory"
				<< endl;
			}
			
			tmpVector.push_back(m_Courbe.at(i)->configAtParam(param - prevSoFar));
			param += delta;
		}
		// TODO watch out
		if (i < nloc - 1)
		{
			prevSoFar = soFar;
		}
	}
	
	tmpVector.push_back(m_Courbe.at(nloc - 1)->configAtParam(param - prevSoFar));
	
	return tmpVector;
	
}

LocalPath* Trajectory::getLocalPathPtrAt(unsigned int id) const
{
	return m_Courbe.at(id);
}

int Trajectory::getNbOfPaths() const
{
	return m_Courbe.size();
}

double Trajectory::computeSubPortionRange(vector<LocalPath*> portion)
{
	
	double range(0.0);
	
	for (unsigned int i = 0; i < portion.size(); i++)
	{
		range += portion[i]->getParamMax();
	}
	
	return range;
}

void Trajectory::updateRange()
{
	
	nloc = m_Courbe.size();
	range_param = computeSubPortionRange(m_Courbe);
}

bool Trajectory::isValid()
{
	for (unsigned int i = 0; i < m_Courbe.size(); i++)
	{
		if (!m_Courbe[i]->isValid())
		{
			return false;
		}
		//                cout <<"LocalPath["<<i<<"] = "<< m_Courbe[i]->getNbColTest() << endl;
	}
	
	return true;
}

void Trajectory::resetCostComputed()
{
	for (unsigned int i = 0; i < m_Courbe.size(); i++)
	{
		m_Courbe[i]->resetCostComputed();
	}
}

vector< pair<double,double > > Trajectory::getCostProfile()
{
	vector< pair<double,double> > vectOfCost;
	
	double previousPathParam = 0.0;
	
	for (unsigned int i = 0; i < m_Courbe.size(); i++)
	{
		vector< pair<double,double > > tmp = m_Courbe[i]->getCostProfile();
		
		for (unsigned int j = 0; j < tmp.size(); j++)
		{
			tmp[j].first += previousPathParam;
			vectOfCost.push_back( tmp[j] );
		}
		
		previousPathParam += m_Courbe[i]->getParamMax();
		
		//		vectOfCost.insert( vectOfCost.end() , 
		//											tmp.begin(),
		//											tmp.end());
	}
	
	return vectOfCost;
}

double Trajectory::computeSubPortionCost(vector<LocalPath*>& portion)
{
	double sumCost(0.0);
	double cost(0.0);
	
	for (unsigned int i = 0; i < portion.size(); i++)
	{
		cost = portion[i]->cost();
		//cout << "Cost["<<i<<"] = "<< cost << endl;
		sumCost += cost;
	}
	
	return sumCost;
}

double Trajectory::ReComputeSubPortionCost(vector<LocalPath*>& portion)
{
	double sumCost(0.0);
	double cost(0.0);
	
	for (unsigned int i = 0; i < portion.size(); i++)
	{
		portion[i]->resetCostComputed();
		cost = portion[i]->cost();
		//		cout << "Cost["<<i<<"] = "<< cost << endl;
		sumCost += cost;
	}
	
	return sumCost;
}

double Trajectory::computeSubPortionIntergralCost(vector<LocalPath*>& portion)
{
	double cost(0.0);
	//	double dmax = p3d_get_env_dmax()/2;
	double dmax = 0;
	
	//	cout << "Computing resolution" << endl;
	
	for( unsigned int i=0; i<m_Courbe.size();i++)
	{
		double resol = m_Courbe[i]->getResolution();
		//		cout << "resol["<< i <<"] = "<< resol << endl;
		dmax += resol;
	}
	
	dmax /= (double)m_Courbe.size();
	
	double currentParam(0.0);
	
	double currentCost, prevCost;
	
#ifdef LIGHT_PLANNER
	Vector3d taskPos, prevTaskPos;
	
	prevCost = portion[0]->getBegin()->cost();
	if( ENV.getBool(Env::HRIPlannerWS) )
	{
		prevTaskPos = portion[0]->getBegin()->getTaskPos();
	}
#endif
	
	double range = computeSubPortionRange(portion);
	double distStep = dmax;
	shared_ptr<Configuration> confPtr;
	
	int nbStep =0;
	
	while (currentParam <= range)
	{
		currentParam += dmax;
		confPtr = configAtParam(currentParam);
		currentCost = confPtr->cost();
		
#ifdef LIGHT_PLANNER
		// Case of task space
		if( ENV.getBool(Env::HRIPlannerWS) )
		{
			taskPos = confPtr->getTaskPos();
			distStep = ( taskPos - prevTaskPos ).norm();
			prevTaskPos = taskPos;
		}
#endif
		//the cost associated to a small portion of curve
		double step_cost =
		//p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep);
		global_costSpace->deltaStepCost(prevCost,currentCost,distStep);
		
		cost += step_cost;
		prevCost = currentCost;
		nbStep++;
	}
	cout << "Nb Step = " << nbStep << endl;
	//        cout << "range/p3d_get_env_dmax() = " << range/p3d_get_env_dmax() << endl;
	
	return cost;
}

double Trajectory::computeSubPortionCostVisib( vector<LocalPath*>& portion )
{
	double epsilon = 0.002;
	double cost(0.0);
	double dmax = 0;
	
	bool inVisibilty(false);
	
	for( unsigned int i=0; i<m_Courbe.size();i++)
	{
		double resol = m_Courbe[i]->getResolution();
		dmax += resol;
	}
	
	dmax /= (double)m_Courbe.size();
	
	double currentParam(0.0);
	double prevCost;
	double currentCost = portion[0]->getBegin()->cost();
	double range = computeSubPortionRange(portion);
	
	int jnt_id=0;
	
#if defined( HRI_COSTSPACE ) && defined ( HRI_PLANNER )
	//jnt_id = hriSpace->getTask();
	cout << "Be carefull not defined" << endl;
#else
	cout << "Error : HRI Planner not compiled nor linked" << endl;
	return 0;
#endif
	
	m_Robot->setAndUpdate(*m_Source);
	Vector3d prevPos;
	Vector3d currentPos = m_Robot->getJointPos(jnt_id);
	
	while (currentParam <= range)
	{
		currentParam += dmax;
		
		shared_ptr<Configuration> currentConf = configAtParam(currentParam);
		
		prevCost = currentCost;
		prevPos = currentPos;
		
		m_Robot->setAndUpdate(*currentConf);
		currentPos = m_Robot->getJointPos(jnt_id);
		double distStep=0;
		for(int k=0;k<currentPos.size();k++)
		{
			distStep += pow((currentPos[k]-prevPos[k]),2);
		}
		distStep = sqrt(distStep);
		
		double step_cost;
		
		if(!inVisibilty)
		{
			currentCost = currentConf->cost();
			//the cost associated to a small portion of curve
			
			step_cost = 
			global_costSpace->deltaStepCost(prevCost, currentCost,distStep);
			//			p3d_ComputeDeltaStepCost(prevCost, currentCost, distStep);
			
			cout << " Step Cost = " << step_cost << endl;
			
			if( currentCost < ENV.getDouble(Env::visThresh) )
			{
				inVisibilty = true;
			}
		}
		else
		{
			step_cost =  epsilon * distStep;
		}
		
		cost += step_cost;
	}
	
	return cost;
}

double Trajectory::cost()
{
	double cost(0.0);
	
	if( p3d_GetDeltaCostChoice() == VISIBILITY )
	{
		cost = computeSubPortionCostVisib(m_Courbe);
	}
	else
	{
		cost = computeSubPortionCost(m_Courbe);
		//            cost =  computeSubPortionIntergralCost(m_Courbe);
		//        cost = ReComputeSubPortionCost(m_Courbe);
		
	}
	return cost;
}

double Trajectory::costNoRecompute()
{
	return computeSubPortionCost(m_Courbe);
}

double Trajectory::costDeltaAlongTraj()
{
	cout << "Sum of LP cost = " << computeSubPortionCost(m_Courbe) << endl;
	Trajectory tmp(*this);
	cout << "Recomputed Sum of LocalPaths = "  << tmp.ReComputeSubPortionCost(tmp.m_Courbe) << endl;
	return computeSubPortionIntergralCost(m_Courbe);
}

vector<shared_ptr<Configuration> > Trajectory::getTowConfigurationAtParam(
																																					double param1, double param2, uint& lp1, uint& lp2)
{
	vector<shared_ptr<Configuration> > conf;
	
	if (param1 < 0)
	{
		cout << "Error: the parameter is out of band" << endl;
	}
	if (param2 > this->range_param)
	{
		cout << "Error: the parameter is out of band" << endl;
	}
	
	if (param1 > param2)
	{
		cout
		<< "Error: not possible to replace trajectory because of the parameters"
		<< endl;
		return conf;
	}
	
	// Looks for the local paths to be changed linear in NLOC
	//-------------------------------------------------------------------
	double soFar(0.0);
	double prevSoFar(0.0);
	
	shared_ptr<Configuration> q1;
	shared_ptr<Configuration> q2;
	
	for (uint i = 0; i < m_Courbe.size(); i++)
	{
		soFar = soFar + m_Courbe.at(i)->getParamMax();
		
		if (param1 < soFar)
		{
			q1 = m_Courbe.at(i)->configAtParam(param1 - prevSoFar);
			//			q1->print();
			//			param_start = param1-prevSoFar;
			lp1 = i;
			break;
		}
		prevSoFar = soFar;
		if (i == (nloc - 1))
		{
			q1 = m_Courbe.at(i)->getEnd();
			lp1 = i;
			cout << "Error : conf ends local path" << endl;
		}
	}
	
	// id_LP_1 is the id of the path of which param1 lies in
	// we start searching for parm2 from here
	soFar = prevSoFar;
	
	for (uint i = lp1; i < m_Courbe.size(); i++)
	{
		soFar = soFar + m_Courbe.at(i)->getParamMax();
		
		if (param2 < soFar)
		{
			q2 = m_Courbe.at(i)->configAtParam(param2 - prevSoFar);
			//			q2->print();
			//			param_end = param2-soFar;
			lp2 = i;
			break;
		}
		prevSoFar = soFar;
		
		if (i == (nloc - 1))
		{
			q2 = m_Courbe.at(nloc - 1)->getEnd();
			lp2 = i;
		}
	}
	
	//        q1->setConstraints();
	//        q2->setConstraints();
	
	conf.push_back(q1);
	conf.push_back(q2);
	return conf;
}

double Trajectory::extractCostPortion(double param1, double param2)
{
	double totalCost(0.0);
	
	vector<shared_ptr<Configuration> > conf;
	
	uint first;
	uint last;
	
	conf = getTowConfigurationAtParam(param1, param2, first, last);
	
	shared_ptr<Configuration> q1 = conf.at(0);
	shared_ptr<Configuration> q2 = conf.at(1);
	
	if (first > last)
	{
		cout
		<< "Error: extractCostPortion: inconsistent query for subpath extraction"
		<< endl;
		return 0;
	}
	
	// Case where they lie in the same path
	//----------------------------------------------------------------------------
	if (first == last)
	{
		if (q1->equal(*q2))
		{
			cout
			<< "Error: extractCostPortion: q1 and q2 are the same in subportion extraction"
			<< endl;
			return 0;
		}
		LocalPath LP(q1, q2);
		return LP.cost();
	}
	
	shared_ptr<Configuration> start = m_Courbe.at(first)->getBegin();
	shared_ptr<Configuration> end = m_Courbe.at(last)->getEnd();
	
	// Adds the modified first local path to subpaths
	// Verifies that the configuration is not starting the local path
	if (!start->equal(*q1))
	{
		if (!m_Courbe.at(first)->getEnd()->equal(*q1))
		{
			LocalPath LP(q1, m_Courbe.at(first)->getEnd());
			
			if (LP.isValid())
			{
				totalCost += LP.cost();
			}
			else
			{
				cout << "Error: extractCostPortion: portion of path not valid"
				<< endl;
				return -1;
			}
		}
	}
	else
	{
		
		if (m_Courbe.at(first)->isValid())
		{
			totalCost += m_Courbe.at(first)->cost();
		}
		else
		{
			cout << "Error: extractCostPortion: portion of path not valid"
			<< endl;
			return -1;
		}
	}
	
	// Adds all the paths between First and Last
	for (uint i = first + 1; i < last; i++)
	{
		totalCost += m_Courbe.at(i)->cost();
	}
	
	// Verifies that the configuration is not ending the local path
	// Adds the modified Last local path to subpaths
	if (!end->equal(*q2))
	{
		if (!m_Courbe.at(last)->getBegin()->equal(*q2))
		{
			LocalPath LP(m_Courbe.at(last)->getBegin(), q2);
			
			if (LP.isValid())
			{
				totalCost += LP.cost();
			}
			else
			{
				cout << "Error: extractCostPortion: portion of path not valid"
				<< endl;
				return -1;
			}
		}
	}
	else
	{
		if (m_Courbe.at(last)->isValid())
		{
			totalCost += m_Courbe.at(last)->cost();
		}
		else
		{
			cout << "Error: extractCostPortion: portion of path not valid"
			<< endl;
			return -1;
		}
	}
	
	// Verifies the integrity of the sub paths
	/*if ((!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(
	 *paths.back()->getEnd())))
	 {
	 paths.at(0)->getBegin()->print();
	 q1->print();
	 paths.back()->getEnd()->print();
	 q2->print();
	 cout << "Error: extractCostPortion: in extract sub portion integrity" << endl;
	 }*/
	
	return totalCost;
}

vector<LocalPath*> Trajectory::extractSubPortion(double param1, double param2,
																								 unsigned int& first, unsigned int& last)
{
	vector<LocalPath*> paths;
	vector<shared_ptr<Configuration> > conf;
	
	conf = getTowConfigurationAtParam(param1, param2, first, last);
	
	shared_ptr<Configuration> q1 = conf.at(0);
	shared_ptr<Configuration> q2 = conf.at(1);
	
	if (first > last)
	{
		cout << "Error: inconsistent query for subpath extraction" << endl;
		return paths;
	}
	
	// Case where they lie in the same path
	//----------------------------------------------------------------------------
	if (first == last)
	{
		if (q1->equal(*q2))
		{
			paths.resize(0);
			cout << "Error: q1 and q2 are the same in subportion extraction"
			<< endl;
			return paths;
		}
		paths.push_back(new LocalPath(q1, q2));
		return paths;
	}
	
	shared_ptr<Configuration> start = m_Courbe.at(first)->getBegin();
	shared_ptr<Configuration> end = m_Courbe.at(last)->getEnd();
	
	// Adds the modified first local path to subpaths
	// Verifies that the configuration is not starting the local path
	if (!start->equal(*q1))
	{
		if (!m_Courbe.at(first)->getEnd()->equal(*q1))
		{
			LocalPath* startNew =
			new LocalPath(q1, m_Courbe.at(first)->getEnd());
			
			if (startNew->isValid())
			{
				paths.push_back(startNew);
			}
			else
			{
				cout << "Error: portion of path not valid" << endl;
				return paths;
			}
		}
	}
	else
	{
		LocalPath* startNew = new LocalPath(*m_Courbe.at(first));
		
		if (startNew->isValid())
		{
			paths.push_back(startNew);
		}
		else
		{
			cout << "Error: portion of path not valid" << endl;
			return paths;
		}
	}
	
	// Adds all the paths between First and Last
	for (uint i = first + 1; i < last; i++)
	{
		paths.push_back(new LocalPath(*m_Courbe.at(i)));
	}
	
	// Verifies that the configuration is not ending the local path
	// Adds the modified Last local path to subpaths
	if (!end->equal(*q2))
	{
		if (!m_Courbe.at(last)->getBegin()->equal(*q2))
		{
			LocalPath* endNew = new LocalPath(m_Courbe.at(last)->getBegin(), q2);
			
			if (endNew->isValid())
			{
				paths.push_back(endNew);
			}
			else
			{
				cout << "Error: portion of path not valid" << endl;
				return paths;
			}
		}
	}
	else
	{
		LocalPath* endNew = new LocalPath(*m_Courbe.at(last));
		
		if (endNew->isValid())
		{
			paths.push_back(endNew);
		}
		else
		{
			cout << "Error: portion of path not valid" << endl;
			return paths;
		}
	}
	
	// Verifies the integrity of the sub paths
	if ((!q1->equal(*paths.at(0)->getBegin())) || (!q2->equal(
																														*paths.back()->getEnd())))
	{
		paths.at(0)->getBegin()->print();
		q1->print();
		paths.back()->getEnd()->print();
		q2->print();
		cout << "Error: in extract sub portion integrity" << endl;
	}
	
	return paths;
}

Trajectory Trajectory::extractSubTrajectory(unsigned int id_start, unsigned int id_end)
{
	vector<LocalPath*> path;
	
	if (id_start > id_end)
	{
		cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
	}
	else
	{
    for (unsigned int i=id_start; i<=id_end; i++) {
      path.push_back(new LocalPath(*m_Courbe[i]));
    }
	}
	
	Trajectory newTraj(m_Robot);
	
	newTraj.m_Courbe = path;
	newTraj.nloc = path.size();
	
	if (path.size() == 0)
	{
		newTraj.m_Source = m_Courbe[id_start]->getBegin();
		newTraj.m_Target = m_Courbe[id_end]->getEnd();
		newTraj.range_param = 0;
	}
	else
	{
		newTraj.m_Source = path.at(0)->getBegin();
		newTraj.m_Target = path.back()->getEnd();
		newTraj.range_param = computeSubPortionRange(path);
	}
	
	return newTraj;
}

Trajectory Trajectory::extractSubTrajectory(double param1, double param2)
{
	
	uint first(0);
	uint last(0);
	
	vector<LocalPath*> path;
	
	if (param1 > param2)
	{
		cout << "Warning: inconsistant query in extractSubTrajectory" << endl;
	}
	else
	{
		path = extractSubPortion(param1, param2, first, last);
	}
	
	Trajectory newTraj(m_Robot);
	
	newTraj.m_Courbe = path;
	newTraj.nloc = path.size();
	
	if (path.size() == 0)
	{
		newTraj.m_Source = configAtParam(param1);
		newTraj.m_Target = newTraj.m_Source;
		newTraj.range_param = 0;
	}
	else
	{
		newTraj.m_Source = path.at(0)->getBegin();
		newTraj.m_Target = path.back()->getEnd();
		newTraj.range_param = computeSubPortionRange(path);
	}
	
	return newTraj;
}

extern double ZminEnv;
extern double ZmaxEnv;

#ifndef P3D_PLANNER
double ZminEnv;
double ZmaxEnv;
#endif

extern void* GroundCostObj;

void Trajectory::draw(int nbKeyFrame)
{
	
	double du = range_param / nbKeyFrame;
	double u = du;
	
	int val1, val2;
	double Cost1, Cost2;
	
	p3d_obj *o;
	
	int NumBody = m_Robot->getRobotStruct()->no - 1;
	
	if ((NumBody >= m_Robot->getRobotStruct()->no) || (NumBody < 0))
		return;
	
	if (!(o = m_Robot->getRobotStruct()->o[NumBody]))
		return;
	
	shared_ptr<Configuration> qSave = m_Robot->getCurrentPos();
	shared_ptr<Configuration> q = m_Source;
	m_Robot->setAndUpdate(*q);
	
	p3d_vector3 pi, pf;
	p3d_jnt_get_cur_vect_point(o->jnt, pi);
	
	int saveColor;
	bool red = false;
	
	while (u < range_param)
	{
		/* position of the robot corresponding to parameter u */
		q = configAtParam(u);
		m_Robot->setAndUpdate(*q);
		p3d_jnt_get_cur_vect_point(o->jnt, pf);
		
		if (isHighestCostIdSet)
		{
			if (getIdOfPathAt(u) == HighestCostId && !red)
			{
				red = true;
				saveColor = mColor;
				mColor = 3;
			}
			if ((mColor == 3) && (getIdOfPathAt(u) != HighestCostId) && red)
			{
				mColor = saveColor;
				red = false;
			}
		}
		
		if ((!ENV.getBool(Env::isCostSpace)) || (GroundCostObj == NULL))
		{
			glLineWidth(3.);
			g3d_drawOneLine(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2], mColor,
											NULL);
			glLineWidth(1.);
		}
		else
		{
			val1 = GHintersectionVerticalLineWithGround(GroundCostObj, pi[0],
																									pi[1], &Cost1);
			val2 = GHintersectionVerticalLineWithGround(GroundCostObj, pf[0],
																									pf[1], &Cost2);
			glLineWidth(3.);
			g3d_drawOneLine(pi[0], pi[1], Cost1 + (ZmaxEnv - ZminEnv) * 0.02,
											pf[0], pf[1], Cost2 + (ZmaxEnv - ZminEnv) * 0.02, mColor,
											NULL);
			glLineWidth(1.);
		}
		p3d_vectCopy(pf, pi);
		u += du;
	}
	
	m_Robot->setAndUpdate(*qSave);
}

double Trajectory::costOfPortion(double param1, double param2)
{
	
	uint first(0);
	uint last(0);
	
	vector<LocalPath*> path = extractSubPortion(param1, param2, first, last);
	
	double Cost = computeSubPortionCost(path);
	
	for (unsigned int i = 0; i < path.size(); i++)
	{
		delete path.at(i);
	}
	return Cost;
	
}
//double Trajectory::costOfPortion(double param1,double param2){
//
//	double soFar(0.0);
//	double prevSoFar(0.0);
//	uint id_start;
//	uint id_end;
//	shared_ptr<Configuration> confPtrStart;
//	shared_ptr<Configuration> confPtrEnd;
//
//	// TODO change that function
//
//	if( param1 > param2 ){
//		cout << "Error: in Trajectory::costofPortion() " << endl;
//		return 0;
//	}
//
//	for(uint i=0;i<nloc;i++){
//
//		soFar = soFar + m_Courbe.at(i)->getParamMax();
//
//		// Parameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param1 < soFar){
//
//			if(param1 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrStart = m_Courbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrStart = m_Courbe.at(i)->configAtParam(param1-prevSoFar);
//			id_start = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	soFar = prevSoFar;
//
//	for(uint i=id_start;i<nloc;i++){
//
//		soFar = soFar + m_Courbe.at(i)->getParamMax();
//
//		// Paramameter lies in the previous local path
//		// return configuration inside previous LP
//		if(param2 < soFar){
//
//			if(param2 < prevSoFar)
//				cout << "Error getting Conf at param" << endl;
//
//			confPtrEnd = m_Courbe.at(i)->configAtParam(param2-prevSoFar);
//			id_end = i;
//			break;
//		}
//		if(i==nloc-1){
//			confPtrEnd = m_Courbe.at(i)->getEnd();
//			id_end = i;
//		}
//		prevSoFar = soFar;
//	}
//
//	LocalPath pathStart(confPtrStart, m_Courbe.at(id_start)->getEnd());
//	LocalPath pathEnd(m_Courbe.at(id_end)->getBegin(),confPtrEnd);
//
//	double cost =0.0;
//
//	cost += pathStart.cost() + pathEnd.cost();
//
//	for(uint i=id_start+1;i<id_end;i++){
//		cost+=m_Courbe.at(i)->cost();
//	}
//
//	return cost;
//}

void Trajectory::push_back(shared_ptr<Configuration> q)
{
	if( m_Courbe.empty() )
	{
		if( m_Source->getConfigStruct() == NULL )
		{
			m_Source = q;
		}
		else
		{
			m_Courbe.push_back(new LocalPath(m_Source,q));
			m_Target = q;
			updateRange();
		}
	}
	else
	{
		m_Courbe.push_back(new LocalPath(m_Target,q));
		m_Target = q;
		updateRange();
	}
}

unsigned int Trajectory::cutPortionInSmallLP(vector<LocalPath*>& portion, unsigned int nLP)
{
	double range = computeSubPortionRange(portion);
	
	double soFar = 0.0;
	double prevSoFar = 0.0;
	double param = 0.0;
	
	if (nLP == 0)
	{
		cout << "Error: cutPortionInSmallLP" << endl;
	}
	
	if (nLP == 1)
	{
		return nLP;
	}
	
	cout << "NB Of LP = " << portion.size() << endl;
	cout << "NB Of LP = " << nLP << endl;
	
	shared_ptr<Configuration> confPtr;
	shared_ptr<Configuration> confPtrTmp;
	
	prevSoFar			= 0.0;
	soFar					= portion[0]->getParamMax();
	confPtrTmp		= portion[0]->getBegin();
	
	const double dMax = range / nLP;
	
	// Compute real number of small LP
	unsigned int nbOfSmallLP = 0;

	for (unsigned int i = 0; i<portion.size(); i++)
	{
		double resol;
    double length = portion[i]->getParamMax();
    
    if ( length < dMax )
		{
			resol = length;
		}
		else if ( floor(length/dMax) == length/dMax ) 
		{
			resol = dMax;
		}
		else 
		{
			double n = floor(length/dMax); // minimal number of segment
			resol = length / n;
		}
    
    unsigned int nbOfDMax = floor((portion[i]->getParamMax()/resol)+0.5);
    nbOfSmallLP += nbOfDMax;
//    cout << "dMax = " << dMax << endl;
//    cout << "resol = " << resol << endl;
//    cout << "nbOfDMax = " << nbOfDMax << endl;
//    cout << "getParamMax() = " << portion[i]->getParamMax() << endl;
	}
	cout << "range = " << range << endl;
  cout << "nbOfSmallLP = " << nbOfSmallLP << endl;
	vector<LocalPath*> portionTmp(nbOfSmallLP);
	
	try
	{
		unsigned int j = 0;
		
		// Loop for every Small LP
		for (unsigned int i = 0; i<nbOfSmallLP-1; i++)
		{
			double resol;
      double length = portion[j]->getParamMax();
      
      if ( length < dMax )
      {
        resol = length;
      }
      else if ( floor(length/dMax) == length/dMax ) 
      {
        resol = dMax;
      }
      else 
      {
        double n = floor(length/dMax); // minimal number of segment
        resol = length / n;
      }
      
			param += resol;
			//			cout << "nb of smal lp on big lp : " <<  portion[j]->getParamMax()/resol << endl;
			//			cout << "nb of smal lp on big lp : " <<  floor((portion[j]->getParamMax()/resol)+0.5) << endl;
			
			// Check to go seek the conf on next big LP
			while ( param > soFar )
			{
				j++;
				if ( j >= portion.size() ) 
				{
					cout << "Warning : Went too far on traj" << endl;
					j = portion.size() - 1;
					break;
				}
				prevSoFar = soFar;
				soFar += portion[j]->getParamMax();
			}
			
			// Create small local path
			confPtr = portion[j]->configAtParam(param - prevSoFar);
			portionTmp[i] = new LocalPath(confPtrTmp, confPtr);
			confPtrTmp = confPtr;
		}
	}
	catch(string str)
	{
		cout << "Exeption in cutPortionInSmallLP" << endl;
		cout << str << endl;
		return 0;
	}
	catch (...) 
	{
		cout << "Exeption in cutPortionInSmallLP" << endl;
		return 0;
	}
	
	// The last LP is made of the end configuration
	confPtrTmp = portionTmp[nbOfSmallLP-2]->getEnd();
	confPtr = portion.back()->getEnd();
	
	portionTmp.back() = new LocalPath(confPtrTmp, confPtr);
	
	cout <<  "old range = " << range << endl;
	cout <<  "new range = " << computeSubPortionRange(portionTmp) << endl;
	
	// Delete and replace every thing
	for (unsigned int i = 0; i < portion.size(); i++)
	{
		delete portion.at(i);
	}
	
	portion.clear();
	portion = portionTmp;
	
	if (nLP != portion.size())
	{
		cout << "Error: cutPortionInSmallLP" << endl;
	}
	
	return nLP;
}

void Trajectory::cutTrajInSmallLP(unsigned int nLP)
{
	try
	{
		cutPortionInSmallLP(m_Courbe, nLP);
	}
	catch(string str)
	{
		cout << "Exeption in cutTrajInSmallLP" << endl;
		cout << str << endl;
		return;
	}
	catch (...) 
	{
		cout << "Exeption in cutTrajInSmallLP" << endl;
		return;
	}
	
	updateRange();
	
	cout << "Cutting into " << nLP << " local paths" << endl;
	cout << "Traj Size = " << nloc << endl;
	cout << "Cost Of trajectory :" << this->cost() << endl;
	
	if (!m_Source->equal(*configAtParam(0)))
	{
		cout << "Error" << endl;
	}
	
	if (!m_Target->equal(*configAtParam(range_param)))
	{
		m_Target->print();
		configAtParam(range_param)->print();
		cout << "Error" << endl;
	}
}

bool Trajectory::concat(const Trajectory& traj)
{
    for( unsigned int i=0; i<traj.m_Courbe.size(); i++ )
    {
        m_Courbe.push_back( new LocalPath( *traj.m_Courbe[i] ));
    }

    updateRange();
    return true;
}

bool Trajectory::replacePortion(unsigned int id1, unsigned int id2, vector<LocalPath*> paths, bool freeMemory )
{
	// WARNING: the ids are ids of nodes and not LocalPaths
	if (id1 == id2)
	{
		cout << "Error: in replace local (id1 and id2 are the same)" << endl;
    return false;
	}
  if (id2 < id1) 
  {
    cout << "Error: in replace local (id2 > id2)" << endl;
    return false;
  }
  if (freeMemory) 
  {
    for (uint i = id1; i < id2; i++)
    {
      delete m_Courbe.at(i);
    }
  } 
  
    m_Courbe.erase(m_Courbe.begin() + id1, m_Courbe.begin() + id2);
	m_Courbe.insert(m_Courbe.begin() + id1, paths.begin(), paths.end());
	
	updateRange();
  return true;
}

bool Trajectory::replacePortion(double param1, double param2,
																vector<LocalPath*> paths , bool freeMemory )
{
	shared_ptr<Configuration> q11 = paths.at(0)->getBegin();
	shared_ptr<Configuration> q12 = paths.back()->getEnd();
	
	shared_ptr<Configuration> q21;
	shared_ptr<Configuration> q22;

	if (param1 > param2)
	{
		cout
		<< "Warning: Error not possible to replace trajectory because of the parameters"
		<< endl;
		return false;
	}
	
	// TODO replace with extratSubPortion
	
	// Looks for the local paths to be changed linear in NLOC
	//-------------------------------------------------------------------
	double soFar(0.0);
	double prevSoFar(0.0);
	
	double param_start(0.0);
	double param_end(0.0);
	
	unsigned int id_LP_1(0);
	unsigned int id_LP_2(0);
	
	for (unsigned int i = 0; i < nloc; i++)
	{
		soFar = soFar + m_Courbe[i]->getParamMax();
		
		// param1 is in local path i
		if (param1 < soFar)
		{
			// get configuration in local path i
			q21 = m_Courbe.at(i)->configAtParam(param1 - prevSoFar);
			param_start = param1 - prevSoFar;
			id_LP_1 = i;
			break;
		}
		prevSoFar = soFar;
		if (i == (nloc - 1))
		{
			cout << "Warning: first parameter not found on trajectory" << endl;
			//			return;
		}
	}
	
	soFar = prevSoFar;
	
	for (unsigned int i = id_LP_1; i < nloc; i++)
	{
		soFar = soFar + m_Courbe[i]->getParamMax();
		
		// param2 is in local path i
		if (param2 < soFar)
		{
			// get configuration in local path i
			q22 = m_Courbe.at(i)->configAtParam(param2 - prevSoFar);
			param_end = param2 - soFar;
			id_LP_2 = i;
			break;
		}
		prevSoFar = soFar;
		
		if (i == (nloc - 1))
		{
			q22 = m_Courbe.at(nloc - 1)->getEnd();
			id_LP_2 = i;
			param_end = soFar;
		}
	}
	
	// Condition has to be false if the trajectory is consistent
	if ((!q11->equal(*q21)) || (!q12->equal(*q22)))
	{
		if (!q11->equal(*q21))
		{
			cout << "q11 and q21" << endl;
			q11->print();
			q21->print();
		}
		if (!q12->equal(*q22))
		{
			cout << "q12 and q22" << endl;
			q12->print();
			q22->print();
		}
		cout
		<< "Warning: Error not possible to replace trajectory because configuration are not the same"
		<< endl;
		return false;
	}
	
	// Adds to paths the portion of local path (At begin and End of the portion)
	//----------------------------------------------------------------------------
	shared_ptr<Configuration> start = m_Courbe.at(id_LP_1)->getBegin();
	shared_ptr<Configuration> end = m_Courbe.at(id_LP_2)->getEnd();
	
	/*cout << "Start and End" << endl;
	 start->print(); end->print();
	 cout << "q21 and q22" << endl;
	 q21->print(); q22->print();*/
	
	// Verifies that the configuration is not starting the local path
	if (!start->equal(*q21))
	{
		LocalPath* startNew = new LocalPath(start, q21);
		
		if (startNew->isValid())
		{
			paths.insert(paths.begin(), new LocalPath(start, q21));
		}
		else
		{
			//cout << "Error: portion of path not valid" << endl;
			return false;
		}
	}
	
	// Verifies that the configuration is not ending the local path
	if (!end->equal(*q22))
	{
		LocalPath* endNew = new LocalPath(q22, end);
		
		if (endNew->isValid())
		{
			paths.push_back(endNew);
		}
		else
		{
			//cout << "Error: portion of path not valid" << endl;
			return false;
		}
	}
	
	/*for(int i=0;i<paths.size();i++)
	 {
	 paths[i]->print();
	 }*/
	//	cout << "PathSizeOf() = " << paths.size() << endl;
	// TODO optional
	//cutPortionInSmallLP(paths);
	
	
	// Free, Erases the old ones and the Inserts the new ones
	//---------------------------------------------------------------------------
	unsigned int id1_erase = id_LP_1;
	unsigned int id2_erase = id_LP_2 + 1;
	
	replacePortion(id1_erase, id2_erase, paths, freeMemory );
	return true;
}

unsigned int Trajectory::getIdOfPathAt(double param)
{
	double soFar(0.0);
	
	for (unsigned int i = 0; i < nloc; i++)
	{
		soFar = soFar + m_Courbe.at(i)->getParamMax();
		
		if (param < soFar)
		{
			return i;
		}
	}
	return nloc - 1;
}

int Trajectory::meanCollTest()
{
	int CollTest = 0.0;
	for(unsigned int i=0;i<m_Courbe.size();i++)
	{
		if(!(m_Courbe[i]->isValid()))
		{
			cout << "Trajectory::Warning => LocalPath is not valid in trajectory" << endl;
		}
		CollTest += m_Courbe[i]->getNbColTest();
	}
	return (int)(CollTest/m_Courbe.size());
}

vector<double> Trajectory::getCostAlongTrajectory(int nbSample)
{
	
	double step = this->getRangeMax() / (double) nbSample;
	
	vector<double> cost;
	
	for( double param=0; param<this->getRangeMax(); param = param + step)
	{
		cost.push_back(this->configAtParam(param)->cost());
		//        cout << this->configAtParam(param)->cost() << endl;
	}
	
	cout << "Compute Cost Along Traj of " << cost.size() << " samples" << endl;
	
	cost.resize(nbSample);
	
	return cost;
}

void Trajectory::print()
{
	
	cout << "-------------- Trajectory --------------" << endl;
	cout << " Number of LP " << nloc << endl;
	cout << " Range Parameter " << this->range_param << endl;
	
//	for (uint i = 0; i < nloc; i++)
//	{
//		cout << "Number " << i << endl;
//		m_Courbe.at(i)->print();
//	}
  
  if( nloc > 0 )
  {
    int size1 = nloc+1;
    int size2 = m_Courbe.at(0)->getBegin()->getEigenVector().size();
    
    // size1 = nRow
    // size2 = nCol
    Eigen::MatrixXd mat(size1,size2);
    //Eigen::VectorXd vect
    for (unsigned int j=0; j<nloc; j++)
    {
      mat.row(j) = m_Courbe.at(j)->getBegin()->getEigenVector();
    }
    
    if( nloc-1 >= 0 )
    {
      mat.row(nloc-1) = m_Courbe.at(nloc-1)->getEnd()->getEigenVector();
    }
    
    cout << mat << endl;
	}
	cout << "-----------------------------------" << endl;
}

void draw_traj_debug()
{
  if( ENV.getBool(Env::debugCostOptim) || ENV.getBool(Env::drawTrajVector) )
  {
    //std::cout << "Should be drawing traj" << std::endl;
    for(unsigned i=0;i<trajToDraw.size();i++)
    {
      trajToDraw.at(i).draw(500);
      //std::cout << "Drawing traj" << std::endl;
    }
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    if (!robotPt->tcur)
    {
      trajToDraw.clear();
    }
  }	
}

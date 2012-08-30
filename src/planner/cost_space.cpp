#include "cost_space.hpp"
#include <iostream>

#include "API/project.hpp"
#include "API/Roadmap/compco.hpp"

#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"
#include "planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"
#include "GroundHeight-pkg.h"
#include "Planner-pkg.h"

#include "Collision-pkg.h"

#include <algorithm>

#include <boost/function.hpp>
#include <boost/bind.hpp>

extern void* GroundCostObj;

CostSpace* global_costSpace(NULL);

using namespace std;
using namespace tr1;

//using std::string;
//------------------------------------------------------------------------------

void GlobalCostSpace::initialize()
{
  if (global_costSpace != NULL) {
    return ;
  }
  
	// initialize the cost function object.
	global_costSpace = new CostSpace();
  
  std::cout << "Initializing the dummy cost Function" << std::endl;
	global_costSpace->addCost("NoCost",boost::bind(computeFlatCost, _1));
	global_costSpace->setCost("NoCost");
	
	std::cout << "Initializing the dist to obst costmap cost function" << std::endl;
	global_costSpace->addCost("costDistToObst",boost::bind(computeDistanceToObstacles, _1));
	global_costSpace->setCost("costDistToObst");
  
  std::cout << "Initializing the collision costmap cost function" << std::endl;
	global_costSpace->addCost("costIsInCollision",boost::bind(computeInCollisionCost, _1));
	global_costSpace->setCost("costIsInCollision");
	
	if(GroundCostObj)
	{
		std::cout << "Initializing the 2d costmap cost function" << std::endl;
		global_costSpace->addCost("costMap2D",boost::bind(computeIntersectionWithGround, _1));
		global_costSpace->setCost("costMap2D");
	}
}

//------------------------------------------------------------------------------
CostSpace::CostSpace() : mSelectedCostName("No Cost"), m_deltaMethod(cs_integral), m_resolution(cs_classic) 
{
  m_dmax = p3d_get_env_dmax();
  
  m_resolution = cs_pr2_manip;
}

//------------------------------------------------------------------------------
std::string CostSpace::getSelectedCostName()
{
  return mSelectedCostName;
}

//------------------------------------------------------------------------------
std::vector<string> CostSpace::getAllCost()
{
	std::vector<string> functions;
	std::map< string, boost::function<double(Configuration&)> >::iterator it;
	
	for ( it=mFunctions.begin() ; it != mFunctions.end(); it++ )
		functions.push_back((*it).first);
	
	return functions;
}

//------------------------------------------------------------------------------
bool CostSpace::setCost(string name)
{
	if(mFunctions.find(name) != mFunctions.end())
	{
    mSelectedCostName = name;
		mSelectedCost = mFunctions[name];
    return true;
	}
	else
	{
		std::cout << "Warning : in CostSpace::setCost(string name), could not find a cost function named " << name  << std::endl;
    return false;
	}
}
//------------------------------------------------------------------------------
void CostSpace::addCost(string name, 
												boost::function<double(Configuration&)> f)
{
	if(mFunctions.find(name) == mFunctions.end())
	{
		mFunctions[name] = f;
	}
	else
	{
		std::cout << "Warning : in CostSpace::addCost, replacing the cost function named " << name << "by another." << std::endl;
		mFunctions[name] = f;
	}
}
//------------------------------------------------------------------------------
void CostSpace::deleteCost(string name)
{
  std::map< string, boost::function<double(Configuration&)> >::iterator 
  it = mFunctions.find(name);
	
  if(it != mFunctions.end())
	{
		mFunctions.erase (it);
	}
	else 
	{
		std::cout << "Warning : in CostSpace::deleteCost, the cost function " << name << " does not exist." << std::endl;
	}
}

//------------------------------------------------------------------------------
double CostSpace::cost(Configuration& conf)
{
  shared_ptr<Configuration> q_tmp = conf.copy();
  
	if(!mSelectedCost.empty())
	{
		double cost = mSelectedCost(conf);
    
    if( !conf.equal( *q_tmp ) )
    {
      cout << "Cost function modifies the config" << endl;
    }
    return cost;
	}
	else
	{
		std::cout << "Warning : CostSpace::cost(Configuration& conf) called, but \
		the cost function has not been set." << std::endl;
		return(1.0);
	}
}

//------------------------------------------------------------------------------
/**
 * ComputeDeltaStepCost
 * Compute the cost of a portion of path */
double CostSpace::deltaStepCost(double cost1, double cost2, double length)
{
	const double epsilon = 0.002;
	//double alpha;
	const double kb = 0.00831;
  const double temp = 310.15;
	
	if ( ENV.getBool(Env::isCostSpace) )
	{
		switch (m_deltaMethod)
		{
			case cs_mechanical_work:
      {
				double cost;
				if (cost2 > cost1)
				{
					cost = length * epsilon + cost2 - cost1;
				}
				else
				{
					cost = epsilon * length;
				}
				return cost;
      }
        
        //    case MECHANICAL_WORK:
				//
				//      if(cost2 > cost1)
				//      {
				//    	  return length*(epsilon+ cost2 - cost1);
				//      }
				//      else
				//      {
				//		  return epsilon*length;
				//      }
				
			case cs_integral:
			case cs_visibility:
      {
        double powerOnIntegral = 1.0;
        double cost = pow(((cost1 + cost2)/2),powerOnIntegral)*length;
        
        // Warning, length added for dynamic shortcut (HRI)
        cost += epsilon*length;
        
        return cost;
      }
				
			case cs_average:
				return (cost1 + cost2) / 2.;
				
				//			case config_cost_and_dist:
				//				alpha = p3d_GetAlphaValue();
				//				return alpha * (cost1 + cost2) / 2. + (1. - alpha) * length;
				
			case cs_boltzman_cost:
				
				if (cost2 > cost1)
					return 1;
				return 1/exp(ENV.getInt(Env::maxCostOptimFailures)*(cost2-cost1)/(kb*temp));
				
			default:
				std::cout << "Warning: " << __func__ <<  std::endl;
		}
	}
	//no cost function
	return length;
}
//----------------------------------------------------------------------
void CostSpace::setNodeCost( Node* node, Node* parent )
{
  //------------------------------------------------------
  // New node cost
  //------------------------------------------------------
  if ( parent != NULL )
  {
    shared_ptr<Configuration> q_tmp = parent->getConfiguration()->copy();
    
    // Compute the sum of cost of the node
    // TODO fix task space bug (should remove the copy)
    LocalPath path(parent->getConfiguration()->copy(),
                   node->getConfiguration());
    
    node->sumCost() = parent->sumCost() + path.cost();
    
    // Min and max cost of the compco
    node->getConnectedComponent()->getCompcoStruct()->minCost 
    = std::min(node->getConfiguration()->cost(),
               node->getConnectedComponent()->getCompcoStruct()->minCost );
    
    node->getConnectedComponent()->getCompcoStruct()->maxCost 
    = std::max(node->getConfiguration()->cost(),
               node->getConnectedComponent()->getCompcoStruct()->maxCost );
    
    if ( !parent->getConfiguration()->equal(*q_tmp) )
    {
      cout << "Configuration was modified" << endl;
    }
  }
}


/*if (p3d_GetCostMethodChoice() == URMSON_TRANSITION)
 {
 node->getConnectedComponent()->getCompcoStruct()->maxUrmsonCost
 = MAX(NodePt->sumCost +
 p3d_ComputeUrmsonCostToGoal(_Graph->getGraphStruct(),NodePt) ,
 node->getConnectedComponent()->getCompcoStruct()->maxUrmsonCost);
 }	*/

void CostSpace::getPr2ArmConfiguration( Eigen::VectorXd& x, confPtr_t q )
{ 
  x[0] = (*q)[16];
  x[1] = (*q)[17];
  x[2] = (*q)[18];
  x[3] = (*q)[19];
  x[4] = angle_limit_PI((*q)[20]);
  x[5] = (*q)[21];
  x[6] = angle_limit_PI((*q)[22]);
}

//----------------------------------------------------------------------
double CostSpace::cost(LocalPath& path)
{
  confPtr_t q_tmp_begin = path.getBegin()->copy();
  confPtr_t q_tmp_end   = path.getEnd()->copy();
  
  double cost = 0;
  
	if (ENV.getBool(Env::isCostSpace))
	{
    int nStep;
    double deltaStep;
    
    if( m_resolution == cs_classic )
    {
      deltaStep = path.getResolution();
      nStep = path.getParamMax()/deltaStep; 
    }
    
    if( m_resolution == cs_pr2_manip )
    {
      Eigen::VectorXd a( Eigen::VectorXd::Zero(7) );
      Eigen::VectorXd b( Eigen::VectorXd::Zero(7) );
      
      path.getLocalpathStruct();
      
      getPr2ArmConfiguration( a, path.getBegin() );
      getPr2ArmConfiguration( b, path.getEnd() );
      
      double param_max = ( a - b ).norm();
      nStep = ceil(param_max/(PlanEnv->getDouble(PlanParam::costResolution)*m_dmax));
      deltaStep = param_max/nStep;
//      cout << "param_max : " << param_max << " , nStep : " << nStep << endl;
//      cout << "a : " << endl << a << endl;
//      cout << "b : " << endl << b << endl;
    }
    
    confPtr_t q;
    
    double currentCost, prevCost;
    double currentParam = 0.0;
    
    prevCost = path.configAtParam(0.0)->cost();
    
    for (int i=0; i<int(nStep); i++)
    {
      q = path.configAtParam(currentParam);
      currentCost = this->cost(*q);
      
      cost += deltaStepCost( prevCost, currentCost, deltaStep );
      
      prevCost = currentCost;
      currentParam += deltaStep;
    }
  }
  else 
  {
    cost = path.getParamMax();
	}
  
  if( !q_tmp_begin->equal(*path.getBegin()) )
  {
    cout << "Warning => begin was modified by local path cost computation" << endl;
  }
  if( !q_tmp_end->equal(*path.getEnd()) ) 
  {
    cout << "Warning => end was modified by local path cost computation" << endl;
  }
	
	return cost;
}

//----------------------------------------------------------------------
// Basic cost functions
//----------------------------------------------------------------------

double computeFlatCost(Configuration& conf)
{
	return 1.0;
}

extern void* GroundCostObj;

double computeIntersectionWithGround(Configuration& conf)
{
	double cost(0);
	if(GroundCostObj)
	{
		GHintersectionVerticalLineWithGround(GroundCostObj, 
																				 conf.getConfigStruct()[6],
																				 conf.getConfigStruct()[7], 
																				 &cost);
	}
  //cout << "Ground Cost = " << cost << endl;
	return(cost);
}

double computeDistanceToObstacles(Configuration& conf)
{
  if( conf.isInCollision() )
  {
    return 10000;
  }
	Robot* robot = conf.getRobot();
	robot->setAndUpdate(conf);
	double cost = p3d_GetMinDistCost(robot->getRobotStruct());
	return cost;
}

double computeInCollisionCost(Configuration& conf)
{
	Robot* robot = conf.getRobot();
	shared_ptr<Configuration> qActual = robot->getCurrentPos();
  
  double cost = 0.1;
  
  if( conf.isInCollision() )
  {
    cost = 20.0;
  }
  
	robot->setAndUpdate(*qActual);
	return cost;
}

double computeCollisionSpaceCost(Configuration& conf)
{
  double cost = 0.1;
  if( optimizer.get() != NULL ) {
    cost = PlanEnv->getDouble(PlanParam::trajOptimObstacWeight)*optimizer->getCollisionSpaceCost( conf );
  }
	return cost;
}

double computeLocalpathKinematicCost(p3d_rob* rob, p3d_localpath* LP)
{
  if (LP == NULL) {
    return 1;
  }
  
  Robot* currRob = global_Project->getActiveScene()->getRobotByNameContaining(rob->name);
  LocalPath path(currRob,LP);
  double cost = path.cost();
  cout << "Kinematic cost = " << cost << endl;
  return cost;
}

//----------------------------------------------------------------------
void CostSpace::initMotionPlanning(p3d_graph* graph, p3d_node* start, p3d_node* goal)
{
  Robot* robot = global_Project->getActiveScene()->getRobotByName( graph->rob->name );
	start->temp = ENV.getDouble(Env::initialTemperature);
	start->comp->temperature = ENV.getDouble(Env::initialTemperature);
	start->nbFailedTemp = 0;
	
	p3d_SetGlobalNumberOfFail(0);
	
  Configuration qStart( robot, start->q );
	p3d_SetNodeCost( graph, start, qStart.cost() );
	p3d_SetCostThreshold( start->cost );
	
#ifdef P3D_PLANNER
	p3d_SetInitCostThreshold( start->cost );
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
	if ( ENV.getBool(Env::expandToGoal) && (goal != NULL))
	{
		goal->temp					= ENV.getDouble(Env::initialTemperature);
		goal->comp->temperature	= ENV.getDouble(Env::initialTemperature);
		start->temp				= ENV.getDouble(Env::initialTemperature);
		goal->nbFailedTemp = 0;
		//    Ng->NbDown = 0;
    
    Configuration qGoal( robot, goal->q );
		p3d_SetNodeCost( graph, goal, qGoal.cost());
		p3d_SetCostThreshold(MAX(qStart.cost(), qGoal.cost() ));
		
		//        p3d_SetCostThreshold(MAX(
		//								p3d_GetNodeCost(this->getStart()->getNodeStruct()), 
		//								p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
		
		p3d_SetAverQsQgCost(
												( graph->search_start->cost
												 +graph->search_goal->cost) / 2.);
	}
	else
	{
#ifdef P3D_PLANNER
		p3d_SetCostThreshold( start->cost );
		p3d_SetInitCostThreshold(start->cost );
		p3d_SetAverQsQgCost( graph->search_start->cost );
#else
		printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	}
}

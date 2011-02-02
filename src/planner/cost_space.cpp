#include "cost_space.hpp"
#include <iostream>

#include "API/planningAPI.hpp"

#include "Roadmap/compco.hpp"

#include "P3d-pkg.h"
#include "GroundHeight-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"

#include <algorithm>

#include <boost/function.hpp>
#include <boost/bind.hpp>

extern void* GroundCostObj;

using namespace std;
using namespace tr1;

void GlobalCostSpace::initialize()
{
	// initialize the cost function object.
	global_costSpace = new CostSpace();
	
	std::cout << "Initializing the dist to obst costmap cost function" << std::endl;
	global_costSpace->addCost("costDistToObst",boost::bind(computeDistanceToObstacles, _1));
	global_costSpace->setCost("costDistToObst");
	
	if(GroundCostObj)
	{
		std::cout << "Initializing the 2d costmap cost function" << std::endl;
		global_costSpace->addCost("costMap2D",boost::bind(computeIntersectionWithGround, _1));
		global_costSpace->setCost("costMap2D");
	}
}

//using std::string;
//------------------------------------------------------------------------------
CostSpace* global_costSpace(NULL);

//------------------------------------------------------------------------------
CostSpace::CostSpace() : m_deltaMethod(cs_integral)
{
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
void CostSpace::setCost(string name)
{
	if(mFunctions.find(name) != mFunctions.end())
	{
		mSelectedCost = mFunctions[name];
	}
	else
	{
		std::cout << "Warning : in CostSpace::setCost(string name), could not find a cost function named " << name  << std::endl;
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
	if(mFunctions.find(name) != mFunctions.end())
	{
		std::map< string, boost::function<double(Configuration&)> >::iterator it;
		it=mFunctions.find(name);
		mFunctions.erase (it);
	}
	else 
	{
		std::cout << "Warning : in CostSpace::deleteCost, the cost function " << name << " does not exist." << std::endl;
	}
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
/**
 * ComputeDeltaStepCost
 * Compute the cost of a portion of path */
double CostSpace::deltaStepCost(double cost1, double cost2, double length)
{
	double epsilon = 0.002;
	//double alpha;
	double kb = 0.00831, temp = 310.15;
	
	//length *= ENV.getDouble(Env::KlengthWeight);
	double powerOnIntegral = ENV.getDouble(Env::KlengthWeight);
	
	if ( ENV.getBool(Env::isCostSpace) )
	{
		switch (m_deltaMethod)
		{
			case cs_mechanical_work:
				
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
				
			case cs_integral:
			case cs_visibility:
				
				return pow(((cost1 + cost2)/2),powerOnIntegral)*length;
				
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
				
			case cs_average:
				return (cost1 + cost2) / 2.;
				
				//			case config_cost_and_dist:
				//				alpha = p3d_GetAlphaValue();
				//				return alpha * (cost1 + cost2) / 2. + (1. - alpha) * length;
				
			case cs_boltzman_cost:
				
				if (cost2 > cost1)
					return 1;
				return		1/exp(ENV.getInt(Env::maxCostOptimFailures)*(cost2-cost1)/(kb*temp));
				
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
	//	p3d_SetNodeCost(this->getActivGraph()->getGraphStruct(), 
	//									this->getGoal()->getNodeStruct(), 
	//									this->getGoal()->getConfiguration()->cost());
	//	double cost = node->getConfiguration()->cost();
	
  //------------------------------------------------------
  // Old node cost
  //------------------------------------------------------
	if(ENV.getBool(Env::use_p3d_structures))
	{
		p3d_node* NodePt = node->getNodeStruct();
		
		NodePt->cost = node->cost();
		
		if (node->getNodeStruct()->parent != NULL)
		{
			//       NodePt->sumCost = NodePt->parent->sumCost+cost;
			double cost1 = NodePt->parent->cost;
			double cost2 = NodePt->cost;
			double length = p3d_get_env_dmax();
			
			NodePt->sumCost = deltaStepCost(cost1, cost2, length) 
			+ NodePt->parent->sumCost;
			
			//		NodePt->sumCost = p3d_ComputeDeltaStepCost(cost1, cost2, length)
			//		+ NodePt->parent->sumCost;
		}
		else
		{
			NodePt->sumCost = node->cost();
		}
    
		node->getConnectedComponent()->getCompcoStruct()->minCost 
		= MIN(node->getConfiguration()->cost(),
					node->getConnectedComponent()->getCompcoStruct()->minCost );
		
		node->getConnectedComponent()->getCompcoStruct()->maxCost 
		= MAX(node->getConfiguration()->cost(),
					node->getConnectedComponent()->getCompcoStruct()->maxCost );
	}
	else 
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
}


/*if (p3d_GetCostMethodChoice() == URMSON_TRANSITION)
 {
 node->getConnectedComponent()->getCompcoStruct()->maxUrmsonCost
 = MAX(NodePt->sumCost +
 p3d_ComputeUrmsonCostToGoal(_Graph->getGraphStruct(),NodePt) ,
 node->getConnectedComponent()->getCompcoStruct()->maxUrmsonCost);
 }	*/

//----------------------------------------------------------------------
double CostSpace::cost(LocalPath& path)
{
  shared_ptr<Configuration> q_tmp_begin = path.getBegin()->copy();
  shared_ptr<Configuration> q_tmp_end   = path.getEnd()->copy();
  
  double Cost = 0;
  
	if (ENV.getBool(Env::isCostSpace))
	{
    double currentCost, prevCost;
    Eigen::Vector3d taskPos;
    Eigen::Vector3d prevTaskPos(0, 0, 0);
    
    double currentParam = 0;
    
    const double DeltaStep = path.getResolution();
    const unsigned int nStep = floor( ( path.getParamMax() / DeltaStep) + 0.5) ;
    
    double CostDistStep = DeltaStep;
    
    //	cout << "nStep = " << nStep <<  endl;
    
    shared_ptr<Configuration> confPtr;
    prevCost = path.getBegin()->cost();
    //	cout << "prevCost = " << prevCost << endl;
    
#ifdef LIGHT_PLANNER
    // If the value of Env::HRIPlannerWS changes while executing this
    // function, it could lead to the use of the incorrectly initialized 
    // prevTaskPos variable, and this triggers a compiler warning.
    // So, save the value of Env::HRIPlannerWS in a local variable.
    const bool isHRIPlannerWS = ENV.getBool(Env::HRIPlannerWS);
    //const bool isHRIPlannerWS = false;
    if(isHRIPlannerWS)
    {
      if( !q_tmp_begin->equal(*path.getBegin()) )
      {
        cout << "Warning => begin was modified by local path cost computation" << endl;
      }
      
      //!TODO fix task space bug in Configuration class
      prevTaskPos = path.getBegin()->getTaskPos();
      
      if( !q_tmp_begin->equal(*path.getBegin()) )
      {
        cout << "Warning => begin was modified by local path cost computation" << endl;
      }
    }
#endif
    // Case of task space
    vector<double> Pos;
    
    //                cout << "nStep =" << nStep << endl;
    for (unsigned int i = 0; i < nStep; i++)
    {
      currentParam += DeltaStep;
      
      confPtr = path.configAtParam(currentParam);
      currentCost = cost(*confPtr);
      //cout << "CurrentCost = " << currentCost << endl;
#ifdef LIGHT_PLANNER		
      // Case of task space
      if(isHRIPlannerWS)
      {
        taskPos = confPtr->getTaskPos();
        CostDistStep = ( taskPos - prevTaskPos ).norm();
        prevTaskPos = taskPos;
      }
#endif		
      Cost += deltaStepCost(prevCost, currentCost, CostDistStep);
      
      prevCost = currentCost;
    }
  }
  else 
  {
    Cost = path.getParamMax();
	}
  
  if( !q_tmp_begin->equal(*path.getBegin()) )
  {
    cout << "Warning => begin was modified by local path cost computation" << endl;
  }
  if( !q_tmp_end->equal(*path.getEnd()) ) 
  {
    cout << "Warning => end was modified by local path cost computation" << endl;
  }
	
	return Cost;
}

//----------------------------------------------------------------------
// Basic cost functions
//----------------------------------------------------------------------

double computeBasicCost(Configuration& conf)
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
	Robot* robotPt = conf.getRobot();
	shared_ptr<Configuration> qActual = robotPt->getCurrentPos();
	robotPt->setAndUpdate(conf);
	double cost = p3d_GetMinDistCost(robotPt->getRobotStruct());
	//	cout << "cost = "<< cost << endl;
	robotPt->setAndUpdate(*qActual);
	return cost;
}
//----------------------------------------------------------------------
void CostSpace::initMotionPlanning(Graph* graph, Node* start, Node* goal)
{
	start->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
	start->getNodeStruct()->comp->temperature = ENV.getDouble(Env::initialTemperature);
	start->getNodeStruct()->nbFailedTemp = 0;
	
	p3d_SetGlobalNumberOfFail(0);
	
	//  GlobalNbDown = 0;
	//  Ns->NbDown = 0;
	p3d_SetNodeCost(graph->getGraphStruct(),
									start->getNodeStruct(), 
									start->getConfiguration()->cost());
	
	p3d_SetCostThreshold(start->cost() );
	
#ifdef P3D_PLANNER
	p3d_SetInitCostThreshold(start->cost() );
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
	if ( ENV.getBool(Env::expandToGoal) && (goal != NULL))
	{
		goal->getNodeStruct()->temp					= ENV.getDouble(Env::initialTemperature);
		goal->getNodeStruct()->comp->temperature	= ENV.getDouble(Env::initialTemperature);
		start->getNodeStruct()->temp				= ENV.getDouble(Env::initialTemperature);
		goal->getNodeStruct()->nbFailedTemp = 0;
		//    Ng->NbDown = 0;
		p3d_SetNodeCost(graph->getGraphStruct(), 
										goal->getNodeStruct(), 
										goal->getConfiguration()->cost());
		
		p3d_SetCostThreshold(MAX(start->cost() , 
														 goal->cost() ));
		
		//        p3d_SetCostThreshold(MAX(
		//								p3d_GetNodeCost(this->getStart()->getNodeStruct()), 
		//								p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
		
		p3d_SetAverQsQgCost(
												( graph->getGraphStruct()->search_start->cost
												 +graph->getGraphStruct()->search_goal->cost) / 2.);
	}
	else
	{
#ifdef P3D_PLANNER
		p3d_SetCostThreshold(start->cost() );
		p3d_SetInitCostThreshold(start->cost() );
		p3d_SetAverQsQgCost(graph->getGraphStruct()->search_start->cost);
#else
		printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	}
}

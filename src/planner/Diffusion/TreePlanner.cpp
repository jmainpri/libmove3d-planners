/*
 * TreePlanner.cpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#include "TreePlanner.hpp"

#include "API/Grids/PointCloud.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"

#include "../p3d/env.hpp"
#include "planEnvironment.hpp"
#include "replanningSimulators.hpp"

#include "Planner-pkg.h"
#include "Collision-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

/*!
 * Constructor
 */
TreePlanner::TreePlanner(Robot* R, Graph* G) :
Planner(R,G),
m_nbConscutiveFailures(0),
m_nbExpansion(0),
m_nbFailedExpansion(0)
{	
#ifdef DEBUG_STATUS
  cout << "TreePlanner::TreePlanner(R,G)" << endl;
#endif
}

/*!
 * Destructor
 */
TreePlanner::~TreePlanner()
{
	
}

unsigned TreePlanner::init()
{
	int added = 0;
	Planner::init();
	m_nbConscutiveFailures = 0;
	m_nbExpansion = 0;
	m_nbInitNodes = _Graph->getNumberOfNodes();
	return added;
}

/*!
 * Checks out that the plannification
 * problem fits such requirement
 */
bool TreePlanner::preConditions()
{
	//    cout << "Entering preCondition" << endl;
	
  //	if (ENV.getBool(Env::isCostSpace) && (ENV.getExpansionMethod()
  //																				== Env::Connect))
  //	{
  //		cout
  //		<< "Warning: Connect expansion strategy is usually unadapted for cost spaces\n"
  //		<< endl;
  //	}
  if (!_Robot->setAndUpdate(*_Start->getConfiguration()) )
  {
    cout << "TreePlanner::preConditions => Start config. does not respect kin. constraints" << endl;
    return false;
  }
  
  if (_Start->getConfiguration()->isOutOfBounds())
	{
		cout << "TreePlanner::preConditions => Start is out of bounds" << endl;
    _Start->getConfiguration()->isOutOfBounds(true);
		return false;
	}
	
	if (_Start->getConfiguration()->isInCollision())
	{
		cout << "TreePlanner::preConditions => Start in collision" << endl;
		p3d_print_col_pair();
		return false;
	}
  
	if (ENV.getBool(Env::biDir) && ENV.getBool(Env::expandToGoal))
	{
    if (*_Start->getConfiguration() == *_Goal->getConfiguration()) 
    {
      cout << "TreePlanner::preConditions => Tree Expansion failed: root nodes are the same" << endl;
      return false;
    }
    
    if (!_Graph->searchConf(*_Start->getConfiguration())) 
    {
      cout << "TreePlanner::preConditions => Config. start not in graph" << endl;
      return false;
    }
    
    if (!_Graph->searchConf(*_Goal->getConfiguration())) 
    {
      cout << "TreePlanner::preConditions => Config. goal not in graph" << endl;
      return false;
    }
	}
	
	if (ENV.getBool(Env::expandToGoal))
	{
		if( _Goal->getConfiguration()->isOutOfBounds() )
		{
			cout << "TreePlanner::preConditions => Goal is out of bounds" << endl;
      _Goal->getConfiguration()->isOutOfBounds(true);
			return false;
		}
		
    if (!_Robot->setAndUpdate( *_Goal->getConfiguration() ) )
    {
      cout << "TreePlanner::preConditions => Goal config. does not respect kin. constraints" << endl;
      return false;
    }
    
		if( _Goal->getConfiguration()->isInCollision() )
		{
			cout << "TreePlanner::preConditions => Goal in collision" << endl;
			p3d_print_col_pair();
			return false;
		}
	}
	
	if(ENV.getBool(Env::drawPoints))
	{
		PointsToDraw = new PointCloud;
	}
	
	return true;
}

/*!
 * Checks out if the plannification
 * problem has reach its goals
 */
bool TreePlanner::checkStopConditions()
{
  if( int(m_nbExpansion) > PlanEnv->getInt(PlanParam::plannerMaxIterations) )
  {
    cout
		<< "Failure: the maximum number of expansion is reached : " 
    << PlanEnv->getInt(PlanParam::plannerMaxIterations) 
		<< endl;
		return (true);
  }
  
	if ( ENV.getBool(Env::expandToGoal) && trajFound() && (!PlanEnv->getBool(PlanParam::rrtExtractShortestPath)))
	{
#ifdef DEBUG_STATUS
    cout << "Success: the start and goal components are connected." << endl;
#endif
		return (true);
	}
	
  if (/*ENV.getBool(Env::ligandExitTrajectory)*/false)
	{
		double d(_Start->getConfiguration()->dist(
                                              *_Graph->getLastNode()->getConfiguration()));
    
		if (d > 12.0)
		{
			ENV.setBool(Env::expandToGoal, true);
			_Goal = _Graph->getLastNode();
			_Graph->getGraphStruct()->search_goal = _Goal->getNodeStruct();
			_Goal->getNodeStruct()->rankFromRoot = 1;
			_Goal->getNodeStruct()->type = ISOLATED;
			_Robot->getGoTo() = _Goal->getConfiguration()->copy();
			cout << "Success: distance from start is " << d << endl;
			return (true);
		}
	}
	
	if (_Start->maximumNumberNodes())
	{
		cout
		<< "Failure: the maximum number of nodes in the start component is reached."
		<< endl;
		return (true);
	}
	
	if (ENV.getBool(Env::biDir) && ENV.getBool(Env::expandToGoal) )
	{
		if (_Goal->maximumNumberNodes())
		{
			cout
			<< "Failure: the maximum number of nodes in the goal component is reached."
			<< endl;
			return (true);
		}
	}
	
	if (_Graph->getNumberOfNodes() >= ((unsigned int)ENV.getInt(Env::maxNodeCompco)))
	{
		cout << "Failure: the maximum number of nodes is reached." << endl;
		return (true);
	}
	
	if ( int(m_nbConscutiveFailures) > ENV.getInt(Env::NbTry))
	{
		cout
		<< "Failure: the maximum number of consecutive failures (" << m_nbConscutiveFailures 
		<< ") to expand a component is reached."
		<< endl;
		return (true);
	}
	
	if ( PlanEnv->getBool(PlanParam::stopPlanner) )
	{
		cout << "Tree expansion cancelled by user." << endl;
		return (true);
	}
  
  if( PlanEnv->getBool(PlanParam::planWithTimeLimit) )
	{
		if ( m_time > PlanEnv->getDouble(PlanParam::timeLimitPlanning) ) 
		{
			cout << "Tree expansion has reached time limit ( " << m_time << " ) " << endl;
			return (true);
		}
	}
	
	return (false);
}

/*!
 * Tries to connect one node from
 * one component to the other
 */
bool TreePlanner::connectNodeToCompco(Node* N, Node* Compco)
{
    return _Graph->linkNodeAndMerge(N,Compco,false);
}


/*!
 * Main function to connect 
 * to the other component
 */
bool TreePlanner::connectionToTheOtherCompco( Node* toNode )
{
	bool connected(false);
	
	if( ENV.getBool(Env::tryClosest) )
	{
		bool WeigtedRot = ENV.getBool(Env::isWeightedRotation);
		ENV.setBool(Env::isWeightedRotation,false);
		
		Node* closestNode = _Graph->nearestWeightNeighbour(toNode,_Graph->getLastNode()->getConfiguration(),
																											 false, ENV.getInt(Env::DistConfigChoice));
		
		ENV.setBool(Env::isWeightedRotation,WeigtedRot);
		
		connected = connectNodeToCompco(_Graph->getLastNode(), closestNode );
	}
	
	else if(ENV.getBool(Env::randomConnectionToGoal))
	{
        connected = connectNodeToCompco( _Graph->getLastNode(), _Graph->randomNodeFromComp(toNode));
	}
	else
	{
		connected = connectNodeToCompco( _Graph->getLastNode(), toNode );
	}
	
	return connected;
}

const int nb_Graph_Saved = 100;
int ith_Graph_Saved = 0;

//! Main Function of the Tree Planner,
//! Bi-Directionality is handled here
unsigned int TreePlanner::run()
{
	shared_ptr<Configuration> tmp = _Robot->getCurrentPos();
	
	//	cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << endl;
	if(!preConditions())
	{
    m_fail = true;
		cout << "Stoped in Tree planner, pre condition" << endl;
		return 0;
	}
	
	unsigned int NbCurCreatedNodes = 0;
	unsigned int NbTotCreatedNodes = 0;
	
	Node* fromNode = _Start;
	Node* toNode = _Goal;
  
  confPtr_t goal_extract;
  
  double ts(0.0); 
  bool first_iteration(true);
  bool connected(false);
  
  if( !global_rePlanningEnv ) 
  {
    m_time = 0.0; ChronoOn();
  }
  else {
    // Stores the init time in ts on first call
    m_time = global_rePlanningEnv->time_since_last_call( first_iteration , ts );
  }
  
	while ( !checkStopConditions() )
	{
		ENV.setInt(Env::progress,(int)((double)_Graph->getNumberOfNodes()/(double)ENV.getInt(Env::maxNodeCompco)));
    
		// Do not expand in the case of a balanced bidirectional expansion,
		// if the components are unbalanced.
		if (  !ENV.getBool(Env::expandToGoal) || 
				!((ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced))
					&& (fromNode->getConnectedComponent()->getNumberOfNodes()
							> toNode->getConnectedComponent()->getNumberOfNodes() + 2)))
		{
      
			// expand one way
			// one time (Main function of Tree like planners
			NbCurCreatedNodes = expandOneStep( fromNode, toNode ); m_nbExpansion++;
			
			if ( NbCurCreatedNodes > 0 )
			{
        
				if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawExploration))
				{
				  _draw_func();
				}
				
				NbTotCreatedNodes += NbCurCreatedNodes;
				
				m_nbConscutiveFailures = 0;
				
				if (ENV.getBool(Env::expandToGoal))
				{
					// If it expands towards a goal, tries to link with local method
					if( connected || connectionToTheOtherCompco( toNode )  )
					{
            connected = true;
            //cout << "connected" << endl;
            
            if( PlanEnv->getBool( PlanParam::rrtExtractShortestPath ) ) 
            {
              extractTrajectory();
            }
					}
				}
      }
			else
			{
				m_nbConscutiveFailures++;
			}
		}
		if (ENV.getBool(Env::biDir))
		{
			swap( fromNode, toNode );
		}
    
    if( global_rePlanningEnv ) 
    {
      m_time += global_rePlanningEnv->time_since_last_call( first_iteration , ts );
    }
    else {
      ChronoTimes( &m_time , &ts );
    }
	}
  
  if( !global_rePlanningEnv ) {
    ChronoOff();
  }
	
	if ( (!ENV.getBool(Env::drawDisabled)) && (ENV.getBool(Env::drawExploration) || ENV.getBool(Env::drawTraj)) )
	{
	  _draw_func();
	}
	
	ENV.setInt(Env::nbQRand,m_nbExpansion);
	_Robot->setAndUpdate(*tmp);
	return (NbTotCreatedNodes);
}

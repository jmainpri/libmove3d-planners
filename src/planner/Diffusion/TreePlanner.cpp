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

using namespace std;
using namespace tr1;

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

int TreePlanner::init()
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
		return false;
	}
	
	if (_Start->getConfiguration()->isInCollision())
	{
		cout << "TreePlanner::preConditions => Start in collision" << endl;
		// TODO print out collision status!!! See FORMenv print call
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
			cout << "TreePlanner::preConditions => Goal in out of bounds" << endl;
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
			// TODO print out collision status!!! See FORMenv print call
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
  
	if ( ENV.getBool(Env::expandToGoal) && trajFound())
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
	
	if ( Env_stopUser() )
	{
		PlanEnv->setBool(PlanParam::stopPlanner,true);
		//p3d_SetStopValue(true);
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
	//    return p3d_ConnectNodeToComp(N->getGraph()->getGraphStruct(),
	//                                 N->getNodeStruct(), CompNode->getCompcoStruct());
	
	return _Graph->connectNodeToCompco(N,Compco);
}

/*!
 * Main function to connect 
 * to the other component
 */
bool TreePlanner::connectionToTheOtherCompco(Node* toNode)
{
	bool isConnectedToOtherTree(false);
	
	if( ENV.getBool(Env::tryClosest))
	{
		bool WeigtedRot = ENV.getBool(Env::isWeightedRotation);
		ENV.setBool(Env::isWeightedRotation,false);
		
		Node* closestNode = _Graph->nearestWeightNeighbour(
																											 toNode,
																											 _Graph->getLastNode()->getConfiguration(),
																											 false,
																											 ENV.getInt(Env::DistConfigChoice));
		
		ENV.setBool(Env::isWeightedRotation,WeigtedRot);
		
		isConnectedToOtherTree = connectNodeToCompco(_Graph->getLastNode(), closestNode );
		
		if(isConnectedToOtherTree)
		{
			return true;
		}
	}
	
	if(ENV.getBool(Env::randomConnectionToGoal))
	{
		isConnectedToOtherTree = connectNodeToCompco(_Graph->getLastNode(), toNode->randomNodeFromComp());
	}
	else
	{
		isConnectedToOtherTree = connectNodeToCompco( _Graph->getLastNode(), toNode );
	}
	
	return isConnectedToOtherTree;
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
  
  double ts(0.0); 
  bool first_iteration(true);
  
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
			
			if (NbCurCreatedNodes > 0)
			{
        if( PlanEnv->getBool( PlanParam::rrtExtractShortestPath ) ) 
        {
          API::Trajectory* traj1 = _Graph->extractAStarShortestPathsTraj( _q_start, _q_goal );
          //        API::Trajectory* traj2 = _Graph->extractBestTrajSoFar( _q_start, _q_goal );
          //        vector<Node*> path = _Graph->extractAStarShortestNodePaths( _q_start, _q_goal );
          if( traj1 ) {
            cout << "traj1 cost : " << traj1->cost() << endl;
            //          cout << "traj2 cost : " << traj2->cost() << endl;
            //          cout << "traj3 cost : " << path.back()->sumCost(true) << endl;
          }
          else {
            cout << "no traj" << endl;
          }
        }
        
				if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawExploration))
				{
          //_Graph->extractBestTrajSoFar( _q_start, _q_goal );
				  _draw_func();
				}
				
				NbTotCreatedNodes += NbCurCreatedNodes;
				
				m_nbConscutiveFailures = 0;
				
				if (ENV.getBool(Env::expandToGoal))
				{
					// If it expands towards a goal, tries to link with local method
					if( connectionToTheOtherCompco( toNode ) )
					{
						cout << "connected" << endl;
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

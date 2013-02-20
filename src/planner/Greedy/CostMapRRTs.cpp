/*
 *  CostMapRRTs.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CostMapRRTs.hpp"
#include "Costmap-RRT.hpp"

#include "Util-pkg.h"

#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/graphConverter.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "planEnvironment.hpp"

#include "cost_space.hpp"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

CostmapPlanner::CostmapPlanner(Robot* R, Graph* G) : TreePlanner(R,G)
{
	m_bestPathCost	= numeric_limits<double>::max();
	m_bestSoFar			= numeric_limits<double>::max();
}

CostmapPlanner::~CostmapPlanner()
{
	
}

/*!
 * Initializes the planner with start and goal configurations
 */
unsigned CostmapPlanner::init()
{
	Planner::init();
	
	int added = 0;
	
	added += setInit(_Robot->getInitialPosition());
	added += setGoal(_Robot->getGoTo());
	
	return added;
}

/*!
 * Expand One Step
 */
int CostmapPlanner::expandOneStep(Node* fromComp, Node* toComp)
{
	return 0;
}

/*!
 * Checks out if the plannification
 * problem has reach its goals
 */
bool CostmapPlanner::checkStopConditions()
{	
	if ( Env_stopUser() )
	{
		PlanEnv->setBool(PlanParam::stopPlanner,true);
		//p3d_SetStopValue(true);
	}
	
	if ( PlanEnv->getBool(PlanParam::stopPlanner) )
	{
		cout << "Costmap planner cancelled." << endl;
		return (true);
	}
	
	if( m_withTimeLimit )
	{
		if ( m_time > 20.0 ) 
		{
			cout << "Smoothin has reached time limit ( " << m_time << " ) " << endl;
			return true;
		}
	}
	
	if ( m_withGainLimit ) 
	{
		const int n = 10; 
		if ( gainOfLastIterations( n ) < 0.005 )
		{
			cout << "Smooting has reached maximal gain ( " << gainOfLastIterations( n ) << " ) " << endl;
			return true;
		}
	}
	
	return (false);
}

double CostmapPlanner::gainOfLastIterations(unsigned int n)
{
	if ( /*m_GainOfIterations.empty()*/ m_GainOfIterations.size() < n ) 
	{
		return 1.0;
	}
	else 
	{
		unsigned int start(m_GainOfIterations.size() - n);
		
		double gain(0.0);
		
		for ( unsigned int i = start ; 
				 i< m_GainOfIterations.size(); i++ ) 
		{
			gain += m_GainOfIterations[i];
		}
		
		gain /= ((double)( m_GainOfIterations.size() - start ));
		return gain;
	}	
}

/*!
 * 
 */ 
void CostmapPlanner::computeAllEdgeCost()
{
	for (unsigned int i=0; i<_Graph->getNumberOfEdges(); i++) 
	{
		cout << "Edge[" << i << "] = " << _Graph->getEdge(i)->cost() << endl;
	}
}

/*!
 * Removes the parts of the trees that are of higher cost
 */ 
void CostmapPlanner::autumnLeaves()
{	
	const double alpha2 = 0.80;
	
	m_bestPathCost *= alpha2;
	
	vector<Node*> nodesToRemove;
	
	for (unsigned int i=0; i<_Graph->getNumberOfNodes(); i++) 
	{
		Node* node = _Graph->getNode(i);
		
		if ( node == _Start  ) // || node == _Goal ) 
		{
			continue;
		}
		
		//cout << "Node[" << i <<"] : SumOfCost = " << node->sumCost() << endl;
		
		if( node->sumCost() > m_bestPathCost )
		{
			Node* parent = node->parent();
			
			if ( parent->sumCost() > m_bestPathCost ) 
			{
				_Graph->removeEdges( node , parent  );
				nodesToRemove.push_back( node );
			}
			else 
			{
				// Wraning undirected form
				Edge* E = _Graph->isEdgeInGraph(parent,node);
				
				if (E == 0x0) 
				{
					throw string("Problem in the costmap planner taking out leaves");
				}
				
				double thresh = m_bestPathCost - parent->sumCost();
				double parame = E->getLocalPath()->whenCostIntegralPasses(thresh);
				
				cout <<  "param (%)  = " << 100*parame/E->getLocalPath()->getParamMax() << endl;
				
				// Get configuration at maximal threshold
				// And insert in graph as leaf
				shared_ptr<Configuration> q = E->getLocalPath()->configAtParam( parame );
				
				_Graph->removeEdges( node , parent  );
				nodesToRemove.push_back( node );
				
				LocalPath path( parent->getConfiguration() , q );
				_Graph->insertNode( parent , path );
			}
		}
	}
	
	// 
	for (unsigned int i=0; i<nodesToRemove.size(); i++) 
	{
		_Graph->removeNode( nodesToRemove[i] );
	}
	
	m_bestPathCost /= alpha2;
}

/*!
 * Removes high cost portion
 */
bool CostmapPlanner::deleteHighCostPortion(LocalPath& path, double thresh)
{
	double pathCost = path.cost();
	/*double param =*/ path.whenCostIntegralPasses(pathCost/2);
	return false;
}

/*!
 * Removes the parts of the trees that are of higher cost
 */ 
void CostmapPlanner::deleteHighIntergralOfCost()
{	
	const double alpha2 = 0.80;
	
	m_bestPathCost *= alpha2;
	
	vector<Node*> nodesToRemove;
	
//	for (unsigned int i=0; i<_Graph->getNumberOfEdges(); i++) 
//	{
//		Edge
//	}
	
	// 
	for (unsigned int i=0; i<nodesToRemove.size(); i++) 
	{
		_Graph->removeNode( nodesToRemove[i] );
	}
	
	m_bestPathCost /= alpha2;
}


/*!
 * Returns the trajectory
 */ 
API::Trajectory* CostmapPlanner::getTrajectoryAndComputeCost()
{
	cout << "Export the cpp graph to new graph" << endl;
	
  // Warning broken
	// Graph graphTraj( _Robot , _Graph->exportCppToGraphStruct() );
	
  GraphConverter gc;
  Graph graphTraj( _Robot , gc.convert(*_Graph) );
  
	API::Trajectory* traj = graphTraj.extractBestTraj(_Start->getConfiguration(),
																										_Goal->getConfiguration());
	
	// Compute new cost and record the gain
	m_bestPathCost = traj->cost();
	PlanEnv->setDouble(PlanParam::costTraj,m_bestPathCost);
	
	cout << "Traj cost : " << m_bestPathCost << endl;
	m_GainOfIterations.push_back( ( m_bestSoFar - m_bestPathCost ) / m_bestSoFar );
	
	//computeAllEdgeCost();
	traj->replaceP3dTraj();
	
	if (ENV.getBool(Env::drawExploration))
	{
		(*_draw_func)();
	}
	
	//traj->getRangeMax()/;
	
	return traj;
}


/*!
 * Main function of the planner
 * Sevral RRT are run from with different thresholds
 */
unsigned int CostmapPlanner::run()
{	
	if( !preConditions() )
	{
		cout << "Stoped in CostmapPlanner planner, preCondition failed" << endl;
		return 0;
	}
	
	// CostSpaceDeltaStepMethod
	global_costSpace->setDeltaStepMethod(cs_integral);
	cout << "Cost Type = " << cs_integral << endl;
	
	m_GainOfIterations.clear();
	
	API::Trajectory* lastTraj = NULL;
	
	double alpha = 0.99;
	
	double ts(0.0); m_time = 0.0; ChronoOn();
	
	shared_ptr<Configuration> gGoal = _Goal->getConfiguration();
	
	while ( !checkStopConditions() )
	{
		cout << "----------------------------------------------" << endl;
		cout << "m_bestPathCost = " << m_bestPathCost << endl;
		
		CostmapRRT rrt(_Robot,_Graph);
		rrt.init();
		rrt.setLastBestPath( m_bestPathCost );
		rrt.run();
		
		if( PlanEnv->getBool(PlanParam::stopPlanner) )
		{
			break;
		}
		
		if (lastTraj) 
		{
			delete lastTraj;
		}
		
		lastTraj = getTrajectoryAndComputeCost();
		
		m_bestSoFar = m_bestPathCost;
		m_bestPathCost *= alpha;
		
		//autumnLeaves();
		
		delete _Graph;
		_Graph = API_activeGraph = new Graph(_Robot);
		setInit(_Robot->getInitialPosition());
		setGoal(gGoal);

		// Set goal to the graph (might be out)
		setGoal(gGoal);
		
		ChronoTimes( &m_time , &ts );
	}
	
	ChronoOff();
	
	lastTraj->replaceP3dTraj();
	
	return _Graph->getNumberOfNodes();
}

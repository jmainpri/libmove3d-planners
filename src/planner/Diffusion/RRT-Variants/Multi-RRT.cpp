/*
 *  Multi-RRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 09/06/10.
 *  Copyright 2010 CNRS/LAAS. All rights reserved.
 *
 */

#include "Multi-RRT.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"

#include "P3d-pkg.h"
#include "Graphic-pkg.h"

using namespace std;
using namespace tr1;

MultiRRT::MultiRRT(Robot* R, Graph* G) :
RRT(R,G)
{
    cout << "Multi-RRT Constructor" << endl;
}

MultiRRT::~MultiRRT()
{
	
}

int MultiRRT::init()
{
	int added = TreePlanner::init();
	
	m_Roots.push_back( _Start );
	m_Roots.push_back( _Goal );
	
	_expan = new RRTExpansion(_Graph);
	
	if(_Robot->getName().compare("ROBOT_JUSTIN") == 0)
	{
#ifdef LIGHT_PLANNER
		_expan->setDirectionMethod(NAVIGATION_BEFORE_MANIPULATION);
		
		for (int i=0; i<ENV.getInt(Env::nbOfSeeds); i++)
		{
			shared_ptr<Configuration> q = _Robot->shootBaseWithoutCC();
			while ((!q->setConstraintsWithSideEffect()) || q->isInCollision()) 
			{
				q = _Robot->shootBaseWithoutCC();
			}
			g3d_draw_allwin_active();
			addSeed(q);
		}
#endif
	}
	else 
	{
		for (int i=0; i<ENV.getInt(Env::nbOfSeeds); i++)
		{
			shared_ptr<Configuration> q = _Robot->shoot();
			while ( q->isInCollision()) 
			{
				q = _Robot->shoot();
			}
			addSeed(q);
		}
	}
	
	setInit(true);
	
	return added;
}

bool MultiRRT::addSeed(shared_ptr<Configuration> q)
{	
	if (!_Init)
	{
		Node* N = _Graph->searchConf(*q);
		
		if (N == NULL)
		{
			Node* newNode = new Node( _Graph, q );
			_Graph->insertNode(newNode );
			m_Roots.push_back( newNode );
			return true;
		}
		
		return false;
	}
	
	return false;
}

Node* MultiRRT::getRandomCompcoForExpansion(Node* fromNode)
{
	Node* randomCompco;
	unsigned int i=0;
	int initNum = fromNode->getConnectedComponent()->getId() ;
	int randNum = fromNode->getConnectedComponent()->getId() ;
	
	//	cout << "Number of Component : " << _Graph->getNumberOfCompco() << endl;
	//	cout << "Init Component : " << initNum << endl;
	
	while ((initNum == randNum ) && (i<50))
	{
		double randomNumber = p3d_random(0,_Graph->getNumberOfCompco());
		unsigned int randId = (unsigned int)randomNumber;
//		cout << "Rand Number : " << randomNumber << endl;
//		cout << "Rand Id : " << randId << endl;
		randomCompco = _Graph->getCompco(randId);
		randNum = randomCompco->getConnectedComponent()->getCompcoStruct()->num;
		//		cout << "Rand Component : " << randNum << endl;
		i++;
	}
	
	if (i == 50 ) {
		cout << "No Other componnent" << endl;
	}
	
	return randomCompco;
}

Node* MultiRRT::getInitCompco()
{
	return getRandomCompcoForExpansion(
									   _Graph->getCompco(
														 (unsigned int)p3d_random(0,_Graph->getNumberOfCompco())));
	
	// Warning not used
	Node* leastExpandedCompco;
	unsigned int numberOfNodes = numeric_limits<unsigned int>::max();
	
	for (unsigned int i=0; i<_Graph->getNumberOfCompco(); i++) 
	{
		Node* comp = _Graph->getCompco(i);
		unsigned int nbNodes = comp->getNumberOfNodesInCompco();
		
		if( nbNodes < numberOfNodes ) 
		{
			numberOfNodes = nbNodes;
			leastExpandedCompco = comp;
		}
	}
	
	return leastExpandedCompco;
}

Node* MultiRRT::getGoalCompco()
{
	return getRandomCompcoForExpansion(m_initNode);
}

unsigned int MultiRRT::run()
{
	shared_ptr<Configuration> tmp = _Robot->getCurrentPos();
	
	//	cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << endl;
	if(!preConditions())
	{
		return 0;
	}
	
	int NbCurCreatedNodes = 0;
	int NbTotCreatedNodes = 0;
	
	
	//	unsigned int m_RootId=0;
	//	
	while (!checkStopConditions())
	{
		// expand one way
		// one time (Main function of Tree like planners
		m_initNode = getInitCompco();
		m_goalNode = getGoalCompco();
		
		NbCurCreatedNodes = expandOneStep(m_initNode,m_goalNode);
		
		if (NbCurCreatedNodes > 0)
		{
			if(ENV.getBool(Env::drawGraph))
			{
				(*_draw_func)();
			}
			
			NbTotCreatedNodes += NbCurCreatedNodes;
			
			//                cout << "NbTotCreatedNodes  = "  << NbTotCreatedNodes << endl;
			m_nbConscutiveFailures = 0;
			
			if (ENV.getBool(Env::expandToGoal))
			{
				// If it expands towards a goal
				// Tries to link with local method
				if( connectNodeToCompco(_Graph->getLastNode(), m_goalNode ) )
				{
					cout << "connected" << endl;
					//return (NbTotCreatedNodes);
				}
			}
		}
		else
		{
			m_nbConscutiveFailures++;
		}
	}
	if (ENV.getBool(Env::drawGraph))
	{
		(*_draw_func)();
	}
	ENV.setInt(Env::nbQRand,m_nbExpansion);
	
	_Robot->setAndUpdate(*tmp);
	
	return (NbTotCreatedNodes);
}
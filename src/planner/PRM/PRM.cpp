//
// C++ Implementation: prm
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PRM.hpp"

#include "API/Device/robot.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include "../p3d/env.hpp"
#include "planEnvironment.hpp"

#include <iostream>

#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

PRM::PRM(Robot* R, Graph* G) :
        Planner(R,G)
{
    cout << " New PRM "  << endl;
    m_nbConscutiveFailures = 0;
}

PRM::~PRM()
{
}

int PRM::init()
{
	int ADDED = 0;

	Planner::init();
	m_nbConscutiveFailures = 0;
	ADDED += Planner::setInit(_Robot->getInitialPosition());
	ADDED += Planner::setGoal(_Robot->getGoTo());
	_Init = true;

	return ADDED;
}

bool PRM::checkStopConditions()
{
	if (ENV.getBool(Env::expandToGoal) && trajFound())
	{
#ifdef DEBUG_STATUS
                cout << "Success: the start and goal components are connected." << endl;
#endif
		return (true);
	}

	if (m_nbConscutiveFailures > ENV.getInt(Env::NbTry))
	{
		cout
				<< "Failure: the maximum number of consecutive failures is reached."
				<< endl;
		PlanEnv->setBool(PlanParam::stopPlanner,true);
		//p3d_SetStopValue(true);
		return (true);
	}

	if (_Graph->getGraphStruct()->nnode >= ENV.getInt(Env::maxNodeCompco))
	{
		cout << "Stop: the maximum number of nodes in the graph is reached."
				<< endl;
		return (true);
	}

        if (_stop_func)
	{
                if (!(*_stop_func)())
		{
			PrintInfo(("basic PRM building canceled\n"));
			return true;
		}
	}

	return false;
}

/**
 * Checks out the preconditions
 */
bool PRM::preConditions()
{
	if (ENV.getBool(Env::expandToGoal) && 
		_Start->getConfiguration()->equal(*_Goal->getConfiguration()))
	{
		cout << "graph creation failed: start and goal are the same" << endl;
		return false;
	}
	
	return true;
}

/**
 * Main function
 */
void PRM::expandOneStep()
{
	shared_ptr<Configuration> q = _Robot->shoot();
	
	//                newConf->print();
	
	if ( q->setConstraintsWithSideEffect() && (!q->isInCollision()) )
	{
		Node* N = new Node(_Graph,q);
		
		_Graph->insertNode(N);
		_Graph->linkNode(N);
		
		m_nbConscutiveFailures = 0;
		m_nbAddedNode++;
		
		if (ENV.getBool(Env::drawExploration))
		{
			cout << "Number of nodes added : " << m_nbAddedNode << endl;
			(*_draw_func)();
		}
	}
	else
	{
		m_nbConscutiveFailures++;
	}
}

/* Main function of the PRM algorithm*/
unsigned int PRM::run()
{
	m_nbAddedNode = 0;
	
	shared_ptr<Configuration> tmp = _Robot->getCurrentPos();

	if (!preConditions()) 
	{
		return m_nbAddedNode;
	}

	while (!checkStopConditions())
	{
		expandOneStep();
	}
	
	_Robot->setAndUpdate(*tmp);
	
	return m_nbAddedNode;
}


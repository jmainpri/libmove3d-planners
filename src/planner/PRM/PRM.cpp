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
MOVE3D_USING_SHARED_PTR_NAMESPACE

PRM::PRM(Robot* R, Graph* G) :
    Planner(R,G)
{
    cout << " New PRM "  << endl;
    m_nbConscutiveFailures = 0;
}

PRM::~PRM()
{
}

unsigned PRM::init()
{
    int ADDED = 0;

    Planner::init();
    m_nbConscutiveFailures = 0;
    ADDED += Planner::setInit(_Robot->getInitPos());
    ADDED += Planner::setGoal(_Robot->getGoalPos());
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

    if (int(_Graph->getNumberOfNodes()) >= ENV.getInt(Env::maxNodeCompco))
    {
        cout << "Stop: the maximum number of nodes in the graph is reached."
             << endl;
        return (true);
    }

    if ( PlanEnv->getBool(PlanParam::stopPlanner) )
    {
        cout << "PRM expansion cancelled by user." << endl;
        return (true);
    }

    if( PlanEnv->getBool(PlanParam::planWithTimeLimit) )
    {
        if ( m_time > PlanEnv->getDouble(PlanParam::timeLimitPlanning) )
        {
            cout << "PRM expansion has reached time limit ( " << m_time << " ) " << endl;
            return (true);
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

    if(!ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::expandToGoal) )
    {
        LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
        if (direct.isValid())
        {
            _Graph->linkNodeAndMerge(_Start,_Goal,false);
#ifdef DEBUG_STATUS
            cout << "Direct connection" << endl;
#endif
            return true;
        }
    }
    return true;
}

/**
 * Main function
 */
void PRM::expandOneStep()
{
    confPtr_t q = _Robot->shoot();

    if ( q->setConstraintsWithSideEffect() && (!q->isInCollision()) )
    {
        Node* N = new Node(_Graph,q);

        _Graph->insertNode(N);
        _Graph->linkNode(N);

        m_nbConscutiveFailures = 0;
        m_nbAddedNode++;

        if (ENV.getBool(Env::drawExploration)) {
            cout << "Number of nodes added : " << m_nbAddedNode << endl;
            (*_draw_func)();
        }
    }
    else {
        m_nbConscutiveFailures++;
    }
}

/* Main function of the PRM algorithm*/
unsigned int PRM::run()
{
    m_time = 0.0;  double ts(0.0); ChronoOn();

    m_nbAddedNode = 0;
    m_nbExpansions = 0;

    confPtr_t tmp = _Robot->getCurrentPos();

    if (!preConditions())
    {
        return m_nbAddedNode;
    }

    while (!checkStopConditions())
    {
        expandOneStep(); m_nbExpansions++;
        ChronoTimes( &m_time , &ts );
    }

    ChronoOff();
    _Robot->setAndUpdate(*tmp);

    return m_nbAddedNode;
}


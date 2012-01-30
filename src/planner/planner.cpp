//
// C++ Implementation: planner
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "planner/planner.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Graphic-pkg.h" 

using namespace std;
using namespace tr1;

Planner* global_Move3DPlanner = NULL;

Planner::Planner() :
		  _stop_func(fct_stop),
		  _draw_func(ext_g3d_draw_allwin_active),
		  _Start(NULL),
		  _Goal(NULL),
		  _Robot(NULL),
		  _Graph(NULL),
		  _Init(false),
          m_fail(false)
{	
//  cout << "------------------------------------------------" << endl;
//  cout << " Planner::Planner() ";
}

Planner::Planner(Robot* rob, Graph* graph) :
		  _stop_func(fct_stop),
		  _draw_func(ext_g3d_draw_allwin_active),
		  _Start(NULL),
		  _Goal(NULL),
		  _Robot(rob),
		  _Graph(graph),
          _Init(false),
          m_fail(false)
{
}


Planner::~Planner()
{
}

bool Planner::trajFound()
{
  if( _Goal == NULL )
    return false;
  
	bool inSameCompco  = (_Goal ? _Start->inSameComponent(_Goal) : false);
	bool isTheSame = _Start->equal(_Goal);
	
	return (!isTheSame) && inSameCompco && (!m_fail);
}

Robot* Planner::getActivRobot()
{
	return _Robot;
}

void Planner::setRobot(Robot* R)
{
	_Robot = R;
}

Graph* Planner::getActivGraph()
{
	return _Graph;
}

void Planner::setGraph(Graph* G)
{
	_Graph = G;
}

Node* Planner::getInit()
{
	return _Start;
}

Node* Planner::getGoal()
{
	return _Goal;
}

int Planner::init()
{
	if( _Robot == NULL || _Graph == NULL )
	{
		cout << "Planner : Error in init is not well initialized" << endl;
	}
	
	_stop_func = fct_stop;

	return 0;
}

//! Set the start and goal configuration
bool Planner::setInit(shared_ptr<Configuration> Cs)
{
	bool b = false;
	
	if (!_Init)
	{
		Node* N = _Graph->searchConf(*Cs);
		if (N == NULL)
		{
			_Start = new Node(_Graph, Cs);
			_Graph->insertNode(_Start);
			_Graph->linkNode(_Start);
			b = true;
		}
		else
		{
			_Start = N;
		}
	}

	if (_Init && (*_Start->getConfiguration() != *Cs))
	{
		_Start = new Node(_Graph, Cs);
		_Graph->insertNode(_Start);
		_Graph->linkNode(_Start);
		b = true;
	}

	_Graph->getGraphStruct()->search_start = _Start->getNodeStruct();
	_q_start = Cs;
	return b;
}

//! Set the start and goal configuration
bool Planner::setGoal(shared_ptr<Configuration> Cg)
{
	bool b = false;
  
	if (ENV.getBool(Env::expandToGoal))
	{
		if (!_Init)
		{
			Node* N = _Graph->searchConf(*Cg);
			if (N == NULL)
			{
				_Goal = new Node(_Graph, Cg);
				_Graph->insertNode(_Goal);
				// Warning
				//				_Graph->linkNode(_Goal);
				b = true;
			}
			else
			{
				_Goal = N;
			}
		}

		if (_Init && (_Goal == NULL || (*_Goal->getConfiguration() != *Cg)))
		{
			_Goal = new Node(_Graph, Cg);
			_Graph->insertNode(_Goal);
			//			_Graph->linkNode(_Goal);
			b = true;
		}
		_Graph->getGraphStruct()->search_goal = _Goal->getNodeStruct();
	}

	else
	{
		_Goal = NULL;
	}

  _q_goal = Cg;
	return b;
}

bool Planner::getInitialized()
{
	return _Init;
}

void Planner::setInitialized(bool b)
{
	_Init = b;
}


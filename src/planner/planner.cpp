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
#include "Move3d-pkg.h"

using namespace std;
using namespace tr1;

extern void draw_opengl();

Planner::Planner() :
		  _stop_func(fct_stop),
		  _draw_func(draw_opengl),
		  _Start(NULL),
		  _Goal(NULL),
		  _Robot(NULL),
		  _Graph(NULL),
		  _Init(false)
{	
}

Planner::Planner(Robot* rob, Graph* graph) :
		  _stop_func(fct_stop),
		  _draw_func(draw_opengl),
		  _Start(NULL),
		  _Goal(NULL),
		  _Robot(rob),
		  _Graph(graph),
		  _Init(false)
{
}


Planner::~Planner()
{
}

bool Planner::trajFound()
{
	bool inSameCompco  = (_Goal ? _Start->inSameComponent(_Goal) : false);
	bool isTheSame = _Start->equal(_Goal);
	
	return (!isTheSame) && inSameCompco;
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

Node* Planner::getStart()
{
	return _Start;
}

Node* Planner::getGoal()
{
	return _Goal;
}

bool Planner::getInit()
{
	return _Init;
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

/*!
 * Set the start and goal configuration
 */
bool Planner::setStart(shared_ptr<Configuration> Cs)
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
	
	return b;
}

/*!
 * Set the start and goal configuration
 */
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

	return b;
}

void Planner::setInit(bool b)
{
	_Init = b;
}


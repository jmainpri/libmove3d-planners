/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef P3D_GREEDY_PROTO_HH
#define P3D_GREEDY_PROTO_HH

#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/Variants//Transition-RRT.hpp"

#include "API/Roadmap/graph.hpp"

/**
  * Planner based on trajectory optimization
  */
namespace Move3D
{

class GreedyCost {

public:

	GreedyCost(graph* G, int (*stop_func)(), void (*draw_func)());
	~GreedyCost();

	bool run();

	void createVectorLocalPath();

	int strait(Node& expansionNode,
             confPtr_t directionConfig,
		     Node* directionNode,
		     Env::expansionMethod method,
		     bool toGoal);

	void optimizePhaze();
	void optimizeLinear();
	void shortCutLinear();
	bool checkStopConditions();

	int getOptimFail() { return nb_OptimFail; }
	int getOptimSuccess() {return nb_OptimSuccess; }

private:

	int mConsecutiveFailures;

	int (*_stop_func)();
	void (*_draw_func)();

	Robot* mRobot;
	Graph* mGraph;

	Node* mStart;
	Node* mGoal;

	RRTExpansion* Expansion;
	RRT* Diffusion;
    Move3D::CostOptimization* optimTrj;

	int nb_Loops;
	int nb_LocalPaths;
	int nb_CostCompare;
	int nb_CObstFail;

	int nb_OptimSuccess;
	int nb_OptimFail;

};

extern bool p3d_RunGreedyCost(graph* GraphPt, int (*fct_stop)(void),
		void (*fct_draw)(void));

}

#endif

#ifndef P3D_GREEDY_PROTO_HH
#define P3D_GREEDY_PROTO_HH

#include "API/planningAPI.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/Variants//Transition-RRT.hpp"

/**
  * Planner based on trajectory optimization
  */
class GreedyCost {

public:

	GreedyCost(graph* G, int (*stop_func)(), void (*draw_func)());
	~GreedyCost();

	bool run();

	void createVectorLocalPath();

	int strait(Node& expansionNode,
			MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig,
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
	API::CostOptimization* optimTrj;

	int nb_Loops;
	int nb_LocalPaths;
	int nb_CostCompare;
	int nb_CObstFail;

	int nb_OptimSuccess;
	int nb_OptimFail;

};

extern bool p3d_RunGreedyCost(graph* GraphPt, int (*fct_stop)(void),
		void (*fct_draw)(void));

#endif

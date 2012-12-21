/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include "../p3d/env.hpp"

#include "planEnvironment.hpp"
#include "plannerFunctions.hpp"
#include "replanningSimulators.hpp"
#include "../p3d/env.hpp"

#include "planner/planner.hpp"

#include "planner/PRM/PRM.hpp"
#include "planner/PRM/Visibility.hpp"
#include "planner/PRM/ACR.hpp"
#include "planner/PRM/PerturbationRoadmap.hpp"

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/EST.hpp"
#include "planner/Diffusion/Variants/Transition-RRT.hpp"
#include "planner/Diffusion/Variants/ManhattanLike-RRT.hpp"
#include "planner/Diffusion/Variants/Multi-RRT.hpp"
#include "planner/Diffusion/Variants/Multi-TRRT.hpp"
#include "planner/Diffusion/Variants/Threshold-RRT.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_costspace.hpp"
#include "HRI_costspace/RRT/HRICS_rrt.hpp"
#include "HRI_costspace/RRT/HRICS_rrtPlan.hpp"
#endif

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"
#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace std;
using namespace tr1;

// ---------------------------------------------------------------------------------
// Exctracted trajectory Id (used for UI)
// ---------------------------------------------------------------------------------
unsigned int trajId = 0;

// ---------------------------------------------------------------------------------
// Last trajectory
// ---------------------------------------------------------------------------------
API::Trajectory last_traj;

API::Trajectory p3d_get_last_trajectory()
{
  return last_traj;
}

// ---------------------------------------------------------------------------------
// Run Id (used for multi-run)
// ---------------------------------------------------------------------------------
unsigned int runId = 0;

void p3d_planner_functions_set_run_id( unsigned int idRun )
{
	runId = idRun;
}

unsigned int p3d_planner_functions_get_run_id()
{
  return runId;
}

// ---------------------------------------------------------------------------------
// RRT statistcis
// ---------------------------------------------------------------------------------
RRTStatistics rrt_statistics;

void p3d_get_rrt_statistics( RRTStatistics& stat )
{
  stat = rrt_statistics;
}

// ---------------------------------------------------------------------------------
// Trajectory statistcis
// ---------------------------------------------------------------------------------
TrajectoryStatistics traj_statistics;

void p3d_get_traj_statistics( TrajectoryStatistics& stat )
{
  stat = traj_statistics;
}

// ---------------------------------------------------------------------------------
// Set back costspace after RRT
// ---------------------------------------------------------------------------------
static bool set_costspace=false;

// ---------------------------------------------------------------------------------
// Extract Traj
// ---------------------------------------------------------------------------------
p3d_traj* p3d_extract_traj(bool is_traj_found, int nb_added_nodes, Graph* graph, confPtr_t q_source, confPtr_t q_target) 
{
  cout << "--- p3d_extract_traj ---------------------------" << endl;
  API::Trajectory* traj = NULL;
  Robot* rob = graph->getRobot();
  
  // If traj is found, extract it from the graph
  if (/*rrt->trajFound()*/ is_traj_found ) 
  {
    // Case of direct connection
    if( nb_added_nodes == 2 )
    {
      vector<confPtr_t> configs;
      
      configs.push_back( q_source );
      configs.push_back( q_target );
      
      cout << "Creating trajectory from two confgurations" << endl;
      traj = new API::Trajectory( configs );
    }
    else 
    {
      //traj = graph->extractBestTraj( q_source, q_target ); // Old extract
      traj = graph->extractAStarShortestPathsTraj( q_source, q_target );
    }
  }

  trajId++;
  
  cout << "compute traj cost" << endl;
  
  // Return trajectory or NULL if falses
  if (traj) 
  {
    //traj->costDeltaAlongTraj();
    if( PlanEnv->getBool(PlanParam::trajComputeCostAfterPlannif) )
    {
      bool is_cost_space = ENV.getBool(Env::isCostSpace);
      ENV.setBool(Env::isCostSpace, true);
      
      traj->resetCostComputed();
      traj->costStatistics( traj_statistics );
      
      cout << "--- stats on traj ---" << endl;
      cout << " length = " << traj_statistics.length << endl;
      cout << " max = " << traj_statistics.max << endl;
      cout << " average = " << traj_statistics.average << endl;
      cout << " integral = " << traj_statistics.integral << endl;
      cout << " mecha_work = " << traj_statistics.mecha_work << endl;
      cout << "---------------------" << endl;
      
      // Compute traj cost
      rrt_statistics.cost = traj->cost();
      cout << "is_cost_space : " << ENV.getBool(Env::isCostSpace) << " , traj_cost : " << rrt_statistics.cost << endl ;
      ENV.setBool(Env::isCostSpace, is_cost_space);
    }
    
    last_traj = *traj;
    
    p3d_traj* result = traj->replaceP3dTraj(NULL); 
    rob->getRobotStruct()->tcur = result;
    
    if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
    {
      g3d_draw_allwin_active();
    }
    
    cout << "result->nlp = " << result->nlp << endl;
    cout << "result->range_param = " << result->range_param << endl;
    
    char trajName[] = "Specific";
		g3d_add_traj( trajName, trajId, rob->getRobotStruct(), rob->getRobotStruct()->tcur );
    
    // Prevent segfault if p3d graph deleted outside
    delete traj;
    return result;
  }
  else 
  {
    cout << __FILE__ << " , " << __func__ << " : No traj found" << endl;
    return NULL;
  }
}

// ---------------------------------------------------------------------------------
// Allocates an RRT depending on env variables
// ---------------------------------------------------------------------------------
RRT* p3d_allocate_rrt(Robot* rob,Graph* graph)
{
  RRT* rrt;
  
  if(ENV.getBool(Env::isManhattan))
	{
		rrt = new ManhattanLikeRRT(rob,graph);
	}
	else if(ENV.getBool(Env::isMultiRRT) && ENV.getBool(Env::isCostSpace)  )
	{
		rrt = new MultiTRRT(rob,graph);
	}
	else if(ENV.getBool(Env::isMultiRRT))
	{
		rrt = new MultiRRT(rob,graph);
	}
#ifdef HRI_COSTSPACE
	else if(ENV.getBool(Env::HRIPlannerWS) && ENV.getBool(Env::HRIPlannerTRRT))
	{
		rrt = new HRICS_RRT(rob,graph);
	}
	else if(ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::HRIPlannerTRRT))
	{
		rrt = new HRICS_RRTPlan(rob,graph);
	}
#endif
	else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::costThresholdRRT) )
	{
		rrt = new ThresholdRRT(rob,graph);
	}
	else if(ENV.getBool(Env::isCostSpace) && PlanEnv->getBool(PlanParam::starRRT) )
	{
		rrt = new StarRRT(rob,graph);
	}
	else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::useTRRT) )
	{
		rrt = new TransitionRRT(rob,graph);
	}
	else
	{
		if( ENV.getBool(Env::isCostSpace) && (!ENV.getBool(Env::useTRRT)) )
		{
			ENV.setBool(Env::isCostSpace,false);
      set_costspace = true;
		}
		
		rrt = new RRT(rob,graph);
	}
  
  return rrt;
}

// ---------------------------------------------------------------------------------
// Planner function (RRT) for connection with Manipulation planner
// ---------------------------------------------------------------------------------
p3d_traj* p3d_planner_function(p3d_rob* robotPt, configPt qs, configPt qg)
{
  cout << "* PLANNING ***************" << endl;
  ChronoTimeOfDayOn();
  
  // Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName( robotPt->name );
  confPtr_t q_source( new Configuration(rob,qs) );
  confPtr_t q_target( new Configuration(rob,qg) );
  
  // Allocate the p3d_graph if does't exist
  // Delete graph if it exists, creates a new graph , Allocate RRT
  delete API_activeGraph;
  Graph* graph = API_activeGraph = new Graph(rob);
  
  // Delete last global planner 
  if( global_Move3DPlanner ) delete global_Move3DPlanner;
  
	// Main Run functions of all RRTs
  // All RRTs are initilized with init and run here
  int nb_added_nodes = 0;
  RRT* rrt = p3d_allocate_rrt(rob,graph);
  global_Move3DPlanner = rrt;
  nb_added_nodes += rrt->setInit(q_source);
	nb_added_nodes += rrt->setGoal(q_target);
	nb_added_nodes += rrt->init();
  rrt->setInitialized(true);
  rrt->setRunId( runId );
	
  nb_added_nodes += rrt->run();
  
  if ((rrt->getNumberOfExpansion() - rrt->getNumberOfFailedExpansion() + rrt->getNumberOfInitialNodes()) 
      != graph->getNumberOfNodes() ) 
  {
    cout << "Error in RRT nb of expansion ";
    cout << "compared to initial nb of nodes in graph total nb of nodes in the graph" << endl;
	}
	
	graph->getGraphStruct()->totTime = graph->getGraphStruct()->rrtTime;
  
  if( ENV.getBool(Env::drawGraph) && global_rePlanningEnv ) 
  {
    global_rePlanningEnv->store_graph_to_draw(*graph);
  }
  
  double time;
  ChronoTimeOfDayTimes(&time);
  cout << "Time before trajectory extraction :"  << time << " sec." << endl;
  
  // Extract the trajectory if one exists, else return NULL
  p3d_traj* traj = p3d_extract_traj(rrt->trajFound(), nb_added_nodes, graph, q_source, q_target);
  
  ChronoTimeOfDayTimes(&time);
  ChronoTimeOfDayOff();
  
  cout << "** ** --------------------------" << endl; 
  
  if( traj ) { 
    cout << "Success" << endl;
  }
  else {
    cout << "Fail" << endl;
  }
  
  cout << "TIME ="  << time << " sec." << endl;
  cout << "NB NODES " << graph->getNumberOfNodes() << endl;
  cout << "NB EXPANSION " << rrt->getNumberOfExpansion() << endl;
  cout << "** ** --------------------------" << endl;
  
  rrt_statistics.runId = rrt->getRunId();
  rrt_statistics.succeeded = (traj!=NULL);
  rrt_statistics.time = time;
  //rrt_statistics.cost = 0.0;
  rrt_statistics.nbNodes = graph->getNumberOfNodes();
  rrt_statistics.nbExpansions = rrt->getNumberOfExpansion();
  
  if( set_costspace )
    ENV.setBool(Env::isCostSpace,true);
  
  return traj;
}

p3d_traj* pathPt=NULL;

// ---------------------------------------------------------------------------------
// Smoothing function (Shortcut) for connection with Manipulation planner
// ---------------------------------------------------------------------------------
void p3d_smoothing_function( p3d_rob* robotPt, p3d_traj* traj, int nbSteps, double maxTime )
{
  cout << "** SMOOTHING ***************" << endl;
  
  if( robotPt == NULL )
  {
    cout << "robot not defined in " << __func__ << endl;
    return;
  }
  
  if( maxTime != -1 )
    PlanEnv->setDouble( PlanParam::timeLimitSmoothing, maxTime );

  Robot* rob = global_Project->getActiveScene()->getRobotByName( robotPt->name );  

  API::Trajectory t;
  
  if(PlanEnv->getBool(PlanParam::withDeformation) || PlanEnv->getBool(PlanParam::withShortCut) )
  {
    double optTime = 0.0;
    
    if( traj == NULL ) {
      cout << "trajectory NULL in " << __func__ << endl;
      return;
    }
    cout << "traj->nlp = " << traj->nlp << endl;
    cout << "traj->range_param = " << traj->range_param << endl;
    
    API::CostOptimization optimTrj( rob, traj );
    
    optimTrj.setRunId( runId );
#ifdef QT_LIBRARY
    optimTrj.setContextName( ENV.getString(Env::nameOfFile).toStdString() );
#endif
    
    if(PlanEnv->getBool(PlanParam::withDeformation))
    {
      optimTrj.resetCostComputed();
      optimTrj.runDeformation( nbSteps , runId );
      optTime += optimTrj.getTime();
    }
    
    if(PlanEnv->getBool(PlanParam::withShortCut))
    {
      optimTrj.resetCostComputed();
      optimTrj.runShortCut( nbSteps, runId );
      optTime += optimTrj.getTime();
    }
    
    optimTrj.replaceP3dTraj();
    optimTrj.resetCostComputed();
    cout << "optimTrj.getRangeMax() : " << optimTrj.getRangeMax()  << endl;
    
    if( PlanEnv->getBool(PlanParam::trajComputeCostAfterPlannif) )
    {
      optimTrj.costStatistics( traj_statistics );
      
      cout << "--- stats on traj ---" << endl;
      cout << " length = " << traj_statistics.length << endl;
      cout << " max = " << traj_statistics.max << endl;
      cout << " average = " << traj_statistics.average << endl;
      cout << " integral = " << traj_statistics.integral << endl;
      cout << " mecha_work = " << traj_statistics.mecha_work << endl;
      cout << "---------------------" << endl;
    }
    
    last_traj = API::Trajectory(optimTrj);
  }
  
  if( PlanEnv->getBool( PlanParam::withStomp ) )
  {
    traj_optim_runStompNoReset( runId );
    
    t = API::Trajectory( rob, rob->getRobotStruct()->tcur );
    t.resetCostComputed();
    
    if( PlanEnv->getBool(PlanParam::trajComputeCostAfterPlannif) )
    {
      t.costStatistics( traj_statistics );
      
      cout << "--- stats on traj ---" << endl;
      cout << " length = " << traj_statistics.length << endl;
      cout << " max = " << traj_statistics.max << endl;
      cout << " average = " << traj_statistics.average << endl;
      cout << " integral = " << traj_statistics.integral << endl;
      cout << " mecha_work = " << traj_statistics.mecha_work << endl;
      cout << "---------------------" << endl;
    }
  }
  
  pathPt = robotPt->tcur;
}

// ---------------------------------------------------------------------------------
// set IK function
// ---------------------------------------------------------------------------------
configPt (*ext_generate_goal_configuration)();

void p3d_set_goal_solution_function( configPt (*fct)() )
{
  ext_generate_goal_configuration = fct;
}

// ---------------------------------------------------------------------------------
// Generate IK function
// ---------------------------------------------------------------------------------
bool p3d_generate_goal_soution( Configuration& q )
{
  if( ext_generate_goal_configuration != NULL )
  {
    configPt q_new = ext_generate_goal_configuration();
    
    q.Clear();
    
    if( q_new != NULL )
    {
      q.setConfiguration( q_new );
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------------
// Tree Planners
// ---------------------------------------------------------------------------------
int p3d_run_rrt(p3d_rob* robotPt)
{	
  Robot* rob = global_Project->getActiveScene()->getRobotByName( robotPt->name );
  
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  p3d_traj* path = p3d_planner_function( rob->getRobotStruct(), q_source->getConfigStruct(), q_target->getConfigStruct() );
  
  if( path != NULL && 
     !PlanEnv->getBool(PlanParam::stopPlanner) && 
     PlanEnv->getBool(PlanParam::withSmoothing) )
  {
    double max_iteration = PlanEnv->getInt(PlanParam::smoothMaxIterations);
    double max_time = PlanEnv->getDouble( PlanParam::timeLimitSmoothing );
    
    p3d_smoothing_function(rob->getRobotStruct(), path, max_iteration, max_time);
  }
  
  return (path != NULL);
}


bool p3d_run_est(p3d_rob* robotPt)
{	
	// Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  // Allocate the p3d_graph if does't exist
  // Removes graph if it exists, creates a new graph , Allocate RRT
  Graph* graph = API_activeGraph =  new Graph(rob);
	
	printf("------------- Running EST --------------\n");
#ifdef LIST_OF_PLANNERS
	RRT* rrt = (RRT*)plannerlist[0];
#else
	EST* est = new EST(rob,graph);
#endif
    est->setInit(q_source);
    est->setGoal(q_target);
	int nb_added_nodes = est->init();
	
	graph = est->getActivGraph();
	
	cout <<"nb nodes " << graph->getNumberOfNodes() << endl;
	
	nb_added_nodes += est->run();
	ENV.setBool(Env::isRunning,false);
	
	printf("nb added nodes %d\n", nb_added_nodes);
	printf("nb nodes %zu\n",graph->getNodes().size());
	bool res = est->trajFound();
	
#ifndef LIST_OF_PLANNERS
	delete est;
#endif
	
	return res;
}


// ---------------------------------------------------------------------------------
// PRMs
// ---------------------------------------------------------------------------------
int p3d_run_prm(p3d_rob* robotPt)
{	
  // Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  // Allocate the p3d_graph if does't exist
  Graph* graph;
  if(!API_activeGraph)
  {
    API_activeGraph =  new Graph(rob);
  }
  graph = API_activeGraph;

	cout << "Initializing PRM " << endl;
  int nb_added_nodes=0;
  PRM* prm = new PRM(rob, graph);
	nb_added_nodes = prm->init();
	nb_added_nodes += prm->run();
	
	cout << "nb added nodes " << nb_added_nodes << endl;
	cout << "nb nodes " << graph->getNumberOfNodes() << endl;
  
  p3d_extract_traj( prm->trajFound(), nb_added_nodes, graph, q_source, q_target);
	delete prm;
	return nb_added_nodes;
}

int p3d_run_perturb_prm(p3d_rob* robotPt)
{
  if( !PlanEnv->getBool(PlanParam::withCurrentTraj) )
  {
    ENV.setBool(Env::isCostSpace,false);
    ENV.setBool(Env::expandToGoal,true);
    ENV.setBool(Env::biDir,true);
    
    p3d_run_rrt( robotPt );
    
    ENV.setBool(Env::biDir,false);
    ENV.setBool(Env::isCostSpace,true);
    ENV.setBool(Env::expandToGoal,false);
  }
  
  // Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  // Allocate the p3d_graph if does't exist
  // Removes graph if it exists, creates a new graph , Allocate RRT
  Graph* graph = API_activeGraph =  new Graph(rob);
	
	cout << "Initializing PRM " << endl;
  int nb_added_nodes=0;
  PerturbationRoadmap* prm = new PerturbationRoadmap(rob, graph);
	nb_added_nodes = prm->init();
	nb_added_nodes += prm->run();
  
	cout << "nb added nodes " << nb_added_nodes << endl;
	cout << "nb nodes " << graph->getNumberOfNodes() << endl;
  
  p3d_extract_traj( prm->trajFound(), nb_added_nodes, graph, q_source, q_target);
	delete prm;
	return nb_added_nodes;
}

int p3d_run_vis_prm(p3d_rob* robotPt)
{	
  // Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  // Allocate the p3d_graph if does't exist
  // Removes graph if it exists, creates a new graph , Allocate RRT
  Graph* graph = API_activeGraph =  new Graph(rob);
	
  int nb_added_nodes=0;
	Vis_PRM* vprm = new Vis_PRM(rob,graph);
	nb_added_nodes = vprm->init();
	nb_added_nodes += vprm->run();
	
	cout << "nb added nodes " << nb_added_nodes << endl;
	cout << "nb nodes " << graph->getNumberOfNodes() << endl;
  
  p3d_extract_traj( vprm->trajFound(), nb_added_nodes, graph, q_source, q_target);
	delete vprm;
	return nb_added_nodes;
}

int p3d_run_acr(p3d_rob* robotPt)
{	
  // Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  // Allocate the p3d_graph if does't exist
  // Removes graph if it exists, creates a new graph 
  Graph* graph = API_activeGraph =  new Graph(rob);
	
  int nb_added_nodes=0;
	ACR* acr = new ACR(rob, graph);
	nb_added_nodes = acr->init();
	nb_added_nodes += acr->run();
	
	cout << "nb added nodes " << nb_added_nodes << endl;
	cout << "nb nodes " << graph->getNumberOfNodes() << endl;
  
  p3d_extract_traj( acr->trajFound(), nb_added_nodes, graph, q_source, q_target);
	delete acr;
	return nb_added_nodes;
}

/**
 * Function To replace p3d_Learn in Case of C++ API Use
 */
void p3d_learn_cxx(int NMAX,
									 int (*fct_stop)(void), void (*fct_draw)(void)) {
	p3d_graph *G;
	int inode, ADDED;
	double tu, ts;
	int nbInitGraphNodes, nbGraphNodes;
	
	ChronoOn();
	
	if (!XYZ_GRAPH) G = p3d_create_graph();
	else           G = XYZ_GRAPH;
	/*debut modif fpilarde*/
	inode = 0;
  
  p3d_rob* robotPt = XYZ_GRAPH->rob;
	
#ifdef P3D_PLANNER
	p3d_set_planning_type(P3D_GLOBAL);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
	ENV.setBool(Env::expandToGoal,false);
	
	if (p3d_get_MOTION_PLANNER() != P3D_DIFFUSION) {
		//while (inode < NMAX) {
		/* Call basic PRM or Visibility method */
		switch (p3d_get_MOTION_PLANNER()) {
				
			case 1:
				cout << "CXX_PLANNER c++ API : p3d_run_prm" << endl;
				ADDED = p3d_run_prm(robotPt);
				break;
				
			case 2:
				cout << "CXX_PLANNER c++ API : p3d_run_vis_prm" << endl;
				ADDED = p3d_run_vis_prm(robotPt);
				break;
				
			case P3D_ALL_PRM:
				cout << "CXX_PLANNER c++ API : p3d_run_acr" << endl;
				ADDED = p3d_run_acr(robotPt);
				break;
				
			default:
				PrintInfo(("p3d_learn : ERREUR : pas de planificateur global...\n"));
				return;
		}
		inode += ADDED;
	}
	
	else {
		nbInitGraphNodes = G->nnode;
		ADDED = p3d_run_rrt(robotPt);
		nbGraphNodes = G->nnode;
		inode  = nbGraphNodes - nbInitGraphNodes;
	}
	
#ifdef P3D_PLANNER
	p3d_set_planning_type(P3D_NONE);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
	PrintInfo(("Pour la creation de %d noeuds : ", inode));
	
	ChronoTimes(&tu, &ts);
	G->time = G->time + tu;
	
#ifdef P3D_PLANNER
	/* When retrieving statistics;
	 Commit Jim; date: 01/10/2008 */
	if(getStatStatus()){
		G->stat->preTime += tu;
	}
#endif
	
	ChronoPrint("");
	ChronoOff();
	
	MY_ALLOC_INFO("After p3d_learn");
	p3d_print_info_graph(G);
}

/**
 * Function To replace p3d_specific_learn in Case of C++ API Use
 */
int p3d_specific_learn_cxx(double *qs, double *qg, int *iksols, int *iksolg,
													 int (*fct_stop)(void), void (*fct_draw)(void)) {
	
	p3d_graph *G;
	
	int       inode = 0, fail = 1, ADDED;
	double    tu, ts;
	
	int nbInitGraphNodes, nbGraphNodes;
#ifdef ENERGY
	int n_coldeg, icoldeg;
	double *coldeg_qs;
#endif
	/* Avoid some stupid errors */
	if (qs == NULL) {
		PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration initiale\n"));
		return(FALSE);
	}
	
	if ((qg == NULL) && (ENV.getBool(Env::expandToGoal) == true)) {
		PrintInfo(("p3d_specific_learn : ERREUR : pas de configuration finale\n"));
		return(FALSE);
	}
	
	ENV.setBool(Env::expandToGoal,true);
	
	ChronoOn();
	
	if (!XYZ_GRAPH)  G = p3d_create_graph();
	else            G = XYZ_GRAPH;
  
   p3d_rob* robotPt = XYZ_GRAPH->rob;
	
	if (p3d_GetIsWeightedChoice() == TRUE) {
		p3d_init_root_weight(G);
	}
	if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH) {
		p3d_init_pb_variables(G);
		
#ifdef ENERGY
		if (p3d_get_MOTION_PLANNER() ==  BIO_COLDEG_RRT) {
			n_coldeg = bio_get_num_collective_degrees();
			// init coldeg_q in Ns
			coldeg_qs = bio_alloc_coldeg_config(n_coldeg);
			for (icoldeg = 0; icoldeg < n_coldeg; icoldeg++) {
				coldeg_qs[icoldeg] = 0.0;
			}
			bio_copy_coldeg_q_in_N(Ns, coldeg_qs, n_coldeg);
			bio_destroy_coldeg_config(coldeg_qs, n_coldeg);
			// WARNING : currently Ng is not considered !!!
		}
#endif
		
		
	}
	ADDED = TRUE;
	if (p3d_get_MOTION_PLANNER() != P3D_DIFFUSION) {
		/* While solution does not exists, insert new nodes with basic PRM or Visibility or RRT */
		//     while ((Ns->numcomp != Ng->numcomp) && !p3d_compco_linked_to_compco(Ns->comp, Ng->comp)) {  modif fpilarde
		switch (p3d_get_MOTION_PLANNER()) {
				
			case P3D_BASIC:
				cout << "CXX_PLANNER c++ API : p3d_run_prm" << endl;
				ADDED = p3d_run_prm(robotPt);
				break;
				
			case P3D_ISOLATE_LINKING:
				cout << "CXX_PLANNER c++ API : p3d_run_vis_prm" << endl;
				ADDED = p3d_run_vis_prm(robotPt);
				break;
				
			case P3D_ALL_PRM:
				cout << "CXX_PLANNER c++ API : p3d_run_acr" << endl;
				ADDED = p3d_run_acr(robotPt);
				break;
				
#ifdef ENERGY
			case BIO_COLDEG_RRT:
				ADDED = bio_expand_coldeg_rrt(G, fct_stop);
				break;
#endif
			default:
				PrintInfo(("p3d_specific_learn : ERREUR : pas de planificateur global...\n"));
				return(FALSE);
		}
		
		inode += ADDED;
		
	} else {
		nbInitGraphNodes = G->nnode;
		cout << "CXX_PLANNER c++ API : p3d_run_rrt" << endl;
		ADDED = p3d_run_rrt(robotPt);
		nbGraphNodes = G->nnode;
		inode  = nbGraphNodes - nbInitGraphNodes;
	}
	
#ifdef P3D_PLANNER
  p3d_set_planning_type(P3D_NONE);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
  
	PrintInfo(("Pour la creation de %d noeuds : ", inode));
	ChronoPrint("");
	
	ChronoTimes(&tu, &ts);
	G->time = G->time + tu;
#ifdef P3D_PLANNER
	/* When retrieving statistics;
	 Commit Jim; date: 01/10/2008 */
	if(getStatStatus()){
		G->stat->preTime += tu;
	}
	ChronoOff();
#endif
	
	p3d_print_info_graph(G);
	MY_ALLOC_INFO("After p3d_specific_learn");
	if (p3d_get_saveInfoInFile()) {
		//    save_infos_in_file(G, ADDED);
	}
	
	PrintInfo(("\n"));
	return(ADDED || !fail);
}

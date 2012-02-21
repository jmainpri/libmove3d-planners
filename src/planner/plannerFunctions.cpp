/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include "../p3d/env.hpp"

#include "planEnvironment.hpp"
#include "plannerFunctions.hpp"
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
// Run number that is counting planner run
// ---------------------------------------------------------------------------------
unsigned int runNum = 0;

// ---------------------------------------------------------------------------------
// Run Id that is used in multiRun
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
// Delete graph if it exists
// ---------------------------------------------------------------------------------
void p3d_delete_graph( p3d_graph* &G )
{
  if (API_activeGraph) 
  {
    p3d_graph* graph =  API_activeGraph->getRobot()->getRobotStruct()->GRAPH;
    
    delete API_activeGraph;
    API_activeGraph = NULL;
    cerr << "Delete C++ API Graph" << endl;
    
    if(graph!=G)
    {
      if(!p3d_del_graph(G))
      {
        cerr << "Graph allready deleded" << endl;
      }
    }
  }
  else
  {
    if(!p3d_del_graph(G))
    {
      cerr << "Graph allready deleded" << endl;
    }
  }
  
  G = NULL;
  
  if( !p3d_del_graph(XYZ_GRAPH) )
  {
    cerr << "XYZ_GRAPH allready deleted" << endl;
  }
  XYZ_GRAPH = NULL;
  
  cerr << "G = " << G << endl;
  cerr <<  "XYZ_GRAPH = " << XYZ_GRAPH << endl;
}

// ---------------------------------------------------------------------------------
// Extract Traj
// ---------------------------------------------------------------------------------
p3d_traj* p3d_extract_traj(bool is_traj_found, int nb_added_nodes, Graph* graph, confPtr_t q_source, confPtr_t q_target) 
{
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
      cout << "Extract graph to traj" << endl;
      traj = graph->extractBestTraj( q_source, q_target );
    }
  }

  runNum++;
  
  // Return trajectory or NULL if falses
  if (traj) 
  {
    p3d_traj* result = traj->replaceP3dTraj(NULL); 
    g3d_draw_allwin_active();
    cout << "result->nlp = " << result->nlp << endl;
    cout << "result->range_param = " << result->range_param << endl;
    
    rob->getRobotStruct()->tcur = result;
    char trajName[] = "Specific";
		g3d_add_traj( trajName, runNum, rob->getRobotStruct(), rob->getRobotStruct()->tcur );
    
    // Prevent segfault if p3d graph deleted outside
    delete traj;
    
    ChronoTimeOfDayTimes(&(graph->getGraphStruct()->rrtTime));
    
    // Debug
    cout << "** ** --------------------------" << endl;
    cout << "TIME ="  << graph->getGraphStruct()->rrtTime << endl;
    cout << "NB NODES " << (int)graph->getNodes().size() << endl;
    cout << "** ** --------------------------" << endl;
    
    ChronoTimeOfDayOff();
    return result;
  }
  else 
  {
    cout << __FILE__ << " , " << __func__ << " : No traj found" << endl;
    //rob->getRobotStruct()->tcur = NULL;
    //if( graph == API_activeGraph ) API_activeGraph = NULL;
    //delete graph;
    
    ChronoTimeOfDayPrint("");
    ChronoTimeOfDayOff();
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
	else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::costStarRRT) )
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
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source( new Configuration(rob,qs) );
  confPtr_t q_target( new Configuration(rob,qg) );
  
  // Allocate the p3d_graph if does't exist
  // Delete graph if it exists, creates a new graph , Allocate RRT
  p3d_graph* GraphPt = robotPt->GRAPH;
  p3d_delete_graph( GraphPt );
  Graph* graph = API_activeGraph =  new Graph(rob,GraphPt);
  
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
	
  nb_added_nodes += rrt->run();
  
  if ((rrt->getNumberOfExpansion() - rrt->getNumberOfFailedExpansion() + rrt->getNumberOfInitialNodes()) 
      != graph->getNumberOfNodes() ) 
  {
    cout << "----------------------" << endl;
    cout << "Nb of nodes differ from number of expansion succes " << endl;
    cout << "nb added nodes " << nb_added_nodes << endl;
    cout << "nb nodes " << graph->getGraphStruct()->nnode << endl;
    cout << " - m_nbExpansion = " << rrt->getNumberOfExpansion() << endl;
    cout << " - m_nbInitNodes = " << rrt->getNumberOfInitialNodes() << endl;
    cout << " - m_nbExpansion + m_nbInitNodes - m_nbExpansionFailed  =  " << (rrt->getNumberOfExpansion() + rrt->getNumberOfInitialNodes() - rrt->getNumberOfFailedExpansion() ) << endl;
    cout << " - _Graph->getNumberOfNodes() = " << graph->getNumberOfNodes() << endl;
	}
	
	graph->getGraphStruct()->totTime = graph->getGraphStruct()->rrtTime;
  
  // Extracj the trajectory if one exists, else return NULL
  p3d_traj* traj = p3d_extract_traj(rrt->trajFound(), nb_added_nodes, graph, q_source, q_target);
  return traj;
}

p3d_traj* pathPt=NULL;

// ---------------------------------------------------------------------------------
// Smoothing function (Shortcut) for connection with Manipulation planner
// ---------------------------------------------------------------------------------
void p3d_smoothing_function(p3d_rob* robotPt, p3d_traj* traj, int nbSteps, double maxTime)
{
  cout << "* SMOOTHING ***************" << endl;
  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  cout << "traj->nlp = " << traj->nlp << endl;
  cout << "traj->range_param = " << traj->range_param << endl;
  
  API::CostOptimization optimTrj(rob,traj);
  
#ifdef QT_LIBRARY
  optimTrj.setContextName( ENV.getString(Env::nameOfFile).toStdString() );
#endif
  
  double optTime = 0.0;
  if(PlanEnv->getBool(PlanParam::withDeformation))
  {
    //ENV.setBool(Env::FKShoot,true);
    optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , runId );
    //ENV.setBool(Env::FKShoot,false);
    optTime += optimTrj.getTime();
  }
  
  optimTrj.resetCostComputed();
  
  //XYZ_GRAPH->rrtCost2 = optimTrj.cost();
  
  if(PlanEnv->getBool(PlanParam::withShortCut))
  {
    optimTrj.runShortCut( ENV.getInt(Env::nbCostOptimize) , runId );
    optTime += optimTrj.getTime();
  }
  
  double dmax = global_Project->getActiveScene()->getDMax();
  double range = optimTrj.getRangeMax();
  
  cout << "optimTrj range is : " << range << endl;
  
  bool doCutInsmallLP = false;
  
  if (doCutInsmallLP) {
    // Cut localpaths in small localpaths
    const unsigned int nLP_max = 20; 
    unsigned int nLP = optimTrj.getNbOfPaths();
    unsigned int nLP_toCut = floor( range / (8*dmax));
    
    if ( (nLP_toCut<nLP_max) && (nLP_toCut>nLP) && (nLP_toCut>1) )
    {
      cout << "Cut the traj in several local paths" << endl;
      optimTrj.cutTrajInSmallLP( nLP_toCut );
    }
  }
  
  // Replace current trajectory
  optimTrj.replaceP3dTraj();
  optimTrj.resetCostComputed();
  
  pathPt = robotPt->tcur;
  
  //XYZ_GRAPH->totTime += optTime;
  
  cout << "After optim rounds cost : " << optimTrj.cost() << endl;
  cout << "Nb of localpaths : " << robotPt->tcur->nlp << endl;
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
    p3d_smoothing_function(rob->getRobotStruct(), path, 100, 4.0);
  }
  
  return true;
}


bool p3d_run_est(p3d_rob* robotPt)
{	
	// Gets the robot pointer and the 2 configurations
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  confPtr_t q_source = rob->getInitialPosition();
  confPtr_t q_target = rob->getGoTo();
  
  // Allocate the p3d_graph if does't exist
  // Removes graph if it exists, creates a new graph , Allocate RRT
  p3d_graph* GraphPt = robotPt->GRAPH;
  p3d_delete_graph( GraphPt );
  Graph* graph = API_activeGraph =  new Graph(rob,GraphPt);
	
	printf("------------- Running EST --------------\n");
#ifdef LIST_OF_PLANNERS
	RRT* rrt = (RRT*)plannerlist[0];
#else
	EST* est = new EST(rob,graph);
#endif
	
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
  // Removes graph if it exists, creates a new graph , Allocate RRT
  p3d_graph* GraphPt = robotPt->GRAPH;
  p3d_delete_graph( GraphPt );
  Graph* graph = API_activeGraph =  new Graph(rob,GraphPt);
	
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
  p3d_graph* GraphPt = robotPt->GRAPH;
  p3d_delete_graph( GraphPt );
  Graph* graph = API_activeGraph =  new Graph(rob,GraphPt);
	
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
  p3d_graph* GraphPt = robotPt->GRAPH;
  p3d_delete_graph( GraphPt );
  Graph* graph = API_activeGraph =  new Graph(rob,GraphPt);
	
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
  p3d_graph* GraphPt = robotPt->GRAPH;
  p3d_delete_graph( GraphPt );
  Graph* graph = API_activeGraph =  new Graph(rob,GraphPt);
	
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

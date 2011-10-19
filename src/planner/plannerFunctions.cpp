/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include "../p3d/env.hpp"

#include "planEnvironment.hpp"
#include "planners_cxx.hpp"
#include "plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#include "Diffusion/RRT-Variants/Threshold-RRT.hpp"
#include "Diffusion/RRT-Variants/Star-RRT.hpp"

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

void p3d_planner_functions_SetRunId( unsigned int idRun )
{
	runId = idRun;
}

unsigned int p3d_planner_functions_GetRunId()
{
  return runId;
}


// ---------------------------------------------------------------------------------
// Delete graph if it exists
// ---------------------------------------------------------------------------------
void delete_graph( p3d_graph* &G )
{
  if (API_activeGraph) 
  {
    delete API_activeGraph;
    API_activeGraph = NULL;
    cerr << "Delete C++ API Graph" << endl;
  }
  
  if( !p3d_del_graph(G) ) 
  {
    cerr << "Graph allready deleded" << endl;
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
// Allocates an RRT depending on env variables
// ---------------------------------------------------------------------------------
RRT* allocate_RRT(Robot* rob,Graph* graph)
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
p3d_traj* planner_Function(p3d_rob* robotPt, configPt qs, configPt qg)
{
//  fixAllJointsWithoutArm(robotPt,0);
  cout << "* PLANNING ***************" << endl;
  
  double ts;
  
  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  // Gets the 2 configurations
  shared_ptr<Configuration> q_Source( new Configuration(rob,qs) );
  shared_ptr<Configuration> q_Target( new Configuration(rob,qg) );
  
  // Allocate the graph if does't exist
  p3d_graph* GraphPt = robotPt->GRAPH;
  
  // Removes graph if it exists
  delete_graph( GraphPt );
    
  Graph* graph = 	API_activeGraph =  new Graph(rob,GraphPt);
  
  // Allocate RRT
  RRT* rrt = allocate_RRT(rob,graph);
  
	// Main Run functions of all RRTs
  // All RRTs are initilized with init and run here
  int nb_added_nodes = 0;
  
  nb_added_nodes += rrt->setInit(q_Source);
	nb_added_nodes += rrt->setGoal(q_Target);
  
	nb_added_nodes += rrt->init();
  
  rrt->setInitialized(true);
	
  nb_added_nodes += rrt->run();
	
	// Gets the graph pointer
	// in case it has been modified by the planner
	graph = rrt->getActivGraph();
	
	ChronoTimes(&(graph->getGraphStruct()->rrtTime), &ts);
	
  // Debug
  cout << "** ** --------------------------" << endl;
	cout << "TIME ="  << graph->getGraphStruct()->rrtTime << endl;
	cout << "NB NODES " << (int)graph->getNodes().size() << endl;
	cout << "** ** --------------------------" << endl;
  
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
  
  API::Trajectory* traj = NULL;
  
  // If traj is found
  // Extract it from the graph
  if (rrt->trajFound()) 
  {
    if( !ENV.getBool(Env::use_p3d_structures) )
    {
      // Export the graph when not using the classical p3d_sctructures
      // Copies the graph to a p3d_structured graph
//      cout << "Export the cpp graph to new graph" << endl;
      XYZ_GRAPH = robotPt->GRAPH = graph->exportCppToGraphStruct();
      
      // Case of direct connection
      if( nb_added_nodes == 2 )
      {
        vector < shared_ptr<Configuration> > configs;
        
        configs.push_back( q_Source );
        configs.push_back( q_Target );
        
        cout << "Creating trajectory from two confgurations" << endl;
        traj = new API::Trajectory( configs );
      }
      else 
      {
        Graph graphTraj( rob, XYZ_GRAPH );
        cout << "Extract graph to traj" << endl;
        traj = graphTraj.extractBestTraj(q_Source,q_Target);
      }
    }
    else
    {
      traj = graph->extractBestTraj(q_Source,q_Target);
    }
  }
  
  delete rrt;
  runNum++;
  
  // Return trajectory or NULL if falses
  if (traj) 
  {
    p3d_traj* result=NULL; 

    result = traj->replaceP3dTraj(result); 
    
//    cout << "result = " << result << endl;
    cout << "result->nlp = " << result->nlp << endl;
    cout << "result->range_param = " << result->range_param << endl;
    
    robotPt->tcur = result;
    char trajName[] = "Specific";
		g3d_add_traj( trajName , runNum ,robotPt , robotPt->tcur );
    delete traj;
    return result;
  }
  else 
  {
    rob->getRobotStruct()->tcur = NULL;
		cout << __FILE__ << " , " << __func__ << " : No traj found" << endl;
    return NULL;
  }
}

p3d_traj* pathPt=NULL;

// ---------------------------------------------------------------------------------
// Smoothing function (Shortcut) for connection with Manipulation planner
// ---------------------------------------------------------------------------------
void smoothing_Function(p3d_rob* robotPt, p3d_traj* traj, int nbSteps, double maxTime)
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
    ENV.setBool(Env::FKShoot,true);
    optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , runId );
    ENV.setBool(Env::FKShoot,false);
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
// Tree Planners
// ---------------------------------------------------------------------------------
int p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void))
{	
  Robot* rob = global_Project->getActiveScene()->getActiveRobot();
  
  shared_ptr<Configuration> q_source = rob->getInitialPosition();
  shared_ptr<Configuration> q_target = rob->getGoTo();
  
  p3d_traj* path = planner_Function(rob->getRobotStruct(), 
                                    q_source->getConfigStruct(),  
                                    q_target->getConfigStruct() );
  
  if( path != NULL && 
     !PlanEnv->getBool(PlanParam::stopPlanner) &&
     PlanEnv->getBool(PlanParam::withSmoothing) )
  {
    smoothing_Function(rob->getRobotStruct(), path, 100, 4.0);
  }
  
  return true;
}


bool p3d_run_est(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void))
{
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();
	
	
	printf("------------- Running EST --------------\n");
	
#ifdef LIST_OF_PLANNERS
	RRT* rrt = (RRT*)plannerlist[0];
#else
	Robot* _Robot = new Robot(GraphPt->rob,false);
	Graph* _Graph = API_activeGraph = new Graph(_Robot,GraphPt);
	
	EST* est;
	
	est = new EST(_Robot,_Graph);
#endif
	
	int nb_added_nodes = est->init();
	
	_Graph = est->getActivGraph();
	
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	
	nb_added_nodes += est->run();
	ENV.setBool(Env::isRunning,false);
	
	printf("nb added nodes %d\n", nb_added_nodes);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	bool res = est->trajFound();
	
#ifndef LIST_OF_PLANNERS
	delete est;
#endif
	
	return res;
}


// ---------------------------------------------------------------------------------
// PRMs
// ---------------------------------------------------------------------------------
int p3d_run_prm(p3d_graph* GraphPt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
	int ADDED;
	
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();
	cout << "Create Robot and Graph " << endl;
	
#ifdef LIST_OF_PLANNERS
	PRM* prm = (PRM*)plannerlist[2];
#else
	Robot* _Robot = new Robot(GraphPt->rob);
	Graph* _Graph = API_activeGraph = new Graph(_Robot,GraphPt);
	
	PRM* prm = new PRM(_Robot,_Graph);
#endif
	cout << "Initializing PRM " << endl;
	ADDED = prm->init();
	
	cout << "Expanding PRM " << endl;
	ADDED += prm->run();
	
	printf("nb added nodes %d\n", ADDED);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	*fail = !prm->trajFound();
	
#ifndef LIST_OF_PLANNERS
	delete prm;
#endif
	
	return ADDED;
}


int p3d_run_vis_prm(p3d_graph* GraphPt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
	int ADDED;
	
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();
	
#ifdef LIST_OF_PLANNERS
	Vis_PRM* vprm = (Vis_PRM*)plannerlist[1];
#else
	Robot* _Robot = new Robot(GraphPt->rob);
	Graph* _Graph = API_activeGraph = new Graph(_Robot,GraphPt);
	
	Vis_PRM* vprm = new Vis_PRM(_Robot,_Graph);
#endif
	
	ADDED = vprm->init();
	
	ADDED += vprm->run();
	
	printf("nb added nodes %d\n", ADDED);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	*fail = !vprm->trajFound();
	
#ifndef LIST_OF_PLANNERS
	delete vprm;
#endif
	
	return ADDED;
}

int p3d_run_acr(p3d_graph* GraphPt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
	int ADDED;
	
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();
	
#ifdef LIST_OF_PLANNERS
	ACR* acr = (ACR*)plannerlist[3];
#else
	Robot* _Robot = new Robot(GraphPt->rob);
	Graph* _Graph = API_activeGraph = new Graph(_Robot,GraphPt);
	
	ACR* acr = new ACR(_Robot,_Graph);
#endif
	
	ADDED = acr->init();
	
	ADDED += acr->run();
	
	printf("nb added nodes %d\n", ADDED);
	printf("nb nodes %zu\n",_Graph->getNodes().size());
	*fail = !acr->trajFound();
	
#ifndef GLOBAL
	delete acr;
#endif
	
	return ADDED;
}


/**
 * Function To replace p3d_Learn in Case of C++ API Use
 */
void p3d_learn_cxx(int NMAX,
									 int (*fct_stop)(void), void (*fct_draw)(void)) {
	p3d_graph *G;
	int inode, ADDED;
	double tu, ts;
	int fail = 1;
	int nbInitGraphNodes, nbGraphNodes;
	
	ChronoOn();
	
	if (!XYZ_GRAPH) G = p3d_create_graph();
	else           G = XYZ_GRAPH;
	/*debut modif fpilarde*/
	inode = 0;
	
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
				ADDED = p3d_run_prm(G, &fail, fct_stop, fct_draw);
				break;
				
			case 2:
				cout << "CXX_PLANNER c++ API : p3d_run_vis_prm" << endl;
				ADDED = p3d_run_vis_prm(G, &fail, fct_stop, fct_draw);
				break;
				
			case P3D_ALL_PRM:
				cout << "CXX_PLANNER c++ API : p3d_run_acr" << endl;
				ADDED = p3d_run_acr(G, &fail, fct_stop, fct_draw);
				break;
				
			default:
				PrintInfo(("p3d_learn : ERREUR : pas de planificateur global...\n"));
				return;
		}
		inode += ADDED;
	}
	
	else {
		nbInitGraphNodes = G->nnode;
		ADDED = p3d_run_rrt(G, fct_stop, fct_draw);
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
				ADDED = p3d_run_prm(G, &fail, fct_stop, fct_draw);
				break;
				
			case P3D_ISOLATE_LINKING:
				cout << "CXX_PLANNER c++ API : p3d_run_vis_prm" << endl;
				ADDED = p3d_run_vis_prm(G, &fail, fct_stop, fct_draw);
				break;
				
			case P3D_ALL_PRM:
				cout << "CXX_PLANNER c++ API : p3d_run_acr" << endl;
				ADDED = p3d_run_acr(G, &fail, fct_stop, fct_draw);
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
		ADDED = p3d_run_rrt(G, fct_stop, fct_draw);
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

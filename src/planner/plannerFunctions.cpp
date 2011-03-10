/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include "env.hpp"

#include "planEnvironment.hpp"
#include "planners_cxx.hpp"
#include "plannerFunctions.hpp"
#include "API/Trajectory/costOptimization.hpp"

#include "Diffusion/RRT-Variants/Threshold-RRT.hpp"
#include "Diffusion/RRT-Variants/Star-RRT.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_costspace.hpp"
#include "HRI_costspace/RRT/HRICS_rrt.hpp"
#include "HRI_costspace/RRT/HRICS_rrtPlan.hpp"
#endif

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "LightPlanner-pkg.h"

using namespace std;
using namespace tr1;

// ---------------------------------------------------------------------------------
// Run number that is counting the run
// ---------------------------------------------------------------------------------
unsigned int runNum = 0;

// ---------------------------------------------------------------------------------
// Run Id that is used in multiRun
// ---------------------------------------------------------------------------------
unsigned int runId = 0;

void p3d_planner_functions_SetRunId( unsigned int idRun)
{
	runId = idRun;
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
  cout << "* PLANNING ***************" << endl;
  
  double ts;
  
  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  // Gets the 2 configurations
  shared_ptr<Configuration> q_Source( new Configuration(rob,qs) );
  shared_ptr<Configuration> q_Target( new Configuration(rob,qg) );
  
  // Allocate the graph if does't exist
  p3d_graph* GraphPt = robotPt->GRAPH;
  GraphPt = GraphPt ? GraphPt : p3d_create_graph();
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
	cout << "graph time ="  << graph->getGraphStruct()->rrtTime << endl;
	cout << "nb added nodes " << nb_added_nodes << endl;
	cout << "nb nodes (Wrapper) " << (int)graph->getNodes().size() << endl;
	cout << "nb nodes " << graph->getGraphStruct()->nnode << endl;
  
  if ((rrt->getNumberOfExpansion() - rrt->getNumberOfFailedExpansion() + rrt->getNumberOfInitialNodes()) 
      != graph->getNumberOfNodes() ) 
  {
    cout << "Nb of nodes differ from number of expansion succes " << endl;
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
      cout << "Export the cpp graph to new graph" << endl;
      XYZ_GRAPH = robotPt->GRAPH = graph->exportCppToGraphStruct();
      
      Graph graphTraj( rob, XYZ_GRAPH );
      
      cout << "Extract graph to traj" << endl;
      traj = graphTraj.extractBestTraj(q_Source,q_Target);
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
    cout << "result = " << result << endl;
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
  
  API::CostOptimization optimTrj(rob,traj);
  
#ifdef QT_LIBRARY
  optimTrj.setContextName( ENV.getString(Env::nameOfFile).toStdString() );
#endif
  
  //XYZ_GRAPH->rrtCost1 = optimTrj.cost();
  
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
  
  cout << "Cut the traj in several local paths" << endl;
  cout << "optimTrj range is : " << range << endl;
  
  optimTrj.cutTrajInSmallLP( floor( range / (15*dmax) ) );
  optimTrj.replaceP3dTraj();
  optimTrj.resetCostComputed();
  
  pathPt = robotPt->tcur;
  
  //XYZ_GRAPH->totTime += optTime;
  
  cout << "After optim rounds cost : " << optimTrj.cost() << endl;
}

// ---------------------------------------------------------------------------------
// Re-Planning
// ---------------------------------------------------------------------------------
void replanning_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint)
{
  cout << "* REPLANNING ***************" << endl;
  
  //traj = pathPt;
  ManipulationUtils::printConstraintInfo(robotPt);
  p3d_multilocapath_print_group_info(robotPt);
  p3d_multilocalpath_switch_to_linear_groups (robotPt);
  
  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  API::Trajectory oldTraj(rob,traj);
  
#ifdef QT_LIBRARY
  //oldTraj.setContextName( ENV.getString(Env::nameOfFile).toStdString() );
#endif
  
  unsigned int lastViaPoint = oldTraj.getNbPaths();
  
  API::CostOptimization optimTrj = oldTraj.extractSubTrajectory( (unsigned int)deformationViaPoint, lastViaPoint-1 );
  
  Eigen::Vector3d WSPoint;
  WSPoint[0] = target[0];
  WSPoint[1] = target[1];
  WSPoint[2] = target[2];
  
  unsigned int validShortCutId,endId;
  double* q = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->testTransferPointToTrajectory(WSPoint, optimTrj,validShortCutId);
  if ( q ) 
  {
    endId = optimTrj.getNbPaths();
    
    vector<LocalPath*> path;
    
    shared_ptr<Configuration> q_source(optimTrj.getLocalPathPtrAt(validShortCutId)->getEnd());
    shared_ptr<Configuration> q_target(new Configuration(rob,q));
    
    path.push_back( new LocalPath(q_source,q_target) );
    
//    vector<LocalPath*> courbe = optimTrj.getCourbe();
//    optimTrj.copyPaths( courbe );
    optimTrj.replacePortion(validShortCutId,endId,path);
  }
  
  double optTime = 0.0;
//  if(PlanEnv->getBool(PlanParam::withDeformation))
//  {
    //ENV.setBool(Env::FKShoot,true);
//    optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , runId );
    //ENV.setBool(Env::FKShoot,false);
//    optTime += optimTrj.getTime();
//  }
  
  optimTrj.resetCostComputed();
  
  //XYZ_GRAPH->rrtCost2 = optimTrj.cost();
  
//  if(PlanEnv->getBool(PlanParam::withShortCut))
//  {
//    optimTrj.runShortCut( ENV.getInt(Env::nbCostOptimize) , runId );
//    optTime += optimTrj.getTime();
//  }
  
  double dmax = global_Project->getActiveScene()->getDMax();
  double range = optimTrj.getRangeMax();
  
  cout << "** END CUTTING **************" << endl;
  cout << "NLP = " << floor( range / (15*dmax)) << endl;
  
  optimTrj.cutTrajInSmallLP( floor( range / (15*dmax) ) );
  
  vector<LocalPath*> courbe = optimTrj.getCourbe();
  optimTrj.copyPaths( courbe );
  
  oldTraj.replacePortion((unsigned int)deformationViaPoint, lastViaPoint, courbe);
  oldTraj.replaceP3dTraj();
}

// ---------------------------------------------------------------------------------
// Tree Planners
// ---------------------------------------------------------------------------------
int p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void))
{	
	double /*tu,*/ts;
  
  cout << "------------------------------------------------" << endl;
  cout << "------------------------------------------------" << endl;
  cout << "p3d_run_rrt (motionPlanner-libs)" << endl;
	
	GraphPt = GraphPt ? GraphPt : p3d_create_graph();
	
	Robot* rob = global_Project->getActiveScene()->getActiveRobot();
	Graph* graph = 	API_activeGraph =  new Graph(rob,GraphPt);
	
	cout << "Planning for robot : " << rob->getName() << endl;
	
	RRT* rrt = allocate_RRT(rob,graph);

  // -------------------------------------------------------------------------
	// Main Run functions of all RRTs
	// -------------------------------------------------------------------------
  
  int nb_added_nodes = 0;
  
  nb_added_nodes += rrt->setInit(rob->getInitialPosition());
	nb_added_nodes += rrt->setGoal(rob->getGoTo());
  
	nb_added_nodes += rrt->init();
  
  rrt->setInitialized(true);
  
  cout << "Start in Graph : " << graph->searchConf( *rob->getInitialPosition() ) << endl;
  cout << "Goal_ in Graph : " << graph->searchConf( *rob->getGoTo() ) << endl;
	
	nb_added_nodes += rrt->run();
  
  cout << "Start in Graph : " << graph->searchConf( *rob->getInitialPosition() ) << endl;
  cout << "Goal_ in Graph : " << graph->searchConf( *rob->getGoTo() ) << endl;
	
	// Gets the graph pointer
	// in case it has been modified by the planner
	graph = rrt->getActivGraph();
	
	ChronoTimes(&(graph->getGraphStruct()->rrtTime), &ts);
	
	printf("graph time = %f\n",graph->getGraphStruct()->rrtTime);
	printf("nb added nodes %d\n", nb_added_nodes);
	printf("nb nodes (Wrapper) %d\n",(int)graph->getNodes().size());
	printf("nb nodes %d\n",graph->getGraphStruct()->nnode);
	
	bool res = rrt->trajFound();
	
	if( (!ENV.getBool(Env::isCostSpace)) && (!ENV.getBool(Env::useTRRT)) )
	{
		ENV.setBool(Env::isCostSpace,true);
	}
	
	graph->getGraphStruct()->totTime = graph->getGraphStruct()->rrtTime;
	
	// Smoothing phaze
	// -------------------------------------------------------------------------
	bool trajExtractSucceded = false;
  
  if(res)
	{
		ENV.setBool(Env::isRunning,true);
		
		if( !ENV.getBool(Env::use_p3d_structures) )
		{
			cout << "Export the cpp graph to new graph" << endl;
      
      cout << "Start in Graph : " << graph->searchConf( *rob->getInitialPosition() ) << endl;
      cout << "Goal_ in Graph : " << graph->searchConf( *rob->getGoTo() ) << endl;
      
			// Copies the graph to a p3d_structured graph
			Graph graphTraj( rob, graph->exportCppToGraphStruct() );
			
			cout << "Extract graph to traj" << endl;
			// Extract traj
			 trajExtractSucceded = graphTraj.extractBestTraj(rob->getInitialPosition(),
                                                       rob->getGoTo());
		}
		else
		{
			// Extract traj
			trajExtractSucceded = graph->extractBestTraj(rob->getInitialPosition(),
                                                   rob->getGoTo());
		}
    
		if ( !ENV.getBool(Env::use_p3d_structures) ) 
		{
			XYZ_GRAPH = graph->exportCppToGraphStruct();
		}
		
		
		if(PlanEnv->getBool(PlanParam::withSmoothing) && trajExtractSucceded )
		{
			API::CostOptimization optimTrj(rob,rob->getTrajStruct());
			
#ifdef QT_LIBRARY
			optimTrj.setContextName( ENV.getString(Env::nameOfFile).toStdString() );
#endif
			
			XYZ_GRAPH->rrtCost1 = optimTrj.cost();
			
			double optTime = 0.0;
			if(PlanEnv->getBool(PlanParam::withDeformation))
			{
				ENV.setBool(Env::FKShoot,true);
				optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , runId );
				ENV.setBool(Env::FKShoot,false);
				optTime += optimTrj.getTime();
			}
			
			optimTrj.resetCostComputed();
			
			XYZ_GRAPH->rrtCost2 = optimTrj.cost();
			
			if(PlanEnv->getBool(PlanParam::withShortCut))
			{
				optimTrj.runShortCut( ENV.getInt(Env::nbCostOptimize) , runId );
				optTime += optimTrj.getTime();
			}
			
			optimTrj.replaceP3dTraj();
			optimTrj.resetCostComputed();
			
			XYZ_GRAPH->totTime += optTime;
			
			cout << "After optim rounds cost : " << optimTrj.cost() << endl;
		}
	}
	
	if ((rrt->getNumberOfExpansion() - rrt->getNumberOfFailedExpansion() + rrt->getNumberOfInitialNodes()) 
			!= graph->getNumberOfNodes() ) 
	{
		cout << "Nb of nodes differ from number of expansion succes " << endl;
		cout << " - m_nbExpansion = " << rrt->getNumberOfExpansion() << endl;
		cout << " - m_nbInitNodes = " << rrt->getNumberOfInitialNodes() << endl;
		cout << " - m_nbExpansion + m_nbInitNodes - m_nbExpansionFailed  =  " << (rrt->getNumberOfExpansion() + rrt->getNumberOfInitialNodes() - rrt->getNumberOfFailedExpansion() ) << endl;
		cout << " - _Graph->getNumberOfNodes() = " << graph->getNumberOfNodes() << endl;
	}
	
	runNum++;
  
//  if( _Start->equal(_Goal) )
//  {
//  }
	
	if( res && trajExtractSucceded )
	{
    char trajName[] = "Specific";
		g3d_add_traj( trajName , runNum , rob->getRobotStruct() , rob->getRobotStruct()->tcur );
		return graph->getNumberOfNodes();
	}
	else
	{
		rob->getRobotStruct()->tcur = NULL;
		cout << "res == false, No traj found" << endl;
		return false;
	}
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

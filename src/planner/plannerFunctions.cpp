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
/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include <libmove3d/p3d/env.hpp>

#include "API/project.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/replanningSimulators.hpp"
#include "planner/planner.hpp"

#include "planner/PRM/PRM.hpp"
#include "planner/PRM/Visibility.hpp"
#include "planner/PRM/ACR.hpp"
#include "planner/PRM/PerturbationRoadmap.hpp"
#include "planner/PRM/sPRM.hpp"
#include "planner/PRM/PRMStar.hpp"

#include "planner/Diffusion/RRT.hpp"
#include "planner/Diffusion/EST.hpp"
#include "planner/Diffusion/Variants/Transition-RRT.hpp"
#include "planner/Diffusion/Variants/ManhattanLike-RRT.hpp"
#include "planner/Diffusion/Variants/Multi-RRT.hpp"
#include "planner/Diffusion/Variants/Multi-TRRT.hpp"
#include "planner/Diffusion/Variants/Threshold-RRT.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"
#include "planner/Diffusion/Variants/RRG.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#ifdef HRI_COSTSPACE
#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/RRT/HRICS_rrt.hpp"
#include "hri_costspace/RRT/HRICS_rrtPlan.hpp"
#endif

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "move3d-headless.h"
#include "Planner-pkg.h"
#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace Move3D;

using std::cout;
using std::endl;
using std::cerr;

MOVE3D_USING_SHARED_PTR_NAMESPACE

// ---------------------------------------------------------------------------------
// Exctracted trajectory Id (used for UI)
// ---------------------------------------------------------------------------------
unsigned int trajId = 0;

// ---------------------------------------------------------------------------------
// Last trajectory
// ---------------------------------------------------------------------------------
Move3D::Trajectory last_traj;

Move3D::Trajectory p3d_get_last_trajectory()
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
p3d_traj* p3d_extract_traj( bool is_traj_found, int nb_added_nodes, Graph* graph, confPtr_t q_source, confPtr_t q_target)
{
    cout << "--- p3d_extract_traj ---------------------------" << endl;
    Move3D::Trajectory* traj = NULL;
    Robot* rob = graph->getRobot();

    // If traj is found, extract it from the graph
    if (/*rrt->trajFound()*/ is_traj_found )
    {
        // Case of direct connection
        if( nb_added_nodes == 2 )
        {
            std::vector<confPtr_t> configs;

            configs.push_back( q_source );
            configs.push_back( q_target );

            cout << "Creating trajectory from two confgurations" << endl;
            traj = new Move3D::Trajectory( configs );
        }
        else
        {
            //traj = graph->extractBestTraj( q_source, q_target ); // Old extract
            traj = graph->extractAStarShortestPathsTraj( q_source, q_target );
            // traj = graph->extractBestAStarPathFromLargestComponent( q_source, q_target );
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

        traj->replaceP3dTraj();
        p3d_traj* result = static_cast<p3d_rob*>(rob->getP3dRobotStruct())->tcur;

        if( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
        {
            g3d_draw_allwin_active();
        }

        cout << "result->nlp = " << result->nlp << endl;
        cout << "result->range_param = " << result->range_param << endl;

        char trajName[] = "Specific";
        g3d_add_traj( trajName, trajId, static_cast<p3d_rob*>(rob->getP3dRobotStruct()), static_cast<p3d_rob*>(rob->getP3dRobotStruct())->tcur );

        // Prevent segfault if p3d graph deleted outside
        delete traj;
        return result;
    }
    else
    {
        // cout << __FILE__ << " , "
        cout << __PRETTY_FUNCTION__ << " : No traj found" << endl;
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
    else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::costThresholdRRT) )
    {
        rrt = new ThresholdRRT(rob,graph);
    }
    else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::useTRRT) )
    {
        rrt = new TransitionRRT(rob,graph);
    }
    else if(ENV.getBool(Env::isCostSpace) && PlanEnv->getBool(PlanParam::starRRT) )
    {
        rrt = new StarRRT(rob,graph);
    }
    else if(ENV.getBool(Env::isCostSpace) && PlanEnv->getBool(PlanParam::rrg) )
    {
        rrt = new RRG(rob,graph);
    }
    else
    {
        if( ENV.getBool(Env::isCostSpace) && (!ENV.getBool(Env::useTRRT)) )
        {
            ENV.setBool( Env::isCostSpace, false );
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
    p3d_traj* traj = p3d_extract_traj( rrt->trajFound(), nb_added_nodes, graph, q_source, q_target );

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

    cout << "av. nb col tests per edge :  " << graph->getNumberOfColTestCalls() / graph->getNumberOfEdges() << endl;
    cout << "av. nb cost calls per edge :  " << graph->getNumberOfCostCalls() / graph->getNumberOfEdges() << endl;

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
        cout << "robot not defined in " << __PRETTY_FUNCTION__ << endl;
        return;
    }

    if( maxTime != -1 )
        PlanEnv->setDouble( PlanParam::timeLimitSmoothing, maxTime );

    Robot* rob = global_Project->getActiveScene()->getRobotByName( robotPt->name );

    Move3D::Trajectory t;

    if(PlanEnv->getBool(PlanParam::withDeformation) || PlanEnv->getBool(PlanParam::withShortCut) )
    {
        double optTime = 0.0;

        if( traj == NULL ) {
            cout << "trajectory NULL in " << __PRETTY_FUNCTION__ << endl;
            return;
        }
        cout << "traj->nlp = " << traj->nlp << endl;
        cout << "traj->range_param = " << traj->range_param << endl;

        Move3D::CostOptimization optimTrj( rob, traj );

        optimTrj.setRunId( runId );
        optimTrj.setContextName( ENV.getString(Env::nameOfFile) );

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
        cout << "optimTrj.getParamMax() : " << optimTrj.getParamMax()  << endl;

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

        t = Move3D::Trajectory(optimTrj);
        last_traj = t;
    }

    if( PlanEnv->getBool( PlanParam::withStomp ) )
    {
        //PlanEnv->setBool(PlanParam::withCurrentTraj,true);
        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( t );

        traj_optim_runStompNoReset( runId );

        t = Move3D::Trajectory( rob, static_cast<p3d_rob*>(rob->getP3dRobotStruct())->tcur );
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

    confPtr_t q_source = rob->getInitPos();
    confPtr_t q_target = rob->getGoalPos();

    p3d_traj* path = p3d_planner_function( rob->getP3dRobotStruct(), q_source->getConfigStruct(), q_target->getConfigStruct() );

    if( path != NULL &&
            !PlanEnv->getBool(PlanParam::stopPlanner) &&
            PlanEnv->getBool(PlanParam::withSmoothing) )
    {
        double max_iteration = PlanEnv->getInt(PlanParam::smoothMaxIterations);
        double max_time = PlanEnv->getDouble( PlanParam::timeLimitSmoothing );

        p3d_smoothing_function( rob->getP3dRobotStruct(), path, max_iteration, max_time );
    }

    return (path != NULL);
}


bool p3d_run_est(p3d_rob* robotPt)
{	
    // Gets the robot pointer and the 2 configurations
    Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
    confPtr_t q_source = rob->getInitPos();
    confPtr_t q_target = rob->getGoalPos();

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
    Robot* rob = global_Project->getActiveScene()->getRobotByName( robotPt->name );
    confPtr_t q_source = rob->getInitPos();
    confPtr_t q_target = rob->getGoalPos();

    // Allocate the graph if does't exist
    if(!API_activeGraph)
        API_activeGraph =  new Graph(rob);

    Graph* graph = API_activeGraph;

    cout << "Initializing PRM " << endl;
    int nb_added_nodes=0;

    PRM* prm;

    switch(ENV.getInt(Env::PRMType))
    {
    case 0:
        prm = new PRM(rob, graph);
        break;
    case 1:
        prm = new Vis_PRM(rob, graph);
        break;
    case 2:
        prm = new ACR( rob, graph );
        break;
    case 3:
        prm = new PerturbationRoadmap( rob, graph );
        break;
    case 4:
        prm = new sPRM( rob, graph );
        break;
    case 5:
        prm = new PRMStar( rob, graph );
        break;
    default:
        return nb_added_nodes;
    }

    nb_added_nodes = prm->init();
    nb_added_nodes += prm->run();

    cout << "nb added nodes " << nb_added_nodes << endl;
    cout << "nb nodes " << graph->getNumberOfNodes() << endl;

    p3d_extract_traj( prm->trajFound(), nb_added_nodes, graph, q_source, q_target);

    cout << "av. nb col tests per edge :  " << graph->getNumberOfColTestCalls() / graph->getNumberOfEdges() << endl;
    cout << "av. nb cost calls per edge :  " << graph->getNumberOfCostCalls() / graph->getNumberOfEdges() << endl;

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
    confPtr_t q_source = rob->getInitPos();
    confPtr_t q_target = rob->getGoalPos();

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

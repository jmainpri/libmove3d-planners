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
#include "GreedyCost.hpp"

#include "planEnvironment.hpp"

#include "Util-pkg.h"
#include "Graphic-pkg.h"
#include "Planner-pkg.h"
#include "Collision-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

GreedyCost::GreedyCost(p3d_graph* G, int(*stop_func)(), void(*draw_func)()) :

    mConsecutiveFailures(0),
    _stop_func(stop_func),
    _draw_func(draw_func),
    mRobot(new Robot(G->rob)),
    mGraph(new Graph(mRobot,G)),
    mStart(NULL),
    mGoal(NULL)
{

    if (!ENV.getBool(Env::isCostSpace))
    {
        cout << "Error Not Cost Space" << endl;
    }

    //	Configuration goalConf(*mRobot->getGoalPos());
    //	Configuration startConf(*mRobot->getInitPos());

    Expansion = new RRTExpansion(mGraph);

    nb_Loops = 0;
    nb_LocalPaths = 0;
    nb_CostCompare = 0;
    nb_CObstFail = 0;

    nb_OptimSuccess = 0;
    nb_OptimFail = 0;
}

GreedyCost::~GreedyCost()
{
    /*	delete Expansion;
     delete trajPt;*/
}

bool GreedyCost::run()
{
    cout << "-------------- Start Diffusion ----------------------" << endl;

    if(ENV.getBool(Env::useTRRT))
    {
        Diffusion = new TransitionRRT(mRobot,mGraph);
    }
    else
    {
        Diffusion = new RRT(mRobot,mGraph);
        ENV.setBool(Env::isCostSpace, false);
    }

    ENV.setBool(Env::biDir, true);
    Diffusion->init();
    mGraph = Diffusion->getActivGraph();
    /* int nb_nodes = */ Diffusion->run();
    ENV.setBool(Env::isCostSpace, true);

    if (Diffusion->trajFound())
    {
        //mGraph->setTraj(mRobot->getTrajStruct());
        cout << "Trajectory exists" << endl;
        ENV.setBool(Env::drawTraj, true);
        g3d_draw_allwin_active();

        cout << "-------------- Start Optimization ----------------------"
             << endl;

        p3d_ExtractBestTraj(mGraph->getGraphStruct());
        optimTrj = new Move3D::CostOptimization( mRobot, mRobot->getP3dRobotStruct()->tcur );

        //                if(ENV.getBool(Env::debugCostOptim))
        //                {
        //                    ENV.setBool(Env::initPlot,false);
        //                    ENV.setVector( Env::costAlongTraj, optimTrj->getCostAlongTrajectory(100) );
        //                }

        double dmax = 0;
        p3d_col_get_dmax(&dmax);
        dmax = ENV.getDouble(Env::dmax);
        cout << "dmax = " << dmax << endl;
        cout << "RangeMax = " << optimTrj->getRangeMax() << endl;

        cout << "Traj Cost = " << optimTrj->cost() << endl;

        optimizeLinear();
        cout << "Traj Cost = " << optimTrj->cost() << endl;

        if(PlanEnv->getBool(PlanParam::withShortCut))
        {
            shortCutLinear();
            cout << "Traj Cost = " << optimTrj->cost() << endl;
        }

        cout << "-------------- End Optimization ------------------------" << endl;
        optimTrj->replaceP3dTraj();
        g3d_draw_allwin_active();


        if (ENV.getBool(Env::debugCostOptim))
        {
            //			optimTrj->printDebugInfo();
        }

        delete optimTrj;
    }
    ENV.setBool(Env::isRunning,false);
    return Diffusion->trajFound();
}

void GreedyCost::shortCutLinear()
{
    int nb_success(0);
    int nLoopTotShort(0);
    int nFailOptim(0);

    bool isOptimSuccess(false);

    const int nMaxCostFail(1000);

    while ( (nFailOptim < nMaxCostFail)
            && (nLoopTotShort < ENV.getInt(Env::nbCostOptimize))
            && !checkStopConditions())
    {
        nLoopTotShort++;

        // TODO
        if(PlanEnv->getBool(PlanParam::trajCostRecompute))
        {
            isOptimSuccess = optimTrj->oneLoopShortCutRecompute();
        }
        else
        {
            isOptimSuccess = optimTrj->oneLoopShortCut();
        }

        if (isOptimSuccess == false)
        {
            nFailOptim++;
            nb_OptimFail++;
        }
        else
        {
            //			optimTrj->replaceP3dTraj();
            //			g3d_draw_allwin_active();
            nb_OptimSuccess++;
            nFailOptim = 0;
            nb_success++;
        }

    }

    cout << "---------------------- Short Cut -----------------------" << endl;
    cout << "Nb Success = " << nb_success << endl;
    cout << "Nb Loops = " << nLoopTotShort << endl;
    cout << "--------------------------------------------------------" << endl;

}
void GreedyCost::optimizeLinear()
{
    double dmax = 0;
    p3d_col_get_dmax(&dmax);
    dmax = ENV.getDouble(Env::dmax);

    double maxFactor = dmax * (PlanEnv->getDouble(PlanParam::MaxFactor));
    double minFactor = dmax * (PlanEnv->getDouble(PlanParam::MinStep));

    int nb_success(0);
    int nb_collision(0);
    int nTotFail(0);
    //	int nFailOptim(0);
    int nLoopTot(0);
    int isOptimSuccess(0);
    //	int MaxCostFail = 10 * ENV.getInt(Env::maxCostOptimFailures);
    const int nLoopTotMax(10000);
    // const int nMaxCostFail(1000);

    double factor = maxFactor;

    while (/*(nFailOptim < nMaxCostFail)*/true && (nLoopTot < nLoopTotMax)
           && (nLoopTot < ENV.getInt(Env::nbCostOptimize))
           && !checkStopConditions())
    {

        factor = factor - ((maxFactor - minFactor) / (double) ENV.getInt(
                               Env::nbCostOptimize));

        // cout << factor << endl;

        nLoopTot++;

        if(	nLoopTot%5 == 0 )
        {
            optimTrj->setCheat();
        }

        if( PlanEnv->getBool(PlanParam::trajCostRecompute) )
        {
            isOptimSuccess = optimTrj->oneLoopDeformRecompute();
        }
        else {
            isOptimSuccess = optimTrj->oneLoopDeform();
        }

        if (isOptimSuccess == false)
        {
            if(optimTrj->deformInCollision())
            {
                nb_collision++;
            }
            nb_OptimFail++;
            nTotFail++;
        }
        else
        {
            // optimTrj->replaceP3dTraj();
            // g3d_draw_allwin_active();
            //nFailOptim = 0;
            nb_success++;
            nb_OptimSuccess++;
        }
    }

    cout << "------------------ Optimize Linear ---------------------" << endl;
    cout << "Factor = " << factor << endl;
    cout << "Nb Success = " << nb_success << endl;
    cout << "Nb Fail = " << nTotFail << endl;
    cout << "Nb Loops = " << nLoopTot << endl;
    cout << "Nb In collision = " << nb_collision << endl;
    cout << "--------------------------------------------------------" << endl;
}

void GreedyCost::optimizePhaze()
{

    int nFailOptim = 0;
    int nLoopTot = 0;
    int nLoopTotMax = 2000;
    int isOptimSuccess;
    //mGraph->

    const int IterationMax(3);
    //	const double FactorMax(1000);
    //	const double FactorMin(0.5);

    std::vector<int> MaxCostFail;

    for (int i = 0; i < IterationMax; i++)
    {
        MaxCostFail.push_back(ENV.getInt(Env::maxCostOptimFailures) / (i + 1));
    }

    std::vector<int> Factor;

    Factor.push_back(100);
    Factor.push_back(10);
    Factor.push_back(1);

    double factor(0.0);
    // double it(0.0);

    int i = 0;
    int nb_success(0);
    int nTotFail(0);

    while (i < IterationMax)
    {
        //Loop done until an optimization failed a given number of times or when it reaches
        // a maximal number of loops
        //		it = (double)i;
        //		factor = ((FactorMin-FactorMax)/(double)IterationMax)*(it+1)+FactorMax;
        factor = Factor.at(i);

        while ((nFailOptim < MaxCostFail.at(i)) && (nLoopTot < nLoopTotMax))
        {
            nLoopTot++;
            isOptimSuccess = optimTrj->oneLoopDeform();

            if (isOptimSuccess == false)
            {
                nFailOptim++;
                nTotFail++;
            }
            else
            {
                nFailOptim = 0;
                nb_success++;
                g3d_draw_allwin_active();
            }
        }
        i++;
        cout << "Factor = " << factor << endl;
        cout << "Nb Success = " << nb_success << endl;
        cout << "Nb Fail = " << nTotFail << endl;
        cout << "Nb Loops = " << nLoopTot << endl;
        cout << "--------------------------------------------" << endl;

        nb_success = 0;
        nTotFail = 0;
        nFailOptim = 0;
        nLoopTot = 0;
        ENV.setBool(Env::drawTraj, true);
    }

    cout << "nb_Loops = " << nb_Loops << endl;
    cout << "nb_LocalPaths = " << nb_LocalPaths << endl;
    cout << "nb_CostCompare = " << nb_CostCompare << endl;
    cout << "nb_CObstFail = " << nb_CObstFail << endl;
}

bool GreedyCost::checkStopConditions()
{

    if (!(*_stop_func)())
        p3d_SetStopValue(true);
    if (p3d_GetStopValue())
    {
        cout << "Greedy search canceled." << endl;
        return (true);
    }
    return (false);
}

bool p3d_RunGreedyCost(p3d_graph* GraphPt, int(*fct_stop)(void),
                       void(*fct_draw)(void))
{

    cout << endl << "**************************************" << endl
         << " Beginning of Greedy search process" << endl << endl;

    MY_ALLOC_INFO("Before the graph creation");
    double tu, ts;

    if (!GraphPt)
    {
        //p3d_del_graph(GraphPt);
        GraphPt = p3d_create_graph();
    }

    p3d_rob* RobotPt = GraphPt->rob;

    p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 3, false);
    p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 4, false);
    p3d_jnt_set_dof_is_active_for_planner(RobotPt->joints[0], 5, false);

    ChronoOn();

    GreedyCost* OptPlanner = new GreedyCost(GraphPt, fct_stop, fct_draw);

    bool trajExists = OptPlanner->run();

    ChronoPrint("");
    ChronoTimes(&tu, &ts);
    GraphPt->time = GraphPt->time + tu;

    ChronoOff();

    cout << "Nb. of fail : " << OptPlanner->getOptimFail() << endl;
    cout << "Nb. of success : " << OptPlanner->getOptimSuccess() << endl;

    cout << endl << " End of Greedy search process" << endl
         << "**************************************" << endl << endl;

    if (trajExists == false)
    {
        trajExists = false;
        cout
                << "No solution path: the exploration didn't \
                   link a start and a goal configuration."
                << endl;
    }

    g3d_draw_allwin_active();

    cout << endl;

    delete OptPlanner;
    return trajExists;
}

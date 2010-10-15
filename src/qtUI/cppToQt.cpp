/*
 * cppToQt.cpp
 *
 *  Created on: Aug 19, 2009
 *      Author: jmainpri
 *
 *      This file implements a pipe to read out commands from qt and
 *      pass it on to the XForms thread, the main loop runs in a
 *      distinct thread using X11 and XForms doesn't permit qt's thread to act upon X11
 *      at the same time (causing a segmentation fault).
 *
 *      This can be solved by passing the 3D display to Ogre or having the OpenGl in a
 *      Qt window.
 */

/**
 * C++ basic headers (they all have name spaces)
 */
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <string>
//#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <tr1/memory>

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"

#include "cppToQt.hpp"
#include "API/project.hpp"

#include "API/Trajectory/smoothing.hpp"
#include "API/Trajectory/costOptimization.hpp"
//#include "Greedy/GreedyCost.hpp"
#include "API/Search/Dijkstra/dijkstra.hpp"
#include "MultiRun.hpp"

#ifdef CXX_PLANNER
#include "SaveContext.hpp"
#endif

//#ifdef HRI_PLANNER
//#include "HRICS_costspace.hpp"
//#endif
#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_costspace.hpp"
#endif

const char *qt_fileName = NULL;

using namespace std;
using namespace tr1;

// -------------------------------------------------------------
// ------------------   Callback functions  --------------------
// -------------------------------------------------------------
/**
 * Reset Graph
 */
void qt_resetGraph()
{
	try 
	{
		if (API_activeGraph) 
		{
			delete API_activeGraph;
			API_activeGraph = NULL;
			cerr << "Delete C++ API Graph" << endl;
		}
		
#ifdef P3D_PLANNER
		if( !p3d_del_graph(XYZ_GRAPH) )
		{
			cerr << "XYZ_GRAPH allready deleted" << endl;
		}
		
		cerr <<  "XYZ_GRAPH = " << XYZ_GRAPH << endl;
		
#endif
	}
	catch (string str) 
	{
		cerr << str << endl;
	}
	catch (...) 
	{
		cerr << "Exeption in qt_resetGraph()" << endl;
	}
}

/**
 * Draw All Win Active
 */
void qt_drawAllWinActive()
{
	g3d_draw_allwin_active();
}

/**
 * Run Diffusion
 */
void qt_runDiffusion()
{
	cout << "Diffusion" << endl;
	
	try 
	{
#ifdef P3D_PLANNER
		p3d_SetStopValue(FALSE);
#endif
		ChronoOn();
		
		int res;
		cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
		if (ENV.getBool(Env::treePlannerIsEST))
		{
#if defined( CXX_PLANNER ) || defined( WITH_OOMOVE3D )
			res = p3d_run_est(XYZ_GRAPH, fct_stop, fct_draw);
		}
		else
		{
			res = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
#endif
		}
		ChronoPrint("");
		ChronoOff();
		
		g3d_draw_allwin_active();
	}
	catch (string str) 
	{
		cerr << "Exeption in run qt_runDiffusion : " << endl;
		cerr << str << endl;
		ENV.setBool(Env::isRunning,false);
	}
	catch (...) 
	{
		cerr << "Exeption in run qt_runDiffusion" << endl;
		ENV.setBool(Env::isRunning,false);
	}
}

/**
 * Run PRM
 */
void qt_runPRM()
{
	try 
	{
#ifdef P3D_PLANNER
		p3d_SetStopValue(FALSE);
#endif
		
		int res;
		int fail;
		
		ChronoOn();
		
		//        cout << "ENV.getInt(Env::PRMType)  = "  << ENV.getInt(Env::PRMType) << endl;
		
		switch(ENV.getInt(Env::PRMType))
		{
#ifdef CXX_PLANNER
			case 0:
				res = p3d_run_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);
				break;
			case 1:
				res = p3d_run_vis_prm(XYZ_GRAPH, &fail, fct_stop, fct_draw);
				break;
			case 2:
				res = p3d_run_acr(XYZ_GRAPH, &fail, fct_stop, fct_draw);
				break;
#endif
			default:
				cout << "Error No Other PRM"  << endl;
				ChronoPrint("");
				ChronoOff();
				return;
		}
		
		
		ChronoPrint("");
		ChronoOff();
		
		if (ENV.getBool(Env::expandToGoal))
		{
			if (res)
			{
				if (ENV.getBool(Env::isCostSpace))
				{
#ifdef P3D_PLANNER
					p3d_ExtractBestTraj(XYZ_GRAPH);
#endif
				}
				else
				{
					if (p3d_graph_to_traj(XYZ_ROBOT))
					{
						g3d_add_traj((char*) "Globalsearch",
												 p3d_get_desc_number(P3D_TRAJ));
					}
					else
					{
						printf("Problem during trajectory extraction\n");
					}
				}
				g3d_draw_allwin_active();
			}
		}
	}
	catch (string str) 
	{
		cerr << "Exeption in run qt_runPRM : " << endl;
		cerr << str << endl;
	}
	catch (...) 
	{
		cerr << "Exeption in run qt_runPRM" << endl;
	}
	
	ENV.setBool(Env::isRunning,false);
}

/**
 * Shortcut optimization
 */
void qt_shortCut()
{
	cout << "Random : ShortCut "  << endl;
	//ENV.setBool(Env::isRunning,true);
#ifdef CXX_PLANNER
	Robot* trajRobot = global_Project->getActiveScene()->getActiveRobot();
	API::Smoothing optimTrj(trajRobot->getCurrentTraj());
	optimTrj.runShortCut(ENV.getInt(Env::nbCostOptimize));
	optimTrj.replaceP3dTraj();
#endif
	g3d_draw_allwin_active();
	ENV.setBool(Env::isRunning,false);
}

/**
 * Deformation optimization
 */
void qt_optimize()
{
	cout << "Random : Deformation "  << endl;
	//ENV.setBool(Env::isRunning,true);
#ifdef CXX_PLANNER
	Robot* trajRobot = global_Project->getActiveScene()->getActiveRobot();
	API::CostOptimization optimTrj(trajRobot->getCurrentTraj());
	optimTrj.runDeformation(ENV.getInt(Env::nbCostOptimize));
	optimTrj.replaceP3dTraj();
#endif
	g3d_draw_allwin_active();
	ENV.setBool(Env::isRunning,false);
}

/**
 * Optimize one step
 */
void qt_oneStepOptim()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	//	  	p3d_SetIsCostFuncSpace(TRUE);
#ifdef CXX_PLANNER
	Robot* trajRobot = new Robot(robotPt);
	API::CostOptimization optimTrj(trajRobot,CurrentTrajPt);
	
	optimTrj.oneLoopDeform();
	//		optimTrj.removeRedundantNodes();
	optimTrj.replaceP3dTraj(CurrentTrajPt);
#endif
	g3d_draw_allwin_active();
	
	if (CurrentTrajPt == NULL)
	{
		PrintInfo(("Warning: no current trajectory to optimize\n"));
	}
	return;	
}

void qt_removeRedundantNodes()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	if (CurrentTrajPt == NULL)
	{
		PrintInfo(("Warning: no current trajectory to optimize\n"));
	}
#ifdef CXX_PLANNER
	Robot* trajRobot = new Robot(robotPt);
	API::CostOptimization optimTrj(trajRobot,CurrentTrajPt);
	delete trajRobot;
	
	optimTrj.removeRedundantNodes();
	optimTrj.replaceP3dTraj(CurrentTrajPt);
	g3d_draw_allwin_active();
	
	if (optimTrj.isValid())
	{
		cout << "Trajectory valid" << endl;
        }
        else
        {
            cout << "Trajectory not valid" << endl;
        }
#endif
		
        return;	
}

/**
 * Read Scenario
 */
void qt_readScenario()
{
	std::string fileToOpen(qt_fileName);
	cout <<" Should Open scenarion " << fileToOpen << endl;
	
	p3d_rw_scenario_init_name();
	p3d_read_scenario(qt_fileName);
}

/**
 * Save Scenario
 */
void qt_saveScenario()
{
	std::string fileToOpen(qt_fileName);
	cout <<" Should Open scenarion " << fileToOpen << endl;
	
	p3d_rw_scenario_init_name();
	p3d_save_scenario(qt_fileName);
}

void qt_readTraj()
{
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	int ir;
	configPt qi, qf;
	pp3d_traj trajPt;
	
	if (qt_fileName!=NULL) 
	{
		if (p3d_read_traj(qt_fileName)) 
		{
			trajPt = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
			ir = p3d_get_desc_curnum(P3D_ROBOT);
//			g3d_add_traj(p3d_get_desc_curname(P3D_TRAJ),
//						 p3d_get_desc_number(P3D_TRAJ));
			qi = p3d_alloc_config(robotPt);
			qf = p3d_alloc_config(robotPt);
			p3d_ends_and_length_traj(trajPt, &qi, &qf);   
			p3d_copy_config_into(robotPt, qf, &(robotPt->ROBOT_GOTO));
			p3d_copy_config_into(robotPt, qi, &(robotPt->ROBOT_POS));
			
//			g3d_draw_allwin_active();
			p3d_destroy_config(robotPt, qi);
			p3d_destroy_config(robotPt, qf);
		}
	}
	
	cout << "Apres lecture de la trajectoire" << endl;
}

#ifdef HRI_COSTSPACE
void qt_load_HRICS_Grid(std::string docname)
{
	ENV.setBool(Env::drawGrid,false);
	
	HRICS_activeNatu  = new HRICS::Natural;
	
//	string docname(qt_fileName);
	
	HRICS::NaturalGrid* myGrid = new HRICS::NaturalGrid;
	myGrid->setNaturalCostSpace(HRICS_activeNatu);
	
	bool reading_OK=false;
	
	for (int i=0; (i<5)&&(!reading_OK) ; i++) 
	{
		cout << "Reading grid at : " << docname << endl;
		reading_OK = myGrid->loadFromXmlFile(docname);
	}
	//		myGrid->initReachable();
	//		myGrid->resetCellCost();
	API_activeGrid = myGrid;
	
	if( HRICS_MotionPL != NULL )
	{
		if( HRICS_activeNatu->IsHuman() )
		{
			cout << "Set Reachability space" << endl;
			HRICS_MotionPL->setReachability(HRICS_activeNatu);
		}
		else 
		{
			cout << "Set Natural space" << endl;
			HRICS_MotionPL->setNatural(HRICS_activeNatu);
		}
	}
	
	ENV.setBool(Env::drawGrid,true);
}
#endif

// -------------------------------------------------------------
// ------------------   Pipe  ----------------------------------
// -------------------------------------------------------------
void read_pipe(int fd, void* data)
{
    char buffer[256];
    while (read(fd, buffer, sizeof(buffer)) > 0);
    //printf("Qt to XForms => %s\n", buffer);
	
    string bufferStr(buffer);
	
    //	cout << bufferStr << endl;
	//cout << "Reading Pipe!!!!!!!" << endl;
	
    if (bufferStr.compare("ResetGraph") == 0)
    {
		qt_resetGraph();
		return;
    }
	
    if (bufferStr.compare("g3d_draw_allwin_active") == 0)
    {
		qt_drawAllWinActive();
		return;
    }
	
    if (bufferStr.compare("RunDiffusion") == 0)
    {
		qt_runDiffusion();
		return;
    }
	
    if (bufferStr.compare("RunPRM") == 0)
    {
		qt_runPRM();
		return;
    }
	
	if (bufferStr.compare("shortCut") == 0)
    {
		qt_shortCut();
		return;
    }
	
	if(bufferStr.compare("readScenario") == 0 )
    {
		qt_readScenario();
		return;
    }
	
	if(bufferStr.compare("saveScenario") == 0 )
    {
		qt_saveScenario();
		return;
    }
	
    /*if (bufferStr.compare("p3d_RunGreedy") == 0)
    {
#ifdef P3D_PLANNER
        p3d_SetStopValue(FALSE);
#ifdef CXX_PLANNER
        p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
#endif
#endif
        return;
    }*/
	
    if (bufferStr.compare("MultiSmooth") == 0)
    {
#ifdef CXX_PLANNER
        MultiRun multiSmooths;
        multiSmooths.runMutliSmooth();
#endif
        return;
    }
	
    if (bufferStr.compare("MultiRRT") == 0)
    {
#ifdef CXX_PLANNER
        MultiRun multiRRTs;
        multiRRTs.runMutliRRT();
#endif
        return;
    }
	
    if (bufferStr.compare("optimize") == 0)
    {
		qt_optimize();
    }
	
	//	if ( bufferStr.compare("computeNaturalGrid") == 0 )
	//	{
	//		HRICS_Natural->computeNaturalGrid();
	//	}
	
	
    if (bufferStr.compare("oneStepOptim") == 0)
    {
        qt_oneStepOptim();
    }
	
	
    if (bufferStr.compare("removeRedunantNodes") == 0)
    {
		
    }
	
	//    if (bufferStr.compare("graphSearchTest") == 0)
	//    {
	//        //		Dijkstra graphS;
	//        //		graphS.example();
	//
	//        Graph* ptrGraph = new Graph(XYZ_GRAPH);
	//
	//        Dijkstra graphS(ptrGraph);
	//
	//        //		int start = 1;
	//        //		int goal = (int)ptrGraph->getNbNode()/2;
	//
	//        shared_ptr<Configuration> Init =
	//                ptrGraph->getRobot()->getInitialPosition();
	//        shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();
	//
	//        Trajectory* traj = graphS.extractTrajectory(Init, Goal);
	//        traj->replaceP3dTraj();
	//
	//        g3d_draw_allwin_active();
	//        return;
	//    }
	
#ifdef HRI_COSTSPACE
	
    if (bufferStr.compare("computeWorkspacePath") == 0)
    {
#ifdef HRI_PLANNER
        hriSpace->computeWorkspacePath();
#else
		cout << "HRI_PLANNER not compiled" << endl;
#endif
        g3d_draw_allwin_active();
        return;
    }
	
    if (bufferStr.compare("computeHoleManipulationPath") == 0)
    {
#ifdef HRI_PLANNER
        hriSpace->computeHoleManipulationPath();
#else
		cout << "HRI_PLANNER not compiled" << endl;
#endif
        g3d_draw_allwin_active();
        return;
    }
	
    if (bufferStr.compare("runHRICSRRT") == 0)
    {
		//        if( HRICS_MOPL->runHriRRT() )
		//        {
		//            Trajectory optimTrj(new Robot(XYZ_ROBOT),XYZ_ROBOT->tcur);
		//            if( !optimTrj.isValid() )
		//            {
		//                cout << "Trajector NOT VALID!!!"  << endl;
		//            }
		//            cout << "Trajectory mean coll test : "  << optimTrj.meanCollTest() << endl;
		//        }
		//        ENV.setBool(Env::drawTraj,true);
		//        g3d_draw_allwin_active();
		//        return;
    }
	
#endif
	
    //    if( )
    //    {
    //
    //    }
	
    else
    {
        printf("Error, pipe not implemented\n");
		//#ifdef CXX_PLANNER
		//        Graph* ptrGraph = new Graph(XYZ_GRAPH);
		//#endif
    }
	
}

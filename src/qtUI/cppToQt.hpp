/*
 * cppToQt.hpp
 *
 *  Created on: Aug 19, 2009
 *      Author: jmainpri
 */

#ifndef CPPTOQT_HPP_
#define CPPTOQT_HPP_

#include "qtLibrary.hpp"

#ifdef ENERGY
#include "bio/BioEnergy/include/Energy-pkg.h"
#endif

#if defined( CXX_PLANNER )
#include "plannerFunctions.hpp"
#include "MultiRun.hpp"
#endif

#if defined( WITH_OOMOVE3D ) 
#include "planner/plannerFunctions.hpp"
#include "utils/MultiRun.hpp"
#endif

#include "API/Trajectory/smoothing.hpp"
#include "API/Trajectory/costOptimization.hpp"
#include "API/Search/Dijkstra/dijkstra.hpp"

#ifdef HRI_COSTSPACE
#include "HRI_costspace/HRICS_Workspace.hpp"
#endif

#if defined( HRI_PLANNER ) && defined( HRI_COSTSPACE )
#include "HRI_costspace/HRICS_HAMP.hpp"
#endif

#include "Graphic-pkg.h"

#ifdef QT_GL
extern G3D_Window *G3D_WIN;
extern QSemaphore* sem;
#endif

// Planner functions
void qt_resetGraph();
void qt_drawAllWinActive();
void qt_runDiffusion();
void qt_runPRM();
void qt_shortCut();
void qt_optimize();

// Read/Write functions
void qt_readScenario();
void qt_saveScenario();
void qt_readTraj();

#ifdef HRI_COSTSPACE
void qt_load_HRICS_Grid(std::string gridName);
#endif

  /**
    * @ingroup qtWindow
    * @brief Function details the pipe between the XForm thread and the Qt Interface thread
    */
void read_pipe(int fd, void* data);
extern int qt_fl_pipe[2];
extern const char *qt_fileName;

#endif /* CPPTOQT_HPP_ */

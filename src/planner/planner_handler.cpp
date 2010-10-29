#include "planner_handler.hpp"
#include "planEnvironment.hpp"
#include "cppToQt.hpp"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"
#include "Util-pkg.h"
#include "Collision-pkg.h"
#include <QtCore/QMutexLocker>
#include <iostream>

extern int mainMhp(int argc, char** argv);

//------------------------------------------------------------------------------
PlannerHandler::PlannerHandler(int argc, char** argv) :
  mState(none),
  mArgc(argc),
  mArgv(argv)
{
}
//------------------------------------------------------------------------------
void PlannerHandler::init()
{
  mainMhp(mArgc, mArgv);
  emit initIsDone();
}
//------------------------------------------------------------------------------
void PlannerHandler::startPlanner(QString plannerName)
{
  if(mState == running) // already running, do nothing
  {
    printf("Error: PlannerHandler::startPlanner called, but a planner \
is already running.\n");
    return;
  }
  mState = running;
#ifdef P3D_PLANNER
  p3d_SetStopValue(FALSE);
#endif
  ENV.setBool(Env::isRunning, true);
  try
  {
    if(plannerName == "Diffusion")
    {
      std::cout << "Planning thread : starting diffusion." << std::endl;
      qt_runDiffusion();
    }
    else if(plannerName == "PRM")
    {
      std::cout << "Planning thread : starting PRM." << std::endl;
      qt_runPRM();
    }
  }
  catch(std::string what)
  {
    std::cerr << "Planner thread : caught exception : " << what << std::endl;
  }
  catch(std::exception& e)
  {
    std::cerr << "Planner thread : caught exception : " << e.what() << std::endl;
  }
  catch(...)
  {
    std::cerr << "Planner thread : caught exception of unknown type." << std::endl;
  }
  ENV.setBool(Env::isRunning, false);
  mState = stopped;
  emit plannerIsStopped();
}
//------------------------------------------------------------------------------
void PlannerHandler::stopPlanner()
{
  if(mState != running) // not running, do nothing
  {
    printf("Error: PlannerHandler::stopPlanner called, but there is no planner \
running.\n");
    return;
  }
#ifdef P3D_PLANNER
  p3d_SetStopValue(true);
#endif
  ENV.setBool( Env::isRunning, false );
  PlanEnv->setBool( PlanParam::stopPlanner, true );
}
//------------------------------------------------------------------------------
void PlannerHandler::resetPlanner()
{
  if(mState == running) // running, do nothing
  {
    printf("Error: PlannerHandler::resetPlanner called, but there is a planner \
currently running.\n");
    return;
  }
  if(mState == none) // no planner, do nothing
  {
    printf("Error: PlannerHandler::resetPlanner called, but there is no planner.\n");
    return;
  }
  mState = none;
  qt_resetGraph();
#ifdef P3D_PLANNER
  p3d_SetStopValue(false);
#endif
  PlanEnv->setBool( PlanParam::stopPlanner, false );
  emit plannerIsReset();
}
//------------------------------------------------------------------------------

//
//  replanning.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "replanning.hpp"

#include "API/Trajectory/costOptimization.hpp"
#include "API/project.hpp"

#include "planner/plannerFunctions.hpp"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"

#include "move3d-headless.h"

#include <sys/time.h>

using namespace std;
using namespace tr1;

// ---------------------------------------------------------------------------------
// Local
// ---------------------------------------------------------------------------------
static bool m_init=false;
static Robot* m_robot=NULL;


// ---------------------------------------------------------------------------------
// Getters 
// ---------------------------------------------------------------------------------
p3d_rob* replann_getRobot()
{
  if (!m_robot) {
    cout << "Replanning not initialized!!!" << endl;
    return NULL;
  }
  
  return m_robot->getRobotStruct();
}

// ---------------------------------------------------------------------------------
// Init function
// ---------------------------------------------------------------------------------
bool replann_initialize()
{
  m_robot = global_Project->getActiveScene()->getRobotByName( global_ActiveRobotName );
  return true;
}

// ---------------------------------------------------------------------------------
// Re-Planning
// ---------------------------------------------------------------------------------
p3d_traj* replanning_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint)
{
  cout << "* REPLANNING ***************" << endl;
  
  unsigned int runSmoothId = p3d_planner_functions_GetRunId();
  
  if ( robotPt->lpl_type == MULTI_LOCALPATH ) 
  {
    ManipulationUtils::printConstraintInfo(robotPt);
    p3d_multilocalpath_switch_to_linear_groups (robotPt);
    p3d_multilocapath_print_group_info(robotPt);
  }
  
  //  traj = pathPt;
  
  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  API::Trajectory oldTraj(rob,traj);
  
#ifdef QT_LIBRARY
  //oldTraj.setContextName( ENV.getString(Env::nameOfFile).toStdString() );
#endif
  
  unsigned int lastViaPoint = oldTraj.getNbOfPaths();
  
  if (lastViaPoint <= (unsigned int)deformationViaPoint) 
  {
    cout << "No optimization possible (lastViaPoint <= deformationViaPoint)" << endl;
    return NULL;
  }
  
  API::CostOptimization optimTrj = oldTraj.extractSubTrajectory( (unsigned int)deformationViaPoint, lastViaPoint-1 );
  
  Eigen::Vector3d WSPoint;
  WSPoint[0] = target[0];
  WSPoint[1] = target[1];
  WSPoint[2] = target[2];
  
  /**
   unsigned int validShortCutId,endId;
   double* q = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->testTransferPointToTrajectory(WSPoint, optimTrj,validShortCutId);
   if ( q ) 
   {
   endId = optimTrj.getNbOfPaths();
   
   vector<LocalPath*> path;
   
   shared_ptr<Configuration> q_source(optimTrj.getLocalPathPtrAt(validShortCutId)->getEnd());
   shared_ptr<Configuration> q_target(new Configuration(rob,q));
   
   //    cout << "q_source = " << endl;
   //    q_source->print(true);
   //    
   //    cout << "q_target = " << endl;
   //    q_target->print(true);
   
   LocalPath* pathPt = new LocalPath(q_source,q_target);
   
   if (!pathPt->isValid()) {
   cout << "ERROR : LocalPath Not Valid" << endl;
   }
   else {
   path.push_back( pathPt );
   
   //    vector<LocalPath*> courbe = optimTrj.getCourbe();
   //    optimTrj.copyPaths( courbe );
   optimTrj.replacePortion(validShortCutId+1,endId,path);
   }
   }
   */
  
  double optTime = 0.0;
  //  if(PlanEnv->getBool(PlanParam::withDeformation))
  //  {
  ENV.setBool(Env::FKShoot,true);
  optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , runSmoothId );
  ENV.setBool(Env::FKShoot,false);
  optTime += optimTrj.getTime();
  //  }
  
  cout << "optTime = " << optTime << endl;
  
  optimTrj.resetCostComputed();
  
  //XYZ_GRAPH->rrtCost2 = optimTrj.cost();
  
  //  if(PlanEnv->getBool(PlanParam::withShortCut))
  //  {
  //    optimTrj.runShortCut( ENV.getInt(Env::nbCostOptimize) , runId );
  //    optTime += optimTrj.getTime();
  //  }
  
  //-----------------------------------
  
  //  double dmax = global_Project->getActiveScene()->getDMax();
  //  double range = optimTrj.getRangeMax();
  //  
  //  cout << "** END CUTTING **************" << endl;
  //  cout << "NLP = " << floor( range / (15*dmax)) << endl;
  //  
  //  optimTrj.cutTrajInSmallLP( floor( range / (15*dmax) ) );
  
  //-----------------------------------
  
  vector<LocalPath*> courbe = optimTrj.getCourbe();
  optimTrj.copyPaths( courbe );
  
  oldTraj.replacePortion((unsigned int)deformationViaPoint, lastViaPoint, courbe);
  oldTraj.replaceP3dTraj();
  
  return rob->getTrajStruct();
}

// ---------------------------------------------------------------------------------
// Execute simulated traj
// ---------------------------------------------------------------------------------
int replann_execute_simulation_traj( int (*fct)(p3d_rob* robot, p3d_localpath* curLp) ) 
{
  if (!m_init) {
    replann_initialize();
    m_init = true;
  }
  
  p3d_rob* robotPt = m_robot->getRobotStruct();
  
  if (robotPt->tcur == NULL) 
  {
    PrintInfo(("execute_simulation_traj : no current trajectory\n"));
    return false;
  }
  
  pp3d_localpath localpathPt = robotPt->tcur->courbePt;
  
  if( localpathPt->type_lp != SOFT_MOTION )
  {
    PrintInfo(("execute_simulation_traj : the trajectory is not a softmotion\n"));
    return false;
  }
  
  bool RunShowTraj=true;
  
  configPt q;
  double tu = 0.0, tu_tmp = 0.0; //, ts = 0.0;
  
  int end = FALSE;
  int count = 0;
  double u = 0.0, du = 0.0; /* parameters along the local path */
  
  double umax = p3d_compute_traj_rangeparam(robotPt->tcur);
  
  ChronoOn();
  bool isFirstLoop=true;
  
  while ( (u < umax - EPS6) && RunShowTraj )
  {
    /* position of the robot corresponding to parameter u */
    q = p3d_config_at_param_along_traj(robotPt->tcur,u);
    p3d_set_and_update_robot_conf(q);
    
//    /* collision checking */
//#ifdef P3D_COLLISION_CHECKING
//    p3d_numcoll = p3d_col_test_all();
//#endif
//    m_robot->isInCollision();

	  RunShowTraj = (*fct_stop)();
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return (count);
    
    count++;
    
#ifdef MULTILOCALPATH
    if (localpathPt->type_lp == SOFT_MOTION)
    {
      timeval tim;
      gettimeofday(&tim, NULL);
      tu=tim.tv_sec+(tim.tv_usec/1000000.0);
      
      if(isFirstLoop)
      {
        tu_tmp = tu;
        isFirstLoop =false;
      }
      //    ChronoTimes(&tu,&ts);
      du = tu - tu_tmp;
      tu_tmp = tu;
      du *= ENV.getDouble(Env::showTrajFPS);
    } 
#endif
    
    u += du;
    //cout << "u = " << u << endl;
    if (u > umax - EPS6) {
      u = umax;
      end = TRUE;
    }
  }
  cout << "Final time(sec) on traj : " << umax << endl;
  ChronoPrint("");
  ChronoOff();
  return count;
}



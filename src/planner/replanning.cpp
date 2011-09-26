//
//  replanning.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "replanning.hpp"

#include "API/project.hpp"

#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"

#include "move3d-headless.h"

#include <boost/thread/thread.hpp>
#include <sys/time.h>

using namespace std;
using namespace tr1;

p3d_traj* replanning_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint);
p3d_traj* replanning_Deform_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint);
p3d_traj* replanning_RRT_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint);

//! The function pointer to the replanning function
p3d_traj* (*replann_fct)(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint) = NULL;

//! The manipulation planner used to convert a path into a trajectory
ManipulationPlanner* m_manipPlanner=NULL;

// ---------------------------------------------------------------------------------
// Local
// ---------------------------------------------------------------------------------
static bool m_init=false;
static Robot* m_robot=NULL;

static int m_BaseMLP=0;
static int m_HeadMLP=0;
static int m_UpBodyMLP=0;
static int m_UpBodySmMLP=0;

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

//! set mlp for this robot
void replann_set_MultiLP()
{
  for (int i = 0; m_robot && i < m_robot->getRobotStruct()->mlp->nblpGp; i++) {
    if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
      m_BaseMLP = i;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
      m_HeadMLP = i;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
      m_UpBodyMLP = i;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
      m_UpBodySmMLP = i;
    }
  }
}

//! invalidate all constraints
// -----------------------------------------------
void replann_invalidate_cntrts()
{
  if (!m_robot) {
    cout << "robot not initialized in file " 
    << __FILE__ << " ,  " << __func__ << endl;
    return;
  }
  
  p3d_rob* rob = m_robot->getRobotStruct();
  p3d_cntrt* ct;
  
  // over all constraints
	for(int i=0; i<rob->cntrt_manager->ncntrts; i++) 
	{
    // get constraint from the cntrts manager
    ct = rob->cntrt_manager->cntrts[i];
    p3d_desactivateCntrt( rob, ct );
  }
}

//! replanning initialiwation loop
// -----------------------------------------------
bool replann_initialize()
{
  cout << "Initializes Replanning" << endl;
  
  m_robot = global_Project->getActiveScene()->getRobotByName( global_ActiveRobotName );
  
  m_manipPlanner = new ManipulationPlanner( m_robot->getRobotStruct() );
  m_manipPlanner->setReplanningMethod( replanning_RRT_Function );
  
  m_init = true;
  
  return true;
}

bool replan_plan_initial_path()
{	
  if (m_robot == NULL || m_manipPlanner == NULL) {
    return false;
  }
  
  cout << "Planning Initial Path" << endl;
  MANIPULATION_TASK_TYPE_STR type = ARM_FREE;
  
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs;
                                   
  configPt qInit  = p3d_copy_config( m_robot->getRobotStruct(), m_robot->getRobotStruct()->ROBOT_POS );
  configPt qGoal  = p3d_copy_config( m_robot->getRobotStruct(), m_robot->getRobotStruct()->ROBOT_GOTO );
	
	std::string m_OBJECT_NAME;
  std::vector<double> objStart, objGoto;
  
  objGoto.resize(6);
  
  objGoto[0] = P3D_HUGE;
  objGoto[1] = P3D_HUGE;
  objGoto[2] = P3D_HUGE;
  
  objGoto[3] = P3D_HUGE;
  objGoto[4] = P3D_HUGE;
  objGoto[5] = P3D_HUGE;
	
	MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_NO_TRAJ_FOUND;
  
  string str = m_manipPlanner->robot()->name;
  
  cout << "Planning for robot : " << str << endl;
  
  if( str == "PR2_ROBOT" )
  {
    fixAllJointsWithoutArm(m_manipPlanner->robot(),0);
  }
  
	switch ( (unsigned int) m_manipPlanner->robot()->lpl_type ) 
	{
    case LINEAR : {
      cout << "armPlanTask :: Linear " << endl;
      gpGrasp grasp;
			status = m_manipPlanner->armPlanTask(type, 0, qInit, qGoal, objStart, objGoto, /* m_OBJECT_NAME.c_str() */ "", "", (char*)"", grasp, trajs);
			
			if(status == MANIPULATION_TASK_OK ) 
      {
				m_manipPlanner->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
				
				for(unsigned int i = 1; i < trajs.size(); i++){
					p3d_concat_traj(m_manipPlanner->robot()->tcur, trajs[i]);
				}
        
        //m_manipulation->setRobotPath(m_manipulation->robot()->tcur);
			}
			break;
		}
			
		case MULTI_LOCALPATH : {
      cout << "armPlanTask :: MultiLocalPath " << endl;
      gpGrasp grasp;
			status = m_manipPlanner->armPlanTask(type, 0, qInit, qGoal, objStart, objGoto, /*m_OBJECT_NAME.c_str()*/ "", "", (char*)"", grasp, confs, smTrajs);
			break;
    }
			
		case SOFT_MOTION : {
			cout << "Manipulation : localpath softmotion should not be called" << endl;
			break;
    }
	}
  
	return (status == MANIPULATION_TASK_OK );
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
// Re-Planning
// ---------------------------------------------------------------------------------
p3d_traj* replanning_Deform_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint)
{
  cout << "* REPLANNING ***************" << endl;
  
  unsigned int runSmoothId = p3d_planner_functions_GetRunId();
  
  if ( robotPt->lpl_type == MULTI_LOCALPATH ) 
  {
    ManipulationUtils::printConstraintInfo(robotPt);
    p3d_multilocalpath_switch_to_linear_groups (robotPt);
    p3d_multilocapath_print_group_info(robotPt);
  }

  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  API::Trajectory oldTraj(rob,traj);
  
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
  
  double optTime = 0.0;

  ENV.setBool(Env::FKShoot,true);
  optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize) , runSmoothId );
  ENV.setBool(Env::FKShoot,false);
  optTime += optimTrj.getTime();
  
  cout << "optTime = " << optTime << endl;
  
  optimTrj.resetCostComputed();
  
  vector<LocalPath*> courbe = optimTrj.getCourbe();
  optimTrj.copyPaths( courbe );
  
  oldTraj.replacePortion((unsigned int)deformationViaPoint, lastViaPoint, courbe);
  oldTraj.replaceP3dTraj();
  
  return rob->getTrajStruct();
}

// ---------------------------------------------------------------------------------
// Re-RRT
// ---------------------------------------------------------------------------------
p3d_traj* replanning_RRT_Function(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint)
{
  cout << "* REPLANNING ***************" << endl;
  
  if (traj == NULL) {
    cout << "Error : No intput path" << endl;
    return NULL;
  }
  
//  unsigned int runSmoothId = p3d_planner_functions_GetRunId();
  
  if ( robotPt->lpl_type == MULTI_LOCALPATH ) 
  {
    //ManipulationUtils::printConstraintInfo(robotPt);
    p3d_multilocalpath_switch_to_linear_groups (robotPt);
    //p3d_multilocapath_print_group_info(robotPt);
  }
  
  // Gets the robot pointer
  Robot* rob = global_Project->getActiveScene()->getRobotByName(robotPt->name);
  
  API::Trajectory oldTraj(rob,traj);
  
  unsigned int lastViaPoint = oldTraj.getNbOfPaths();
  
  if (lastViaPoint <= (unsigned int)deformationViaPoint) 
  {
    cout << "No optimization possible (lastViaPoint <= deformationViaPoint)" << endl;
    return NULL;
  }
  
  shared_ptr<Configuration> q_init = oldTraj.getLocalPathPtrAt( deformationViaPoint )->getBegin();
  shared_ptr<Configuration> q_goal = oldTraj.getLocalPathPtrAt( lastViaPoint-1 )->getEnd();
  
  // Delete graph
  if ( API_activeGraph ) 
  {
    delete API_activeGraph;
    API_activeGraph = NULL;
    cerr << "Delete C++ API Graph" << endl;
  }
  
  if( !p3d_del_graph(rob->getRobotStruct()->GRAPH) )
  {
    cerr << "XYZ_GRAPH allready deleted" << endl;
  }
  
  p3d_traj* path = planner_Function(rob->getRobotStruct(), 
                                    q_init->getConfigStruct(),  
                                    q_goal->getConfigStruct() );
  
  if (path != NULL) 
  {  
    smoothing_Function(rob->getRobotStruct(), path, 100, 4.0);
    
    API::Trajectory optimTrj( rob, path );
    
//    double optTime = 0.0;
    
    optimTrj.resetCostComputed();
    
    vector<LocalPath*> courbe = optimTrj.getCourbe();
    optimTrj.copyPaths( courbe );
    
    oldTraj.replacePortion((unsigned int)deformationViaPoint, lastViaPoint, courbe);
    oldTraj.replaceP3dTraj();
    
    return rob->getTrajStruct();
  }
  else 
  {
    return NULL;
  }
}

// ---------------------------------------------------------------------------------
// Replanning environment 
// ---------------------------------------------------------------------------------
static p3d_rob*     traj_sim_Robot=NULL;
static p3d_traj*    traj_sim_Traj=NULL;
static p3d_vector3  traj_sim_End;
static int          traj_sim_ViaPoint=0;
static bool         traj_sim_ReplanRunning=false;


//! executes the replanning function for a given
//! robot, trajectory, end workspace point, traj via point
void replann_fct_main()
{
  cout << "Start replanning" << endl;
  if( m_manipPlanner )
  {
    SM_TRAJ smTraj;
    m_manipPlanner->armReplan( traj_sim_End , traj_sim_ViaPoint, smTraj);
  }
  else
  {
    cout << "The manipulation planner is not initilized" << endl;
  }
  
  //traj_sim_Traj = (*replann_fct)(traj_sim_Robot, traj_sim_Robot->tcur, traj_sim_End, traj_sim_ViaPoint );
//  for (unsigned int i=0; i<1000000000; i++) {
//    int j=0;
//    j++;
//  }
  traj_sim_ReplanRunning = false;
}

// ---------------------------------------------------------------------------------
// Execute simulated replanning phase
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
  
  p3d_traj* traj = p3d_create_traj_by_copy(robotPt->tcur);
  
  pp3d_localpath localpathPt = traj->courbePt;
  
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
  
  traj_sim_Robot = robotPt;
  traj_sim_Traj = robotPt->tcur;
  traj_sim_ViaPoint = 0;
  traj_sim_ReplanRunning = false;
  
  ChronoOn();
  bool isFirstLoop=true;
  
  while ( (u < umax - EPS6) && RunShowTraj )
  {
    if( !traj_sim_ReplanRunning )
    {
      double tau =  u; // in seconds
      double t_rep = 2.0; // in seconds
      
      p3d_getQSwitchIDFromMidCVS(tau, t_rep, &traj_sim_ViaPoint);
      
      traj_sim_ReplanRunning = true;
      boost::thread workerThread( replann_fct_main );
//      workerThread.join();
    }
    
    /* position of the robot corresponding to parameter u */
    q = p3d_config_at_param_along_traj(traj,u);
    p3d_set_and_update_robot_conf(q);
    
    //robotPt->tcur = (*replann_fct)();
    
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



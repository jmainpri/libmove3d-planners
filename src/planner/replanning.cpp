//
//  replanning.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "replanning.hpp"

#include "API/project.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

#include "move3d-headless.h"

#include <boost/thread/thread.hpp>
#include <sys/time.h>

using namespace std;
using namespace tr1;

//! Extern variables

extern SM_TRAJ ManipPlannerLastTraj;


//! Local variables
static bool m_init=false;

static Robot* m_robot=NULL;
static Robot* m_rosim=NULL;
static int m_BaseMLP=0;
static int m_BaseSmMLP=0;
static int m_HeadMLP=0;
static int m_UpBodyMLP=0;
static int m_UpBodySmMLP=0;

static shared_ptr<Configuration> m_qSwitch;
static shared_ptr<Configuration> m_qGoal;

static API::Trajectory m_CurrentTraj;

static bool m_isPlanning=false;
static bool m_planningSucceded=true;
static int m_switch_id = 0;

ManipulationPlanner* m_manipPlanner=NULL;

static std::vector< std::vector<double> > m_currentLine;
static std::vector< std::vector<double> > m_lastLine;

//! Initialize the robot strucutres
void replan_init(string robotName)
{
  // Check if allready initialized
  m_robot = global_Project->getActiveScene()->getRobotByName( robotName );

  if (m_robot) 
  {
    for (int i = 0; i<m_robot->getRobotStruct()->mlp->nblpGp; i++) {
      if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
        m_BaseMLP = i;
      } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "baseSm")) {
        m_BaseSmMLP = i;
      } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
        m_HeadMLP = i;
      } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
        m_UpBodyMLP = i;
      } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
        m_UpBodySmMLP = i;
      }
    }
  }
}

//! Set the DoFs of the robot for navigation
void replan_init_for_navigation()
{  
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
  p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);
#endif
  
  // Fix all joints for planner
  fixAllJointsExceptBase( m_robot->getRobotStruct() );
  
  // Deactivate all kinematic constraints
  p3d_cntrt_management* ct_manager = m_robot->getRobotStruct()->cntrt_manager; 
  p3d_cntrt** cntrts = ct_manager->cntrts;
  
	for(int i=0; i<ct_manager->ncntrts; i++) 
	{
    p3d_desactivateCntrt( m_robot->getRobotStruct() , cntrts[i] );
  }
  
  // Set base to joint to draw
  p3d_set_user_drawnjnt(1);
}

//! Create a straight line trajectory
void replan_create_straightline()
{
  Robot* rob=NULL;
  
  if( m_robot && m_init )
  {
    rob = m_robot;
  }
  else
  {
    rob = global_Project->getActiveScene()->getActiveRobot();
  }
  
  if (!rob) {
    cout << "robot not initialized in file " 
    << __FILE__ << " ,  " << __func__ << endl;
    return;
  }
  
  shared_ptr<Configuration> q_init( rob->getInitialPosition() );
  shared_ptr<Configuration> q_goal( rob->getGoTo() );
  
  if( q_init->equal( *q_goal ) )
  {
    cout << "init equal q_goal in file "
    << __FILE__ << " ,  " << __func__ << endl; 
    return;
  }
  
  vector< shared_ptr<Configuration> > confs(2);
  
  confs[0] = q_init;
  confs[1] = q_goal;
  
  m_CurrentTraj = API::Trajectory( confs );
  m_CurrentTraj.replaceP3dTraj();
}

//! Optimize current traj
void replan_deform_trajectory()
{
	API::CostOptimization optimTrj( m_CurrentTraj );
	optimTrj.runDeformation(ENV.getInt(Env::nbCostOptimize));
	optimTrj.replaceP3dTraj();
}

//! Draws replanning features
void replan_draw()
{
  if(m_currentLine.size()>=2)
  {
    double color[4];
    color[0] = 1.0;
    color[1] = 0.8;
    color[2] = 0.2;
    color[3] = 1.0;
    
    for( int i=0; i<int(m_currentLine.size()-1); i++)
    {
      g3d_drawSphere(m_currentLine[i+0][0],m_currentLine[i+0][1],m_currentLine[i+0][2],0.05);
      
      g3d_drawOneLine(m_currentLine[i+0][0],m_currentLine[i+0][1],m_currentLine[i+0][2],
                      m_currentLine[i+1][0],m_currentLine[i+1][1],m_currentLine[i+1][2],Any,color);
      
      if( i == int(m_currentLine.size()-2))
      {
        g3d_drawSphere(m_currentLine[i+1][0],m_currentLine[i+1][1],m_currentLine[i+1][2],0.05);
      }
    }
  }
  
  if(m_lastLine.size()>=2)
  {
    double color[4];
    color[0] = 0.8;
    color[1] = 1.0;
    color[2] = 0.2;
    color[3] = 1.0;
    
    for( int i=0; i<int(m_lastLine.size()-1); i++)
    {
      g3d_drawSphere(m_lastLine[i+0][0],m_lastLine[i+0][1],m_lastLine[i+0][2],0.05);
      
      g3d_drawOneLine(m_lastLine[i+0][0],m_lastLine[i+0][1],m_lastLine[i+0][2],
                      m_lastLine[i+1][0],m_lastLine[i+1][1],m_lastLine[i+1][2],Any,color);
      
      if( i == int(m_lastLine.size()-2))
      {
        g3d_drawSphere(m_lastLine[i+1][0],m_lastLine[i+1][1],m_lastLine[i+1][2],0.05);
      }
    }
  }
}

//! Initialization phaze
bool replan_init_manipulationPlanner()
{
  if(m_init == true)
  {
    cout << "Allready initialized" << endl;
    return true;
  }
  
  replan_init("PR2_ROBOT");
  m_rosim = global_Project->getActiveScene()->getRobotByName("PR2_SIMUL");
  
  if( m_robot == NULL || m_rosim == NULL )
  {
    return m_init;
  }
  
  m_robot->getRobotStruct()->display_mode = P3D_ROB_NO_DISPLAY;
  m_currentLine.clear();
  m_lastLine.clear();
  
  g3d_set_multi_thread_mode( true );
  ext_g3d_draw_multi_thread = replan_draw;
  
  if( m_init == true )
  {
    m_manipPlanner =  new ManipulationPlanner(m_robot->getRobotStruct());
    m_manipPlanner->setUseBaseMotion( true );
    m_manipPlanner->setPlanningMethod( planner_Function );
    m_manipPlanner->setSmoothingMethod( smoothing_Function );
  }
  
  return m_init;
}

//! Concatanate the initial trajectory with the new one
//
bool replan_compute_softmotion(MANPIPULATION_TRAJECTORY_CONF_STR &confs, p3d_traj* traj, SM_TRAJ& smTraj)
{
  bool approximate = false;
  if (!traj) {
    cout << "SoftMotion : ERREUR : no generated traj\n";
    return false;
  }
  if (!traj || traj->nlp < 1) {
    cout << "Optimization with softMotion not possible: current trajectory contains one or zero local path\n";
    return false;
  }
  if (p3d_local_get_planner() != 9) {
    cout << "Optimization with softMotion not possible: current trajectory is not multi-localpath one\n";
    return false;
  }
  for (int i = 0; i < int((*traj->rob->armManipulationData).size()); i++) {
    ArmManipulationData& armData  = (*traj->rob->armManipulationData)[i];
    if (armData.getCartesian()) {
      approximate = true;
    }
  }
  if (p3d_convert_traj_to_softMotion(traj, ENV.getBool(Env::smoothSoftMotionTraj), true, 
                                     approximate, confs.first, confs.second, smTraj) == 1) {
    cout << "p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n";
    return false;
  }
  return true;
}

p3d_traj* replan_concat_to_current_traj(const vector<p3d_traj*>& trajs)
{
  if ( m_robot==NULL ) {
    cout <<  "concateneAllTrajectories: robot is NULL.\n";
    return NULL;
  }
  if ( trajs.empty() || !trajs[0] ) {
    cout <<  "concateneAllTrajectories: the trajectory vector is empty.\n" ;
    return NULL;
  }
  
  int init_id;
  p3d_traj* concatTraj;
  
  if (m_CurrentTraj.size()>0) {
    concatTraj = m_CurrentTraj.extractSubTrajectoryOfLocalPaths(0,m_switch_id-1).replaceP3dTraj(NULL);
    init_id = 0; 
  }
  else {
    concatTraj = trajs[0];
    init_id = 1; 
  }
  
  for (int i=init_id; i<int(trajs.size()); i++) 
  {
    if(trajs[i])
    {
      if (p3d_concat_traj( concatTraj, trajs[i]) == TRUE)
      {
        cout << "Concat traj fails" << endl;  
        return NULL;
      }
    }
  }
  //  m_robot->tcur = (*concatTraj);
  return concatTraj;
}

bool replan_compute_softmotion(p3d_traj* entireTraj)
{
  if (entireTraj != NULL)
  {
    MANPIPULATION_TRAJECTORY_CONF_STR conf;
    SM_TRAJ smTraj;
    replan_compute_softmotion(conf, entireTraj, smTraj);
    ManipPlannerLastTraj = smTraj;
    return true;
  }
  else
    return false;
}

bool replan_generate_new_trajectory(const vector<p3d_traj*>& trajs)
{
  if( m_planningSucceded )
  {
    p3d_traj* entireTraj = replan_concat_to_current_traj( trajs );
    
    if ( entireTraj == NULL ) 
    {
      cout << "Fail to concat traj" << endl;
      m_planningSucceded = false;
    }
    else if( !replan_compute_softmotion(entireTraj) )
    {
      cout << "Fail to compute softmotion" << endl;
      m_planningSucceded = false;
    }
  }
  
  if( m_planningSucceded )
  {
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct(), false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false );
    
    p3d_traj* lin_traj =  p3d_get_last_linear_traj();
    
    if( lin_traj )
    {
      m_CurrentTraj = API::Trajectory( m_robot, lin_traj );
    }
    else
    {
      cout << "Failed to get linear traj" << endl;
      m_planningSucceded = false;
    }
  }
  
  return m_planningSucceded;
}

bool replan_plan_trajectory()
{	  
  if (m_robot == NULL || m_manipPlanner == NULL) 
  {
    cout << "replanning not well initialized" << endl;
    return false;
  }
  
  MANIPULATION_TASK_TYPE_STR type = ARM_FREE;
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_NO_TRAJ_FOUND;
	vector <p3d_traj*> trajs;
	string m_OBJECT_NAME;
  vector<double> objStart, objGoto;
  objGoto.resize(6,P3D_HUGE);
  
  configPt qInit  = m_qSwitch->getConfigStructCopy();
  configPt qGoal  = m_qGoal->getConfigStructCopy();
  
  string str = m_manipPlanner->robot()->name;
  
  cout << "Planning for robot : " << str << endl;
  
  if( str == "PR2_ROBOT" ) 
  {
    fixAllJointsWithoutArm(m_manipPlanner->robot(),0);
  }
  
	switch ( (unsigned int) m_manipPlanner->robot()->lpl_type ) 
	{			
		case MULTI_LOCALPATH : 
    {
      cout << "armPlanTask :: MultiLocalPath" << endl;
      gpGrasp grasp;
      trajs.clear();
      status = m_manipPlanner->armPlanTask(type, 0, qInit, qGoal, objStart, objGoto,  
                                           "", "", "", grasp, trajs);
			break;
    }
      
    case LINEAR :
      cout << "Manipulation : linear not implemented" << endl;
      break;
      
    case SOFT_MOTION : 
      cout << "Manipulation : softmotion not implemented" << endl;
      break;
	}
  
  m_isPlanning = false;
  m_planningSucceded = ( status == MANIPULATION_TASK_OK );
  m_planningSucceded = ( replan_generate_new_trajectory(trajs) );
	return m_planningSucceded;
}

//! Before execution
//! robot, trajectory, end workspace point, traj via point
bool replan_init_execution()
{
  if (!m_init) 
  {
    if(replan_init_manipulationPlanner())
    {
      m_init = true;
    }
  }
  
  return m_init;
}

//! Before execution
//! robot, trajectory, end workspace point, traj via point
bool replan_init_simple_replanning()
{
  replan_init("PR2_ROBOT");
  replan_init_for_navigation();
  replan_create_straightline();
  return true;
}

//! Stores the trajectory in a vector
void replan_store_traj_to_vect(SM_TRAJ& smTraj, double current, double step)
{
  m_lastLine = m_currentLine;
  //m_lastLine.clear();
  m_currentLine.clear();
  
  for( double t=current; t<smTraj.getDuration(); t += step)
  {
    std::vector<SM_COND> cond;
    smTraj.getMotionCond(t,cond);
    
    vector<double> point(3);
    point[0] = cond[0].x;
    point[1] = cond[1].x;
    point[2] = 0.2;
    
    m_currentLine.push_back( point );
  }
}

//! Stores the trajectory in a vector
void replan_store_traj_to_vect(API::Trajectory& traj, double step)
{
  m_lastLine = m_currentLine;
  //m_lastLine.clear();
  m_currentLine.clear();
  
  for( double t=0.0; t<traj.getRangeMax(); t += step)
  {
    Configuration q(*traj.configAtParam(t));
    
    vector<double> point(3);
    point[0] = q[6];
    point[1] = q[7];
    point[2] = q[11];
    
    m_currentLine.push_back( point );
  }
}

//! first call stores the time of day in t_init
//! the next calls will returns the substraction
//! between timofday and the argument t_init
double replan_time_since_last_call(bool& is_first_call, double& t_init)
{
  timeval tim;
  gettimeofday(&tim, NULL);
  double tu=tim.tv_sec+(tim.tv_usec/1000000.0);
  
  if (is_first_call) {
    t_init = tu;
    is_first_call = false;
  }
  
  double dt = tu - t_init;
  t_init = tu;
  return dt;
}

// ---------------------------------------------------------------------------------
// Execute simulated replanning phase
// ---------------------------------------------------------------------------------
int replan_execute_simple_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) ) 
{
  if(!replan_init_simple_replanning())
  {
    cout << "replan_init_simple_replanning did not work!!!" << endl;
  }
  
  bool RunShowTraj=true;
  bool StopRun=false;
  bool isFirstLoop=true;
  
  int count=0;
  
  double t = 0.0, t_init = 0.0, lastFloor=0.0;
  
  p3d_rob* robotPt;
  p3d_localpath* localpathPt;
  
  boost::thread workerThread( replan_deform_trajectory );
  
  while ( !StopRun )
  { 
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return(count);
    count++;
    
	  RunShowTraj = (*fct_stop)();
    
    t += replan_time_since_last_call( isFirstLoop, t_init);
    
    if (floor(t) > lastFloor) {
      cout << "t = " << t << endl;
      lastFloor = floor(t);
    }
    
    if (PlanEnv->getBool(PlanParam::stopPlanner)) {
      StopRun = true;
    }
    
    if (!m_isPlanning) 
    {
      cout << "Planning Ended!!!" << endl;
      StopRun = true;
    }
  }
  
  return 0;
}

// ---------------------------------------------------------------------------------
// Execute simulated replanning phase
// ---------------------------------------------------------------------------------
int replan_execute_simulation_traj( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) ) 
{
  if( !replan_init_execution() )
  {
    cout << "replan_init_execution did not work!!!" << endl;
    return false;
  }
  
  p3d_rob* robotPt;
  p3d_localpath* localpathPt;
  SM_TRAJ smTraj;
  
  bool RunShowTraj=true;
  
  Configuration q(m_rosim);
  
  double tu_tmp = 0.0; //, ts = 0.0;
  double t_max = 0.0, t = 0.0, t_init = 0.0, lastFloor=0.0; /* parameters along the local path */
  
  ChronoOn();
  bool isFirstLoop=true;
  bool StopRun=false;
  bool is_switch_done=true;
  
  int count=0;
  
  // Draws the second PR2 transparent
  m_rosim->getRobotStruct()->draw_transparent = true;
  
  // Deactivate robot to simulation robot collision checking
  p3d_col_deactivate_rob_rob(m_robot->getRobotStruct(),
                             m_rosim->getRobotStruct());
  
  // Set global variables
  m_qSwitch = m_robot->getCurrentPos();
  m_qGoal = m_robot->getGoTo();
  m_CurrentTraj = API::Trajectory(m_robot);
  m_isPlanning=false;
  m_lastLine.clear();
  m_currentLine.clear();
  
  if( !replan_plan_trajectory() )
  {
    cout << "Failed to find an inital plan" << endl;
    return false;
  }
  
  // Initial values
  smTraj = ManipPlannerLastTraj;
  t = 0.0;
  t_init = 0.0;
  t_max = smTraj.getDuration();
  is_switch_done = true;
  
  // Store trajectory for drawing
  replan_store_traj_to_vect(smTraj, 0.0, 1.0);
  
  // Replanning values
  const double t_rep = 5.0; // in seconds
  double t_switch= t_max;
  
  while ( !StopRun )
  { 
    // Switch to new trajectory
    if( (t>t_switch) && !m_isPlanning && m_planningSucceded )
    {
      smTraj = ManipPlannerLastTraj;
      t_init = 0.0;
      t_max = smTraj.getDuration();
      is_switch_done = true;
      
      // Store trajectory for drawing
      replan_store_traj_to_vect( smTraj, 0, 1.0 );
    }
    
    // Launch new planning
    if( is_switch_done && !m_isPlanning )
    {
      m_isPlanning = true;
      is_switch_done = false;
      StopRun = true;
      
      if ( m_planningSucceded ) 
      {
        p3d_getQSwitchIDFromMidCVS( t, t_rep, &m_switch_id );
        p3d_getMidCVSTimeOnTraj( m_switch_id, t_switch );
        
        cout << "q switch id : " << m_switch_id << endl;
        cout << "m_CurrentTraj.geMaxParam() : " << m_CurrentTraj.getRangeMax() << endl;
        
        if ( m_switch_id < (m_CurrentTraj.getNbOfViaPoints()-1) )
        {
          m_qSwitch = m_CurrentTraj[m_switch_id];
          boost::thread workerThread( replan_plan_trajectory );
          StopRun = false;
        }
        else if ( m_switch_id == (m_CurrentTraj.getNbOfViaPoints()-1) )
        {
          cout << "Last replanning!!!" << endl;
          StopRun = false;
        }
      }
    }
    
    // Show current trajectory
    std::vector<SM_COND> cond;
    smTraj.getMotionCond(t-t_init,cond);
    
    q = *m_rosim->getCurrentPos();
    
    q[6] = cond[0].x;
    q[7] = cond[1].x;
    q[11] = cond[5].x;
    // torso
    q[12] = cond[6].x;
    //head
    q[13] = cond[7].x;
    q[14] = cond[8].x;
    //leftarm
    q[25] = cond[19].x;
    q[26] = cond[20].x;
    q[27] = cond[21].x;
    q[28] = cond[22].x;
    q[29] = cond[23].x;
    q[30] = cond[24].x;
    q[31] = cond[25].x;
    //rightarm
    q[16] = cond[10].x;
    q[17] = cond[11].x;
    q[18] = cond[12].x;
    q[19] = cond[13].x;
    q[20] = cond[14].x;
    q[21] = cond[15].x;
    q[22] = cond[16].x;
    
    // set and update robot from configuration
    m_rosim->setAndUpdate(q);
    
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return(count);
    count++;
    
	  RunShowTraj = (*fct_stop)();
    
    t += replan_time_since_last_call( isFirstLoop, tu_tmp );
    
    if (floor(t) > lastFloor) {
      cout << "t = " << t << endl;
      lastFloor = floor(t);
    }
    
    if (t > t_max - EPS6) {
      t = t_max;
      StopRun = true;
    }
    
    if (PlanEnv->getBool(PlanParam::stopPlanner)) {
      StopRun = true;
    }
    
    if (!m_planningSucceded) 
    {
      cout << "Planning Failed!!!" << endl;
      StopRun = true;
    }
  }
  
  cout << "Final time(sec) on traj : " << t_max << endl;
  ChronoPrint("");
  ChronoOff();
  PlanEnv->setBool(PlanParam::stopPlanner,true);
  return 0;
}

//
//  replanning.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "replanningAlgorithms.hpp"
#include "replanningSimulators.hpp"

#include "API/project.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/cost_space.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"
#include "utils/ConfGenerator.h"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

#include "move3d-headless.h"

#include <boost/thread/thread.hpp>
#include <sys/time.h>

using namespace std;
using namespace tr1;

//! Deactivate all kinematic constraints
void p3d_deactivate_all_cntrts( Robot* r )
{
  p3d_rob* rob = r->getRobotStruct();
  
  int nb_cntrts = rob->cntrt_manager->ncntrts;
  p3d_cntrt** cntrts = rob->cntrt_manager->cntrts;
  
	for(int i=0; i<nb_cntrts; i++) 
    p3d_desactivateCntrt( rob , cntrts[i] );
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

//! Initialize replanner by setting the robot, the multi-localpaths available
//! and calling the init function
Replanner::Replanner(Robot* r) : m_robot(r), m_human(NULL), m_useMLP(false)
{
  cout << "new base replanner" << endl;
  
  if( !init_mlp() )
  {
    cout << "Error initializing multi-localpahts in " << __FILE__ << " at " << __func__ << endl;
  }
}

Replanner::~Replanner() 
{

}

void Replanner::setHuman(Robot* hum)
{
  m_human = hum;
}

void Replanner::setSwitchData( confPtr_t qSwitch, int switch_id, double t_rep, double lp_avera_length, double initial_step )
{
  m_qSwitch = qSwitch;
  m_switch_id = switch_id;
  m_t_rep = t_rep;
  m_lp_avera_length = lp_avera_length;
  m_initial_step = initial_step;
}

//! Initialize the robot multilocal path
//! disable the display of the main robot
bool Replanner::init_mlp()
{
  m_useMLP = false;
  
  if (m_robot == NULL)
    return false;
  
  for (int i=0; i<m_robot->getRobotStruct()->mlp->nblpGp; i++) 
  {
    if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "base")) {
      m_BaseMLP = i;
      m_useMLP = true;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "baseSm")) {
      m_BaseSmMLP = i;
      m_useMLP = true;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "head")) {
      m_HeadMLP = i;
      m_useMLP = true;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBody")) {
      m_UpBodyMLP = i;
      m_useMLP = true;
    } else if (!strcmp(m_robot->getRobotStruct()->mlp->mlpJoints[i]->gpName, "upBodySm")) {
      m_UpBodySmMLP = i;
      m_useMLP = true;
    }
  }
  
  cout << "Replanner::robot : " << m_robot->getName() << endl;
  cout << "Replanner::m_useMLP : " << m_useMLP << endl;
  m_init = true;
  return true;
}

//! Concat the new portion of the trajectory which is going to be used 
//! The concatanation happens at swith id which has to be computed before
p3d_traj* Replanner::concat_to_current_traj(const vector<p3d_traj*>& trajs)
{
  if ( m_robot==NULL ) {
    cout <<  "concat_to_current_traj: robot is NULL.\n";
    return NULL;
  }
  if ( trajs.empty() || !trajs[0] ) {
    cout <<  "concat_to_current_traj: the trajectory vector is empty.\n" ;
    return NULL;
  }
  
  // if the CurrentTraj is not empty
  // concat the allready executed part begore m_switch_id to the new trajectory
  int init_id;
  p3d_traj* concatTraj;
  if ( !m_CurrentTraj.isEmpty() ) {
    concatTraj = m_CurrentTraj.extractSubTrajectoryOfLocalPaths(0,m_switch_id-1).replaceP3dTraj(NULL);
    init_id = 0; 
  }
  else {
    concatTraj = trajs[0];
    init_id = 1; 
  }
  
  for (int i=init_id; i<int(trajs.size()); i++) {
    if(trajs[i]) {
      if (p3d_concat_traj( concatTraj, trajs[i]) == TRUE) {
        cout << "Concat traj fails" << endl;  
        return NULL;
      }
    }
  }
  
  return concatTraj;
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
SimpleReplanner::SimpleReplanner(Robot* r) : Replanner(r)
{
  cout << "new simple replanner" << endl;
  
  // Call to the virtual function
  if( !init() )
  {
    cout << "Error initializing virtual function of the replanner in " << __FILE__ << " at " << __func__ << endl;
  }
}

SimpleReplanner::~SimpleReplanner()
{
  
}

//! Before simple (no softmotion) replanning execution, this function initilizes the replanner data
//! First 
bool SimpleReplanner::init()
{
  m_isPlanning = false;
  
  set_active_joints( PlanEnv->getInt(PlanParam::setOfActiveJoints) );
  init_active_joints();
  
  // Store initial and final configurations
  m_qSwitch = m_robot->getCurrentPos();
  m_qGoal   = m_robot->getGoTo();
  
  m_idRun = 0;
  
  if( !init_create_straightline() )
  {
    cout << "Error : could not create straightline" << endl;
    return false;
  }

  return true;
}

//! Creates a straight line trajectory
//! uses the main robot of the simulator or uses the current
//! robot if not initialized
bool SimpleReplanner::init_create_straightline()
{
  Robot* rob=NULL;
  
  if( m_robot != NULL ) {
    rob = m_robot;
  }
  else {
    rob = global_Project->getActiveScene()->getActiveRobot();
  }
  
  if (!rob) {
    cout << "Robot not initialized in file " << __FILE__ << " ,  " << __func__ << endl;
    return false;
  }
  
  confPtr_t q_init( rob->getInitialPosition() );
  confPtr_t q_goal( rob->getGoTo() );
  
  if( q_init->equal( *q_goal ) )
  {
    cout << "Init equal q_goal in file " << __FILE__ << " ,  " << __func__ << endl; 
    cout << "for robot : " << rob->getName() << endl;
    q_init->print();
    q_goal->print();
    return false;
  }
  
  vector<confPtr_t> confs(2);
  
  confs[0] = q_init;
  confs[1] = q_goal;
  
  m_CurrentTraj = API::Trajectory( confs );
  m_CurrentTraj.replaceP3dTraj();
  return true;
}

void SimpleReplanner::init_for_manipuation()
{
  if( m_useMLP )
  {
    cout << "Set upbody MLP" << endl;
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
#endif
    
    // Fix all the joints but unfixes the arm 0
    fixAllJointsWithoutArm( m_robot->getRobotStruct(), 0 );
  }
  
  // Deactivate all kinematic constraints
  p3d_deactivate_all_cntrts( m_robot );
}

void SimpleReplanner::init_for_mobile_manip()
{
  if( m_useMLP )
  {
    cout << "Set upbody MLP" << endl;
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);
#endif
  }
  
  // Fix all the joints but unfixes the arm 0
  fixAllJointsWithoutArm( m_robot->getRobotStruct(), 0 );
  
  // Unfix base joint for planning
  unFixJoint( m_robot->getRobotStruct(), m_robot->getRobotStruct()->baseJnt );
  
  // Deactivate all kinematic constraints
  p3d_deactivate_all_cntrts( m_robot );
}

//! Set the DoFs active/passive of the robot for navigation
void SimpleReplanner::init_for_navigation()
{  
  if( m_useMLP )
  {
    cout << "Set base MLP" << endl;
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
    p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_BaseMLP, 1, false);
#endif
    
    // Fix all joints for planner
    // This only works with manipulation planner
    fixAllJointsExceptBase( m_robot->getRobotStruct() );
  }
  
  // Deactivate all kinematic constraints
  p3d_deactivate_all_cntrts( m_robot );
  
  // Set base to joint to draw
  p3d_set_user_drawnjnt(1);
}

//! Choose what part of the robot are active in the planning phase
//! Three type of planning to chose from
//! Navigation, Manipulation, Mobile-anipulation
void SimpleReplanner::set_active_joints(int type)
{
  switch (type) 
  {
    case 0:
      m_ReplanningType = NAVIGATION;
      break;
      
    case 1:
      m_ReplanningType = MANIPULATION;
      break;
      
    case 2:
      m_ReplanningType = MOBILE_MANIP;
      break;
      
    default:
      cout << "Planner type not implemented" << endl;
      break;
  }
}

void SimpleReplanner::init_active_joints()
{
  // Block specific joint sampling
  // also activate localpaths
  switch (m_ReplanningType) 
  {
    case NAVIGATION:
      cout << "Init for Navigation" << endl;
      init_for_navigation();
      break;
      
    case MANIPULATION:
      cout << "Init for Manipulation" << endl;
      init_for_manipuation();
      break;
      
    case MOBILE_MANIP:
      cout << "Init for Mobile-manipulation" << endl;
      init_for_mobile_manip();
      break;
  }
}

void SimpleReplanner::run()
{
  m_isPlanning = true;
  
  // Set 0.3 seconds to do the concat traj
  PlanEnv->setBool( PlanParam::planWithTimeLimit, true );
  PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
  PlanEnv->setDouble( PlanParam::timeLimitSmoothing, m_t_rep-0.3 );
  
  cout << "m_t_rep : " << m_t_rep << endl;
  
  // Run deformation from last path id
  int last_path_id = m_CurrentTraj.getNbOfPaths()-1;
  
	API::CostOptimization optimTrj(m_CurrentTraj.extractSubTrajectoryOfLocalPaths(m_switch_id,last_path_id));
  
  optimTrj.setStep( m_initial_step );
	optimTrj.runDeformation( ENV.getInt(Env::nbCostOptimize), m_idRun++ );
	optimTrj.cutTrajInSmallLP( optimTrj.getRangeMax() / m_lp_avera_length );
  optimTrj.replaceP3dTraj();
  
  vector<p3d_traj*> traj(1);
  traj[0] = optimTrj.replaceP3dTraj(NULL);
  
  // Concatanate traj and store it to current traj
  p3d_traj* lin_traj = concat_to_current_traj(traj);
  
  if( lin_traj )
  {
    m_CurrentTraj = API::Trajectory( m_robot, lin_traj );
    m_planningSucceded = true;
  }
  else {
    m_planningSucceded = false;
  }
  
  if( optimTrj.getBegin() != m_qSwitch ) {
    cout << "Different q switch" << endl;
  }
  
  //store_traj_to_draw( optimTrj 0.0 );
  cout << "End replanning : " << __func__ << endl;
  m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
RRTReplanner::RRTReplanner(Robot* r) : SimpleReplanner(r)
{
  cout << "new star replanner" << endl;
}

RRTReplanner::~RRTReplanner()
{
  
}

bool RRTReplanner::init()
{
  return true;
}

void RRTReplanner::run()
{
  cout << "RRTReplanner::run"<<  endl;
 
  m_isPlanning = true;
  
  // Set 0.3 seconds to do the concat traj
  PlanEnv->setBool( PlanParam::planWithTimeLimit, true );
  PlanEnv->setDouble(PlanParam::timeLimitPlanning, m_t_rep-0.3 );
  
  int last_path_id = m_CurrentTraj.getNbOfPaths()-1;
	API::Trajectory traj(m_CurrentTraj.extractSubTrajectoryOfLocalPaths( m_switch_id, last_path_id) );

  double t_init = 0.0;
  double time = 0.0;
  //bool first_call_to_chrono=true;
  
  m_idRun++;
  
  // Get the initial time of planning
  timeval tim;
  gettimeofday(&tim, NULL); t_init=tim.tv_sec+(tim.tv_usec/1000000.0);
  
  p3d_traj* path = p3d_planner_function( m_robot->getRobotStruct(), 
                                        traj.getBegin()->getConfigStruct(), 
                                        traj.getEnd()->getConfigStruct() );
  
  gettimeofday(&tim, NULL); time=tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
  
  // Set the time left for smoothing
  PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
  PlanEnv->setDouble( PlanParam::timeLimitSmoothing, m_t_rep-0.3-time );
  
  if( path != NULL && 
     !PlanEnv->getBool(PlanParam::stopPlanner) && 
     PlanEnv->getBool(PlanParam::withSmoothing) )
  {
    p3d_smoothing_function( m_robot->getRobotStruct(), path, 100, 4.0);
  }
  
  p3d_traj* lin_traj = NULL;
  
  if( path ) {
    
    // Cutt the new trajectory in lp average piecese
    API::Trajectory path_( m_robot, path );
    path_.cutTrajInSmallLP( traj.getRangeMax() / m_lp_avera_length );
    path_.replaceP3dTraj();
    
    vector<p3d_traj*> traj(1);
    traj[0] = path_.replaceP3dTraj(NULL);
    
    // Concatanate traj and store it to current traj
    lin_traj = concat_to_current_traj( traj );
  }
  
  if( lin_traj )
  {
    m_CurrentTraj = API::Trajectory( m_robot, lin_traj );
    m_planningSucceded = true;
  }
  else {
    m_planningSucceded = false;
  }
  
  //store_traj_to_draw( optimTrj 0.0 );
  cout << "End replanning : " << __func__ << endl;
  m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
HandoverReplanner::HandoverReplanner(Robot* r) : SimpleReplanner(r)
{
  cout << "new star replanner" << endl;
}

HandoverReplanner::~HandoverReplanner()
{
  
}

bool HandoverReplanner::init()
{
  return true;
}

std::pair<bool,confPtr_t> HandoverReplanner::newGoal()
{  
  configPt q = NULL; 
  double dt = 0.0;
  
  timeval tim;
  gettimeofday(&tim, NULL);
  double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);
  
  ConfGenerator generator( m_robot, m_human );
  
  Eigen::Vector3d point = m_human->getJoint("rPalm")->getVectorPos();
  point[2] += 0.10;
  
  // if the generator finds a configuration, return it otherwise return null configuration
  if( generator.computeRobotIkForGrabing( q, point ) )
  {
    gettimeofday(&tim, NULL); double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
    cout << "Valid robotIk computed in : " << dt << " sec" << endl;
    confPtr_t q_rob( new Configuration( m_robot, q ) );
    return std::make_pair( true, q_rob );
  }
  else
  {
    gettimeofday(&tim, NULL); dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
    cout << "Could not compute a valid robot IK configuration in : " << dt << "sec" << endl;
    p3d_destroy_config(m_robot->getRobotStruct(), q);
    confPtr_t q_rob( new Configuration( m_robot, NULL ) );
    return std::make_pair( false, q_rob );
  }
}

void HandoverReplanner::run()
{
  cout << "HandoverReplanner::run"<<  endl;
  
  m_isPlanning = true;
  
  // Set 0.3 seconds to do the concat traj
  PlanEnv->setBool( PlanParam::planWithTimeLimit, true );
  PlanEnv->setDouble(PlanParam::timeLimitPlanning, m_t_rep-0.3 );
  
  int last_path_id = m_CurrentTraj.getNbOfPaths()-1;
	API::Trajectory traj(m_CurrentTraj.extractSubTrajectoryOfLocalPaths( m_switch_id, last_path_id) );
  
  double t_init = 0.0;
  double time = 0.0;
  //  bool first_call_to_chrono=true;
  
  m_idRun++;
  
  // Get the initial time of planning
  timeval tim;
  gettimeofday(&tim, NULL); t_init=tim.tv_sec+(tim.tv_usec/1000000.0);
  
  p3d_traj* path = p3d_planner_function( m_robot->getRobotStruct(), 
                                        traj.getBegin()->getConfigStruct(), 
                                        traj.getEnd()->getConfigStruct() );
  
  //  // Set the time left for smoothing
  //  time = global_rePlanningEnv->time_since_last_call( first_call_to_chrono, t_init );
  gettimeofday(&tim, NULL); time=tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
  
  PlanEnv->setDouble( PlanParam::timeLimitSmoothing, m_t_rep-0.3-time );
  
  if( path != NULL && 
     !PlanEnv->getBool(PlanParam::stopPlanner) && 
     PlanEnv->getBool(PlanParam::withSmoothing) )
  {
    p3d_smoothing_function( m_robot->getRobotStruct(), path, 100, 4.0);
  }
  
  p3d_traj* lin_traj = NULL;
  
  if( path ) {
    
    // Cutt the new trajectory in lp average piecese
    API::Trajectory path_( m_robot, path );
    path_.cutTrajInSmallLP( traj.getRangeMax() / m_lp_avera_length );
    path_.replaceP3dTraj();
    
    vector<p3d_traj*> concat_traj(1);
    concat_traj[0] = path_.replaceP3dTraj(NULL);
    
    // Concatanate traj and store it to current traj
    lin_traj = concat_to_current_traj( concat_traj );
  }
  
  if( lin_traj )
  {
    m_CurrentTraj = API::Trajectory( m_robot, lin_traj );
    m_planningSucceded = true;
  }
  else {
    m_planningSucceded = false;
  }
  
  //store_traj_to_draw( optimTrj 0.0 );
  cout << "End replanning : " << __func__ << endl;
  m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
AStarReplanner::AStarReplanner(Robot* r) : SimpleReplanner(r)
{
  cout << "new star replanner" << endl;
}

AStarReplanner::~AStarReplanner()
{
  
}

bool AStarReplanner::init()
{
  m_navigation = new HRICS::Navigation( m_robot );
  return true;
}

void AStarReplanner::run()
{
  cout << "AStarReplanner::run"<<  endl;
  
  m_isPlanning = true;
  
  m_navigation->reset();
  
  int last_path_id = m_CurrentTraj.getNbOfPaths()-1;
  API::Trajectory traj( m_CurrentTraj.extractSubTrajectoryOfLocalPaths( m_switch_id, last_path_id ) );
  
  m_idRun++;
  
  API::Trajectory* path_ = m_navigation->computeRobotTrajectory( traj.getBegin(), traj.getEnd() );
  
  p3d_traj* lin_traj = NULL;
  
  if( path_ ) 
  {
    // Cut the new trajectory in lp average piecese
    path_->cutTrajInSmallLP( traj.getRangeMax() / m_lp_avera_length );
    
    vector<p3d_traj*> concat_traj(1);
    concat_traj[0] = path_->replaceP3dTraj(NULL);
    
    // Concatanate traj and store it to current traj
    lin_traj = concat_to_current_traj( concat_traj );
  }
  
  if( lin_traj )
  {
    m_CurrentTraj = API::Trajectory( m_robot, lin_traj );
    m_planningSucceded = true;
  }
  else {
    m_planningSucceded = false;
  }
  
  global_rePlanningEnv->store_traj_to_draw( *path_, m_CurrentTraj.getRangeMax()/m_CurrentTraj.getNbOfPaths() );
  delete path_;
  
  //store_traj_to_draw( optimTrj 0.0 );
  cout << "End replanning : " << __func__ << endl;
  m_isPlanning = false;
}



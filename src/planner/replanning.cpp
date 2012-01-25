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
#include "planner/cost_space.hpp"
#include "planner/Diffusion/Variants/Star-RRT.hpp"

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

ReplanningSimulator* global_rePlanningEnv=NULL;

// The replanner drawing function
void replan_current_draw()
{
  if( global_rePlanningEnv )
    global_rePlanningEnv->draw();
}

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
Replanner::Replanner(Robot* r) : m_robot(r) , m_useMLP(false)
{
  cout << "new base replanner" << endl;
  
  // Check if allready initialized
  if( !m_init )
  {
    if( !init_mlp() )
    {
      cout << "Error initializing multi-localpahts in " << __FILE__ << " at " << __func__ << endl;
    }
  }
}

Replanner::~Replanner() 
{

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

//! Before simple (no softmotion) replanning execution this function initilizes the replanner data
//! First 
bool SimpleReplanner::init()
{
  m_isPlanning = false;
  
  set_planner_type( PlanEnv->getInt(PlanParam::plannerType) );
  init_planner_type();
  
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
  
  shared_ptr<Configuration> q_init( rob->getInitialPosition() );
  shared_ptr<Configuration> q_goal( rob->getGoTo() );
  
  if( q_init->equal( *q_goal ) )
  {
    cout << "Init equal q_goal in file " << __FILE__ << " ,  " << __func__ << endl; 
    cout << "for robot : " << rob->getName() << endl;
    q_init->print();
    q_goal->print();
    return false;
  }
  
  vector< shared_ptr<Configuration> > confs(2);
  
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
  // p3d_set_user_drawnjnt(1);
}

//! Choose what part of the robot are active in the planning phase
//! Three type of planning to chose from
//! Navigation, Manipulation, Mobile-anipulation
void SimpleReplanner::set_planner_type(int type)
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

void SimpleReplanner::init_planner_type()
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
  PlanEnv->setBool(PlanParam::withTimeLimit, true );
  PlanEnv->setDouble(PlanParam::optimTimeLimit, m_t_rep-0.3 );
  
  cout << "m_t_rep : " << m_t_rep << endl;
  
  // Run deformation from last path id
  int last_path_id = m_CurrentTraj.getNbOfPaths()-1;
  
	API::CostOptimization optimTrj(m_CurrentTraj.extractSubTrajectoryOfLocalPaths(m_switch_id,last_path_id));
  
  optimTrj.setStep( m_initial_step );
	optimTrj.runDeformation(ENV.getInt(Env::nbCostOptimize),m_idRun++);
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
  
  //store_traj_to_draw( optimTrj 0.0 );
  cout << "End replanning : " << __func__ << endl;
  m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
StarReplanner::StarReplanner(Robot* r) : SimpleReplanner(r)
{
  cout << "new star replanner" << endl;
}

StarReplanner::~StarReplanner()
{

}

bool StarReplanner::init()
{

}

void StarReplanner::run()
{
  m_isPlanning = true;
  
  // Set 0.3 seconds to do the concat traj
  PlanEnv->setBool(PlanParam::withTimeLimit, true );
  PlanEnv->setDouble(PlanParam::optimTimeLimit, m_t_rep-0.3 );
  
  cout << "m_t_rep : " << m_t_rep << endl;
  
  // Run deformation from last path id
  int last_path_id = m_CurrentTraj.getNbOfPaths()-1;
  
  StarRRT* rrt = new StarRRT( m_robot, m_graph );

  
  m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
SoftmotionReplanner::SoftmotionReplanner(Robot* r) : Replanner(r)
{
  // Call to the virtual function
  if( !init() )
  {
    cout << "Error initializing virtual function of the replanner in " << __FILE__ << " at " << __func__ << endl;
  }
}

SoftmotionReplanner::~SoftmotionReplanner()
{
  
}

//! Initialization phaze
bool SoftmotionReplanner::init()
{
  if( m_robot == NULL )
  {
    cout << "Error: robot is not initialized in " << __func__ << endl;
    return false;
  }
  
  m_manipPlanner = new ManipulationPlanner(m_robot->getRobotStruct());
  m_manipPlanner->setUseBaseMotion( true );
  m_manipPlanner->setPlanningMethod( planner_Function );
  m_manipPlanner->setSmoothingMethod( smoothing_Function );
  return true;
}

//! Convert trajectory to softmotion
//! @return the result in SM_TRAJ form
//! @return true or false is succeded
bool SoftmotionReplanner::compute_softmotion(MANPIPULATION_TRAJECTORY_CONF_STR &confs, p3d_traj* traj, SM_TRAJ& smTraj)
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

//! Comutes softmotion on a linear trajectory
//! @param traj the linear trajectory that os going to be converted
//! @return set the result in ManipPlannerLastTraj
bool SoftmotionReplanner::compute_softmotion(p3d_traj* traj)
{
  if (traj != NULL)
  {
    MANPIPULATION_TRAJECTORY_CONF_STR conf;
    SM_TRAJ smTraj;
    compute_softmotion(conf, traj, smTraj);
    ManipPlannerLastTraj = smTraj;
    return true;
  }
  else
    return false;
}

//! Generates the CurrentTraj and the soft motion traj
bool SoftmotionReplanner::generate_new_trajectory(const vector<p3d_traj*>& trajs)
{
  if( m_planningSucceded )
  {
    p3d_traj* entireTraj = concat_to_current_traj( trajs );
    
    if ( entireTraj == NULL ) 
    {
      cout << "Fail to concat traj" << endl;
      m_planningSucceded = false;
    }
    else if( !compute_softmotion(entireTraj) )
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

//! Replan a trajectory for the entire robot using the manipulation planner
//! the trajectory is planned using RRT and a soft motion trajectory is the produced
void SoftmotionReplanner::run()
{
  if ( m_robot == NULL || m_manipPlanner == NULL ) 
  {
    cout << "Error : replanning not well initialized in " << __func__ << __FILE__ << endl;
    return;
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
  
  m_planningSucceded = ( status == MANIPULATION_TASK_OK );
  m_planningSucceded = ( generate_new_trajectory(trajs) );
  m_isPlanning = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
ReplanningSimulator::ReplanningSimulator()
{
  //! Local variables
  m_init=false;
  
  m_robot=NULL;
  m_rosim=NULL;
  m_human=NULL;
  
  //m_manipPlanner=NULL;
  
  global_rePlanningEnv = this;
}

ReplanningSimulator::~ReplanningSimulator()
{
  global_rePlanningEnv = NULL;
}

//! Finds the robot in the P3D file
//! that also have a SIMUL robot
bool ReplanningSimulator::init_simulator()
{  
  string robotBaseName;
  if( !init_find_robot_basename( robotBaseName ) )
  {
    cout << "Error : p3d file not supported for replanning in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  int replanner_type = 0;
  
  switch ( replanner_type )
  {
    case 0:
      m_replanner = new SimpleReplanner( m_robot ); break;
    case 1:
      m_replanner = new SoftmotionReplanner( m_robot ); break;
    default :
      cout << "No replanner selected" << endl;
  }
  
  if( !init_rosim_cntrts_and_collisions() )
  {
    cout << "Error : p3d file not supported for replanning in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  // Create one line traj from current traj for robot simul
  m_ExecuteTraj = API::Trajectory( m_rosim );
  m_ExecuteTraj.clear();
  
  // Set the simulator as initialized and put set the drawing variables
  m_init = true;
  m_isWritingDisplay=false;
  m_isReadingDisplay=false;
  return true;
}

//! Finds the robot in the P3D file
//! that also have a SIMUL robot
bool ReplanningSimulator::init_find_robot_basename( string& robotBaseName )
{
  Scene* sce = global_Project->getActiveScene();
  
  Robot* rob = sce->getRobotByNameContaining( "_ROBOT" );
  if( rob == NULL ) {
    return false; 
  }

  size_t found = rob->getName().find("_ROBOT");
  if (found!=string::npos)
  {
    robotBaseName = rob->getName().substr(0,found);
    cout << "Robot base name is : " << robotBaseName << endl;
  }
  else {
    return false;
  }
  
  if ( sce->getRobotByName( robotBaseName + string("_SIMUL") ) == NULL ) {
    return false;
  }
  
  m_robot = sce->getRobotByName( robotBaseName + string("_ROBOT") );
  m_rosim = sce->getRobotByName( robotBaseName + string("_SIMUL") );
  if( m_robot == NULL || m_rosim == NULL )
  {
    return false;
  }
  
  return true;
}

//! initialize the simulation robot
bool ReplanningSimulator::init_rosim_cntrts_and_collisions()
{
  if( m_robot == NULL || m_rosim == NULL )
  {
    cout << "Error : the robots are not initialized in "  << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }

  // Deactivate robot to simulation robot collision checking
  p3d_col_deactivate_rob_rob( m_robot->getRobotStruct(), m_rosim->getRobotStruct() );
  
  // Deactivate all kinematic constraints
  p3d_deactivate_all_cntrts( m_rosim );
  return true;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
void ReplanningSimulator::set_multithread_graphical(bool enable)
{
  if( m_robot == NULL )
  {
    cout << "Replanner not initiailzed" << endl;
    return;
  }
  
  g3d_set_multi_thread_mode( enable );
  
  if (enable) 
  {
    m_robot->getRobotStruct()->display_mode = P3D_ROB_NO_DISPLAY;
    ext_g3d_draw_multi_thread = replan_current_draw;
  }
  else
  {
    m_robot->getRobotStruct()->display_mode = P3D_ROB_DEFAULT_DISPLAY;
    ext_g3d_draw_multi_thread = NULL;
  }
}

//! Stores the trajectory in a vector
void ReplanningSimulator::store_traj_to_vect(SM_TRAJ& smTraj, double current, double step)
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
    point[2] = 0.20;
    
    m_currentLine.push_back( point );
  }
}

//! Stores the trajectory in a vector
void ReplanningSimulator::store_traj_to_vect(API::Trajectory& traj, double step)
{
  if( !m_init )
    return;
  
  vector<double> point(3);
  //m_lastLine = m_currentLine;
  m_lastLine.clear();
  m_currentLine.clear();
  
  Configuration q(*traj.getBegin());
  point[0] = q[6];
  point[1] = q[7];
  point[2] = 0.20;
  
  m_currentLine.push_back( point );
  
  step = traj.getRangeMax()/20;
  
  for( double t=step; t<traj.getRangeMax(); t += step)
  {
    q = *traj.configAtParam(t);
    point[0] = q[6];
    point[1] = q[7];
    point[2] = 0.20;
    
    m_currentLine.push_back( point );
  }
  
  q = *traj.getEnd();
  point[0] = q[6];
  point[1] = q[7];
  point[2] = 0.20;
  
  m_currentLine.push_back( point );
}

//! Stores the trajectory to a current trajectory
void ReplanningSimulator::store_traj_to_draw(const API::Trajectory& traj, double step)
{
  if( !m_init )
    return;
  
  m_isWritingDisplay = true;
  
  while (m_isReadingDisplay);
  
  vector<double> point(3);
  //m_lastLine = m_currentLine;
  m_lastLine.clear();
  m_currentLine.clear();
  
  Joint* drawJoint = m_robot->getJoint( p3d_get_user_drawnjnt() );
  
//  cout << "user_drawnjnt : " << p3d_get_user_drawnjnt() << endl;
//  cout << "drawJoint : " << drawJoint << endl;
  
  if( drawJoint != NULL )
  {
    for( int i=0; i<traj.getNbOfViaPoints(); i++ )
    {
      m_robot->setAndUpdate(*traj[i]);
      
      Eigen::Vector3d vect = drawJoint->getVectorPos();
      
      point[0] = vect[0];
      point[1] = vect[1];
      point[2] = vect[2];
      
      m_currentLine.push_back( point );
    }
  }
  
  shared_ptr<Configuration> q_hum(m_human->getCurrentPos());
  
  (*q_hum)[6] =  PlanEnv->getDouble(PlanParam::env_futurX);
  (*q_hum)[7] =  PlanEnv->getDouble(PlanParam::env_futurY);
  (*q_hum)[8] =  PlanEnv->getDouble(PlanParam::env_futurZ);
  (*q_hum)[11] = PlanEnv->getDouble(PlanParam::env_futurRZ);
  
  m_human->setAndUpdate(*q_hum);
  
  m_isWritingDisplay = false;
}

//! Stores the trajectory to a current trajectory
void ReplanningSimulator::store_exploration(const API::Trajectory& traj, double lPrev, double lNext, shared_ptr<
                              Configuration> qNew)
{
  if( !m_init )
    return;
  
  m_isWritingDisplay = true;
  
  while (m_isReadingDisplay);
  
  cout << "ReplanningSimulator::store_exploration" << endl;
  
  m_deviateLine.clear();
  
  vector< shared_ptr<Configuration> > vectConf(3);
	vectConf.at(0) = traj.configAtParam(lPrev);
	vectConf.at(1) = qNew;
	vectConf.at(2) = traj.configAtParam(lNext);
  
  Joint* drawJoint = m_robot->getJoint( p3d_get_user_drawnjnt() );
  
  if( drawJoint != NULL )
  {
    vector<double> point(3);
    
    for( int i=0; i<int(vectConf.size()); i++ )
    {
      m_robot->setAndUpdate(*vectConf[i]);
      
      Eigen::Vector3d vect = drawJoint->getVectorPos();
      
      point[0] = vect[0];
      point[1] = vect[1];
      point[2] = vect[2];
      
      m_deviateLine.push_back( point );
    }
  }
  
   m_isWritingDisplay = false;
}

//! Set current traj as executed
void ReplanningSimulator::set_executed_traj_to_current(API::Trajectory& traj)
{
  m_ExecuteTraj.clear();
  
  for( int i=0; i<traj.getNbOfViaPoints(); i++ )
  {
    shared_ptr<Configuration> q(new Configuration(m_rosim,traj[i]->getConfigStruct()));
    m_ExecuteTraj.push_back( q );
  }
}

//! Draws replanning features
void ReplanningSimulator::draw()
{
  while (m_isWritingDisplay);
  
  m_isReadingDisplay = true;
  
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
        g3d_drawSphere(m_currentLine[i+1][0],m_currentLine[i+1][1],m_currentLine[i+1][2],0.02);
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
        g3d_drawSphere(m_lastLine[i+1][0],m_lastLine[i+1][1],m_lastLine[i+1][2],0.02);
      }
    }
  }
  
  if(m_deviateLine.size()>=2 && PlanEnv->getBool(PlanParam::showExploration))
  {
    double color[4];
    color[0] = 1.0;
    color[1] = 0.0;
    color[2] = 0.0;
    color[3] = 1.0;
    
    for( int i=0; i<int(m_deviateLine.size()-1); i++)
    {
      g3d_drawSphere(m_deviateLine[i+0][0], m_deviateLine[i+0][1], m_deviateLine[i+0][2],0.05);
      
      g3d_drawOneLine(m_deviateLine[i+0][0], m_deviateLine[i+0][1], m_deviateLine[i+0][2],
                      m_deviateLine[i+1][0], m_deviateLine[i+1][1], m_deviateLine[i+1][2],Any,color);
      
      if( i == int(m_deviateLine.size()-2))
      {
        g3d_drawSphere(m_deviateLine[i+1][0],m_deviateLine[i+1][1],m_deviateLine[i+1][2],0.02);
      }
    }
  }
  
  if (ext_g3d_traj_debug) {
    ext_g3d_traj_debug();
  }
  
  m_isReadingDisplay = false;
}

//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------

//! first call stores the time of day in t_init
//! the next calls will returns the substraction
//! between timofday and the argument t_init
double ReplanningSimulator::time_since_last_call(bool& is_first_call, double& t_init)
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

bool ReplanningSimulator::time_switch_and_id(double s, double s_rep, int& id_switch, API::Trajectory& traj, double &s_switch)
{
  double p = 0.0;
  
  for (int i=0; i<traj.getNbOfViaPoints(); i++)
  {
    p += traj.getLocalPathPtrAt(i)->getParamMax();
    
    if( p > (s+s_rep) )
    {
      m_switch_id = i;
      s_switch = p;
      return true;
    }
  }
  
  return false;
}
// ---------------------------------------------------------------------------------
// Execute simulated replanning phase
// ---------------------------------------------------------------------------------
int ReplanningSimulator::execute_softmotion_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) ) 
{
  string robotname;
  init_find_robot_basename( robotname );
  
  if (!init_simulator()) {
    cout << "init_simulator did not work!!!" << endl;
    return false;
  }
  
  if (!m_init) 
  {
//    if(dynamic_cast<SoftmotionReplanner*>(m_replanner)->init_manipulationPlanner())
//    {
//      m_init = true;
//    }
  }
  else 
  {
    cout << "ReplanningSimulator::init_execution did not work!!!" << endl;
    return false;
  }
  
  if( global_costSpace == NULL )
  {
    cout << "Error : Cost Space not initialized" << endl;
    return false;
  }
  
  p3d_rob* robotPt;
  p3d_localpath* localpathPt;
  SM_TRAJ smTraj;
  
  bool RunShowTraj=true;
  
  Configuration q(m_rosim);
  
  ChronoOn();
  
  // Set global variables
  confPtr_t qSwitch =   m_replanner->getQSwitch();
  confPtr_t qGoal =     m_replanner->getQGoal();
  API::Trajectory& CurrentTraj = m_replanner->getCurrentTraj();
  
  m_lastLine.clear();
  m_currentLine.clear();
  set_multithread_graphical( true );
  
  //  if( !replanner_manip() )
  //  {
  //    cout << "Failed to find an inital plan" << endl;
  //    return false;
  //  }
  
  // Initial values
  smTraj = ManipPlannerLastTraj;
  
  // Simulator variables
  bool isFirstLoop=true;
  bool StopRun=false;
  int count=0;
  bool planningSucceded = false;
  bool is_switch_done = true;
  double tu_tmp = 0.0; //, ts = 0.0;
  double t_max = smTraj.getDuration();
  double t = 0.0;
  double t_init = 0.0;
  double lastFloor=0.0; /* parameters along the local path */
  double t_rep = 5.0;
  int initial_step =0;
  double lp_avera_length = 0.0;
  
  // Switch
  m_switch_id = 0;
  
  // Store trajectory for drawing
  store_traj_to_vect(smTraj, 0.0, 1.0);
  
  // Replanning values
  double t_switch= t_max;
  
  while ( !StopRun )
  { 
    // Switch to new trajectory
    if( (t>t_switch) && !m_replanner->isPlanning() && planningSucceded )
    {
      smTraj = ManipPlannerLastTraj;
      t_init = 0.0;
      t_max = smTraj.getDuration();
      is_switch_done = true;
      
      // Store trajectory for drawing
      store_traj_to_vect( smTraj, 0, 1.0 );
    }
    
    // Launch new planning
    if( is_switch_done && !m_replanner->isPlanning() )
    {
      is_switch_done = false;
      StopRun = true;
      
      if ( planningSucceded ) 
      {
        p3d_getQSwitchIDFromMidCVS( t, t_rep, &m_switch_id );
        p3d_getMidCVSTimeOnTraj( m_switch_id, t_switch );
        
        cout << "q switch id : " << m_switch_id << endl;
        cout << "m_CurrentTraj.geMaxParam() : " << CurrentTraj.getRangeMax() << endl;
        
        if ( m_switch_id < (CurrentTraj.getNbOfViaPoints()-1) )
        {
          qSwitch = CurrentTraj[m_switch_id];
          
          // Set data for switch in replanner
          m_replanner->setSwitchData( qSwitch, m_switch_id, t_rep, lp_avera_length, initial_step );
          
          // Launch replanning thread
          boost::thread workerThread( boost::bind( &Replanner::run, m_replanner) );
          StopRun = false;
        }
        else if ( m_switch_id == (CurrentTraj.getNbOfViaPoints()-1) )
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
    
    t += time_since_last_call( isFirstLoop, tu_tmp );
    
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
    
    if (!planningSucceded) 
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

// ---------------------------------------------------------------------------------
// Execute simulated replanning phase
// ---------------------------------------------------------------------------------
int ReplanningSimulator::execute_simple_simulation( int (*fct)(p3d_rob* robot, p3d_localpath* localpathPt) ) 
{  
  if(!init_simulator())
  {
    cout << "Error : init simulator did not work in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  if(!m_replanner->init())
  {
    cout << "Error : init did not work in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  m_human = global_Project->getActiveScene()->getRobotByNameContaining("HERAKLES");
  
  if( m_human == NULL )
  {
    cout << "Error : no human found in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  if( global_costSpace == NULL )
  {
    cout << "Error : costspace not initialized in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  bool RunShowTraj=true;
  bool StopRun=false;
  bool isFirstLoop=true;
  bool is_switch_done=false;
  
  int count=0;
  p3d_rob* robotPt;
  p3d_localpath* localpathPt;
  
  // Replanning variables
  double t=0.0, t_init = 0.0;
  double s = 0.0,  s_switch=0.0, s_max=0.0, s_rep=0.0;
  double lastFloor=0.0;
  double t_rep = 1.0; // 5 seconds for SM
  double lp_avera_length;
  double initial_step;
  
  // Switch
  m_switch_id = 0;
  
  const double nb_lp = 60;
  
  // Retreive Current traj & q_switch from replanner
  confPtr_t qSwitch = m_replanner->getQSwitch();
  API::Trajectory& CurrentTraj = m_replanner->getCurrentTraj();
  
  // Compute the initial trajectory data
  CurrentTraj.cutTrajInSmallLP( nb_lp );
  lp_avera_length = CurrentTraj.getRangeMax() / nb_lp;
  initial_step    = CurrentTraj.getRangeMax();
  
  // Set the executed traj & multi-thread display mode
  set_executed_traj_to_current(CurrentTraj);
  set_multithread_graphical(true);
  
  // we want the trajectory to execute in 30 seconds
  // the replanning window is estimated with the param factor
  double paramFactor = CurrentTraj.getRangeMax() / 30; // To end in 3Os seconds
  s_rep = t_rep * paramFactor;
  s_max = CurrentTraj.getRangeMax();
  
  // this function keep track of the time and
  // the switch id
  time_switch_and_id( s, s_rep, m_switch_id, CurrentTraj, s_switch );
  cout << "q switch id : " << m_switch_id << endl;
  cout << "s_max : " << s_max << endl;
  cout << "s_switch : " << s_switch << endl;
  
  // Set data for switch in replanner
  m_replanner->setSwitchData( qSwitch, m_switch_id, t_rep, lp_avera_length, initial_step );
  
  // Launch replanning thread
  boost::thread workerThread( boost::bind(&Replanner::run,m_replanner) );
  
  while ( !StopRun )
  { 
     // Switch to new trajectory
    if( (s>s_switch) && (!m_replanner->isPlanning()) && m_replanner->getPlanSucceeded() )
    {
      set_executed_traj_to_current(CurrentTraj);
      s_max = m_ExecuteTraj.getRangeMax();
      is_switch_done = true;
    }
    
    // Launch new planning
    if( is_switch_done && (!m_replanner->isPlanning()) )
    {
      is_switch_done = false;
      StopRun = true;
      
      if ( m_replanner->getPlanSucceeded() ) 
      {
        time_switch_and_id( s, s_rep, m_switch_id, CurrentTraj, s_switch );
        
        cout << "q switch id : " << m_switch_id << endl;
        cout << "m_ExecuteTraj.geMaxParam() : " << m_ExecuteTraj.getRangeMax() << endl;
        
        if ( m_switch_id < (m_ExecuteTraj.getNbOfViaPoints()-1) )
        {
          qSwitch = m_ExecuteTraj[m_switch_id];
          
          // Set data for switch in replanner
          m_replanner->setSwitchData( qSwitch, m_switch_id, t_rep, lp_avera_length, initial_step );
          
          // Launch thread
          boost::thread workerThread( boost::bind( &Replanner::run, m_replanner ) );
          StopRun = false;
        }
        else if ( m_switch_id == (m_ExecuteTraj.getNbOfViaPoints()-1) )
        {
          cout << "Last replanning!!!" << endl;
          StopRun = false;
        }
      }
    }
    
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return(count);
    count++;
    
	  RunShowTraj = (*fct_stop)();
    
    //cout << "count : " << count << endl;
    m_rosim->setAndUpdate( *m_ExecuteTraj.configAtParam(s) );
    
    // Timer using real time
    t += time_since_last_call( isFirstLoop, t_init);
    
    // s is the parameter on the path
    s = t*paramFactor;
    
    if(s>s_max) {
      StopRun = true;
    }
    
    if (PlanEnv->getBool(PlanParam::stopPlanner)) {
      StopRun = true;
    }
    
    if (floor(t) > lastFloor) {
      cout << "t = " << t << endl;
      lastFloor = floor(t);
    }
  }
  
  //ReplanningSimulator::set_multithread_graphical(false);
  return 0;
}

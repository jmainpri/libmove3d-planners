//
//  replanningSimulator.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 23/04/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#include <iostream>

#include "replanningSimulators.hpp"

#include "API/project.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/cost_space.hpp"

#include "LightPlanner-pkg.h"
#include "Graphic-pkg.h"
#include "Collision-pkg.h"

#include "move3d-headless.h"

#include <boost/thread/thread.hpp>
#include <sys/time.h>

using namespace std;
using namespace tr1;

//! Soft motion trajectory for planner
SM_TRAJ ManipPlannerLastTraj;

//! Replanning simulator pointer
ReplanningSimulator* global_rePlanningEnv=NULL;

//! Replanning graph
Graph* global_rePlanningGraph=NULL;

// Replanning drawing function
void replan_current_draw()
{
  if( global_rePlanningEnv )
    global_rePlanningEnv->draw();
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
  m_manipPlanner->setPlanningMethod( p3d_planner_function );
  m_manipPlanner->setSmoothingMethod( p3d_smoothing_function );
  m_manipPlanner->setCleanningRoadmaps( false );
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
  
  m_draw_final_config = true;
  
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
  
  if( !init_rosim_cntrts_and_collisions() )
  {
    cout << "Error : p3d file not supported for replanning in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  switch ( PlanEnv->getInt( PlanParam::replanningAlgorithm ) )
  {
    case 0:
      m_replanner = new SimpleReplanner( m_robot ); break;
    case 1:
      m_replanner = new SoftmotionReplanner( m_robot ); break;
    case 2:
      m_replanner = new RRTReplanner( m_robot ); break;
    case 3:
      m_replanner = new HandoverReplanner( m_robot ); break;
    case 4:
      m_replanner = new AStarReplanner( m_robot ); break;
    case 5:
      m_replanner = new StompReplanner( m_robot ); break;
    default :
      cout << "No replanner selected" << endl;
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
  
  m_human = global_Project->getActiveScene()->getRobotByNameContaining("HERAKLES");
  if( m_human == NULL ) {
    cout << "Error : no human found in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  m_robot = sce->getRobotByName( robotBaseName + string("_ROBOT") );
  m_rosim = sce->getRobotByName( robotBaseName + string("_SIMUL") );
  m_rocyl = sce->getRobotByNameContaining( string("CYLINDER") );
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
  
  if( m_rocyl ) {
    p3d_col_deactivate_rob_rob( m_rocyl->getRobotStruct(), m_robot->getRobotStruct() );
    p3d_col_deactivate_rob_rob( m_rocyl->getRobotStruct(), m_rosim->getRobotStruct() );
  }
  
  // Deactivate all kinematic constraints
  // p3d_deactivate_all_cntrts( m_rosim );
  
  // Fix all the joints but unfixes the arm 0
  if( m_robot->getName() == "PR2_ROBOT" )
     fixAllJointsWithoutArm( m_rosim->getRobotStruct(), 0 );
  
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
  
  cout << "set graphic mode to multi thread" << endl;
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

void print_point( const vector<double>& p )
{
  cout << "p = ( "<<p[0]<<" , "<<p[1]<<" , "<<p[2]<<" )"<< endl;
}

void ReplanningSimulator::store_human_pos()
{
  confPtr_t q_hum = m_human->getCurrentPos();
  (*q_hum)[6] =  PlanEnv->getDouble(PlanParam::env_futurX);
  (*q_hum)[7] =  PlanEnv->getDouble(PlanParam::env_futurY);
  (*q_hum)[8] =  PlanEnv->getDouble(PlanParam::env_futurZ);
  (*q_hum)[11] = PlanEnv->getDouble(PlanParam::env_futurRZ);
  
  m_human->setAndUpdate(*q_hum);
  //  cout << "PlanEnv->getDouble(PlanParam::env_futurX) = " << PlanEnv->getDouble(PlanParam::env_futurX) << endl;
  //  cout << "PlanEnv->getDouble(PlanParam::env_futurY) = " << PlanEnv->getDouble(PlanParam::env_futurY) << endl;
  //  cout << "PlanEnv->getDouble(PlanParam::env_futurZ) = " << PlanEnv->getDouble(PlanParam::env_futurZ) << endl;
  //  cout << "PlanEnv->getDouble(PlanParam::env_futurRZ) = " << PlanEnv->getDouble(PlanParam::env_futurRZ) <<  endl;
  
  //  cout << "(*q_hum)[6] = " << (*q_hum)[6] << endl;
  //  cout << "(*q_hum)[7] = " << (*q_hum)[7] << endl;
  //  cout << "(*q_hum)[8] = " << (*q_hum)[8] << endl;
  //  cout << "(*q_hum)[11] = " << (*q_hum)[11] <<  endl;
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
  
  // print_point( point );
  m_currentLine.push_back( point );
  
  step = traj.getRangeMax()/20;
  
  for( double t=step; t<traj.getRangeMax(); t += step)
  {
    q = *traj.configAtParam(t);
    point[0] = q[6];
    point[1] = q[7];
    point[2] = 0.20;
    
    // print_point( point );
    m_currentLine.push_back( point );
  }
  
  q = *traj.getEnd();
  point[0] = q[6];
  point[1] = q[7];
  point[2] = 0.20;
  
  // print_point( point );
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
//    for( int i=0; i<traj.getNbOfViaPoints(); i++ )
//    {
//      m_robot->setAndUpdate(*traj[i]);
//      
//      Eigen::Vector3d vect = drawJoint->getVectorPos();
//      point[0] = vect[0];
//      point[1] = vect[1];
//      point[2] = vect[2];
//      
//      m_currentLine.push_back( point );
//    }
    
    double step = m_draw_step;
    //double step = traj.getRangeMax()/20;
    double range_max = traj.getRangeMax();
    
    for( double t=0.0; t<(range_max+step); t += step)
    {
      m_robot->setAndUpdate(*traj.configAtParam(t));
      Eigen::Vector3d vect = drawJoint->getVectorPos();
      point[0] = vect[0];
      point[1] = vect[1];
      point[2] = vect[2];
      
      // print_point( point );
      m_currentLine.push_back( point );
    }
  }
  
  // Store also human position
  store_human_pos();
  
  m_isWritingDisplay = false;
}

//! Stores the graph to a local graph
void ReplanningSimulator::store_graph_to_draw(const Graph& graph)
{
  m_isWritingDisplay = true;
  
  while (m_isReadingDisplay);
  
  cout << "Copy graph" << endl;
  delete global_rePlanningGraph;
  global_rePlanningGraph = new Graph(graph);
  
  // Store also human position
  store_human_pos();
  
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
  
//  cout << "ReplanningSimulator::store_exploration" << endl;
  
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
  
  // Store also human position
  store_human_pos();
  
  m_isWritingDisplay = false;
}

//! Set current traj as executed
bool ReplanningSimulator::set_executed_traj_to_current(API::Trajectory& traj)
{
  m_ExecuteTraj.clear();

  for( int i=0; i<traj.getNbOfViaPoints(); i++ ) {
    confPtr_t q(new Configuration( m_rosim, traj[i]->getConfigStruct()));
    m_ExecuteTraj.push_back( q );
  }
  
  if( traj.getNbOfPaths() != m_ExecuteTraj.getNbOfPaths() ) {
    cout << "Error: The number of path differ" << endl;
    cout << "traj.getNbOfPaths() : " << traj.getNbOfPaths() << endl;
    cout << "m_ExecuteTraj.getNbOfPaths() : " << m_ExecuteTraj.getNbOfPaths() << endl;
    return false;
  }
  
  for( int i=0; i<traj.getNbOfViaPoints(); i++ ) {
    if( *traj[i] != *m_ExecuteTraj[i] ) {
      cout << "Error: config differ" << endl;
      traj[i]->print();
      m_ExecuteTraj[i]->print();
      return false;
    }
  }
  
  for( int i=0; i<m_ExecuteTraj.getNbOfPaths(); i++ )
  {
    double param_1 = traj.getLocalPathPtrAt(i)->getParamMax();
    double param_2 = m_ExecuteTraj.getLocalPathPtrAt(i)->getParamMax();
    
    if( fabs( param_1 - param_2 ) > EPS6 ) {
      
      cout << "param 1 : " << param_1 << endl;
      cout << "param 2 : " << param_2 << endl;
      
      confPtr_t q1_1 = traj.getLocalPathPtrAt(i)->getBegin();
      confPtr_t q2_1 = traj.getLocalPathPtrAt(i)->getEnd();
      cout << "Current traj. conf" << endl;
      q1_1->equal( *q2_1, true );
      
      confPtr_t q1_2 = m_ExecuteTraj.getLocalPathPtrAt(i)->getBegin();
      confPtr_t q2_2 = m_ExecuteTraj.getLocalPathPtrAt(i)->getEnd();
      cout << "Executed traj. conf" << endl;
      q1_2->equal( *q2_2, true );
      cout << "Error : The localpath " << i << " are not the same length" << endl;
      
      LocalPath path3(q1_1,q2_1);
      LocalPath path4(q1_2,q2_2);
      
      if( *q1_1 != *q1_2 ) {
        q1_1->print();
        q1_2->print();
      }
      if( *q2_1 != *q2_2 ) {
        q2_1->print();
        q2_2->print();
      }
      cout << "param 3 : " << path3.getParamMax() << endl;
      cout << "param 4 : " << path4.getParamMax() << endl;
      
      cout << "q1_1->dist( *q2_1, true ) : " << q1_1->dist( *q2_1, true ) << endl;
      cout << "q1_2->dist( *q2_2, true ) : " << q1_2->dist( *q2_2, true ) << endl;
      
      return false;
    }
  }
  
  m_q_end = m_ExecuteTraj.getEnd();
  return true;
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
      //      cout <<"m_currentLine["<<i<<"][0]="<<m_currentLine[i][0]<<endl;
      //      cout <<"m_currentLine["<<i<<"][1]="<<m_currentLine[i][1]<<endl;
      //      cout <<"m_currentLine["<<i<<"][2]="<<m_currentLine[i][1]<<endl;
      g3d_set_color(Any,color);
      //g3d_drawSphere(m_currentLine[i+0][0],m_currentLine[i+0][1],m_currentLine[i+0][2],0.01);
      g3d_draw_solid_sphere(m_currentLine[i+0][0],m_currentLine[i+0][1],m_currentLine[i+0][2],0.02,10);
      
      g3d_drawOneLine(m_currentLine[i+0][0],m_currentLine[i+0][1],m_currentLine[i+0][2],
                      m_currentLine[i+1][0],m_currentLine[i+1][1],m_currentLine[i+1][2],Any,color);
      
      if( i == int(m_currentLine.size()-2))
      {
        //g3d_drawSphere(m_currentLine[i+1][0],m_currentLine[i+1][1],m_currentLine[i+1][2],0.01);
        g3d_draw_solid_sphere(m_currentLine[i+1][0],m_currentLine[i+1][1],m_currentLine[i+1][2],0.01,10);
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
      g3d_set_color(Any,color);
      //g3d_drawSphere(m_lastLine[i+0][0],m_lastLine[i+0][1],m_lastLine[i+0][2],0.01);
      g3d_draw_solid_sphere(m_lastLine[i+0][0],m_lastLine[i+0][1],m_lastLine[i+0][2],0.02,10);
      
      g3d_drawOneLine(m_lastLine[i+0][0],m_lastLine[i+0][1],m_lastLine[i+0][2],
                      m_lastLine[i+1][0],m_lastLine[i+1][1],m_lastLine[i+1][2],Any,color);
      
      if( i == int(m_lastLine.size()-2))
      {
        //g3d_drawSphere(m_lastLine[i+1][0],m_lastLine[i+1][1],m_lastLine[i+1][2],0.01);
        g3d_draw_solid_sphere(m_lastLine[i+1][0],m_lastLine[i+1][1],m_lastLine[i+1][2],0.01,10);
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
      g3d_set_color(Any,color);
      
      g3d_draw_solid_sphere(m_deviateLine[i+0][0], m_deviateLine[i+0][1], m_deviateLine[i+0][2],0.02,10);
      
      g3d_drawOneLine(m_deviateLine[i+0][0], m_deviateLine[i+0][1], m_deviateLine[i+0][2],
                      m_deviateLine[i+1][0], m_deviateLine[i+1][1], m_deviateLine[i+1][2],Any,color);
      
      if( i == int(m_deviateLine.size()-2))
      {
        g3d_draw_solid_sphere(m_deviateLine[i+1][0],m_deviateLine[i+1][1],m_deviateLine[i+1][2],0.02,10);
      }
    }
  }
  
  if (ENV.getBool(Env::drawGraph) && global_rePlanningGraph) 
  {
    global_rePlanningGraph->setRobot( m_rosim );
    global_rePlanningGraph->draw();
  }
  
  if (ext_g3d_traj_debug) {
    ext_g3d_traj_debug();
  }
  
//  if( m_draw_final_config ) {
//    confPtr_t q_tmp = m_rosim->getCurrentPos();
//    G3D_Window* win = g3d_get_cur_win();
//    win->vs.transparency_mode= G3D_TRANSPARENT_AND_OPAQUE;
//    
//    m_rosim->setAndUpdate( *m_q_end );
//    m_rosim->getRobotStruct()->draw_transparent = false;
//    g3d_draw_robot( m_rosim->getRobotStruct()->num, win, 0);
//    m_rosim->getRobotStruct()->draw_transparent = false;
//    m_rosim->setAndUpdate( *q_tmp );
//  }
  
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

bool ReplanningSimulator::time_switch_and_id(double s, double s_rep, int& id_switch, 
                                             API::Trajectory& traj, double &s_switch)
{
  double p = 0.0;
  
  for (int i=0; i<traj.getNbOfPaths(); i++)
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
          m_replanner->setSwitchData( qSwitch, m_switch_id, t_switch, t_rep, lp_avera_length, initial_step );
          
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
    cout << "Error : in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  if(!m_replanner->init())
  {
    cout << "Error : in " << __FILE__ << " at " << __LINE__ << endl;
    return false;
  }
  
  // If HERAKLES exists in the sceen, set it
  m_replanner->setHuman( m_human );
  
  if( global_costSpace == NULL )
  {
    cout << "Warning : costspace not initialized in " << __FILE__ << " at " << __LINE__ << endl;
    //return false;
  }
  
  bool RunShowTraj=true;
  bool StopRun=false;
  bool isFirstLoop=true;
  bool do_switch=false;
  
  int count=0;
  p3d_rob* robotPt;
  p3d_localpath* localpathPt;
  
  // Replanning variables
  double t=0.0, t_init = 0.0;
  double s = 0.0,  s_switch=0.0, s_max=0.0, s_rep=0.0;
  double lastFloor=0.0;
  double t_rep = PlanEnv->getDouble( PlanParam::trajReplanningWindow );
  double t_total = PlanEnv->getDouble( PlanParam::trajReplanningTotalTime );
  double initial_step = 0.0;
  
  // Retreive Current traj & q_switch from replanner
  confPtr_t q_switch;
  API::Trajectory& CurrentTraj = m_replanner->getCurrentTraj();
  initial_step = CurrentTraj.getRangeMax()/1;
  m_q_end = CurrentTraj.getEnd();
  
  // Set the executed traj & multi-thread display mode
  set_executed_traj_to_current(CurrentTraj);
  set_multithread_graphical(true);
  
  // we want the trajectory to execute in 30 seconds
  // the replanning window is estimated with the param factor
  double paramFactor = CurrentTraj.getRangeMax() / t_total;
  s_rep = t_rep * paramFactor;
  s_max = CurrentTraj.getRangeMax();
  s_switch = s+s_rep;

  cout << "q switch id : " << m_switch_id << endl;
  cout << "s_max : " << s_max << endl;
  cout << "s_switch : " << s_switch << endl;
  
  // Set data for switch in replanner
  m_replanner->setSwitchData( q_switch, 0, s_switch, t_rep, 0.0, initial_step );
  
  // Launch replanning thread
  boost::thread workerThread( boost::bind(&Replanner::run,m_replanner) );
  
  while ( !StopRun )
  { 
    // Switch to new trajectory
    if( s>s_switch )
    {
      if( m_replanner->isPlanning() && (!m_replanner->getPlanSucceeded()) )
      {
        cout << "replanner still planning or not succeeded!!!" << endl;
      }
      else 
      {
        if( set_executed_traj_to_current(CurrentTraj) ) {
          do_switch = true;
          s_max = m_ExecuteTraj.getRangeMax();
        }
        else {
          cout << "Executed traj and current traj differ" << endl;
          StopRun = true;
        }
      }
      
      // Launch new planning
      if( do_switch && ((s_max-s) > (5*paramFactor)) )
      {
        do_switch = false;
        StopRun = true;
        
        if ( m_replanner->getPlanSucceeded() ) 
        {
          StopRun = false;
          s_switch = s+s_rep;
          cout << " ------------- time_switch_and_id ------------- " << endl;
          cout << "s : " << s << endl;
          cout << "s_rep : " << s_rep << endl;
          cout << "s_switch : " << s_switch << endl;
          
          if ( s < s_max-s_rep )
          {          
            // Set data for switch in replanner
            m_replanner->setSwitchData( q_switch, 0, s_switch, t_rep, 0.0, initial_step );
            
            // Launch thread
            boost::thread workerThread( boost::bind( &Replanner::run, m_replanner ) );
          }
          else
          {
            cout << "Last replanning!!!" << endl;
          }
        }
      }
    }
    
	  RunShowTraj = (*fct_stop)();
    
    // Timer using real time &  s is the parameter on the path
    t += time_since_last_call( isFirstLoop, t_init );
    s = t*paramFactor;
    m_rosim->setAndUpdate( *m_ExecuteTraj.configAtParam(s) );
    //cout << "s : " << s << endl;
    
    // Set and update human pos
    store_human_pos();
    
    if (fct) if (((*fct)(robotPt, localpathPt)) == FALSE) return(count);
    count++;
    
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
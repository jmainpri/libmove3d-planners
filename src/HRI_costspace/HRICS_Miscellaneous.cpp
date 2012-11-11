//
//  HRICS_Miscellaneous.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 21/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "HRICS_Miscellaneous.hpp"
#include "HRICS_costspace.hpp"

#include "API/project.hpp"
#include "API/ConfigSpace/configuration.hpp"

#include "plannerFunctions.hpp"

#include "P3d-pkg.h"
#include "GraspPlanning-pkg.h"
#include "LightPlanner-pkg.h"
#include "Collision-pkg.h"

#include <algorithm>

using namespace HRICS;
using namespace std;
using namespace tr1;
using namespace Eigen;

extern string global_ActiveRobotName;
#ifdef MULTILOCALPATH
extern ManipulationTestFunctions* global_manipPlanTest;
#endif

void HRICS::printHumanConfig()
{
  Scene* sc = global_Project->getActiveScene();
  
  //Robot* rob = sc->getRobotByName("HERAKLES_HUMAN1");
  Robot* rob = sc->getRobotByName("PR2_ROBOT");
  
  if( rob == NULL ) 
  {
    cout << "No robot named HERAKLES_HUMAN1 in env" << endl;
  }
  
  for( int i=0; i<int(rob->getNumberOfJoints()); i++)
  {
    Joint* jnt = rob->getJoint(i);
    
    for( int j=0; j<int(jnt->getNumberOfDof()); j++)
    {
      cout << "jnt->getName() : " << jnt->getName() << "(" <<i<< ") , index_dof : " << jnt->getIndexOfFirstDof()+j << endl;
    }
  }
}

void HRICS::printPr2Config()
{
  Scene* sc = global_Project->getActiveScene();
  
  Robot* rob = sc->getRobotByName("PR2_ROBOT");
  
  if( rob == NULL ) 
  {
    cout << "No robot named PR2_ROBOT in env" << endl;
  }
  
  confPtr_t q = rob->getCurrentPos();
  
  Joint* jnt = rob->getJoint("platformJoint");
  double base_tx = (*q)[jnt->getJointStruct()->index_dof+0];
  double base_ty = (*q)[jnt->getJointStruct()->index_dof+1];
  double base_tz = (*q)[jnt->getJointStruct()->index_dof+2];
  double base_rx = (*q)[jnt->getJointStruct()->index_dof+3];
  double base_ry = (*q)[jnt->getJointStruct()->index_dof+4];
  double base_rz = (*q)[jnt->getJointStruct()->index_dof+5];
  
  jnt = rob->getJoint("Torso");
  double torso = (*q)[jnt->getJointStruct()->index_dof+0];
  
  jnt = rob->getJoint("pan_cam");
  double head_pan = (*q)[jnt->getJointStruct()->index_dof+0]; 
  
  jnt = rob->getJoint("tilt_cam");
  double head_tilt = (*q)[jnt->getJointStruct()->index_dof+0];
  
  jnt = rob->getJoint("laser-jnt");
  double laser_tilt = (*q)[jnt->getJointStruct()->index_dof+0];
  
  jnt = rob->getJoint("right-Arm1");
  double r_shoulder_pan     = (*q)[jnt->getJointStruct()->index_dof+0]; 
  double r_shoulder_lift    = (*q)[jnt->getJointStruct()->index_dof+1];
  double r_upper_arm_roll   = (*q)[jnt->getJointStruct()->index_dof+2];
  double r_elbow_flex       = (*q)[jnt->getJointStruct()->index_dof+3];
  double r_forearm_roll     = (*q)[jnt->getJointStruct()->index_dof+4];
  double r_wrist_flex       = (*q)[jnt->getJointStruct()->index_dof+5];
  double r_writ_roll        = (*q)[jnt->getJointStruct()->index_dof+6];
  
  jnt = rob->getJoint("fingerJointGripper_0");
  double r_gripper = (*q)[jnt->getJointStruct()->index_dof+0]; 
  
  jnt = rob->getJoint("right-grip2");
  double r_gripper_false = (*q)[jnt->getJointStruct()->index_dof+0]; 
  
  jnt = rob->getJoint("left-Arm1");
  double l_shoulder_pan     = (*q)[jnt->getJointStruct()->index_dof+0];
  double l_shoulder_lift    = (*q)[jnt->getJointStruct()->index_dof+1];
  double l_upper_arm_roll   = (*q)[jnt->getJointStruct()->index_dof+2];
  double l_elbow_flex       = (*q)[jnt->getJointStruct()->index_dof+3];
  double l_forearm_roll     = (*q)[jnt->getJointStruct()->index_dof+4];
  double l_wrist_flex       = (*q)[jnt->getJointStruct()->index_dof+5];
  double l_wrist_roll       = (*q)[jnt->getJointStruct()->index_dof+6];
  
  jnt = rob->getJoint("fingerJointGripper_1");
  double l_gripper = (*q)[jnt->getJointStruct()->index_dof+0]; 
  
  jnt = rob->getJoint("left-grip2");
  double l_gripper_false = (*q)[jnt->getJointStruct()->index_dof+0]; 
  
  cout << "CONFIG FOR PR2 SOFTMOTION" << endl;
  cout << base_tx << " ";
  cout << base_ty << " ";
  cout << base_tz << " ";
  cout << base_rx << " ";
  cout << base_ry << " ";
  cout << base_rz << " ";
  
  cout << torso << " ";
  cout << head_pan << " ";
  cout << head_tilt << " ";
  cout << laser_tilt << " ";
  
  cout << r_shoulder_pan  << " ";   
  cout << r_shoulder_lift  << " ";
  cout << r_upper_arm_roll << " ";  
  cout << r_elbow_flex << " "; 
  cout << r_forearm_roll  << " ";
  cout << r_wrist_flex   << " "; 
  cout << r_writ_roll   << " "; 
  cout << r_gripper << " ";
  cout << r_gripper_false << " ";
  
  cout << l_shoulder_pan  << " ";
  cout << l_shoulder_lift  << " "; 
  cout << l_upper_arm_roll << " "; 
  cout << l_elbow_flex  << " ";  
  cout << l_forearm_roll << " ";  
  cout << l_wrist_flex << " ";  
  cout << l_wrist_roll << " ";  
  cout << l_gripper << " ";
  cout << l_gripper_false << " ";
  cout << endl;  
}

void HRICS::setSimulationRobotsTransparent()
{
  Scene* sc = global_Project->getActiveScene();
  
  Robot* rob = sc->getRobotByName("PR2_ROBOT");
  Robot* sim = sc->getRobotByName("PR2_SIMUL");
  if( rob == NULL || sim == NULL )
  {
    return;
  }
  
  // Draws the second PR2 transparent
  sim->getRobotStruct()->draw_transparent = true;
  
  // Deactivate robot to simulation robot collision checking
  p3d_col_deactivate_rob_rob(rob->getRobotStruct(),sim->getRobotStruct());
}


static int switch_cost = 0;

void HRICS::setTenAccessiblePositions()
{
  API_activeGrid = HRICS_activeNatu->getGrid();
  
  Robot* human = HRICS_activeNatu->getGrid()->getRobot();
  
  vector< NaturalCell* > reachable_cells = HRICS_activeNatu->getGrid()->getAllReachableCells();
  vector< pair<double,NaturalCell*> > sorted_cells;
  
  for(int i=0;i<int(reachable_cells.size());i++)
  {
    switch( switch_cost ) {
      case 0: 
      {
        Distance* dist = HRICS_humanCostMaps->getAgentGrid(human)->getDistance();
        sorted_cells.push_back( make_pair(dist->getWorkspaceCost(reachable_cells[i]->getWorkspacePoint()),reachable_cells[i]) );
      }
        break;
      case 1:
      {
        Visibility* visi = HRICS_humanCostMaps->getAgentGrid(human)->getVisibility();
        sorted_cells.push_back( make_pair(visi->getWorkspaceCost(reachable_cells[i]->getWorkspacePoint()),reachable_cells[i]) );
      }
        break;
      case 2:
      {
        Natural* reach = HRICS_humanCostMaps->getAgentGrid(human)->getNatural();
        sorted_cells.push_back( make_pair(reach->getWorkspaceCost(reachable_cells[i]->getWorkspacePoint()),reachable_cells[i]) );
      }
        break;
    }
  }
  
  if( switch_cost != 2 ) {
    switch_cost++;
  }
  else {
    switch_cost = 0;
  }
  
  sort( sorted_cells.begin(), sorted_cells.end() );
  
  Scene* sce = global_Project->getActiveScene();
  sce->getRobotByName("Doggy")->setAndUpdateFreeFlyer( sorted_cells[0].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy1")->setAndUpdateFreeFlyer( sorted_cells[1].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy2")->setAndUpdateFreeFlyer( sorted_cells[2].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy3")->setAndUpdateFreeFlyer( sorted_cells[3].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy4")->setAndUpdateFreeFlyer( sorted_cells[4].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy5")->setAndUpdateFreeFlyer( sorted_cells[5].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy6")->setAndUpdateFreeFlyer( sorted_cells[6].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy7")->setAndUpdateFreeFlyer( sorted_cells[7].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy8")->setAndUpdateFreeFlyer( sorted_cells[8].second->getWorkspacePoint() );
  sce->getRobotByName("Doggy9")->setAndUpdateFreeFlyer( sorted_cells[9].second->getWorkspacePoint() );
}

void HRICS::generateGraspConfigurations()
{
  p3d_rob* robot = p3d_get_robot_by_name("PR2_ROBOT");
  if( robot == NULL )
    return;
  
  p3d_rob* object = p3d_get_robot_by_name("GREY_TAPE");
  if( object == NULL )
    return;
  
  double confCost(-1);
  configPt q(NULL);
  gpHand_properties handProp((*robot->armManipulationData)[0].getHandProperties());
  
  // Create and generates the config generator
  ManipulationConfigs configGen(robot);
  configGen.setMobileBaseMode( true );
  configGen.setDebugMode( false );
  
  // Compute or reads the grasp list if available
  list<gpGrasp> graspList;
  graspList.clear();
  gpGet_grasp_list(object->name, handProp.type, graspList);
  cout << "graspList.size() = " << graspList.size() << endl;
  
  ManipulationUtils::fixAllHands(robot, NULL, true);
  
  vector< pair<double,configPt> > configs;
  
  for (list<gpGrasp>::iterator iter = graspList.begin(); 
       iter != graspList.end(); iter++)
  {
    // Set hand frame
    p3d_matrix4 tAtt,handFrame;
    p3d_mat4Mult((*iter).frame, handProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, (*robot->armManipulationData)[0].getCcCntrt()->Tatt2, tAtt);
    
    // Generate a grasp configuration
    q = configGen.getGraspConf(object, 0, (*iter), tAtt, confCost);
    
    if( q != NULL )
    {
      configs.push_back(make_pair( confCost , q ));
    }
  }
  
  std::sort(configs.begin(),configs.end());
  
  for (vector<pair<double,configPt> >::iterator iter = configs.begin(); 
       iter != configs.end(); iter++)
  {
    cout << "Cost : " << iter->first << endl;
  }
  
  if(!configs.empty())
    p3d_set_and_update_this_robot_conf( robot, configs[0].second );
  else
    cout << "No config" << endl;
  
  g3d_draw_allwin_active();
}

void HRICS::setThePlacemateInIkeaShelf()
{
  Scene* sc = global_Project->getActiveScene();
  
  std::vector<Robot*> placemates;
  
  // Get all robots that conain the name placemat
  for (int i=0; i<int(sc->getNumberOfRobots()); i++) 
  {
    Robot* rob = sc->getRobot(i);
    
    if( rob->getName().find("PLACEMAT") !=string::npos)
    {
      placemates.push_back( rob );
    }
  }
  
  if( placemates.size() < 4 )
    return;
  
//  for (int i=0; i<int(placemates.size()); i++) 
//  {
//    cout << placemates[i]->getName() << endl; 
//  }
  
  Configuration q( *placemates[0]->getCurrentPos() );
  q[6] = 0.00;
  q[7] = 0.18;
  q[8] = 0.07;
  q[9] = 0.00;
  q[10] = 0.00;
  q[11] = 0.00;
  placemates[0]->setAndUpdate( q );
  
  q = *placemates[1]->getCurrentPos();
  q[6] = 0.00;
  q[7] = -0.18;
  q[8] = 0.07;
  q[9] = 0.00;
  q[10] = 0.00;
  q[11] = 0.00;
  placemates[1]->setAndUpdate( q );
  
  q = *placemates[2]->getCurrentPos();
  q[6] = 0.00;
  q[7] = 0.18;
  q[8] = 0.41;
  q[9] = 0.00;
  q[10] = 0.00;
  q[11] = 0.00;
  placemates[2]->setAndUpdate( q );
  
  q = *placemates[3]->getCurrentPos();
  q[6] = 0.00;
  q[7] = -0.18;
  q[8] = 0.41;
  q[9] = 0.00;
  q[10] = 0.00;
  q[11] = 0.00;
  placemates[3]->setAndUpdate( q );
  
  // Place the placemat in the IKEA_SHELF
  // by moving the initial positions to the shelf
  Robot* shelf = sc->getRobotByName( "IKEA_SHELF" );
  if (shelf == NULL) 
    return;
  
  cout << "Set placemats" << endl;
  Transform3d Tshelf = shelf->getJoint( 1 )->getMatrixPos();
  for( int i=0; i<int(placemates.size()); i++)
  {
    Joint* jnt = placemates[i]->getJoint( 1 );
    Transform3d Tplace =  Tshelf*jnt->getMatrixPos();
    q = *placemates[i]->getCurrentPos();
    
    Eigen::Vector3d trans = Tplace.translation();
    q[6] = trans[0];
    q[7] = trans[1];
    q[8] = trans[2];
    
    Eigen::Vector3d rot = Tplace.linear().eulerAngles(0, 1, 2);
    q[9] = rot[0];
    q[10] = rot[1];
    q[11] = rot[2];
    placemates[i]->setAndUpdate( q );
  }
  
  Robot* rob = sc->getRobotByName( "GREY_TAPE" );
  if (rob == NULL)
    return;
  q = *rob->getCurrentPos();
  Configuration q_place( *placemates[0]->getCurrentPos() );
  q[6] = q_place[6];
  q[7] = q_place[7];
  q[8] = q_place[8];
  q[9] = 0.0;
  q[10] = 0.0;
  q[11] = 0.0;
  rob->setAndUpdate(q);
  
  rob = sc->getRobotByName( "LOTR_TAPE" );
  if (rob == NULL)
    return;
  q = *rob->getCurrentPos();
  q_place = *placemates[1]->getCurrentPos();
  q[6] = q_place[6];
  q[7] = q_place[7];
  q[8] = q_place[8];
  q[9] = 0.0;
  q[10] = 0.0;
  q[11] = 0.0;
  rob->setAndUpdate(q);
  
  rob = sc->getRobotByName( "ORANGE_BOTTLE" );
  if (rob == NULL)
    return;
  q = *rob->getCurrentPos();
  q_place = *placemates[2]->getCurrentPos();
  q[6] = q_place[6];
  q[7] = q_place[7];
  q[8] = q_place[8];
  q[9] = 0.0;
  q[10] = 0.0;
  q[11] = 0.0;
  rob->setAndUpdate(q);
  
  rob = sc->getRobotByName( "BLUE_BOTTLE" );
  if (rob == NULL)
    return;
  q = *rob->getCurrentPos();
  
  q_place = *placemates[3]->getCurrentPos();
  q[6] = q_place[6];
  q[7] = q_place[7];
  q[8] = q_place[8];
  q[9] = 0.0;
  q[10] = 0.0;
  q[11] = 0.0;
  rob->setAndUpdate(q);
}

bool HRICS::initShelfScenario()
{
  ManipulationPlanner* planner = NULL;
  
  Robot* rob = global_Project->getActiveScene()->getRobotByName( global_ActiveRobotName );
  
  if (!rob)
  {
    cout << "Robot with name : " << global_ActiveRobotName << " does not exist" << endl;
    return false;
  }
  
  if (global_manipPlanTest)
  {
    delete global_manipPlanTest;
  }
  
  global_manipPlanTest = new ManipulationTestFunctions( global_ActiveRobotName  );
  
  // Set init and goal config
  confPtr_t qInit = rob->getCurrentPos();
  confPtr_t qGoal = rob->getGoTo();
  global_manipPlanTest->setGoalConfiguration(qInit->getConfigStructCopy());
  global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
  
  // Initialize planner
  global_manipPlanTest->initManipulationGenom();
  planner = global_manipPlanTest->getManipPlanner();
  
  // Set Planning functions
  planner->setPlanningMethod( p3d_planner_function );
  planner->setSmoothingMethod( p3d_smoothing_function );
//  planner->setReplanningMethod( replanning_Function );

  planner->setArmCartesian( 0, false );
  planner->setUseBaseMotion( true );
  
  global_manipPlanTest->setDebugMode( false );
  global_manipPlanTest->resetToPoint();
  
  g3d_draw_allwin_active();
  
  return true;
}

static int default_drawtraj(p3d_rob* robot, p3d_localpath* curLp)
{
	g3d_draw_allwin_active();
	usleep(20000);
	//cout << "ENV.getDouble(Env::showTrajFPS) = " << ENV.getDouble(Env::showTrajFPS) << endl;
	return TRUE;
}

bool HRICS::execShelfScenario()
{
  ManipulationPlanner* planner = global_manipPlanTest->getManipPlanner();
  
  if (planner ==NULL) {
    cout << "planner not defined in : " << __func__ << endl;
    return false;
  }
  
  Scene* sc = global_Project->getActiveScene();
  Robot* rob = sc->getRobotByName( planner->robot()->name );
  
  if (rob ==NULL) {
    cout << "robot not defined in : " << __func__ << endl;
    return false;
  }
  
  confPtr_t qCur = rob->getCurrentPos();
  confPtr_t qObj;
  Robot* object;
  
  // Get the grey tape
  global_manipPlanTest->resetToPoint();
  global_manipPlanTest->setObject( "GREY_TAPE" );
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  if(!global_manipPlanTest->runTest(2)) {
    return false;
  }
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
    return false;
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Bring it to the trashbin
  // Point where the trashbin is
  vector<double> trashbinPoint(6,P3D_HUGE);
  trashbinPoint[0] = 0.14;
  trashbinPoint[1] = 1.00;
  trashbinPoint[2] = 1.15;
  global_manipPlanTest->setToPoint( trashbinPoint );
  if(!global_manipPlanTest->runTest(3))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Escape Object
  if(!global_manipPlanTest->runTest(7))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Get the object down 20cm
  object = sc->getRobotByName( "GREY_TAPE" );
  qObj = object->getCurrentPos(); (*qObj)[8] -= 0.35;
  object->setAndUpdate(*qObj);
  g3d_draw_allwin_active();
  
  // Get the orange bottle
  global_manipPlanTest->resetToPoint();
  global_manipPlanTest->setObject( "ORANGE_BOTTLE" );
  if(!global_manipPlanTest->runTest(2))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Bring it to the trashbin
  global_manipPlanTest->setToPoint( trashbinPoint );
  if(!global_manipPlanTest->runTest(3))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Escape Object
  if(!global_manipPlanTest->runTest(7))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Get the object down 20cm
  object = sc->getRobotByName( "ORANGE_BOTTLE" );
  qObj = object->getCurrentPos(); (*qObj)[8] -= 0.35;
  object->setAndUpdate(*qObj);
  g3d_draw_allwin_active();
  
  // Get the lotr tape
  global_manipPlanTest->resetToPoint();
  global_manipPlanTest->setObject( "LOTR_TAPE" );
  if(!global_manipPlanTest->runTest(2))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Bring it to the trashbin
  global_manipPlanTest->setToPoint( trashbinPoint );
  if(!global_manipPlanTest->runTest(3))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Escape Object
  if(!global_manipPlanTest->runTest(7))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Get the object down 20cm
  object = sc->getRobotByName( "LOTR_TAPE" );
  qObj = object->getCurrentPos(); (*qObj)[8] -= 0.35;
  object->setAndUpdate(*qObj);
  g3d_draw_allwin_active();
  
  return true;
}

bool HRICS::simpShelfScenario()
{
  ManipulationPlanner* planner = global_manipPlanTest->getManipPlanner();
  
  if (planner ==NULL) {
    cout << "planner not defined in : " << __func__ << endl;
    return false;
  }
  
  Scene* sc = global_Project->getActiveScene();
  Robot* rob = sc->getRobotByName( planner->robot()->name );
  
  if (rob ==NULL) {
    cout << "robot not defined in : " << __func__ << endl;
    return false;
  }
  
  confPtr_t qCur = rob->getCurrentPos();
  confPtr_t qObj;
  
  // Point where the trashbin is
  vector<double> trashbinPoint(6,P3D_HUGE);
  trashbinPoint[0] = 0.14;
  trashbinPoint[1] = 1.00;
  trashbinPoint[2] = 1.15;
  
  // Get the grey tape
  global_manipPlanTest->resetToPoint();
  global_manipPlanTest->setObject( "GREY_TAPE" );
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  if(!global_manipPlanTest->runTest(2))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  qCur = rob->getCurrentPos();
  if( qCur->isOutOfBounds() )
  {
    cout << "qCur out of bounds" << endl;
    qCur->adaptCircularJointsLimits();
    return false;
  }
  global_manipPlanTest->setInitConfiguration(qCur->getConfigStructCopy());
  
  // Bring it to the trashbin
  global_manipPlanTest->setToPoint( trashbinPoint );
  if(!global_manipPlanTest->runTest(3))
    return false;
  
  g3d_show_tcur_rob( planner->robot(), default_drawtraj );
  return true;
}


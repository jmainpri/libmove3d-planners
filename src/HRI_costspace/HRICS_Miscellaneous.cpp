//
//  HRICS_Miscellaneous.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 21/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "HRICS_Miscellaneous.hpp"

#include "API/project.hpp"
#include "API/ConfigSpace/configuration.hpp"

#include "plannerFunctions.hpp"

#include "P3d-pkg.h"
#include "GraspPlanning-pkg.h"
#include "LightPlanner-pkg.h"
#include "Collision-pkg.h"

using namespace HRICS;
using namespace std;
using namespace tr1;
using namespace Eigen;

extern string global_ActiveRobotName;
#ifdef MULTILOCALPATH
extern ManipulationTestFunctions* global_manipPlanTest;
#endif

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
  
  sort(configs.begin(),configs.end());
  
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
  shared_ptr<Configuration> qInit = rob->getCurrentPos();
  shared_ptr<Configuration> qGoal = rob->getGoTo();
  global_manipPlanTest->setGoalConfiguration(qInit->getConfigStructCopy());
  global_manipPlanTest->setGoalConfiguration(qGoal->getConfigStructCopy());
  
  // Initialize planner
  global_manipPlanTest->initManipulationGenom();
  planner = global_manipPlanTest->getManipPlanner();
  
  // Set Planning functions
  planner->setPlanningMethod( p3d_planner_function );
  planner->setSmoothingMethod( p3d_smoothing_function );
  //planner->setReplanningMethod( replanning_Function );
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
  
  shared_ptr<Configuration> qCur = rob->getCurrentPos();
  shared_ptr<Configuration> qObj;
  Robot* object;
  
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
  
  shared_ptr<Configuration> qCur = rob->getCurrentPos();
  shared_ptr<Configuration> qObj;
  
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


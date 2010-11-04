/*
 *  qtRobot.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CRNS. All rights reserved.
 *
 */

#include "qtRobot.hpp"
#include "ui_qtRobot.h"

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"

#include "Util-pkg.h"
#include "P3d-pkg.h"

#ifdef PQP
#include "Collision-pkg.h"
#endif

#if defined( CXX_PLANNER ) || defined( OOMOVE3D_CORE ) 
#include "MultiRun.hpp"
#include "SaveContext.hpp"
#include "testModel.hpp"
#include "API/project.hpp"
//#include "Greedy/GridCollisionChecker.h"
#endif

#ifdef HRI_GENERALIZED_IK
#include "Hri_planner-pkg.h"
#endif

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

using namespace std;
using namespace tr1;

RobotWidget::RobotWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::RobotWidget)
{
	m_ui->setupUi(this);
	
	initModel();
	initManipulation();
	//initVoxelCollisionChecker();
}

RobotWidget::~RobotWidget()
{
	delete m_ui;
}

//---------------------------------------------------------------------
// Robot
//---------------------------------------------------------------------
void RobotWidget::initRobot()
{
	m_mainWindow->Ui()->formRobot->initAllForms(m_mainWindow->getOpenGL());	
}

MoveRobot*  RobotWidget::getMoveRobot()
{
	return m_mainWindow->Ui()->formRobot;
}

//---------------------------------------------------------------------
// TEST MODEL
//---------------------------------------------------------------------
void RobotWidget::initModel()
{
	connect(m_ui->pushButtonCollision,SIGNAL(clicked()),this,SLOT(collisionsTest()));
	connect(m_ui->pushButtonLocalPath,SIGNAL(clicked()),this,SLOT(localpathsTest()));
	connect(m_ui->pushButtonCost,SIGNAL(clicked()),this,SLOT(costTest()));
	connect(m_ui->pushButtonTestAll,SIGNAL(clicked()),this,SLOT(allTests()));
	connect(m_ui->pushButtonSetObjectToCarry,SIGNAL(clicked()),this,SLOT(SetObjectToCarry()));
	
	connect(ENV.getObject(Env::numberOfCollisionPerSec),SIGNAL(valueChanged(QString)),m_ui->labelCollision,SLOT(setText(QString)));
	connect(ENV.getObject(Env::numberOfLocalPathPerSec),SIGNAL(valueChanged(QString)),m_ui->labelLocalPath,SLOT(setText(QString)));
	connect(ENV.getObject(Env::numberOfCostPerSec),SIGNAL(valueChanged(QString)),m_ui->labelTimeCost,SLOT(setText(QString)));
	
	connect(m_ui->pushButtonAttMat,SIGNAL(clicked()),this,SLOT(setAttMatrix()));
	
	QString RobotObjectToCarry("No Object");
	
	ENV.setString(Env::ObjectToCarry,RobotObjectToCarry);
	
	// Grab Object
	for(int i =0;i<XYZ_ENV->nr;i++)
	{
		if(XYZ_ENV->robot[i]->joints[1]->type == P3D_FREEFLYER )
		{
			if( XYZ_ENV->robot[i]->njoints == 1 )
			{
				QString FFname(XYZ_ENV->robot[i]->name);
				m_ui->comboBoxGrabObject->addItem(FFname);
				mFreeFlyers.push_back(FFname);
				//                cout<< " FreeFlyer = "  << XYZ_ENV->robot[i]->name << endl;
			}
		}
	}
	
	m_ui->comboBoxGrabObject->setCurrentIndex(0);
	connect(m_ui->comboBoxGrabObject, SIGNAL(currentIndexChanged(int)),this, SLOT(currentObjectChange(int))/*, Qt::DirectConnection*/);
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsWeightedRot,	Env::isWeightedRotation);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxFKSampling,		Env::FKShoot);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxFKDistance,		Env::FKDistance);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawBox,				Env::drawBox);
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxWeightedRot, m_ui->horizontalSliderWeightedRot , Env::RotationWeight );
	
	connect(m_ui->pushButtonGrabObject,SIGNAL(clicked()),					this,SLOT(GrabObject()));
	connect(m_ui->pushButtonReleaseObject,SIGNAL(clicked()),			this,SLOT(ReleaseObject()));
	
#ifdef HRI_GENERALIZED_IK
	connect(m_ui->pushButtonComputeLeftArmGIK,SIGNAL(clicked()),	this,SLOT(computeHriGikLARM()));
	connect(m_ui->pushButtonComputeRightArmGIK,SIGNAL(clicked()),	this,SLOT(computeHriGikRARM()));
#endif
	
	connect(m_ui->pushButtonPrintCurrentPos,SIGNAL(clicked()),		this,SLOT(printCurrentPos()));
	
#if defined(LIGHT_PLANNER)
	connect(m_ui->pushButtonSwitchFKIK,SIGNAL(clicked()),					this,SLOT(switchFKIK()));
#endif
	
	connect(m_ui->pushButtonPrintPQPColPair,SIGNAL(clicked()),		this,SLOT(printPQPColPair()));
	
	void	setMaximum ( int max );
	void	setMinimum ( int min );
	
}

void RobotWidget::printAbsPos()
{
	
}

void RobotWidget::costTest()
{
	if(ENV.getBool(Env::isCostSpace))
	{
#ifdef CXX_PLANNER
		TestModel tests;
		tests.nbOfCostPerSeconds();
#endif
	}
}

void RobotWidget::collisionsTest()
{
#ifdef CXX_PLANNER
	TestModel tests;
	tests.nbOfColisionsPerSeconds();
#endif
}

void RobotWidget::localpathsTest()
{
#ifdef CXX_PLANNER	
	TestModel tests;
	tests.nbOfLocalPathsPerSeconds();
#endif
}

void RobotWidget::allTests()
{
#ifdef CXX_PLANNER		
	TestModel tests;
	tests.runAllTests();
#endif
}

void RobotWidget::setAttMatrix()
{
#ifdef LIGHT_PLANNER
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	//  p3d_compute_attached_matrix_from_virt_obj(robotPt->ccCntrts[0]);
	for(int i = 0; i < robotPt->nbCcCntrts; i++)
	{
		//p3d_compute_Tatt(robotPt->ccCntrts[i]);
		
		cout << "Tatt = " << endl;
		for (i = 0; i < 4; i++)
		{
			cout << robotPt->ccCntrts[0]->Tatt[i][0] << " " <<
			" " << robotPt->ccCntrts[0]->Tatt[i][1] <<
			" " << robotPt->ccCntrts[0]->Tatt[i][2] <<
			" " << robotPt->ccCntrts[0]->Tatt[i][3] << endl;
		}
		cout << endl;
	}
#endif
}

void RobotWidget::computeHriGik(bool leftArm)
{	
	int i=0;
	for(i=0; i<XYZ_ENV->nr; i++){
		if( strcasestr(XYZ_ENV->robot[i]->name,"VISBALL") )
			break;
	}
	if(i==XYZ_ENV->nr){
		printf("No visball in the environment\n");
		return;
	}
	
#ifdef HRI_GENERALIZED_IK
	// 1 - Select Goto point
	p3d_vector3 Tcoord;
	
	Tcoord[0] = XYZ_ENV->robot[i]->joints[1]->abs_pos[0][3];
	Tcoord[1] = XYZ_ENV->robot[i]->joints[1]->abs_pos[1][3];
	Tcoord[2] = XYZ_ENV->robot[i]->joints[1]->abs_pos[2][3];
	
	
	// 2 - Select Task
	HRI_GIK_TASK_TYPE task;
	
	if (leftArm == true) 
	{
		task = GIK_LAREACH; // Left Arm GIK
	}
	else 
	{
		task = GIK_RAREACH; // Left Arm GIK
	}
	
	// 3 - Select Agent
	HRI_AGENTS * agents = hri_create_agents();
	
	configPt q;
	
	double distance_tolerance = 1.0;
	
	if(	agents->humans_no > 0 ) // Humans
	{
		q = p3d_get_robot_config(agents->humans[0]->robotPt);
		hri_agent_single_task_manip_move(agents->humans[0], task, &Tcoord, distance_tolerance, &q);
		p3d_set_and_update_this_robot_conf(agents->humans[0]->robotPt,q);
	}
	else if ( agents->robots_no > 0) // Robots
	{
		q = p3d_get_robot_config(agents->robots[0]->robotPt);
		hri_agent_single_task_manip_move(agents->robots[0], task, &Tcoord, distance_tolerance, &q);
		p3d_set_and_update_this_robot_conf(agents->robots[0]->robotPt,q);
	}
	else {
		cout << "Warning: No Agent for GIK" << endl;
	}
	
#else
	cout << "HRI_GENERALIZED_IK not defined" << endl;
#endif
	
	
	//delete_config(robotPt,q);	
	m_mainWindow->drawAllWinActive();
}

void RobotWidget::currentObjectChange(int i)
{
	if((mFreeFlyers.size() > 0) && (i != 0))
	{
		
		//        cout << "Env::ObjectToCarry  is "<< mFreeFlyers[i-1] << endl;
		ENV.setString(Env::ObjectToCarry,mFreeFlyers[i-1]);
	}
}

void RobotWidget::SetObjectToCarry()
{
#ifdef LIGHT_PLANNER
	if(mFreeFlyers.size() > 0)
	{
		p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
		p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
		
		// Set the dist of the object to the radius of the carried object
		robotPt->curObjectJnt->dist = robotPt->carriedObject->joints[1]->dist;
		
		double radius = 1.5;
		//take only x and y composantes of the base
		double dof[2][2];
		for(int i = 0; i < 2; i++){
			dof[i][0] = p3d_jnt_get_dof(robotPt->joints[1], i) - radius;
			dof[i][1] = p3d_jnt_get_dof(robotPt->joints[1], i) + radius;
		}
		for(int i = 0; i < 2; i++){
			p3d_jnt_set_dof_rand_bounds(robotPt->curObjectJnt, i, dof[i][0], dof[i][1]);
		}
		
	}
	else
	{
		p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
		// Set the dist of the object to the radius of the carried object
		
		cout << "Setting Dist of Joint : " << robotPt->joints[6]->name << endl;
		cout << "To Joint : "  << robotPt->joints[7]->name << endl;
		
		cout << robotPt->joints[7]->dist << "  Takes "  << robotPt->joints[6]->dist << endl;
		
		robotPt->joints[7]->dist = robotPt->joints[6]->dist;
	}
#endif
}

void RobotWidget::GrabObject()
{
	
#if defined( LIGHT_PLANNER ) && defined( PQP )
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	cout << "Robot = " << robotPt->name <<  endl;
	p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
	p3d_grab_object2(robotPt,0);
	
	//    if(mFreeFlyers.size() > 0)
	//    {
	//        p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	//		//        p3d_rob *carriedObject;
	//		
	//        p3d_set_object_to_carry(robotPt,ENV.getString(Env::ObjectToCarry).toStdString().c_str());
	//		//        p3d_matrix4 saved;
	//		//        p3d_mat4Copy(robotPt->curObjectJnt->abs_pos,saved);
	//        p3d_mat4Copy(robotPt->carriedObject->joints[1]->abs_pos,robotPt->curObjectJnt->abs_pos);
	//        p3d_grab_object(robotPt,0);
	//		//        p3d_mat4Copy(saved,robotPt->curObjectJnt->abs_pos);
	//		//        configPt q = p3d_get_robot_config(robotPt);
	//		
	//		//        robotPt->ROBOT_POS = q;
	//		//        p3d_set_and_update_robot_conf(q);
	//        p3d_mat4Print(robotPt->ccCntrts[0]->Tatt,"curObject Grab");
	//    }
#endif
}

void RobotWidget::ReleaseObject()
{
	p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	cout << "Robot = " << robotPt->name <<  endl;
	/*
	 p3d_release_object(robotPt);
	 deactivateCcCntrts(robotPt, -1);
	 //	configPt qi = p3d_alloc_config(robotPt);
	 //	p3d_copy_config_into(robotPt, _robotPt->ROBOT_POS, &qi);
	 x
	 //p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
	 double tx, ty, tz, ax, ay, az;
	 p3d_mat4ExtractPosReverseOrder2(robotPt->CurObjtJnt->abs_pos, &tx, &ty, &tz, &ax, &ay, &az);
	 
	 //	p3d_set_and_update_this_robot_conf(robotPt, qi);
	 //	p3d_destroy_config(_robotPt, qi);
	 
	 qi = p3d_get_robot_config(robotPt);
	 p3d_copy_config_into(robotPt, qi, &_robotPt->ROBOT_POS);
	 p3d_destroy_config(_robotPt, qi);
	 m_ui->mainWindow->drawAllWindowActive();
	 */
	
#if defined ( LIGHT_PLANNER ) && defined( PQP )
	//    m_ui->comboBoxGrabObject-
	//    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	p3d_release_object(robotPt);
	//    m_ui->comboBoxGrabObject->setCurrentIndex(0);
#endif
};

void RobotWidget::printCurrentPos()
{
	Robot currRobot((p3d_rob*) p3d_get_desc_curid(P3D_ROBOT));
	shared_ptr<Configuration> q = currRobot.getCurrentPos();
	q->print(true );
}

#ifdef LIGHT_PLANNER
void RobotWidget::switchFKIK()
{
	cout << "Switching FK to IK" << endl;
	
	Robot* ptrRob = global_Project->getActiveScene()->getActiveRobot();
	
	if ( ptrRob->isActiveCcConstraint() ) 
	{
		ptrRob->deactivateCcConstraint();
	}
	else
	{
		ptrRob->activateCcConstraint();
	}
	
	getMoveRobot()->setRobotConstraintedDof(ptrRob);
	
}
#endif


void RobotWidget::printPQPColPair()
{
#ifdef PQP
	pqp_print_colliding_pair();	
#else
	cout << "PQP not compiled" << endl;
#endif
}
/*void RobotWidget::initVoxelCollisionChecker()
 {
 m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawVoxelGrid,				Env::drawGrid);
 
 connect(m_ui->pushButtonCreateVoxelCollisionChecker,SIGNAL(clicked()),this,SLOT(createVoxelCC()));
 connect(m_ui->pushButtonDeleteVoxelCollisionChecker,SIGNAL(clicked()),this,SLOT(deleteVoxelCC()));
 
 connect(m_ui->pushButtonVoxelCC,SIGNAL(clicked()),this,SLOT(voxelCCTest()));
 connect(ENV.getObject(Env::numberOfCollisionPerSec),SIGNAL(valueChanged(QString)),m_ui->labelVoxelTime,SLOT(setText(QString)));
 
 connect(m_ui->spinBoxVoxelCCnbCells,SIGNAL(valueChanged(int)),ENV.getObject(Env::nbCells),SLOT(set(int)));
 connect(ENV.getObject(Env::nbCells),SIGNAL(valueChanged(int)),m_ui->spinBoxVoxelCCnbCells,SLOT(setValue(int)));
 }
 
 void RobotWidget::createVoxelCC()
 {
 API_activeGrid = global_GridCollisionChecker = new GridCollisionChecker;
 ENV.setBool(Env::drawGrid, true);
 }
 
 void RobotWidget::deleteVoxelCC()
 {
 ENV.setBool(Env::drawAll,false);
 delete API_activeGrid;
 API_activeGrid = NULL;
 }
 
 void RobotWidget::voxelCCTest()
 {
 #ifdef CXX_PLANNER
 TestModel tests;
 tests.nbOfVoxelCCPerSeconds();
 #endif
 }*/

// ------------------------------------------------------------------------------
// Arm manipulations
// ------------------------------------------------------------------------------
void RobotWidget::initManipulation()
{
	connect(m_ui->pushButtonArmFree,SIGNAL(clicked()),						this,SLOT(armFree()));
	connect(m_ui->pushButtonArmPickGoto,SIGNAL(clicked()),				this,SLOT(armPickGoto()));
	connect(m_ui->pushButtonArmPickTakeToFree,SIGNAL(clicked()),	this,SLOT(armPickTakeToFree()));
	connect(m_ui->pushButtonArmPickGotoAndTakeToFree,SIGNAL(clicked()),	this,SLOT(armPickGotoAndTakeToFree()));
}

#ifdef MULTILOCALPATH
static ManipulationPlanner *manipulation= NULL;

namespace Manip
{
	typedef enum ManipulationType
	{
		armFree,
		pickGoto,
		takeToFree,
		pickGotoAndTakeToFree,
	} 
	ManipulationType;
};

Manip::ManipulationType ManipPhase;

shared_ptr<Configuration> qInit;
shared_ptr<Configuration> qGoal;

static void initManipulationGenom() 
{
  if (manipulation == NULL) 
	{
		Robot* r = global_Project->getActiveScene()->getRobotByName("JIDOKUKA_ROBOT");
		
		p3d_rob * robotPt= r->getRobotStruct();
		
		manipulation= new ManipulationPlanner(robotPt);
		//         manipulation->setArmType(GP_LWR); // set the arm type
		
		qInit = shared_ptr<Configuration>(new Configuration(r,robotPt->ROBOT_POS));
		qGoal = shared_ptr<Configuration>(new Configuration(r,robotPt->ROBOT_GOTO));
  }
	
  return;
}
#endif

/**
 * @ingroup qtWindow
 * @brief Planner thread class
 */
//-----------------------------------------------
Manipulationthread::Manipulationthread(QObject* parent) :
QThread(parent)
{
	
}

void Manipulationthread::run()
{
#ifdef MULTILOCALPATH	
	//         double x, y, theta;
	if (manipulation== NULL) 
	{
		initManipulationGenom();
	}
	
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs; 
	
	cout << "Selected object is : " << ENV.getString(Env::ObjectToCarry).toStdString() << endl;
	
	if ( ENV.getString(Env::ObjectToCarry).toStdString().compare("No Object") == 0) 
	{
		cout << "Warning : No object selected" << endl;
		ENV.setBool(Env::isRunning,false);
		return;
	}
	
	string str = ENV.getString(Env::ObjectToCarry).toStdString();
	const char* OBJECT_NAME = str.c_str();
	
	switch (ManipPhase) 
	{
		case Manip::armFree:
		{
			confs.clear();
			smTrajs.clear();
			trajs.clear();
			
			switch ( manipulation->robot()->lpl_type ) 
			{
				case P3D_LINEAR_PLANNER :
				{
					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_FREE,0,
																																			 qInit->getConfigStruct(), 
																																			 qGoal->getConfigStruct(), 
																																			 OBJECT_NAME, "", trajs);
					if(status == MANIPULATION_TASK_OK )
					{
						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
						
						for(unsigned int i = 1; i < trajs.size(); i++){
							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
						}
					}
				}
					break;
					
				case P3D_MULTILOCALPATH_PLANNER :
					
					manipulation->armPlanTask(ARM_PICK_GOTO,0,
																		qInit->getConfigStruct(), 
																		qGoal->getConfigStruct(), 
																		OBJECT_NAME, "", confs, smTrajs);
					break;
					
				case P3D_SOFT_MOTION_PLANNER:
					cout << "Manipulation : localpath softmotion should not be called" << endl;
					break;
					
				default:
					break;
			}
			break;
		}
			
		break;

		case Manip::pickGoto:
		{
			//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
			//         manipulation->setSupport((char*)SUPPORT_NAME);
			//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
			//         manipulation->setCameraFOV(CAMERA_FOV);
			//         manipulation->setCameraImageSize(200, 200);
			
			confs.clear();
			smTrajs.clear();
			trajs.clear();
			
			switch ( manipulation->robot()->lpl_type ) 
			{
				case P3D_LINEAR_PLANNER :
				{
					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_PICK_GOTO,0,
																						qInit->getConfigStruct(), 
																						qGoal->getConfigStruct(), 
																						OBJECT_NAME, "", trajs);
					if(status == MANIPULATION_TASK_OK )
					{
						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
						
						for(unsigned int i = 1; i < trajs.size(); i++){
							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
						}
					}
				}
					break;
					
				case P3D_MULTILOCALPATH_PLANNER :
					
					manipulation->armPlanTask(ARM_PICK_GOTO,0,
																		qInit->getConfigStruct(), 
																		qGoal->getConfigStruct(), 
																		OBJECT_NAME, "", confs, smTrajs);
					break;
					
				case P3D_SOFT_MOTION_PLANNER:
					cout << "Manipulation : localpath softmotion should not be called" << endl;
					break;

				default:
					break;
			}
			
			break;
		}
			
		case Manip::takeToFree:
		{
			confs.clear();
			smTrajs.clear();
			trajs.clear();
			
			switch ( manipulation->robot()->lpl_type ) 
			{
				case P3D_LINEAR_PLANNER :
				{
					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_PICK_TAKE_TO_FREE,0,qInit->getConfigStruct(), 
								 qGoal->getConfigStruct(), 
								 OBJECT_NAME, (char*)"", trajs);
					
					if( status == MANIPULATION_TASK_OK)
					{
						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
						
						for(unsigned int i = 1; i < trajs.size(); i++){
							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
						}
					}
				}
					break;
					
				case P3D_MULTILOCALPATH_PLANNER :
					
					manipulation->armPlanTask(ARM_PICK_TAKE_TO_FREE,0,
																		qInit->getConfigStruct(), 
																		qGoal->getConfigStruct(),  
																		OBJECT_NAME, "", confs, smTrajs);
					break;
					
				case P3D_SOFT_MOTION_PLANNER:
					cout << "Manipulation : localpath softmotion should not be called" << endl;
					break;

				default:
					break;
			}
			break;
		}
			
		case Manip::pickGotoAndTakeToFree :
		{
			confs.clear();
			smTrajs.clear();
			trajs.clear();
			
			switch ( manipulation->robot()->lpl_type ) 
			{
				case P3D_LINEAR_PLANNER :
				{
					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_PICK_GOTO_AND_TAKE_TO_FREE,0,
								 qInit->getConfigStruct(), 
								 qGoal->getConfigStruct(), 
									OBJECT_NAME, (char*)"", trajs);
					
					if( status == MANIPULATION_TASK_OK)
					{
						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
						
						for(unsigned int i = 1; i < trajs.size(); i++){
							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
							}
						}
					}
					break;
					
				case P3D_MULTILOCALPATH_PLANNER :
					
					manipulation->armPlanTask(ARM_PICK_GOTO_AND_TAKE_TO_FREE,0,
																		qInit->getConfigStruct(), 
																		qGoal->getConfigStruct(),  
																		OBJECT_NAME, "", confs, smTrajs);
					
					//manipulation->robot()
					
					break;
					
				case P3D_SOFT_MOTION_PLANNER:
					cout << "Manipulation : localpath softmotion should not be called" << endl;
					break;
					
				default:
					break;
			}
			break;
		}
		default:
			cout << "Manipulation : request does not exist" << endl;
			break;
	}
	
	
	//	g3d_win *win= NULL;
	//	win= g3d_get_cur_win();
	//	win->fct_draw2= &(genomDraw);
	//	win->fct_key1= &(genomKey);
	g3d_draw_allwin_active();
	ENV.setBool(Env::isRunning,false);
	cout << "Ends Manipulation Thread" << endl;
#endif
}

void RobotWidget::armFree()
{
	cout << "Manipulation : free" << endl;
	
#ifdef MULTILOCALPATH	
	ManipPhase = Manip::armFree;
	Manipulationthread* manip = new Manipulationthread(this);
	m_mainWindow->isPlanning();
	manip->start();
#else
	cout << "Error : use MultiLocalPath" << endl;
#endif
}

void RobotWidget::armPickGoto()
{
	cout << "Manipulation : pick goto" << endl;
	
#ifdef MULTILOCALPATH	
	ManipPhase = Manip::pickGoto;
	Manipulationthread* manip = new Manipulationthread(this);
	m_mainWindow->isPlanning();
	manip->start();
#else
	cout << "Error : use MultiLocalPath" << endl;
#endif
}

void RobotWidget::armPickTakeToFree()
{
	cout << "Manipulation : take to free" << endl;
	
#ifdef MULTILOCALPATH	
	ManipPhase = Manip::takeToFree;
	Manipulationthread* manip = new Manipulationthread(this);
	m_mainWindow->isPlanning();
	manip->start();
#else
	cout << "Error : use MultiLocalPath" << endl;
#endif
}

void RobotWidget::armPickGotoAndTakeToFree()
{
	cout << "Manipulation : pick goto and take to free" << endl;
	
#ifdef MULTILOCALPATH	
	ManipPhase = Manip::pickGotoAndTakeToFree;
	Manipulationthread* manip = new Manipulationthread(this);
	m_mainWindow->isPlanning();
	manip->start();
#else
	cout << "Error : use MultiLocalPath" << endl;
#endif
}

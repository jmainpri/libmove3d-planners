/*
 *  qtMotionPlanner.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtMotionPlanner.hpp"
#include "ui_qtMotionPlanner.h"

#include <iostream>
#include <tr1/memory>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

#include "planEnvironment.hpp"
#include "cppToQt.hpp"
#include "cost_space.hpp"
#include "API/project.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "ParametersEnv.hpp"

#include "SaveContext.hpp"
#include "MultiRun.hpp"

using namespace std;
using namespace tr1;

MotionPlanner::MotionPlanner(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::MotionPlanner)
{
	m_ui->setupUi(this);
	
	initDiffusion();
	initPRM();
	initOptim();
	initMultiRun();
	initGeneral();
	initShowGraph();
}

MotionPlanner::~MotionPlanner()
{
	delete m_ui;
}

//---------------------------------------------------------------------
// GENERAL
//---------------------------------------------------------------------
void MotionPlanner::initGeneral()
{
	connect(m_ui->pushButtonCheckAllEdges,SIGNAL(clicked()),this,SLOT(checkAllEdges()));
	
	// Connecting the Dmax Env variable
	m_ui->doubleSpinBoxDMax->setValue(p3d_get_env_dmax());
	connect(m_ui->doubleSpinBoxDMax, SIGNAL(valueChanged( double )), SLOT(envDmaxSpinBoxValueChanged( double ) ) );
	connect(m_ui->doubleSpinBoxDMax, SIGNAL(valueChanged( double )), ENV.getObject(Env::dmax),SLOT(set(double)));
	connect(ENV.getObject(Env::dmax), SIGNAL(valueChanged(double)),m_ui->doubleSpinBoxDMax, SLOT(setValue(double)));
	
	
	//	connect(m_ui->doubleSpinBoxTest, SIGNAL(valueChanged( double )), PlanEnv->getObject(PlanParam::eleven),SLOT(set(double)));
	//	connect(PlanEnv->getObject(PlanParam::eleven), SIGNAL(valueChanged(double)),m_ui->doubleSpinBoxTest, SLOT(setValue(double)));
	//	connect(PlanEnv->getObject(PlanParam::eleven), SIGNAL(valueChanged(double)),this,SLOT(testParam(double)));
}

void MotionPlanner::testParam(double param)
{
	//cout << "Value changed : " << PlanEnv->getDouble(PlanParam::eleven) << endl;
}

void MotionPlanner::checkAllEdges()
{
	Graph* tmpGraph = new Graph(XYZ_GRAPH);
	
	if(tmpGraph->checkAllEdgesValid())
	{
		cout << "Graph valid" << endl;
	}
	else {
		cout << "Graph Not valid" << endl;
	}
	
}

void MotionPlanner::envDmaxSpinBoxValueChanged( double dmax )
{
	p3d_set_env_dmax( dmax );
}


//---------------------------------------------------------------------
// DIFFUSION
//---------------------------------------------------------------------
void MotionPlanner::initDiffusion()
{
	//    m_mainWindow->connectCheckBoxToEnv(m_ui->isCostSpace,         Env::isCostSpace);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isWithGoal,          Env::expandToGoal);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isManhattan,         Env::isManhattan);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isEST,               Env::treePlannerIsEST);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isBidir,             Env::biDir);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isBalanced,          Env::expandBalanced);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isExpandControl,     Env::expandControl);
	m_mainWindow->connectCheckBoxToEnv(m_ui->isDiscardingNodes,   Env::discardNodes);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsGoalBias,  Env::isGoalBiased);
	//    m_mainWindow->connectCheckBoxToEnv(m_ui->isCostTransition,    Env::costBeforeColl);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRandomInCompCo, Env::randomConnectionToGoal);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxClosestInCompCo, Env::tryClosest);
	
	m_ui->expansionMethod->setCurrentIndex((int)ENV.getExpansionMethod());
	connect(m_ui->expansionMethod, SIGNAL(currentIndexChanged(int)),&ENV, SLOT(setExpansionMethodSlot(int)), Qt::DirectConnection);
	connect(&ENV, SIGNAL(expansionMethodChanged(int)),m_ui->expansionMethod, SLOT(setCurrentIndex(int)));
	
	//    m_ui->lineEditMaxNodes->setText( QString::number(ENV.getInt(Env::maxNodeCompco)));
	m_ui->spinBoxMaxNodes->setValue(ENV.getInt(Env::maxNodeCompco));
	connect(m_ui->spinBoxMaxNodes, SIGNAL(valueChanged( int )), ENV.getObject(Env::maxNodeCompco), SLOT(set(int)));
	connect(ENV.getObject(Env::maxNodeCompco), SIGNAL(valueChanged( int )), m_ui->spinBoxMaxNodes, SLOT(setValue(int)));
	
	m_ui->spinBoxNbTry->setValue(ENV.getInt(Env::NbTry));
	connect(m_ui->spinBoxNbTry, SIGNAL(valueChanged( int )), ENV.getObject(Env::NbTry), SLOT(set(int)));
	connect(ENV.getObject(Env::NbTry), SIGNAL(valueChanged( int )), m_ui->spinBoxNbTry, SLOT(setValue(int)));
	
	//    connect(ENV.getObject(Env::maxNodeCompco),SIGNAL(valueChanged(int)),this,SLOT(setLineEditWithNumber(Env::maxNodeCompco,int)));
	//    connect(m_ui->lineEditMaxNodes,SIGNAL(getText(QString::number(int))),ENV.getObject(Env::maxNodeCompco),SLOT(setInt(int)));
	//    cout << "ENV.getBool(Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
	
	//    connect(m_ui->lineEditExtentionStep,SIGNAL(textEdited(QString)),this,SLOT(lineEditChangedStep()));
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxExtentionStep, m_ui->horizontalSliderExtentionStep , Env::extensionStep );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxBias, m_ui->horizontalSliderBias , Env::Bias );
	
	initMultiRRT();
}

void MotionPlanner::initMultiRRT()
{
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxMultiRRT,         Env::isMultiRRT);
	
	connect(m_ui->spinBoxNumberOfSeeds,SIGNAL(valueChanged(int)),ENV.getObject(Env::nbOfSeeds),SLOT(set(int)));
	connect(ENV.getObject(Env::nbOfSeeds),SIGNAL(valueChanged(int)),m_ui->spinBoxNumberOfSeeds,SLOT(setValue(int)));	
}
//void MainWindow::setLineEditWithNumber(Env::intParameter p,int num)
//{
//    if(p == Env::maxNodeCompco)
//    {
//        m_ui->lineEditMaxNodes->setText(QString::number(num));
//    }
//}

//---------------------------------------------------------------------
// PRM
//---------------------------------------------------------------------
void MotionPlanner::initPRM()
{
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxUseDistance,Env::useDist);
	// PRMType
	connect(m_ui->comboBoxPRMType, SIGNAL(currentIndexChanged(int)),ENV.getObject(Env::PRMType),SLOT(set(int)));
	connect( ENV.getObject(Env::PRMType), SIGNAL(valueChanged(int)),m_ui->comboBoxPRMType, SLOT(setCurrentIndex(int)));
	m_ui->comboBoxPRMType->setCurrentIndex( 0 /*INTEGRAL*/ );
	// 0 => PRM
	// 1 => Visib
	// 2 => ACR
	
	//    connect(ENV.getObject(Env::PRMType), SIGNAL(valueChanged(int)),m_ui->comboBoxPRMType, SLOT(setCurrentIndex(int)));
	
	m_ui->spinBoxMaxConnect->setValue(ENV.getInt(Env::maxConnect));
	connect(m_ui->spinBoxMaxConnect, SIGNAL(valueChanged(int)), ENV.getObject(Env::maxConnect), SLOT(set(int)));
	connect(ENV.getObject(Env::maxConnect), SIGNAL(valueChanged(int)), m_ui->spinBoxMaxConnect, SLOT(setValue(int)));
	
}

//---------------------------------------------------------------------
// OPTIM
//---------------------------------------------------------------------
void MotionPlanner::initOptim()
{	
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxCostSpace2,				Env::isCostSpace );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxDebug2,						Env::debugCostOptim );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxSaveTrajCost,			PlanParam::saveTrajCost );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithDeform,				PlanParam::withDeformation );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithShortCut,			PlanParam::withShortCut );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithTimeLimit,		PlanParam::withTimeLimit );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithGainLimit,		PlanParam::withGainLimit );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithIterLimit,		PlanParam::withMaxIteration );
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxWithDescent,			PlanParam::withDescent );
	
	m_mainWindow->connectCheckBoxToEnv( m_ui->checkBoxPartialShortcut,	PlanParam::partialShortcut );
	
	connect(m_ui->pushButtonRandomShortCut,				SIGNAL(clicked()),this,SLOT(shortCutCost()));
	connect(m_ui->pushButtonRemoveRedundantNodes,	SIGNAL(clicked()),this,SLOT(removeRedundant()));
	connect(m_ui->pushButtonEraseDebugTraj,				SIGNAL(clicked()),this,SLOT(eraseDebugTraj()));
	connect(m_ui->pushButtonCutTrajInSmallLP,				SIGNAL(clicked()),this,SLOT(cutTrajInSmallLP()));
	
	connect(m_ui->pushButtonTriangleDeformation,SIGNAL(clicked()),this,SLOT(optimizeCost()));
	
	// costCriterium
	connect( ENV.getObject(Env::costDeltaMethod),	SIGNAL(valueChanged(int)),this,		SLOT(setCostCriterium(int)), Qt::DirectConnection );
	//		connect( ENV.getObject(Env::costDeltaMethod),	SIGNAL(valueChanged(int)),				m_ui->comboBoxTrajCostExtimation, SLOT(setCurrentIndex(int)));
	connect( m_ui->comboBoxTrajCostExtimation,		SIGNAL(currentIndexChanged(int)),	this, SLOT(setCostCriterium(int)) );
	m_ui->comboBoxTrajCostExtimation->setCurrentIndex( /*MECHANICAL_WORK*/ INTEGRAL );
	setCostCriterium(MECHANICAL_WORK);
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxNbRounds, m_ui->horizontalSliderNbRounds_2 , Env::nbCostOptimize );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxNbMultiSmooth, m_ui->horizontalSliderNbMultiSmooth , Env::nbMultiSmooth );
	
	//QtShiva::SpinBoxSliderConnector* connector1 = 
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxMinDeformStep, m_ui->horizontalSliderMinDeformStep , PlanParam::MinStep );
	
	//QtShiva::SpinBoxSliderConnector* connector2 = 
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxTimeLimit, m_ui->horizontalSliderTimeLimit , PlanParam::optimTimeLimit );
	
	connect(m_ui->pushButtonRunMultiSmooth,SIGNAL(clicked()),this,SLOT(runMultiSmooth()));
	
	// ------------------------------------------------------------
	// ------------------------------------------------------------
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxMaxDeformStep, m_ui->horizontalSliderMaxDeformStep , PlanParam::MaxFactor );
	
	//connect(connector,SIGNAL(valueChanged(double)),PlanEnv->getObject(PlanParam::optimTimeLimit),SLOT(set(double)));
	//connect(PlanEnv->getObject(PlanParam::optimTimeLimit),SIGNAL(valueChanged(double)),this,SLOT(test(double)));
	//connect(connector1,SIGNAL(valueChanged(double)),this,SLOT(test(double)));
	
	//
	//	PlanEnv->getObject(PlanParam::optimTimeLimit)->dumpObjectInfo();
	//	PlanEnv->setDouble(PlanParam::optimTimeLimit,15.0);
	//	PlanEnv->setDouble(PlanParam::optimTimeLimit,30.0);
	//PlanEnv->getObject(PlanParam::optimTimeLimit)->metaObject()->connectSlotsByName ();
}

void MotionPlanner::test(double value)
{
	//cout << "Value of PlanParam::optimTimeLimit : " << PlanEnv->getDouble(PlanParam::optimTimeLimit) << endl;
	cout << "Value of PlanParam::optimTimeLimit : " << PlanEnv->getDouble(PlanParam::MinStep) << endl;
}

void MotionPlanner::cutTrajInSmallLP()
{
	Robot* rob =	global_Project->getActiveScene()->getActiveRobot();
	double dmax = global_Project->getActiveScene()->getDMax();

	API::Trajectory traj = rob->getCurrentTraj();
		
	cout << "Cutting into small LP" << endl;
	traj.cutTrajInSmallLP( floor( traj.getRangeMax() / dmax ) );
	traj.replaceP3dTraj();
}

void MotionPlanner::eraseDebugTraj()
{
	trajToDraw.clear();
}

void MotionPlanner::setCostCriterium(int choice) 
{
  cout << "Set Delta Step Choise to " << choice << endl;
	
#ifdef P3D_PLANNER
	p3d_SetDeltaCostChoice(choice);
#endif
	
	map<int,CostSpaceDeltaStepMethod> methods;
	
	methods.insert ( pair<int,CostSpaceDeltaStepMethod>(MECHANICAL_WORK,cs_mechanical_work) );
	methods.insert ( pair<int,CostSpaceDeltaStepMethod>(INTEGRAL,cs_integral) );
	methods.insert ( pair<int,CostSpaceDeltaStepMethod>(2,cs_visibility) );
	methods.insert ( pair<int,CostSpaceDeltaStepMethod>(3,cs_average) );
	methods.insert ( pair<int,CostSpaceDeltaStepMethod>(4,cs_config_cost_and_dist) );
	
	if(ENV.getBool(Env::isCostSpace))
	{
		global_costSpace->setDeltaStepMethod( methods[choice] );
	}
	
	ENV.setInt(Env::costDeltaMethod,choice);
	//m_ui->comboBoxTrajCostExtimation->setCurrentIndex(choice);
}

void MotionPlanner::computeGrid()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
#ifdef P3D_PLANNER
	p3d_CreateDenseRoadmap(robotPt);
#endif
}

void MotionPlanner::runMultiSmooth()
{
#ifdef WITH_XFORMS
	std::string str = "MultiSmooth";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	cout << "Not implemented" << endl;
#endif
	
}

/**
 * @ingroup qtWindow
 * @brief Planner thread class
 */
//-----------------------------------------------
SmoothThread::SmoothThread(bool isShortCut, QObject* parent) :
QThread(parent),
m_isShortCut(isShortCut)
{
	
}

void SmoothThread::run()
{	
	if (m_isShortCut) 
	{
		qt_shortCut();
	}
	else {
		qt_optimize();
	}
	
	cout << "Ends Smooth Thread" << endl;
}
//-----------------------------------------------

void MotionPlanner::optimizeCost()
{
	if( ENV.getBool(Env::isRunning) )
		return;
	
#ifdef WITH_XFORMS
	std::string str = "optimize";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	m_mainWindow->isPlanning();
	SmoothThread* ptrSmooth = new SmoothThread(false);
	cout << "Start Smooth Thread" << endl;
	ptrSmooth->start();
#endif
	
}

void MotionPlanner::shortCutCost()
{
	if( ENV.getBool(Env::isRunning) )
		return;
	
	ENV.setBool(Env::isRunning,true);
#ifdef WITH_XFORMS
	std::string str = "shortCut";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	m_mainWindow->isPlanning();
	SmoothThread* ptrSmooth = new SmoothThread(true);
	cout << "Start Smooth Thread" << endl;
	ptrSmooth->start();
#endif
	
}

void MotionPlanner::removeRedundant()
{
	ENV.setBool(Env::isRunning,true);
#ifdef WITH_XFORMS
	std::string str = "removeRedunantNodes";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	cout << "Not implemented" << endl;
#endif
}



void MotionPlanner::extractBestTraj()
{
#ifdef P3D_PLANNER
	p3d_ExtractBestTraj(XYZ_GRAPH);
#endif
}

//---------------------------------------------------------------------
// Multiple Runs
//---------------------------------------------------------------------
void MotionPlanner::initMultiRun()
{
	connect(m_ui->pushButtonSaveContext, SIGNAL(clicked()),this,SLOT(saveContext()));
	connect(m_ui->pushButtonSetSelected, SIGNAL(clicked()),this,SLOT(setToSelected()));
	connect(m_ui->pushButtonPrintSelected, SIGNAL(clicked()),this,SLOT(printContext()));
	connect(m_ui->pushButtonDeleteSelected, SIGNAL(clicked()),this,SLOT(deleteSelected()));
	connect(m_ui->pushButtonPrintAllContext, SIGNAL(clicked()),this,SLOT(printAllContext()));
	connect(m_ui->pushButtonResetContext, SIGNAL(clicked()),this,SLOT(resetContext()));
	
	contextList = new QListWidget;
	m_ui->multiRunLayout->addWidget(contextList);
	
	//    new QtShiva::SpinBoxSliderConnector(
	//            this, m_ui->doubleSpinBoxNbRounds, m_ui->horizontalSliderNbRounds , Env::nbMultiRun );
	
	
	connect(m_ui->horizontalSliderNbMutliRun, SIGNAL(valueChanged(int)), m_ui->spinBoxNbMutliRun, SLOT(setValue(int)) );
	connect(m_ui->spinBoxNbMutliRun, SIGNAL(valueChanged(int)),ENV.getObject(Env::nbMultiRun), SLOT(set(int)) );
	connect(m_ui->spinBoxNbMutliRun, SIGNAL(valueChanged(int)),m_ui->horizontalSliderNbMutliRun, SLOT(setValue(int)) );
	connect(ENV.getObject(Env::nbMultiRun), SIGNAL(valueChanged(int)),m_ui->spinBoxNbMutliRun, SLOT(setValue(int)) );
	m_ui->spinBoxNbMutliRun->setValue(ENV.getInt(Env::nbMultiRun));
	
	
	connect(m_ui->pushButtonRunAllRRT, SIGNAL(clicked()),this,SLOT(runAllRRT()));
	connect(m_ui->pushButtonRunAllGreedy, SIGNAL(clicked()),this,SLOT(runAllGreedy()));
	connect(m_ui->pushButtonShowHisto, SIGNAL(clicked()),this, SLOT(showHistoWindow()));
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxStopMultiRun,Env::StopMultiRun);
}

void MotionPlanner::saveContext()
{
	ENV.setString(Env::nameOfFile,m_ui->lineEditContext->text());
	
	QListWidgetItem* item= new QListWidgetItem(contextList);
	itemList.push_back(item);
	itemList.back()->setText(m_ui->lineEditContext->text());
	
	storedContext.saveCurrentEnvToStack();
	storedPlannerContext->saveCurrentEnvToStack();
}

void MotionPlanner::printAllContext()
{
	if( storedContext.getNumberStored()>0){
		
		for(uint i=0;i<storedContext.getNumberStored();i++){
			std::cout << "------------ Context Number " << i << " ------------" << std::endl;
			storedContext.printData(i);
			storedPlannerContext->printData(i);
		}
		std::cout << "-------------------------------------------" << std::endl;
		std::cout << " Total number of contexts in stack =  " << storedContext.getNumberStored() << std::endl;
		std::cout << "-------------------------------------------" << std::endl;
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void MotionPlanner::setToSelected()
{
	if( storedContext.getNumberStored()>0)
	{
		int i =  contextList->currentRow();
		storedContext.switchCurrentEnvTo(i);
		storedPlannerContext->switchCurrentEnvTo(i);
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void MotionPlanner::deleteSelected()
{
	if( storedContext.getNumberStored()>0)
	{
		int i =  contextList->currentRow();
		storedContext.deleteEnv(i);
		storedPlannerContext->deleteEnv(i);
		std::cout << "Delete context " << i << std::endl;
	}
	else{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void MotionPlanner::printContext()
{
	if( storedContext.getNumberStored() > 0 )
	{
		int i =  contextList->currentRow();
		std::cout << "------------ Context Number " << i << " ------------" << std::endl;
		storedContext.printData(i);
		storedPlannerContext->printData(i);
	}
	else
	{
		std::cout << "Warning: no context in stack" << std::endl;
	}
}

void MotionPlanner::resetContext()
{
	storedContext.clear();
	storedPlannerContext->clear();
	//	setContextUserApp(context);
	for(unsigned int i=0;i<itemList.size();i++)
	{
		delete itemList.at(i);
	}
	itemList.clear();
}

/**
 * @ingroup qtWindow
 * @brief Multi Planner thread class
 */
//-----------------------------------------------
MultiThread::MultiThread(bool isRRT, QObject* parent) :
QThread(parent),
m_isRRT(isRRT)
{
	
}

void MultiThread::run()
{	
	if (m_isRRT) 
	{
		MultiRun multiRRTs;
		multiRRTs.runMutliRRT();
	}
	else {
		
	}
	
	cout << "Ends Multi Thread" << endl;
}
//-----------------------------------------------

void MotionPlanner::runAllRRT()
{
	//	runAllRounds->setDisabled(true);
#ifdef WITH_XFORMS
	std::string str = "MultiRRT";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	MultiThread* ptrPlan = new MultiThread(true);
	ptrPlan->start();
	
#endif
}

void MotionPlanner::runAllGreedy()
{
	//	runAllRounds->setDisabled(true);
	std::string str = "MultiGreedy";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MotionPlanner::showHistoWindow()
{
#ifdef QWT
	histoWin = new HistoWindow();
	histoWin->startWindow();
#endif
}

//-----------------------------------------------
// Show Graph
//-----------------------------------------------
void MotionPlanner::initShowGraph()
{
	//connect(m_ui->spinBoxNodeToShow, SIGNAL(valueChanged(int)),ENV.getObject(Env::cellToShow),SLOT(set(int)), Qt::DirectConnection);
	connect(m_ui->spinBoxNodeToShow, SIGNAL(valueChanged(int)),this,SLOT(nodeToShowChanged()), Qt::QueuedConnection);
	connect(m_ui->pushButtonRemoveNode, SIGNAL(clicked()),this,SLOT(removeNode()) );
	
}

void MotionPlanner::nodeToShowChanged()
{
	if ( !API_activeGraph ) {
		return;
	}
	
	Graph* graph = API_activeGraph;
	vector<Node*> nodes = graph->getNodes();
	
	if (nodes.empty()) {
		cout << "Warning :: nodes is empty!!!" << endl;
	}
	
	int ith = m_ui->spinBoxNodeToShow->value();
	
	cout << "Showing node nb : " << ith << endl;
	cout << "graph size nb : " << nodes.size() << endl;
	
	if ( (ith >= 0) && ( ((int)nodes.size()) > ith) ) 
	{
		graph->getRobot()->setAndUpdate(*nodes[ith]->getConfiguration());
		cout << "Node number : " << nodes[ith]->getNodeStruct()->num << endl ;
		cout << "Connected Component : " << nodes[ith]->getConnectedComponent()->getId() << endl ;
	}
	else 
	{
		shared_ptr<Configuration> q_init = graph->getRobot()->getInitialPosition();
		graph->getRobot()->setAndUpdate(*q_init);
		cout << "Exede the number of nodes" << endl;
	}
	m_mainWindow->drawAllWinActive();
}

void MotionPlanner::removeNode()
{
	if ( !API_activeGraph ) 
	{
		return;
	}
	
	try 
	{
		Graph* graph = API_activeGraph;
		
		vector<Node*> nodes = graph->getNodes();
		
		if ( nodes.empty() ) 
		{
			cout << "Warning :: nodes is empty!!!" << endl;
		}
		
		int ith = m_ui->spinBoxNodeToShow->value();
		
		cout << "Removing node nb : " << ith << endl;
		cout << "graph size nb : " << nodes.size() << endl;
		
		if ( (ith >= 0) && ( ((int)nodes.size()) > ith) ) 
		{
			graph->removeNode( nodes[ith] );
		}
		else 
		{
			shared_ptr<Configuration> q_init = graph->getRobot()->getInitialPosition();
			graph->getRobot()->setAndUpdate(*q_init);
			cout << "Exede the number of nodes" << endl;
		}
	}
	catch (std::string str) 
	{
		cerr << "Exeption in MotionPlanner::removeNode()" << endl;
		cerr << str << endl;
	}
	
	m_mainWindow->drawAllWinActive();
}



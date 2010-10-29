/*
 *  qtUtil.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtUtil.hpp"
#include "ui_qtUtil.h"

#include "cppToQt.hpp"

#include "API/project.hpp"


using namespace std;
using namespace tr1;

UtilWidget::UtilWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::UtilWidget)
{
	m_ui->setupUi(this);
	
	initRRTthreshold();
	initRRTstar();
	initGreedy();
	initDistanceAndLocalpath();
	
}

UtilWidget::~UtilWidget()
{
	delete m_ui;
}

void UtilWidget::drawAllWinActive()
{
	m_mainWindow->drawAllWinActive();
}

//---------------------------------------------------------------------
// GREEDY
//---------------------------------------------------------------------
void UtilWidget::initGreedy()
{
	// Greedy
	greedy = new QPushButton("Greedy Planner");
	connect(greedy, SIGNAL(clicked()),this, SLOT(greedyPlan()),Qt::DirectConnection);
	
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDebug,               Env::debugCostOptim);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRecomputeTrajCost,   PlanParam::trajCostRecompute);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxUseTRRT,             Env::useTRRT);
	
	LabeledSlider* nbGreedyTraj = m_mainWindow->createSlider(tr("Number of trajectories"), Env::nbGreedyTraj, 1, 10 );
	LabeledSlider* hight = m_mainWindow->createSlider(tr("Height Factor"), Env::heightFactor, 1, 10 );
	LabeledSlider* numberIterations = m_mainWindow->createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 500 );
//	LabeledDoubleSlider* maxFactor = m_mainWindow->createDoubleSlider(tr("Start Factor"), PlanParam::MaxFactor, 0, 2000 );
//	LabeledDoubleSlider* minStep = m_mainWindow->createDoubleSlider(tr("Min step"), PlanParam::MinStep, 0, 1000 );
	LabeledDoubleSlider* costStep = m_mainWindow->createDoubleSlider(tr("Cost Step"), Env::costStep, 0.01, 10 );
	
	//	double dmax=0;
	//	p3d_col_get_dmax(&dmax);
	//
	//	minStep->setValue(dmax);
	numberIterations->setValue(ENV.getInt(Env::nbCostOptimize));
	
	//    QPushButton* biasPos = new QPushButton("Biased Pos");
	//    connect(biasPos, SIGNAL(clicked()),this, SLOT(biasPos()),Qt::DirectConnection);
	
	//    QComboBox* costCriterium = new QComboBox();
	//    costCriterium->insertItem(INTEGRAL, "Integral");
	//    costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
	//    costCriterium->insertItem(VISIBILITY, "Visibility");
	//    costCriterium->setCurrentIndex((int)(INTEGRAL));
	//
	//    connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);
	
	m_ui->greedyLayout->addWidget(greedy);
	m_ui->greedyLayout->addWidget(nbGreedyTraj);
	m_ui->greedyLayout->addWidget(numberIterations);
	m_ui->greedyLayout->addWidget(hight);
//	m_ui->greedyLayout->addWidget(maxFactor);
//	m_ui->greedyLayout->addWidget(minStep);
	m_ui->greedyLayout->addWidget(costStep);
	//    m_ui->greedyLayout->addWidget(costCriterium);
	//    m_ui->greedyLayout->addWidget(biasPos);
	
}

void UtilWidget::biasPos() 
{
	//    Robot* R = new Robot(XYZ_ROBOT);
	//    CostOptimization* costOptim = new CostOptimization(R,R->getTrajStruct());
	//    tr1::shared_ptr<Configuration> q = costOptim->cheat();
	//    costOptim->getRobot()->setAndUpdate(*q);
	//    this->drawAllWinActive();
}

void UtilWidget::greedyPlan() 
{
	//	  runButton->setDisabled(true);
	//	  resetButton->setDisabled(true);
	std::string str = "p3d_RunGreedy";
	
	m_mainWindow->isPlanning();
#ifdef WITH_XFORMS	
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#endif
}

//---------------------------------------------------------------------
// @ingroup qtWindow
// @brief Planner thread class
//---------------------------------------------------------------------
TestPlannerthread::TestPlannerthread(QObject* parent) :
QThread(parent)
{
	
}

void TestPlannerthread::run()
{	
	
	cout << "Ends Planner Thread" << endl;
}

//---------------------------------------------------------------------
// Threshold
//---------------------------------------------------------------------
void UtilWidget::initRRTthreshold()
{
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsCostThreshold,	Env::costThresholdRRT);
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxCostThresh, m_ui->horizontalSliderCostThresh , Env::costThreshold );
	
	connect(m_ui->pushButtonThresholdPlanner,SIGNAL(clicked()),this,SLOT(runThresholdPlanner()));
}

void UtilWidget::runThresholdPlanner()
{
	cout << "MainWindow::run" << endl;
	m_mainWindow->isPlanning();
#ifdef WITH_XFORMS
	std::string str = "RunDiffusion";
	write(qt_fl_pipe[1],str.c_str(),str.length()+1);
	ENV.setBool(Env::isRunning,true);
#else
	TestPlannerthread* ptrPlan = new TestPlannerthread;
	cout << "Start Test Planning Thread" << endl;
	ptrPlan->start();
#endif
}

//---------------------------------------------------------------------
// RRT*
//---------------------------------------------------------------------
void UtilWidget::initRRTstar()
{
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRRTstar,	Env::costStarRRT);

}


//---------------------------------------------------------------------
// RRT*
//---------------------------------------------------------------------
void UtilWidget::initDistanceAndLocalpath()
{
	connect(m_ui->pushButtonComputeRandomLP,SIGNAL(clicked()),this,SLOT(computeRandomLP()));

}

void UtilWidget::computeRandomLP()
{
// pushButtonComputeRandomLP
	Robot* rob = global_Project->getActiveScene()->getActiveRobot();
	
	shared_ptr<Configuration> q1 = rob->shoot();
	q1->setConstraintsWithSideEffect();
	shared_ptr<Configuration> q2 = rob->shoot();
	q2->setConstraintsWithSideEffect();
	
	LocalPath LP( q1, q2 );
	
	double param = p3d_random(0, LP.getParamMax() );
	
	shared_ptr<Configuration> q3 = LP.configAtParam( param );
	q3->setConstraintsWithSideEffect();
	
	cout << "----------------------------------" << endl;
	cout << "LP.getParamMax() = " << LP.getParamMax() << endl;
	cout << "q1->dist( *q2 )  = " << q1->dist( *q2 ) << endl;
	cout << "Random Param  = " << param << endl;
	cout << "q1->dist( *q3 )  = " << q1->dist( *q3 ) << endl;
	
	rob->setAndUpdate( *q3 );
}

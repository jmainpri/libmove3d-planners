/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "qtCost.hpp"
#include "ui_qtCost.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <tr1/memory>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "cppToQt.hpp"

#include "cost_space.hpp"
#if defined(HRI_COSTSPACE)
#include "HRI_costspace/HRICS_costspace.hpp"
#endif

#ifdef QWT
#include "qtWindow/qtPlot/basicPlot.hpp"
#include "qtWindow/qtPlot/doublePlot.hpp"
#include "qtWindow/qtPlot/tempWin.hpp"
#endif

#include "mainwindow.hpp"
#include "qtMotionPlanner.hpp"

#include "Util-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

CostWidget::CostWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::CostWidget)
{
	m_ui->setupUi(this);
	
	//initCost();
	//initHRI();
	//initHumanLike();
}

CostWidget::~CostWidget()
{
	delete m_ui;
	//#ifdef QWT
	//    delete this->plot;
	//#endif
}

#ifdef HRI_COSTSPACE
HricsWidget* CostWidget::getHriWidget()
{ 
	cout << "Warning : not compiling well HRICS interface" << endl;
	return m_ui->tabHri; 
}
#endif

//---------------------------------------------------------------------
// COST
//---------------------------------------------------------------------
void CostWidget::initCost()
{
	m_mainWindow->connectCheckBoxToEnv(m_ui->isCostSpaceCopy,			Env::isCostSpace);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostBefore,		Env::costBeforeColl);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostExpandToGoal,	Env::costExpandToGoal);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostWithGradient,	Env::tRrtComputeGradient);
	
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxInitTemp, m_ui->horizontalSliderInitTemp , Env::initialTemperature );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxNFailMax, m_ui->horizontalSliderNFailMax , Env::temperatureRate );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxLengthWeight, m_ui->horizontalSliderLengthWeight , Env::KlengthWeight );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxMinConnectGap, m_ui->horizontalSliderMinConnectGap , Env::minimalFinalExpansionGap );
	
#ifdef QWT
	connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this,SLOT(showTrajCost()));
	connect(m_ui->pushButtonShowCostProfile,SIGNAL(clicked()),this,SLOT(showCostProfile()));
	connect(m_ui->pushButtonShowHRITrajCost,SIGNAL(clicked()),this,SLOT(showHRITrajCost()));
	connect(m_ui->pushButtonShowTemp,SIGNAL(clicked()),this,SLOT(showTemperature()));
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRescale, Env::initPlot);
	
	this->plot = new BasicPlotWindow();
#endif
	qRegisterMetaType< std::vector<double> > ("std::vector<double>");
	connect(ENV.getObject(Env::costAlongTraj), SIGNAL(valueChanged(std::vector<double>)), this, SLOT(setPlotedVector(std::vector<double>)));
	//    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this->plot,SLOT(show()));
	connect(m_ui->pushButtonGridInGraph,SIGNAL(clicked()),this,SLOT(putGridInGraph()));
	//    connect(m_ui->pushButtonAStar,SIGNAL(clicked()),this,SLOT(computeAStar()));
	
	// costCriterium 2
	connect(m_ui->comboBoxTrajCostExtimation_2, SIGNAL(currentIndexChanged(int)), m_motionWidget, SLOT(setCostCriterium(int)));
//	connect(ENV.getObject(Env::costDeltaMethod), SIGNAL(valueChanged(int)),m_ui->comboBoxTrajCostExtimation_2, SLOT(setCurrentIndex(int)));
	m_ui->comboBoxTrajCostExtimation_2->setCurrentIndex( MECHANICAL_WORK /*INTEGRAL*/ );
//	
	//connect(m_ui->pushButton2DAstar,SIGNAL(clicked()),this,SLOT(computeGridAndExtract()));
	connect(m_ui->pushButton2DDijkstra,SIGNAL(clicked()),this,SLOT(graphSearchTest()));
	
	connect(m_ui->pushButtonRecomputeGraph,SIGNAL(clicked()),this,SLOT(newGraphAndReComputeCost()));
	connect(m_ui->pushButtonExtractBestPath,SIGNAL(clicked()),this,SLOT(extractBestPath()));
	connect(m_ui->pushButtonStonesGraph,SIGNAL(clicked()),this,SLOT(stonesGraph()));
	
	if (ENV.getBool(Env::isCostSpace)) 
	{
		std::cout << "Initializing the basic cost function" << std::endl;
		global_costSpace->addCost("NoCost",boost::bind(computeBasicCost, _1));
		
		std::cout << "Initializing the distance to obst. cost function" << std::endl;
		global_costSpace->addCost("costDistToObst",boost::bind(computeDistanceToObstacles, _1));
	}
}


extern void* GroundCostObj;

void CostWidget::initCostFunctions()
{
	vector<string> AllCost = global_costSpace->getAllCost();
	m_ui->comboBoxCostFunctions->clear();
	
	for (unsigned int i=0; i<AllCost.size(); i++) 
	{
		QString name(AllCost[i].c_str());
		m_ui->comboBoxCostFunctions->addItem(name);
	}
	
	connect(m_ui->comboBoxCostFunctions, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostFunction(int)));
	m_ui->comboBoxCostFunctions->setCurrentIndex( 0 );
	
	if( GroundCostObj != NULL )
	{
		vector<string> costFunc  = global_costSpace->getAllCost();
		vector<string>::iterator it = std::find( costFunc.begin() , costFunc.end() , "costMap2D" );
		m_ui->comboBoxCostFunctions->setCurrentIndex( it - costFunc.begin() );
	}
	else 
	{
		global_costSpace->setCost("NoCost");
	}
}

void CostWidget::setCostFunction(std::string function)
{
	vector<string> costFunc  = global_costSpace->getAllCost();
	vector<string>::iterator it = std::find( costFunc.begin() , costFunc.end() , function);
	if (it == costFunc.end()) 
	{
		cout << "Wrong cost function name" << endl;
		return;
	}
	m_ui->comboBoxCostFunctions->setCurrentIndex( it - costFunc.begin() );
}

void CostWidget::setCostFunction(int costFunctionId)
{
	vector<string> AllCost = global_costSpace->getAllCost();
	global_costSpace->setCost(AllCost[costFunctionId]);
	cout << "Cost Function is now :  " << AllCost[costFunctionId] << endl;	
}

void CostWidget::extractBestPath()
{
	//	p3d_ExtractBestTraj(XYZ_GRAPH);
	Graph* myGraph = new Graph(XYZ_GRAPH);
	Robot* myRobot = myGraph->getRobot();
	
	myGraph->extractBestTraj(myRobot->getInitialPosition(),myRobot->getGoTo());
	
	m_mainWindow->drawAllWinActive();
}

void CostWidget::newGraphAndReComputeCost()
{
	if (!XYZ_GRAPH) 
	{
		cout << "No XYZ_GRAPH!!!!!!" << endl;
	}
	else 
	{
		Graph* myGraph = new Graph(new Robot(XYZ_GRAPH->rob),XYZ_GRAPH);
		myGraph->recomputeCost();
		cout << "All graph cost recomputed XYZ_GRAPH is uptodate" << endl;
	}
	
}

void CostWidget::stonesGraph()
{
	if (!XYZ_GRAPH) 
	{
		p3d_del_graph(XYZ_GRAPH);
	}
	double du(0.0), ds(0.0);
	char* file = "/Users/jmainpri/workspace/BioMove3DDemos/PathDeform/simple/StonesACR.graph";
	p3d_readGraph(file, DEFAULTGRAPH);
	ENV.setBool(Env::isCostSpace,true);
	ChronoOn();
	this->newGraphAndReComputeCost();
	ChronoTimes(&du,&ds);
	cout << "Time to compute cost on graph : " << ds << endl;
	ENV.setBool(Env::drawGraph,true);
	m_mainWindow->drawAllWinActive();
	ChronoOff();
}


void CostWidget::graphSearchTest()
{
	cout << "Extracting Grid" << endl;
	
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	
#ifdef P3D_PLANNER
	if(XYZ_GRAPH)
	{
		p3d_del_graph(XYZ_GRAPH);
	}
	
	cout << "Creating Dense Roadmap" << endl;
	p3d_CreateDenseRoadmap(robotPt);
#endif
	
#ifdef CXX_PLANNER
	Graph* ptrGraph = new Graph(XYZ_GRAPH);
	
	shared_ptr<Configuration> Init = ptrGraph->getRobot()->getInitialPosition();
	shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();
	
	cout << "Dijkstra graph search on graph" << endl;
	Dijkstra graphSearch(ptrGraph);
	
	cout << "graphSearch.extractTrajectory" << endl;
	API::Trajectory* traj = graphSearch.extractTrajectory(Init,Goal);
	
	cout << "-------------------------------" << endl;
	cout << "Trajectory Cost = "<< traj->cost() << endl;
	cout << "   nl = "<< traj->getNbPaths() << endl;
	cout << "   length = "<< traj->getRangeMax() << endl;
	
	ENV.setBool(Env::drawGraph,false);
	ENV.setBool(Env::drawTraj,true);
	
	traj->replaceP3dTraj();
	
	m_mainWindow->drawAllWinActive();
	
	delete traj;
#endif
}


void CostWidget::showTrajCost()
{
#if defined(QWT) && defined(CXX_PLANNER)
	cout << "showTrajCost" << endl;
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	BasicPlot* myPlot = new BasicPlot(this->plot);
	myPlot->setGeometry(this->plot->getPlot()->geometry());
	int nbSample = myPlot->getPlotSize();
	
	API::Trajectory traj(new Robot(robotPt),CurrentTrajPt);
	
	double step = traj.getRangeMax() / (double) nbSample;
	
	vector<double> cost;
	
	//    cout << "Traj param max = " << traj.getRangeMax() << endl;
	//    cout << "Traj step = " << step << endl;
	
	cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	
	
	for( double param=0; param<traj.getRangeMax(); param = param + step)
	{
		cost.push_back( traj.configAtParam(param)->cost() );
		//cout << cost.back() << endl;
	}
	
	myPlot->setData(cost);
	delete this->plot->getPlot();
	this->plot->setPlot(myPlot);
	this->plot->show();
#endif
}

void CostWidget::showCostProfile()
{
#if defined(QWT) && defined(CXX_PLANNER)
	cout << "showCostProfile" << endl;
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	BasicPlot* myPlot = new BasicPlot(this->plot);
	myPlot->setGeometry(this->plot->getPlot()->geometry());
	int nbSample = myPlot->getPlotSize();
	
	API::Trajectory traj(new Robot(robotPt),CurrentTrajPt);
	
//	cout << "nbSample : " << nbSample << endl;
	const double step = traj.getRangeMax() / (double) nbSample;
	
	std::vector< std::pair<double,double> > profile = traj.getCostProfile();
	
	unsigned int i = 0;
	unsigned int j = 0;
	
	std::vector<double> cost;
	
	for( double param=0.0; param< traj.getRangeMax(); param += step)
	{
		while( profile[i].first < param )
		{
			if( ( profile.size() - 1 ) > i  )
			{
				i++;
			}
			else 
			{
				break;
			}
			//cout << "profile[i].first = " << profile[i].first << endl;
		}
		
		//cout << "param = " << param << endl;
//		cout << "cost.first = " << profile[i].first << endl;
//		cout << "cost.second = " << profile[i].second << endl;
		cost.push_back( profile[i].second );
		
		cout << "cost[" << j++ << "] = " << cost.back() << endl;
	}
	
	myPlot->setData(cost);
	delete this->plot->getPlot();
	this->plot->setPlot(myPlot);
	this->plot->show();
#endif
}

void CostWidget::showHRITrajCost()
{
	if (!ENV.getBool(Env::enableHri) ) 
	{
		cerr << "Hri is not enabled" << endl;
		return;
	}	

  double kDistanceTmp = ENV.getDouble(Env::Kdistance);
	double kVisibiliTmp = ENV.getDouble(Env::Kvisibility);
	double kReachablTmp = ENV.getDouble(Env::Kreachable);

#if defined(QWT) && defined(CXX_PLANNER)
	cout << "--------------------------------" << endl;
	
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	if (!CurrentTrajPt) 
	{
		cerr << "No trajectory" << endl;
		return;
	}
#if defined(HRI_COSTSPACE)
	ENV.setInt(Env::hriCostType,HRICS_Combine);
#endif
	
	DoublePlot* myPlot = new DoublePlot(this->plot);
	myPlot->setGeometry(this->plot->getPlot()->geometry());
	int nbSample = myPlot->getPlotSize();
	
	Robot* thisRob = new Robot(robotPt);
	API::Trajectory traj(thisRob,CurrentTrajPt);
	
	cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	

	
	ENV.setDouble(Env::Kreachable,0.0);
	ENV.setDouble(Env::Kvisibility,0.0);
	ENV.setDouble(Env::Kreachable,0.0);
	
	
	// Compute the 3 costs separatly 
	ENV.setDouble(Env::Kdistance,kDistanceTmp);
	cout << "Distance Cost = " << traj.costDeltaAlongTraj() << endl;
	ENV.setDouble(Env::Kdistance,0.0);
	
	ENV.setDouble(Env::Kvisibility,kVisibiliTmp);
	cout << "Visibility Cost = " << traj.costDeltaAlongTraj() << endl;
	ENV.setDouble(Env::Kvisibility,0);
	
	ENV.setDouble(Env::Kreachable,kReachablTmp);
	cout << "Reachalbilty Cost = " << traj.costDeltaAlongTraj() << endl;
	
	ENV.setDouble(Env::Kdistance,		kDistanceTmp);
	ENV.setDouble(Env::Kvisibility,	kVisibiliTmp);
	ENV.setDouble(Env::Kreachable,	kReachablTmp);
	
	double step = traj.getRangeMax() / (double) nbSample;
	
	//    cout << "Traj param max = " << traj.getRangeMax() << endl;
	//    cout << "Traj step = " << step << endl;
	
	std::ostringstream oss;
	oss << "statFiles/CostAlongTraj.csv";
	
	const char *res = oss.str().c_str();
	
	std::ofstream s;
	s.open(res);
	
	cout << "Opening save file : " << res << endl;
	
	s << "Dista"  << ";";
	s << "Visib"  << ";";
	s << "Reach"  << ";";
	
	s << endl;
	
	vector<double> costDistance;
	vector<double> costVisibili;
	vector<double> costReachabl;
	for( double param=0; param<traj.getRangeMax(); param = param + step)
	{
		shared_ptr<Configuration> q = traj.configAtParam(param);
		
#ifdef HRI_COSTSPACE
		if(ENV.getBool(Env::HRIPlannerCS))
		{
			q->cost();
			
			double dCost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPL)->getLastDistanceCost();
			double vCost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPL)->getLastVisibiliCost();
			costDistance.push_back(dCost);
			costVisibili.push_back(vCost);
		}
		if(ENV.getBool(Env::HRIPlannerWS))
		{
			ENV.setDouble(Env::Kreachable,	0.0);
			ENV.setDouble(Env::Kvisibility,	0.0);
			ENV.setDouble(Env::Kreachable,	0.0);
			
			
			// Compute the 3 costs separatly 
			ENV.setDouble(Env::Kdistance,		kDistanceTmp);
			q->setCostAsNotTested();
			double dCost = q->cost();
			
			ENV.setDouble(Env::Kdistance,		0.0);
			ENV.setDouble(Env::Kvisibility,	kVisibiliTmp);
			q->setCostAsNotTested();
			double vCost = q->cost();
			
			ENV.setDouble(Env::Kvisibility,	0.0);
			ENV.setDouble(Env::Kreachable,	kReachablTmp);
			q->setCostAsNotTested();
			double rCost = q->cost();
			
			// Put 3 costs in vector
			costDistance.push_back( dCost );
			costVisibili.push_back( vCost );
			costReachabl.push_back( rCost );
			
			/*cout	<<	"dCost: "			<< dCost << 
								" , vCost: "	<< vCost << 
								" , rCost: " << rCost << endl;*/
			
			// Save to file
			s << dCost << ";";
			s << vCost << ";";
			s << rCost << ";";
			s << endl;
		}
		//        cout << cost.back() << endl;
#endif
	}
	
	vector< vector<double> > curves;
	curves.push_back( costDistance );
	curves.push_back( costVisibili );
	curves.push_back( costReachabl );
	
	vector< string > plotNames;
	plotNames.push_back( "Distance" );
	plotNames.push_back( "Visibility" );
	plotNames.push_back( "Reachability" );
	
	myPlot->setData( plotNames , curves );
	
	delete this->plot->getPlot();
	this->plot->setPlot(myPlot);
	this->plot->show();
	
	s.close();
#endif
	
	cout << "Closing save file" << endl;
	
	ENV.setDouble(Env::Kdistance,		kDistanceTmp);
	ENV.setDouble(Env::Kvisibility,	kVisibiliTmp);
	ENV.setDouble(Env::Kreachable,	kReachablTmp);

	
}

void CostWidget::setPlotedVector(vector<double> v)
{
	cout << "PLOTTING ------------------------------------------" << endl;
#ifdef QWT
	BasicPlot* myPlot = dynamic_cast<BasicPlot*>(this->plot->getPlot());
	vector<double> cost = ENV.getVector(Env::costAlongTraj);
	cost.resize(myPlot->getPlotSize());
	myPlot->setData(cost);
	this->plot->show();
#endif
}

void CostWidget::showTemperature()
{
#ifdef QWT
	TempWin* window = new TempWin();
	window->show();
#endif
}

void CostWidget::computeAStar()
{
	if(!ENV.getBool(Env::isCostSpace))
	{
		return;
	}
#ifdef P3D_PLANNER
	if(!(XYZ_GRAPH->start_nodePt))
	{
		
		XYZ_GRAPH->search_start = XYZ_GRAPH->nodes->N;
		XYZ_GRAPH->search_goal = XYZ_GRAPH->last_node->N;
		cout << "p3d_initSearch" << endl;
		p3d_initSearch(XYZ_GRAPH);
		
		cout << "Number Of Graph nodes = " << XYZ_GRAPH->nnode << endl;
#ifdef CXX_PLANNNER
		GraphState* InitialState = new GraphState(XYZ_GRAPH->nodes->N);
		
		//        N = new Node(ptrGraph,rob->getGoTo());
		//        ptrGraph->insertNode(N);
		//        ptrGraph->linkNode(N);
		
		API::AStar search;
		vector<API::State*> path = search.solve(InitialState);
		
		if(path.size() == 0 )
		{
			return;
		}
		
		API::Trajectory* traj = new API::Trajectory(new Robot(XYZ_ROBOT));
		
		for (unsigned int i=0;i<path.size();i++)
		{
			configPt conf = dynamic_cast<GraphState*>(path[i])->getGraphNode()->q;
			shared_ptr<Configuration> q(new Configuration(new Robot(XYZ_ROBOT),conf));
			traj->push_back(q);
		}
		
		traj->replaceP3dTraj();
#endif
		m_mainWindow->drawAllWinActive();
		
		cout << "solution : End Search" << endl;
	}
	else
	{
		cout << "No start node" << endl;
	}
#endif
}

void CostWidget::putGridInGraph()
{
	cout << "Computing Grid" << endl;
	
#ifdef CXX_PLANNNER
	Vector3i     gridSize;
	
	gridSize[0] = 10;
	gridSize[1] = 10;
	gridSize[2] = 10;
	
	//    vector<double>  envSize(6);
	//    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
	//    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
	//    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;
	
	//    Grid myGrid(gridSize,envSize);
	//    myGrid.createAllCells();
	//
	//    for(int i=0;i<myGrid.getNumberOfCells();i++)
	//    {
	//        vector<double> center = myGrid.getCell(i)->getCenter();
	//        cout << i << " =  ("<< center[0] << "," << center[1] << "," << center[2] << ")" << endl;
	//    }
	//---------------
	
	GridToGraph theGrid(gridSize);
	theGrid.putGridInGraph();
#endif
	
	m_mainWindow->drawAllWinActive();
}



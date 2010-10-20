/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_COST_H
#define QT_COST_H

#if defined( CXX_PLANNER )

#include "qtLibrary.hpp"
#include "mainwindow.hpp"
#include "qtMotionPlanner.hpp"

#include <string>

#ifdef QWT
#include "qtPlot/basicPlotWindow.hpp"
#endif

#endif

#if defined( OOMOVE3D_CORE )

#include "qtUI/qtLibrary.hpp"
#include "qtUI/qtMainInterface/mainwindow.hpp"
#include "qtUI/qtMainInterface/sideWidgets/qtMotionPlanner.hpp"

#include <string>

#ifdef QWT
#include "qtUI/qtPlot/basicPlotWindow.hpp"
#endif

#endif

#ifdef HRI_COSTSPACE
#include "qtHrics.hpp"
#endif

namespace Ui
{
	class CostWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class CostWidget : public QWidget
{
	Q_OBJECT
	
public:
	CostWidget(QWidget *parent = 0);
	~CostWidget();
	
	void initCost();
	void initCostFunctions();
	void initThreshold();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
#ifdef HRI_COSTSPACE
	HricsWidget* getHriWidget();
#endif
	
public slots:
	void setCostFunction(std::string function);
	void setCostFunction(int costFunctionId);
	
private slots:
	
	// General Cost --------------------------------
	void stonesGraph();
	void extractBestPath();
	void newGraphAndReComputeCost();
	void showTrajCost();
	void showCostProfile();
	void showHRITrajCost();
	void showTemperature();
	void setPlotedVector(std::vector<double> v);
	void putGridInGraph();
	void computeAStar();
	//void computeGridAndExtract();
	void graphSearchTest();
	
private:
	Ui::CostWidget*		m_ui;
	
	MotionPlanner*		m_motionWidget;
	MainWindow*			m_mainWindow;
	
#ifdef QWT
	BasicPlotWindow *plot;
#endif
	
};

#endif
/*
 *  qtMotionPlanner.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_MOTIONPLANNER_H
#define QT_MOTIONPLANNER_H

#include "env.hpp"

#if defined( CXX_PLANNER )

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"

#endif

#if defined( MOVE3D_CORE )

#include "qtUI/qtLibrary.hpp"
#include "qtUI/qtMainInterface/mainwindow.hpp"

#endif

namespace Ui
{
	class MotionPlanner;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class MotionPlanner : public QWidget
{
	Q_OBJECT
	
public:
	MotionPlanner(QWidget *parent = 0);
	~MotionPlanner();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
private slots:
	// Optim -----------------------------
	void test(double value);
	void computeGrid();
	void runMultiSmooth();
	void optimizeCost();
	void shortCutCost();
	void removeRedundant();
	void extractBestTraj();
	void setCostCriterium(int choise);
	void eraseDebugTraj();
	void cutTrajInSmallLP();
	
	// Multi-Run -------------------------
	void saveContext();
	void printContext();
	void deleteSelected();
	void printAllContext();
	void resetContext();
	void setToSelected();
	void runAllRRT();
	void runAllGreedy();
	void showHistoWindow();
	
	// General ---------------------------
	void checkAllEdges();
	void envDmaxSpinBoxValueChanged( double dmax );
	void testParam(double param);
	
	// Show ------------------------------
	void nodeToShowChanged();
	void removeNode();
	
private:
	Ui::MotionPlanner *m_ui;
	
	MainWindow *m_mainWindow;
	
	QListWidget* contextList;
	std::vector<QListWidgetItem*> itemList;
	
#ifdef QWT
	HistoWindow* histoWin;
#endif
	
	void initDiffusion();
	void initMultiRRT();
	void initPRM();
	void initMultiRun();
	void initOptim();
	void initGeneral();
	void initShowGraph();
};

/**
 * @ingroup qtWindow
 * @brief Smoothing thread class 
 */
class SmoothThread: public QThread
{
	Q_OBJECT
	
public:
	SmoothThread(bool isShortCut,QObject* parent = 0);
	
protected:
	void run();
	bool m_isShortCut;
};

/**
 * @ingroup qtWindow
 * @brief Multi Planner thread class 
 */
class MultiThread: public QThread
{
	Q_OBJECT
	
public:
	MultiThread(bool isRRT,QObject* parent = 0);
	
protected:
	void run();
	bool m_isRRT;
};

#endif

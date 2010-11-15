/*
 *  mainwindowTestFunctions.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/08/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#if defined( CXX_PLANNER )
#include "mainwindow.hpp"
#endif

#if defined( MOVE3D_CORE )
#include "qtUI/qtMainInterface/mainwindow.hpp"
#endif

#ifndef MAIN_WINDOW_TEST_FUNCTIONS
#define MAIN_WINDOW_TEST_FUNCTIONS

#include <QtCore/QObject>

class MainWindowTestFunctions : public QObject
{
	
Q_OBJECT
	
public:
	MainWindowTestFunctions(MainWindow* MainWinPt);

public slots:
	void test1();
	void test2();
	void test3();
	
private:
	MainWindow* m_mainWindow;
	
};

/**
 * @ingroup qtWindow
 * @brief Testing thread class 
 */
class Testthread: public QThread
{
	Q_OBJECT
	
public:
	Testthread(QObject* parent = 0);
	
protected:
	void run();
	
};

#endif
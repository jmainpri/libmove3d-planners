/*
 *  qtRobot.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_ROBOT_H
#define QT_ROBOT_H

#if defined( CXX_PLANNER )

#include "qtLibrary.hpp"
#include "qtFormRobot/moverobot.hpp"
#include "mainwindow.hpp"

#endif

#if defined( OOMOVE3D_CORE )

#include "qtUI/qtLibrary.hpp"
#include "qtUI/qtFormRobot/moverobot.hpp"
#include "qtUI/qtMainInterface/mainwindow.hpp"

#endif

namespace Ui
{
	class RobotWidget;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class RobotWidget : public QWidget
{
	Q_OBJECT
	
public:
	RobotWidget(QWidget *parent = 0);
	~RobotWidget();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
	void initRobot();
	
	MoveRobot* getMoveRobot();
	
private slots:
	// Test Model -------------------------
	void costTest();
	void collisionsTest();
	void localpathsTest();
	void allTests();
	void setAttMatrix();
	
	// Hri Planner ------------------------
	// HRI GIK functions
	void computeHriGik(bool leftArm);
	
#if defined (HRI_GENERALIZED_IK)
	void computeHriGikLARM() { this->computeHriGik(true); }
	void computeHriGikRARM() { this->computeHriGik(false); }
#endif
	
	// Grab Object ------------------------
	
	void GrabObject();
	void ReleaseObject();
	void currentObjectChange(int i);
	void SetObjectToCarry();
	
	// MISC -------------------------------
	void printCurrentPos();
	void printAbsPos();
	
#ifdef LIGHT_PLANNER
	void switchFKIK();
#endif
	
	void printPQPColPair();
	
	// Manipulation -----------------------
	void armPickGoto();
	void armPickTakeToFree();
	void armPickGotoAndTakeToFree();
	
	/*void initVoxelCollisionChecker();
	 void createVoxelCC();
	 void deleteVoxelCC();
	 
	 void voxelCCTest();*/
	
private:
	Ui::RobotWidget *m_ui;
	
	MainWindow *m_mainWindow;
	
	std::vector<QString> mFreeFlyers;
	
	void initModel();
	void initManipulation();
};

/**
 * @ingroup qtWindow
 * @brief Planner thread class 
 */
class Manipulationthread: public QThread
{
	Q_OBJECT
	
public:
	Manipulationthread(QObject* parent = 0);
	
protected:
	void run();
	
};

#endif

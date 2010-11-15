/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_HRICS_H
#define QT_HRICS_H

#if defined( CXX_PLANNER )

#include "qtLibrary.hpp"
#include "mainwindow.hpp"
#include "qtMotionPlanner.hpp"
#include "API/Grids/gridsAPI.hpp"

#endif

#if defined( MOVE3D_CORE )

#include "qtUI/qtLibrary.hpp"
#include "qtUI/qtMainInterface/mainwindow.hpp"
#include "qtUI/qtMainInterface/sideWidgets/qtMotionPlanner.hpp"
#include "API/Grids/gridsAPI.hpp"

#endif

namespace Ui
{
	class HricsWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class HricsWidget : public QWidget
{
	Q_OBJECT
	
public:
	HricsWidget(QWidget *parent = 0);
	~HricsWidget();
	
#ifdef HRI_COSTSPACE
	void initHRI();
#endif
	//void initCost();
	void initVectorField();
	void initDrawOneLineInGrid();
	void initNaturalSpace();
	void initConfigSpace();
	void initTaskSpace();
	void initObjectTransferPoint();
	void initHumanLike();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
	Ui::HricsWidget* Ui() { return m_ui; }
	
	private slots:
#ifdef HRI_COSTSPACE
	
	// Vector Field
	void computeVectorField();
	
	// OTP
	void computeObjectTransferPoint();
	
	void set_X_line(bool enable);
	void set_Y_line(bool enable);
	void set_Z_line(bool enable);
	
	void saveActiveGridToFile();
	void loadActiveGridFromFile();
	// HRI ----------------------------------------
	// Natural
	void newNaturalCostSpace();
	void deleteNaturalCostSpace();
	void recomputeReachability();
	void computeAllCellNaturalCost();
	void computeReachability();
	void getSortedReachableWSPoint();
	
	// CSpace
	void newHRIConfigSpace();
	void deleteHRIConfigSpace();
	void makeGridHRIConfigSpace();
	void makePlanHRIConfigSpace();
	void AStarInPlanHRIConfigSpace();
	void writeToOBPlane();
	void hriPlanRRT();
	
	// Workspace
	void make3DHriGrid();
	void delete3DHriGrid();
	void computeGridCost();
	void resetGridCost();
	void AStarIn3DGrid();
	void HRICSRRT();
	void zoneSizeChanged();
	void resetRandomPoints();
	
	// Taskspace
	void computeWorkspacePath();
	void computeHoleMotion();
	void KDistance(double value);
	void KVisibility(double value);
	void make2DGrid();
	
	void enableHriSpace();
	void setWhichTestSlot(int test);
	void setActiveGrid(int grid);
	void mergeGrids();
	void cellToShowChanged();
#endif
	
	void drawAllWinActive();
	
private:
	QtShiva::SpinBoxSliderConnector*	m_k_distance;
	QtShiva::SpinBoxSliderConnector*	m_k_visbility;
	QtShiva::SpinBoxSliderConnector*	m_k_reachability;
	QtShiva::SpinBoxSliderConnector*	m_k_naturality;
	
	Ui::HricsWidget*					m_ui;
	
	MotionPlanner*						m_motionWidget;
	MainWindow*							m_mainWindow;
	
	std::vector<API::BaseGrid*>			m_grids;
};

#endif
/*
 *  PlanningThread.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 28/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef PLANNING_THREAD_H
#define PLANNING_THREAD_H

#include "qtLibrary.h"
#include "planner_cxx/plannerFunctions.hpp"
#include "Planner-pkg.h"
#include "move3d-headless.h"

/**
 * @ingroup qtWindow
 * @brief Planning thread class 
 */
class PlanningThread: public QThread
{
	Q_OBJECT
	
public:
	PlanningThread(QObject* parent = 0);
	
	int getNumberOfNodes()
	{
		return m_nbNodes;
	}
	
protected:
	void run();
	
private:
	int m_nbNodes;
	
};

#endif

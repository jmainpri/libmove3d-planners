/*
 *  PlanningThread.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 28/06/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "PlanningThread.hpp"

PlanningThread::PlanningThread(QObject* parent) : QThread(parent)
{
	
}


void PlanningThread::run()
{	
	m_nbNodes = p3d_run_rrt(XYZ_GRAPH, fct_stop, fct_draw);
	std::cout << "Ends Planner Thread" << std::endl;
}

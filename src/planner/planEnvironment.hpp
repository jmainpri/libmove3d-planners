/*
 *  PlanEnvironment.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 11/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

//**********************************************************
// Module planner
//**********************************************************

#ifndef PLAN_ENVIRONMENT_HPP
#define PLAN_ENVIRONMENT_HPP

#include "../p3d/ParametersEnv.hpp"

bool Env_stopUser();

namespace  PlanParam
{
	enum boolParameter 
	{
		stopPlanner,
		isMaxDisNeigh,
		isWeightedChoice,
		
		// Smoothing stage
		partialShortcut,
		saveTrajCost,
		trajCostRecompute,
		withMaxIteration,
		withGainLimit,
		withTimeLimit,
		withSmoothing,
		withShortCut,
		withDeformation,
		withDescent,
    drawColorConfig
	};
	
	enum intParameter 
	{
		tata
	};
	
	enum doubleParameter 
	{
		// Optimization Variables
		optimTimeLimit,
		MaxFactor,
		MinStep,
		costTraj
	};
	
	enum stringParameter 
	{
		titi
	};
	
	enum vectorParameter 
	{
		tutu
	};
	
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<
PlanParam::boolParameter,
PlanParam::intParameter,
PlanParam::doubleParameter,
PlanParam::stringParameter,
PlanParam::vectorParameter>* PlanEnv;

// Functions that initializes the planner
// Parameters
void initPlannerParameters();

#endif

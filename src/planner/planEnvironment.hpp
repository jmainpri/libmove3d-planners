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
        
        // Drawing (gl) stuff
        drawColorConfig,
        drawOccupVoxels,
        drawSampledPoints,
        drawStaticVoxels,
        drawRandomMap,
        
        //Object Transfert point variable
        env_humanGridDraw,
        env_robotGridDraw,
        env_isStanding,
        env_drawRandomPoint,
        env_drawDistGrid,
        env_drawOnlyBest,
        env_normalRand,
        env_fusedGridRand
	};
	
	enum intParameter 
	{
		tata,

		// Object TransfertPoint variable
		env_maxIter,
		env_nbRandomRotOnly,
		env_totMaxIter,

		// grid cells
		env_xToDraw,
		env_yToDraw,
		env_timeShow,
		env_pow

	};
	
	enum doubleParameter 
	{
		// Optimization Variables
		optimTimeLimit,
		MaxFactor,
		MinStep,
		costTraj,
        distMinToDraw,
        
        // Object TransfertPoint variable
        env_randomXMinLimit,
        env_randomXMaxLimit,
        env_randomYMinLimit,
        env_randomYMaxLimit,
        env_robotSpeed,
        env_humanSpeed,
        env_timeStamp,
        env_psi,
        env_delta,
        env_ksi,
        env_rho,
        env_objectNessecity,
        env_sittingOffset
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

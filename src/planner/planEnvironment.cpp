/*
 *  PlanEnvironment.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 11/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "planEnvironment.hpp"
#include "Move3d-pkg.h"
#include <iostream>

//**********************************************************
//**********************************************************

bool Env_stopUser()
{
	bool stop = !(*fct_stop)();
	
	if (stop) 
	{
		std::cout << "=> Env_stopUser() !!!" << std::endl;
	}
	
	return stop;
}


// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disabled this just acts as a normal container

// Definition of the parameter container
Parameters<
PlanParam::boolParameter,
PlanParam::intParameter,
PlanParam::doubleParameter,
PlanParam::stringParameter,
PlanParam::vectorParameter>* PlanEnv = NULL;

// @brief Function that inizializes the 
// Parameter container
void initPlannerParameters()
{
	// Create 5 maps for all types and fill the 5 maps
	// ------------------------------------------------------------------
	std::map<PlanParam::boolParameter,		boolContainer*>				myBoolMap;
	std::map<PlanParam::intParameter,			intContainer*>				myIntMap;
	std::map<PlanParam::doubleParameter,	doubleContainer*>			myDoubleMap;
	std::map<PlanParam::stringParameter,	stringContainer*>			myStringMap;
	std::map<PlanParam::vectorParameter,	vectorContainer*>			myVectorMap;
	
	// Bool
	// ------------------------------------------------------------------
	myBoolMap.insert( std::make_pair( PlanParam::stopPlanner,				new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::isMaxDisNeigh,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::isWeightedChoice,	new boolContainer(false)));
	
	// Smoothing stage
	myBoolMap.insert( std::make_pair( PlanParam::partialShortcut,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::saveTrajCost,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::trajCostRecompute, new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::withMaxIteration,	new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withGainLimit,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::withTimeLimit,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withSmoothing,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withShortCut,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withDeformation,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::withDescent,				new boolContainer(false)));
	
	// Int
	// ------------------------------------------------------------------
	myIntMap.insert(std::make_pair( PlanParam::tata,								new intContainer(5)));
	
	// Double
	// ------------------------------------------------------------------
	// Post-processing phaze
	myDoubleMap.insert( std::make_pair( PlanParam::optimTimeLimit,	new doubleContainer(3.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::MaxFactor,				new doubleContainer(4.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::MinStep,					new doubleContainer(1.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::costTraj,				new doubleContainer(1.0)));
	
	
	// String
	// ------------------------------------------------------------------
#ifdef QT_LIBRARY
	myStringMap.insert(std::make_pair(PlanParam::titi,							new stringContainer("titi")));
#endif
	
	// Vector
	// ------------------------------------------------------------------
	std::vector<double> tutu;
	tutu.push_back( 1 ); tutu.push_back( 8 );
	
	myVectorMap.insert(std::make_pair(PlanParam::tutu,							new vectorContainer(tutu)));
	
	// Make the new parameter container
	PlanEnv =  new Parameters<
	PlanParam::boolParameter,
	PlanParam::intParameter,
	PlanParam::doubleParameter,
	PlanParam::stringParameter,
	PlanParam::vectorParameter>(
															myBoolMap,
															myIntMap,
															myDoubleMap,
															myStringMap,
															myVectorMap);
}

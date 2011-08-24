/*
 *  PlanEnvironment.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 11/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "planEnvironment.hpp"
#include "move3d-headless.h"
#include <iostream>
//#include "../p3d/env.hpp"


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

using namespace std;


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
	std::map<PlanParam::intParameter,			intContainer*>			myIntMap;
	std::map<PlanParam::doubleParameter,	doubleContainer*>			myDoubleMap;
	std::map<PlanParam::stringParameter,	stringContainer*>			myStringMap;
	std::map<PlanParam::vectorParameter,	vectorContainer*>			myVectorMap;
	
	// Bool
	// ------------------------------------------------------------------
	myBoolMap.insert( std::make_pair( PlanParam::stopPlanner,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::isMaxDisNeigh,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::isWeightedChoice,      new boolContainer(false)));

    // Smoothing stage
	myBoolMap.insert( std::make_pair( PlanParam::partialShortcut,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::saveTrajCost,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::trajCostRecompute,	new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::withMaxIteration,	new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::withGainLimit,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withTimeLimit,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withSmoothing,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withShortCut,			new boolContainer(true)));
	myBoolMap.insert( std::make_pair( PlanParam::withDeformation,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::withDescent,       new boolContainer(false)));
  myBoolMap.insert( std::make_pair( PlanParam::doReplanning,       new boolContainer(false)));
  
  // Stomp/Chomp
  myBoolMap.insert( std::make_pair( PlanParam::withCurrentTraj,			new boolContainer(false)));
  
	// Drawing (gl) stuff
  myBoolMap.insert( std::make_pair( PlanParam::drawColorConfig,		new boolContainer(false)));
  myBoolMap.insert( std::make_pair( PlanParam::drawOccupVoxels,		new boolContainer(false)));
  myBoolMap.insert( std::make_pair( PlanParam::drawSampledPoints,     new boolContainer(false)));
  myBoolMap.insert( std::make_pair( PlanParam::drawStaticVoxels,      new boolContainer(false)));
  myBoolMap.insert( std::make_pair( PlanParam::drawRandomMap,			new boolContainer(false)));
  myBoolMap.insert( std::make_pair( PlanParam::drawBoundingVolumes,			new boolContainer(false)));
    
    // Object TransfertPoint variable
    myBoolMap.insert( std::make_pair( PlanParam::env_humanGridDraw,		new boolContainer(true )));
	myBoolMap.insert( std::make_pair( PlanParam::env_robotGridDraw,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_isStanding,		new boolContainer(false)));
    myBoolMap.insert( std::make_pair( PlanParam::env_drawRandomPoint,	new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_drawDistGrid,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_drawOnlyBest,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_normalRand,		new boolContainer(true )));
	myBoolMap.insert( std::make_pair( PlanParam::env_fusedGridRand,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_useSlice,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_useOrientedSlice,	new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_useAllGrid,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_drawSlice,			new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_showHumanTraj,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_useOldDude,		new boolContainer(false)));
	myBoolMap.insert( std::make_pair( PlanParam::env_isInit,			new boolContainer(false)));

	// Int
	// ------------------------------------------------------------------
	myIntMap.insert(std::make_pair( PlanParam::tata,						new intContainer(5)));
  myIntMap.insert(std::make_pair( PlanParam::nb_pointsOnTraj,	new intContainer(40)));
  
	// Object TransfertPoint variable
	myIntMap.insert(std::make_pair( PlanParam::env_maxIter,					new intContainer(100)));
	myIntMap.insert(std::make_pair( PlanParam::env_nbRandomRotOnly,			new intContainer(10)));
	myIntMap.insert(std::make_pair( PlanParam::env_nbSittingRotation,		new intContainer(500)));
	myIntMap.insert(std::make_pair( PlanParam::env_totMaxIter,				new intContainer(400)));

	myIntMap.insert(std::make_pair( PlanParam::env_xToDraw,				new intContainer(-1)));
	myIntMap.insert(std::make_pair( PlanParam::env_yToDraw,				new intContainer(-1)));
	myIntMap.insert(std::make_pair( PlanParam::env_timeShow,			new intContainer(2)));
	myIntMap.insert(std::make_pair( PlanParam::env_pow,					new intContainer(2)));
	myIntMap.insert(std::make_pair( PlanParam::env_MOTP,				new intContainer(20)));
	
	// Double
	// ------------------------------------------------------------------
	// Post-processing phaze
	myDoubleMap.insert( std::make_pair( PlanParam::optimTimeLimit,          new doubleContainer(2.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::MaxFactor,				new doubleContainer(3.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::MinStep,					new doubleContainer(2.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::costTraj,				new doubleContainer(1.0)));
  myDoubleMap.insert( std::make_pair( PlanParam::distMinToDraw,           new doubleContainer(0.3)));
	myDoubleMap.insert( std::make_pair( PlanParam::trajDuration,           new doubleContainer(5.0)));
  
	// Object TransfertPoint variable
	myDoubleMap.insert( std::make_pair( PlanParam::env_randomXMinLimit,		new doubleContainer(-3.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_randomXMaxLimit,		new doubleContainer(3.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_randomYMinLimit,		new doubleContainer(-3.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_randomYMaxLimit,		new doubleContainer(3.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_robotSpeed,			new doubleContainer(1.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_humanSpeed,			new doubleContainer(1.0)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_timeStamp,			new doubleContainer(0.35)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_psi,					new doubleContainer(0.99)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_delta,				new doubleContainer(0.01)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_ksi,					new doubleContainer(0.43)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_rho,					new doubleContainer(0.57)));
    myDoubleMap.insert( std::make_pair( PlanParam::env_objectNessecity,		new doubleContainer(0.5)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_sittingOffset,		new doubleContainer(0.2)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_limitRot,			new doubleContainer(M_PI/3)));
	myDoubleMap.insert( std::make_pair( PlanParam::env_Cellsize,			new doubleContainer(0.2)));

    //cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble( PlanParam::env_objectNessecity ) << endl;

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
  
  // Planning Environment
	// ------------------------------------------------------------------
  ENV.setBool(Env::biDir,true);
  ENV.setExpansionMethod(Env::Extend);
  ENV.setDouble(Env::extensionStep,6.0);
  
  ENV.setDouble(Env::minimalFinalExpansionGap,5.0);
  ENV.setDouble(Env::temperatureRate,30);
  
  PlanEnv->setBool(PlanParam::withSmoothing,true);
  PlanEnv->setBool(PlanParam::withDeformation,false);

  // Drawing environment
  // -------------------------------------------------------------------
  ENV.setBool(Env::drawMultiColorLocalpath,true);
  
}

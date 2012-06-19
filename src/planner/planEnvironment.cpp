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

#ifdef QT_LIBRARY
PlanParam* EnumPlannerParameterObject = NULL;

PlanParam::PlanParam()
{
  
}

PlanParam::~PlanParam()
{
  
}
#endif

// @brief Function that inizializes the 
// Parameter container
void initPlannerParameters()
{



#ifdef QT_LIBRARY
  EnumPlannerParameterObject = new PlanParam;
#endif

        // Create 5 maps for all types and fill the 5 maps
        // ------------------------------------------------------------------
        std::map<PlanParam::boolParameter,      boolContainer*>                  myBoolMap;
        std::map<PlanParam::intParameter,       intContainer*>                   myIntMap;
        std::map<PlanParam::doubleParameter,    doubleContainer*>                myDoubleMap;
        std::map<PlanParam::stringParameter,    stringContainer*>                myStringMap;
        std::map<PlanParam::vectorParameter,    vectorContainer*>                myVectorMap;

        // Bool
        // ------------------------------------------------------------------
        myBoolMap.insert( std::make_pair( PlanParam::stopPlanner,                new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::isMaxDisNeigh,              new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::isWeightedChoice,           new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::nextIterWaitForGui,         new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::rrtExtractShortestPath,     new boolContainer(false)));
  
        // Time
        myBoolMap.insert( std::make_pair( PlanParam::planWithTimeLimit,          new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::trajWithTimeLimit,          new boolContainer(true)));
  
        // Smoothing stage
        myBoolMap.insert( std::make_pair( PlanParam::trajPrintGain,              new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::trajPartialShortcut,        new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::trajSaveCost,               new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::trajCostRecompute,          new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::trajComputeCollision,       new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::trajStompComputeColl,       new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::trajBiasOptim,              new boolContainer(false)));
  
        myBoolMap.insert( std::make_pair( PlanParam::withMaxIteration,           new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::withGainLimit,              new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::withSmoothing,              new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::withShortCut,               new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::withDeformation,            new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::withDescent,                new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::doReplanning,               new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::showExploration,            new boolContainer(false)));
  
        // Stomp/Chomp
        myBoolMap.insert( std::make_pair( PlanParam::withCurrentTraj,            new boolContainer(false)));
  
        // HRICS
        myBoolMap.insert( std::make_pair( PlanParam::hriSetColorFromConfig,      new boolContainer(false)));
  
        // Drawing (gl) stuff
        myBoolMap.insert( std::make_pair( PlanParam::drawColorConfig,            new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::drawOccupVoxels,            new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::drawSampledPoints,          new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::drawStaticVoxels,           new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::drawRandomMap,              new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::drawBoundingVolumes,        new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::saveVideo,                  new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_oldCriteria,            new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_noRepetition,           new boolContainer(false)));

        // Object TransfertPoint variable
        myBoolMap.insert( std::make_pair( PlanParam::env_humanGridDraw,          new boolContainer(true )));
        myBoolMap.insert( std::make_pair( PlanParam::env_robotGridDraw,          new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_isStanding,             new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::env_drawRandomPoint,        new boolContainer(false)));

        myBoolMap.insert( std::make_pair( PlanParam::env_drawDistGrid,           new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_drawOnlyBest,           new boolContainer(false)));

        myBoolMap.insert( std::make_pair( PlanParam::env_normalRand,             new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_fusedGridRand,          new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_fusedGridAndRotRand,    new boolContainer(true )));

        myBoolMap.insert( std::make_pair( PlanParam::env_useSlice,               new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_useOrientedSlice,       new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_useAllGrid,             new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_drawSlice,              new boolContainer(false)));

        myBoolMap.insert( std::make_pair( PlanParam::env_showHumanTraj,          new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_useOldDude,             new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_isInit,                 new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_showText,               new boolContainer(true)));

        myBoolMap.insert( std::make_pair( PlanParam::trajOptimTestMultiGauss,    new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_realTime,               new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::useSelectedDuration,        new boolContainer(false))); 
        myBoolMap.insert( std::make_pair( PlanParam::env_drawHumanModel,         new boolContainer(false)));

        myBoolMap.insert( std::make_pair( PlanParam::env_softMotionTraj,         new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_createTrajs,            new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::env_computeTrajForHuman,    new boolContainer(true)));

        myBoolMap.insert( std::make_pair( PlanParam::env_trajNormal,             new boolContainer(false)));
        myBoolMap.insert( std::make_pair( PlanParam::env_trajSoftMotion,         new boolContainer(true)));
        myBoolMap.insert( std::make_pair( PlanParam::env_trajRos,                new boolContainer(false)));
  
        // Int
        // ------------------------------------------------------------------
        myIntMap.insert(std::make_pair( PlanParam::tata,                         new intContainer(5)));
        myIntMap.insert(std::make_pair( PlanParam::nb_pointsOnTraj,              new intContainer(15)));
  
        // RRRT
        myIntMap.insert(std::make_pair( PlanParam::plannerMaxIterations,         new intContainer(1000000)));
  
        // Replanning
        myIntMap.insert(std::make_pair( PlanParam::setOfActiveJoints,            new intContainer(0)));
        myIntMap.insert(std::make_pair( PlanParam::replanningAlgorithm,            new intContainer(0)));
  
        // Object TransfertPoint variable
        myIntMap.insert(std::make_pair( PlanParam::env_maxIter,                  new intContainer(40)));
        myIntMap.insert(std::make_pair( PlanParam::env_nbRandomRotOnly,          new intContainer(10)));
        myIntMap.insert(std::make_pair( PlanParam::env_nbSittingRotation,        new intContainer(1)));
        myIntMap.insert(std::make_pair( PlanParam::env_totMaxIter,               new intContainer(2000)));

        myIntMap.insert(std::make_pair( PlanParam::env_xToDraw,                  new intContainer(-1)));
        myIntMap.insert(std::make_pair( PlanParam::env_yToDraw,                  new intContainer(-1)));
        myIntMap.insert(std::make_pair( PlanParam::env_timeShow,                 new intContainer(2)));
        myIntMap.insert(std::make_pair( PlanParam::env_pow,                      new intContainer(2)));
        myIntMap.insert(std::make_pair( PlanParam::env_MOTP,                     new intContainer(20)));
        myIntMap.insert(std::make_pair( PlanParam::env_anglePow,                 new intContainer(3)));

        // Double
        // ------------------------------------------------------------------
        myDoubleMap.insert( std::make_pair( PlanParam::timeLimitPlanning,        new doubleContainer(10.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::timeLimitSmoothing,       new doubleContainer(10.0)));
        // Post-processing phaze
        myDoubleMap.insert( std::make_pair( PlanParam::MaxFactor,                new doubleContainer(3.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::MinStep,                  new doubleContainer(2.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::costTraj,                 new doubleContainer(1.0)));

        myDoubleMap.insert( std::make_pair( PlanParam::distMinToDraw,            new doubleContainer(0.3)));
        myDoubleMap.insert( std::make_pair( PlanParam::trajDuration,             new doubleContainer(5.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::trajOptimStdDev,          new doubleContainer(2.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::trajOptimSmoothWeight,    new doubleContainer(0.1)));
        myDoubleMap.insert( std::make_pair( PlanParam::trajOptimObstacWeight,    new doubleContainer(1.0)));
  
        // Object TransfertPoint variable
        myDoubleMap.insert( std::make_pair( PlanParam::env_randomXMinLimit,      new doubleContainer(-3.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_randomXMaxLimit,      new doubleContainer(3.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_randomYMinLimit,      new doubleContainer(-3.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_randomYMaxLimit,      new doubleContainer(3.0)));

        myDoubleMap.insert( std::make_pair( PlanParam::env_robotSpeed,           new doubleContainer(1.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_humanSpeed,           new doubleContainer(1.0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_timeStamp,            new doubleContainer(0.35)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_psi,                  new doubleContainer(0.99)));

        myDoubleMap.insert( std::make_pair( PlanParam::env_delta,                new doubleContainer(0.01)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_ksi,                  new doubleContainer(0.73)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_rho,                  new doubleContainer(0.57)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_objectNessecity,      new doubleContainer(0.1)));

        myDoubleMap.insert( std::make_pair( PlanParam::env_sittingOffset,        new doubleContainer(0.2)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_limitRot,             new doubleContainer(M_PI/3)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_Cellsize,             new doubleContainer(0.2)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_timeLimitation,       new doubleContainer(10.0)));

        myDoubleMap.insert( std::make_pair( PlanParam::env_sitTimeLimitation,    new doubleContainer(0.2)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_timeToDump,           new doubleContainer(0.01)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_futurX,               new doubleContainer(0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_futurY,               new doubleContainer(0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_futurZ,               new doubleContainer(0)));
        myDoubleMap.insert( std::make_pair( PlanParam::env_futurRZ,              new doubleContainer(0)));

    //cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble( PlanParam::env_objectNessecity ) << endl;

        // String
        // ------------------------------------------------------------------
#ifdef QT_LIBRARY
        myStringMap.insert(std::make_pair(PlanParam::titi,                       new stringContainer("titi")));
#endif

        // Vector
        // ------------------------------------------------------------------
        std::vector<double> tutu;
        tutu.push_back( 1 ); tutu.push_back( 8 );

        myVectorMap.insert(std::make_pair(PlanParam::tutu,                       new vectorContainer(tutu)));

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

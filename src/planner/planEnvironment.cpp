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

bool Env_stopUser() {
  bool stop = !(*fct_stop)();

  if (stop) {
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
Parameters<PlanParam::boolParameter,
           PlanParam::intParameter,
           PlanParam::doubleParameter,
           PlanParam::stringParameter,
           PlanParam::vectorParameter>* PlanEnv = NULL;

#ifdef QT_LIBRARY
PlanParam* EnumPlannerParameterObject = NULL;

PlanParam::PlanParam() {}

PlanParam::~PlanParam() {}
#endif

// @brief Function that inizializes the
// Parameter container
void initPlannerParameters() {
#ifdef QT_LIBRARY
  EnumPlannerParameterObject = new PlanParam;
#endif

  // Create 5 maps for all types and fill the 5 maps
  // ------------------------------------------------------------------
  std::map<PlanParam::boolParameter, boolContainer*> myBoolMap;
  std::map<PlanParam::intParameter, intContainer*> myIntMap;
  std::map<PlanParam::doubleParameter, doubleContainer*> myDoubleMap;
  std::map<PlanParam::stringParameter, stringContainer*> myStringMap;
  std::map<PlanParam::vectorParameter, vectorContainer*> myVectorMap;

  // ------------------------------------------------------------------
  // Bool
  // ------------------------------------------------------------------
  myBoolMap.insert(
      std::make_pair(PlanParam::stopPlanner, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::isMaxDisNeigh, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::isWeightedChoice, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::nextIterWaitForGui, new boolContainer(true)));
  myBoolMap.insert(std::make_pair(PlanParam::rrtExtractShortestPath,
                                  new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::orientedGraph, new boolContainer(false)));

  // Time
  myBoolMap.insert(
      std::make_pair(PlanParam::planWithTimeLimit, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajWithTimeLimit, new boolContainer(true)));

  // Smoothing stage
  myBoolMap.insert(
      std::make_pair(PlanParam::trajUseCost, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajPrintGain, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajPartialShortcut, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajSaveCost, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajCostRecompute, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajComputeCollision, new boolContainer(true)));

  // Stomp
  myBoolMap.insert(std::make_pair(PlanParam::trajStompRunParallel,
                                  new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::trajStompRunMultiple,
                                  new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajStompComputeColl, new boolContainer(true)));
  myBoolMap.insert(std::make_pair(PlanParam::trajStompWithTimeLimit,
                                  new boolContainer(true)));
  myBoolMap.insert(std::make_pair(PlanParam::trajStompWithIterLimit,
                                  new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajStompMultiplyM, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajStompWithRRT, new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::trajStompMatrixAdaptation,
                                  new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajStompNoPrint, new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::trajOptimStopWhenCollisionFree,
                                  new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::trajStompDrawImprovement,
                                  new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::trajStompMoveEndConfig,
                                  new boolContainer(false)));

  myBoolMap.insert(
      std::make_pair(PlanParam::trajBiasOptim, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajMoveHuman, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajUseOtp, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::trajNPoints, new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::trajComputeCostAfterPlannif,
                                  new boolContainer(false)));

  myBoolMap.insert(
      std::make_pair(PlanParam::withMaxIteration, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::withGainLimit, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::withSmoothing, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::withShortCut, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::withDeformation, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::withDescent, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::withStomp, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::doReplanning, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::showExploration, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::setRobotIK, new boolContainer(false)));

  // HRI
  myBoolMap.insert(
      std::make_pair(PlanParam::useLegibleCost, new boolContainer(false)));

  // RRT*
  myBoolMap.insert(
      std::make_pair(PlanParam::starRRT, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::starRewire, new boolContainer(false)));

  // RRG
  myBoolMap.insert(std::make_pair(PlanParam::rrg, new boolContainer(false)));

  // Stomp/Chomp
  myBoolMap.insert(
      std::make_pair(PlanParam::withCurrentTraj, new boolContainer(false)));

  // Drawing (gl)
  myBoolMap.insert(
      std::make_pair(PlanParam::drawModule, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawNaturalColor, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawParallelTraj, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawColorConfig, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawOccupVoxels, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawSampledPoints, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawStaticVoxels, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawRandomMap, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawBoundingVolumes, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::drawReachableGrid, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::saveVideo, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_oldCriteria, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_noRepetition, new boolContainer(false)));

  // Object TransfertPoint variable
  myBoolMap.insert(
      std::make_pair(PlanParam::env_humanGridDraw, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_robotGridDraw, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_isStanding, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_drawRandomPoint, new boolContainer(false)));

  myBoolMap.insert(
      std::make_pair(PlanParam::env_drawDistGrid, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_drawOnlyBest, new boolContainer(false)));

  myBoolMap.insert(
      std::make_pair(PlanParam::env_normalRand, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_fusedGridRand, new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::env_fusedGridAndRotRand,
                                  new boolContainer(true)));

  myBoolMap.insert(
      std::make_pair(PlanParam::env_useSlice, new boolContainer(false)));
  myBoolMap.insert(std::make_pair(PlanParam::env_useOrientedSlice,
                                  new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_useAllGrid, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_drawSlice, new boolContainer(false)));

  myBoolMap.insert(
      std::make_pair(PlanParam::env_showHumanTraj, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_useOldDude, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_isInit, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_showText, new boolContainer(true)));

  myBoolMap.insert(std::make_pair(PlanParam::trajOptimTestMultiGauss,
                                  new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_realTime, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::useSelectedDuration, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_drawHumanModel, new boolContainer(false)));

  myBoolMap.insert(
      std::make_pair(PlanParam::env_softMotionTraj, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_createTrajs, new boolContainer(true)));
  myBoolMap.insert(std::make_pair(PlanParam::env_computeTrajForHuman,
                                  new boolContainer(true)));

  myBoolMap.insert(
      std::make_pair(PlanParam::env_trajNormal, new boolContainer(false)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_trajSoftMotion, new boolContainer(true)));
  myBoolMap.insert(
      std::make_pair(PlanParam::env_trajRos, new boolContainer(false)));

  // Collision Space
  myBoolMap.insert(
      std::make_pair(PlanParam::initColisionSpace, new boolContainer(false)));

  // Sample Graph
  myBoolMap.insert(std::make_pair(PlanParam::samplegraphMultiLoop,
                                  new boolContainer(false)));

  // ------------------------------------------------------------------
  // Int
  // ------------------------------------------------------------------
  myIntMap.insert(std::make_pair(PlanParam::tata, new intContainer(5)));
  myIntMap.insert(
      std::make_pair(PlanParam::nb_pointsOnTraj, new intContainer(100)));

  // RRRT
  myIntMap.insert(std::make_pair(PlanParam::plannerMaxIterations,
                                 new intContainer(1000000)));
  myIntMap.insert(
      std::make_pair(PlanParam::smoothMaxIterations, new intContainer(100)));

  // Replanning
  myIntMap.insert(
      std::make_pair(PlanParam::replanningAlgorithm, new intContainer(0)));
  myIntMap.insert(
      std::make_pair(PlanParam::replanningInitMethod, new intContainer(0)));

  // Object TransfertPoint variable
  myIntMap.insert(std::make_pair(PlanParam::env_maxIter, new intContainer(40)));
  myIntMap.insert(
      std::make_pair(PlanParam::env_nbRandomRotOnly, new intContainer(10)));
  myIntMap.insert(
      std::make_pair(PlanParam::env_nbSittingRotation, new intContainer(1)));
  myIntMap.insert(
      std::make_pair(PlanParam::env_totMaxIter, new intContainer(2000)));

  myIntMap.insert(std::make_pair(PlanParam::env_xToDraw, new intContainer(-1)));
  myIntMap.insert(std::make_pair(PlanParam::env_yToDraw, new intContainer(-1)));
  myIntMap.insert(std::make_pair(PlanParam::env_timeShow, new intContainer(2)));
  myIntMap.insert(std::make_pair(PlanParam::env_pow, new intContainer(2)));
  myIntMap.insert(std::make_pair(PlanParam::env_MOTP, new intContainer(20)));
  myIntMap.insert(std::make_pair(PlanParam::env_anglePow, new intContainer(3)));

  myIntMap.insert(
      std::make_pair(PlanParam::stompDrawIteration, new intContainer(7)));
  myIntMap.insert(
      std::make_pair(PlanParam::stompMaxIteration, new intContainer(250)));

  // Nb samples
  myIntMap.insert(
      std::make_pair(PlanParam::lamp_nb_samples, new intContainer(10)));
  myIntMap.insert(
      std::make_pair(PlanParam::lamp_nb_reused_samples, new intContainer(5)));

  // ------------------------------------------------------------------
  // Double
  // ------------------------------------------------------------------
  myDoubleMap.insert(std::make_pair(PlanParam::drawScaleFactorNodeSphere,
                                    new doubleContainer(1.0)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::timeLimitPlanning, new doubleContainer(10.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::timeLimitSmoothing, new doubleContainer(2.0)));
  // Post-processing phaze
  myDoubleMap.insert(
      std::make_pair(PlanParam::MaxFactor, new doubleContainer(3.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::MinStep, new doubleContainer(2.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::costTraj, new doubleContainer(1.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::costResolution, new doubleContainer(1.0)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::distMinToDraw, new doubleContainer(0.3)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::trajStompTimeLimit, new doubleContainer(7.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::trajStompSmoothVel, new doubleContainer(0.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::trajStompSmoothAcc, new doubleContainer(1.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::trajStompSmoothJerk, new doubleContainer(0.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajStompConstStrength,
                                    new doubleContainer(1.0)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::trajDuration, new doubleContainer(5.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::trajOptimStdDev, new doubleContainer(2.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajOptimSmoothWeight,
                                    new doubleContainer(0.1)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajOptimObstacWeight,
                                    new doubleContainer(0.1)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajOptimGlobalWeight,
                                    new doubleContainer(1.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::trajOptimTermWeight, new doubleContainer(0.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajOptimSmoothFactor,
                                    new doubleContainer(1.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajOptimObstacFactor,
                                    new doubleContainer(1.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajReplanningWindow,
                                    new doubleContainer(1.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajReplanningTotalTime,
                                    new doubleContainer(30.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::trajReplanningTotalTime,
                                    new doubleContainer(30.0)));

  // RRT*
  myDoubleMap.insert(
      std::make_pair(PlanParam::starRadius, new doubleContainer(1.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::starFinish, new doubleContainer(3.0)));

  // Sample Graph
  myDoubleMap.insert(std::make_pair(PlanParam::samplegraphVarianceA,
                                    new doubleContainer(1.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::samplegraphVarianceB,
                                    new doubleContainer(3.0)));

  // AStar
  myDoubleMap.insert(
      std::make_pair(PlanParam::grid_pace, new doubleContainer(1.0)));

  // Collspace
  myDoubleMap.insert(std::make_pair(PlanParam::ratioCollRadiusSpacing,
                                    new doubleContainer(2.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::collison_points_clearance,
                                    new doubleContainer(0.06)));

  // Object TransfertPoint variable
  myDoubleMap.insert(std::make_pair(PlanParam::env_randomXMinLimit,
                                    new doubleContainer(-3.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_randomXMaxLimit, new doubleContainer(3.0)));
  myDoubleMap.insert(std::make_pair(PlanParam::env_randomYMinLimit,
                                    new doubleContainer(-3.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_randomYMaxLimit, new doubleContainer(3.0)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::env_robotSpeed, new doubleContainer(1.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_humanSpeed, new doubleContainer(1.0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_timeStamp, new doubleContainer(0.35)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_psi, new doubleContainer(0.99)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::env_delta, new doubleContainer(0.01)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_ksi, new doubleContainer(0.73)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_rho, new doubleContainer(0.57)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_objectNessecity, new doubleContainer(0.1)));

  myDoubleMap.insert(
      std::make_pair(PlanParam::env_sittingOffset, new doubleContainer(0.2)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_limitRot, new doubleContainer(M_PI / 3)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_Cellsize, new doubleContainer(0.2)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_timeLimitation, new doubleContainer(10.0)));

  myDoubleMap.insert(std::make_pair(PlanParam::env_sitTimeLimitation,
                                    new doubleContainer(0.2)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_timeToDump, new doubleContainer(0.01)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_futurX, new doubleContainer(0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_futurY, new doubleContainer(0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_futurZ, new doubleContainer(0)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::env_futurRZ, new doubleContainer(0)));

  // cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble(
  // PlanParam::env_objectNessecity ) << endl;

  // LAMP
  myDoubleMap.insert(
      std::make_pair(PlanParam::lamp_hessian_factor, new doubleContainer(0.)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::lamp_control_cost, new doubleContainer(0.)));
  myDoubleMap.insert(
      std::make_pair(PlanParam::lamp_eta, new doubleContainer(0.1)));

  // String
  // ------------------------------------------------------------------
  myStringMap.insert(
      std::make_pair(PlanParam::active_cost_function, new stringContainer("")));
  myStringMap.insert(
      std::make_pair(PlanParam::end_effector_joint, new stringContainer("")));

  // Vector
  // ------------------------------------------------------------------
  std::vector<double> tutu;
  tutu.push_back(1);

  myVectorMap.insert(
      std::make_pair(PlanParam::planner_joints, new vectorContainer(tutu)));
  myVectorMap.insert(
      std::make_pair(PlanParam::active_joints, new vectorContainer(tutu)));

  // Make the new parameter container
  PlanEnv = new Parameters<PlanParam::boolParameter,
                           PlanParam::intParameter,
                           PlanParam::doubleParameter,
                           PlanParam::stringParameter,
                           PlanParam::vectorParameter>(
      myBoolMap, myIntMap, myDoubleMap, myStringMap, myVectorMap);

  // Planning Environment
  // ------------------------------------------------------------------
  ENV.setBool(Env::biDir, true);
  ENV.setExpansionMethod(Env::Extend);
  ENV.setDouble(Env::extensionStep, 6.0);

  ENV.setDouble(Env::minimalFinalExpansionGap, 5.0);
  ENV.setDouble(Env::temperatureRate, 0.1);

  PlanEnv->setBool(PlanParam::withSmoothing, true);
  PlanEnv->setBool(PlanParam::withDeformation, false);

  // Drawing environment
  // -------------------------------------------------------------------
  ENV.setBool(Env::drawMultiColorLocalpath, true);
}

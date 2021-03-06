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

#include <libmove3d/p3d/ParametersEnv.hpp>

bool Env_stopUser();

#ifdef QT_LIBRARY
class PlanParam : public QObject {
  Q_OBJECT;
  Q_ENUMS(boolParameter);
  Q_ENUMS(intParameter);
  Q_ENUMS(doubleParameter);
  Q_ENUMS(stringParameter);
  Q_ENUMS(vectorParameter);

 public:
  PlanParam();
  ~PlanParam();

#else
namespace PlanParam {
#endif
  enum boolParameter {
    stopPlanner,
    isMaxDisNeigh,
    isWeightedChoice,
    nextIterWaitForGui,
    rrtExtractShortestPath,
    orientedGraph,

    // Time
    trajWithTimeLimit,
    planWithTimeLimit,

    // Smoothing stage
    trajUseCost,
    trajPrintGain,
    trajPartialShortcut,
    trajSaveCost,
    trajCostRecompute,
    trajComputeCollision,

    // Stomp
    trajStompRunParallel,
    trajStompRunMultiple,
    trajStompWithRRT,
    trajStompComputeColl,
    trajStompWithTimeLimit,
    trajStompWithIterLimit,
    trajStompMultiplyM,
    trajStompMatrixAdaptation,
    trajStompNoPrint,
    trajStompDrawImprovement,
    trajStompMoveEndConfig,

    // Traj
    trajBiasOptim,
    trajMoveHuman,
    trajUseOtp,
    trajNPoints,
    trajComputeCostAfterPlannif,
    trajOptimTestMultiGauss,
    trajOptimStopWhenCollisionFree,

    withMaxIteration,
    withGainLimit,
    withSmoothing,
    withShortCut,
    withDeformation,
    withStomp,
    withDescent,

    setRobotIK,
    showExploration,

    // Collision Space
    initColisionSpace,

    // Sample Graph
    samplegraphMultiLoop,

    // RRT*
    starRRT,
    starRewire,

    // RRG
    rrg,

    // Stomp/Chomp & trajectory optim
    withCurrentTraj,
    doReplanning,
    useSelectedDuration,

    // Use Legible Cost
    useLegibleCost,

    // Drawing (gl) stuff
    drawModule,
    drawNaturalColor,
    drawParallelTraj,
    drawColorConfig,
    drawOccupVoxels,
    drawSampledPoints,
    drawStaticVoxels,
    drawRandomMap,
    drawBoundingVolumes,
    drawReachableGrid,
    saveVideo,

    // Object Transfert point variable
    env_humanGridDraw,
    env_robotGridDraw,
    env_isStanding,
    env_drawRandomPoint,
    env_drawDistGrid,
    env_drawOnlyBest,
    env_normalRand,
    env_fusedGridRand,
    env_useSlice,
    env_useOrientedSlice,
    env_useAllGrid,
    env_drawSlice,
    env_showHumanTraj,
    env_useOldDude,
    env_isInit,
    env_showText,
    env_realTime,
    env_drawHumanModel,
    env_softMotionTraj,
    env_createTrajs,
    env_fusedGridAndRotRand,
    env_oldCriteria,
    env_noRepetition,
    env_computeTrajForHuman,
    env_trajNormal,
    env_trajSoftMotion,
    env_trajRos

  };

  enum intParameter {
    tata,
    nb_pointsOnTraj,

    // RRT & PRM
    plannerMaxIterations,
    smoothMaxIterations,

    // Replanning
    replanningAlgorithm,
    replanningInitMethod,

    // Stomp
    stompDrawIteration,
    stompMaxIteration,

    // Object TransfertPoint variable
    env_maxIter,
    env_nbRandomRotOnly,
    env_nbSittingRotation,
    env_totMaxIter,

    // grid cells
    env_xToDraw,
    env_yToDraw,
    env_timeShow,
    env_pow,
    env_MOTP,
    env_anglePow,

    // Lamp
    lamp_nb_samples,
    lamp_nb_reused_samples
  };

  enum doubleParameter {
    // Drawing
    drawScaleFactorNodeSphere,

    // Time
    timeLimitSmoothing,
    timeLimitPlanning,

    // Optimization Variables
    MaxFactor,
    MinStep,
    costTraj,
    costResolution,
    distMinToDraw,
    trajStompTimeLimit,
    trajStompSmoothVel,
    trajStompSmoothAcc,
    trajStompSmoothJerk,
    trajDuration,
    trajOptimStdDev,
    trajOptimSmoothWeight,
    trajOptimObstacWeight,
    trajOptimGlobalWeight,
    trajOptimTermWeight,
    trajOptimSmoothFactor,
    trajOptimObstacFactor,
    trajReplanningWindow,
    trajReplanningTotalTime,
    trajStompConstStrength,

    // Sample Graph
    samplegraphVarianceA,
    samplegraphVarianceB,

    // RRT*
    starRadius,
    starFinish,

    // Pace
    grid_pace,

    // Collision Space
    ratioCollRadiusSpacing,
    collison_points_clearance,

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
    env_sittingOffset,
    env_limitRot,
    env_Cellsize,
    env_timeLimitation,
    env_sitTimeLimitation,
    env_timeToDump,
    env_futurX,
    env_futurY,
    env_futurZ,
    env_futurRZ,

    // Lamp
    lamp_hessian_factor,
    lamp_control_cost,
    lamp_eta
  };

  enum stringParameter { active_cost_function, end_effector_joint };

  enum vectorParameter { planner_joints, active_joints };
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<PlanParam::boolParameter,
                  PlanParam::intParameter,
                  PlanParam::doubleParameter,
                  PlanParam::stringParameter,
                  PlanParam::vectorParameter>* PlanEnv;

// Functions that initializes the planner
// Parameters
void initPlannerParameters();

#ifdef QT_LIBRARY
extern PlanParam* EnumPlannerParameterObject;
#endif

#endif

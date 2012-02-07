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

#ifdef QT_LIBRARY
class PlanParam : public QObject
{

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
namespace PlanParam
{
#endif
        enum boolParameter
        {
            stopPlanner,
            isMaxDisNeigh,
            isWeightedChoice,

            // Smoothing stage
            trajPartialShortcut,
            trajSaveCost,
            trajCostRecompute,
            trajComputeCollision,
            withMaxIteration,
            withGainLimit,
            withTimeLimit,
            withSmoothing,
            withShortCut,
            withDeformation,
            withDescent,
            trajOptimTestMultiGauss,
            showExploration,

            // Stomp/Chomp
            withCurrentTraj,
            doReplanning,
            useSelectedDuration,


            // Drawing (gl) stuff
            drawColorConfig,
            drawOccupVoxels,
            drawSampledPoints,
            drawStaticVoxels,
            drawRandomMap,
            drawBoundingVolumes,
            saveVideo,

            //Object Transfert point variable
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
            env_drawFinalConf,
            env_isFinalConf
        };

        enum intParameter
        {
            tata,
            nb_pointsOnTraj,
          
            // RRT & PRM
            plannerMaxIterations,
          
            // Replanning
            plannerType,
            planningAlgorithm,

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
            env_anglePow

        };

        enum doubleParameter
        {
            // Optimization Variables
            optimTimeLimit,
            MaxFactor,
            MinStep,
            costTraj,
            distMinToDraw,
            trajDuration,
            trajOptimStdDev,
            trajOptimSmoothWeight,
            trajOptimObstacWeight,
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
            env_futurRZ

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
  
#ifdef QT_LIBRARY
extern PlanParam* EnumPlannerParameterObject;
#endif

#endif

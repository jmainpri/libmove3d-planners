#ifndef HRICS_GESTPARAMETERS_HPP
#define HRICS_GESTPARAMETERS_HPP

/*
 *  PlanEnvironment.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 11/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include <libmove3d/p3d/ParametersEnv.hpp>

#ifdef QT_LIBRARY
class GestParam : public QObject
{

  Q_OBJECT;
  Q_ENUMS(boolParameter);
  Q_ENUMS(intParameter);
  Q_ENUMS(doubleParameter);
  Q_ENUMS(stringParameter);
  Q_ENUMS(vectorParameter);

public:

  GestParam();
  ~GestParam();

#else
namespace GestParam
{
#endif
        enum boolParameter
        {
            draw_robot_sampled_points,
            draw_human_sampled_points,
            draw_ws_occupancy,
            draw_single_class,
            draw_null_cost,
            draw_recorded_motion
        };

        enum intParameter
        {
            human_traj_id
        };

        enum doubleParameter
        {
            tete
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
extern Parameters<GestParam::boolParameter,GestParam::intParameter,GestParam::doubleParameter,
GestParam::stringParameter,GestParam::vectorParameter>* GestEnv;

// Functions that initializes the planner
// Parameters
void initGestureParameters();

#ifdef QT_LIBRARY
extern GestParam* EnumGestureParameterObject;
#endif

#endif // HRICS_GESTPARAMETERS_HPP

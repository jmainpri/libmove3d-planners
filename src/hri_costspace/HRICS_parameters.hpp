/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001).
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014
 */
#ifndef HRICS_PARAMETERS_HPP
#define HRICS_PARAMETERS_HPP

#include <libmove3d/p3d/ParametersEnv.hpp>

#ifdef QT_LIBRARY
class HricsParam : public QObject {
  Q_OBJECT;
  Q_ENUMS(boolParameter);
  Q_ENUMS(intParameter);
  Q_ENUMS(doubleParameter);
  Q_ENUMS(stringParameter);
  Q_ENUMS(vectorParameter);

 public:
  HricsParam();
  ~HricsParam();

#else
namespace HricsParam {
#endif
  enum boolParameter {
    init_spheres_cost,
    init_human_trajectory_cost,
    ioc_single_iteration,
    ioc_load_samples_from_file,
    ioc_draw_demonstrations,
    ioc_draw_samples,
    ioc_draw_one_demo,
    ioc_sample_around_demo,
    ioc_exit_after_run,
    ioc_use_stomp_spetial_cost,
    ioc_use_simulation_demos,
    ioc_user_set_pelvis_bounds,
    ioc_use_baseline,
    ioc_no_replanning,
    ioc_split_motions,
    ioc_conservative_baseline,
    ioc_show_last_simulation,
    ioc_remove_split,
    ioc_parallel_job,
    ioc_show_replanning,
    ioc_training_dataset,
    ioc_set_dof_limits_from_experiments
  };

  enum intParameter {
    ioc_phase,
    ioc_sample_iteration,
    ioc_nb_of_way_points,
    ioc_planner_type,
    ioc_spheres_to_draw,
    ioc_from_file_offset,
    ioc_ik,
    ioc_baseline_type,
    ioc_dataset
  };

  enum doubleParameter {
    ioc_spheres_power,
    ioc_sample_std_dev,
    ioc_sample_std_dev_ik,
    ioc_cost_factor
  };

  enum stringParameter { ioc_traj_split_name, ioc_tmp_traj_folder };

  enum vectorParameter { tutu };
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<HricsParam::boolParameter,
                  HricsParam::intParameter,
                  HricsParam::doubleParameter,
                  HricsParam::stringParameter,
                  HricsParam::vectorParameter>* HriEnv;

// Functions that initializes the planner
// Parameters
void initHricsParameters();

#ifdef QT_LIBRARY
extern HricsParam* EnumHricsParameterObject;
#endif

#endif  // HRICS_PARAMETERS_HPP

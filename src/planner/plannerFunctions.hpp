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
#ifdef LIST_OF_PLANNERS
#define Global
#else
#define Global extern
#endif

#include <libmove3d/include/P3d-pkg.h>

/**
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
/*object global permettant d'acceder aux planner dans tous les fichiers de Move3d*/

#include "planner/planner.hpp"
#include "API/Trajectory/trajectory.hpp"

Global std::vector<Move3D::Planner*> plannerlist;

/**
 * RRT statistics
 */
struct RRTStatistics
{
  int runId;
  bool succeeded;
  double time;
  double cost;
  int nbNodes;
  int nbExpansions;
};

/**
 * Return the last trajectory
 */
Move3D::Trajectory p3d_get_last_trajectory();

/**
 * Get last RRT statistics
 */
void p3d_get_rrt_statistics( RRTStatistics& stat );

/**
 * Get last Trajectory statistics
 */
void p3d_get_traj_statistics( Move3D::TrajectoryStatistics& stat );

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion to set multirun Ids
 */
void p3d_planner_functions_set_run_id( unsigned int idRun );

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion to get multi-run Ids
 */
unsigned int p3d_planner_functions_get_run_id();

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion for planning (Manip. Module)
 */
p3d_traj* p3d_planner_function( p3d_rob* robotPt, configPt qs, configPt qg );

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion for smoothing a trajectory (Manip. Module)
 */
void p3d_smoothing_function( p3d_rob* robotPt, p3d_traj* traj, int nbSteps, double maxTime );

/**
 * Set a function that generate IK solutions
 */ 
void p3d_set_goal_solution_function( configPt (*fct)() );

/**
 * Generate Goal Configuration Function
 * This function can be set outside the library
 */ 
bool p3d_generate_goal_configuration( Move3D::Configuration& q );

/**
  @ingroup NEW_CPP_MODULE
 * \fn int p3d_run_prm(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
 * \brief fonction de lancement de l'algorithme PRM
 * @param robotPt the robot
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_prm(p3d_rob* robotPt);

/**
 @ingroup NEW_CPP_MODULE
 * \fn int p3d_run_perturb_prm
 * \brief function that starts the perturbation algorithm
 * @param robotPt the robot
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_perturb_prm(p3d_rob* robotPt);

/**
  @ingroup NEW_CPP_MODULE
 * \fn bool p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void));
 * \brief fonction de lancement de l'algorithme RRT
 * @param robotPt the robot
 * @return la trajectoire peut être générée
 */
int p3d_run_rrt(p3d_rob* robotPt);

/**
  @ingroup NEW_CPP_MODULE
 * \fn bool p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void));
 * \brief function running the EST algorithm
 * @param robotPt the robot
 * @return if there's a trajectory
 */
bool p3d_run_est(p3d_rob* robotPt);

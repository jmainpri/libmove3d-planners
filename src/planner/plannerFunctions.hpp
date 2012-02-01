#ifdef LIST_OF_PLANNERS
#define Global
#else
#define Global extern
#endif

#include "P3d-pkg.h"

/**
	@author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/
/*object global permettant d'acceder aux planner dans tous les fichiers de Move3d*/

#include "planner/planner.hpp"

Global std::vector<Planner*> plannerlist;

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
bool p3d_generate_goal_configuration( Configuration& q );

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
 * \fn int p3d_run_acr(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
 * \brief fonction de lancement de l'algorithme ACR
 * @param robotPt the robot
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_acr(p3d_rob* robotPt);

/**
  @ingroup NEW_CPP_MODULE
 * \fn int p3d_run_vis_prm(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
 * \brief fonction de lancement de l'algorithme Vis_PRM
 * @param robotPt the robot
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_vis_prm(p3d_rob* robotPt);

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

/**
 * @ingroup NEW_CPP_MODULE
 * @brief LEARN FUNCTION to use with C++ Planner API
 */
void p3d_learn_cxx(int NMAX, int (*fct_stop)(void), void (*fct_draw)(void));

/**
 * @ingroup NEW_CPP_MODULE
 * @brief SPECIFIC LEARN FUNCTION to use with C++ Planner API
 */
int p3d_specific_learn_cxx(double *qs, double *qg, int *iksols, int *iksolg, int (*fct_stop)(void), void (*fct_draw)(void));

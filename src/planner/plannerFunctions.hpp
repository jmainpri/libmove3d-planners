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
void p3d_planner_functions_SetRunId( unsigned int idRun );

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion to get multi-run Ids
 */
unsigned int p3d_planner_functions_GetRunId();

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion for planning (Manip. Module)
 */
p3d_traj* planner_Function(p3d_rob* robotPt, configPt qs, configPt qg);

/**
 * @ingroup NEW_CPP_MODULE
 * Funtion for smoothing a trajectory (Manip. Module)
 */
void smoothing_Function(p3d_rob* p3d_Robot, p3d_traj* traj, int nbSteps, double maxTime);


/**
  @ingroup NEW_CPP_MODULE
 * \fn int p3d_run_prm(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
 * \brief fonction de lancement de l'algorithme PRM
 * @param Graph_Pt le graphPt affiché
 * @param fail in/out la trajectoire peut être générée
 * @param (*fct_stop)(void) la fonction d'arret
 * @param (*fct_draw)(void) la fonction d'affichage
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_prm(
		p3d_graph* Graph_Pt,
		int* fail,
		int (*fct_stop)(void),
		void (*fct_draw)(void));

/**
  @ingroup NEW_CPP_MODULE
 * \fn int p3d_run_acr(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
 * \brief fonction de lancement de l'algorithme ACR
 * @param Graph_Pt le graphPt affiché
 * @param fail in/out la trajectoire peut être générée
 * @param (*fct_stop)(void) la fonction d'arret
 * @param (*fct_draw)(void) la fonction d'affichage
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_acr(
		p3d_graph* Graph_Pt,
		int* fail,
		int (*fct_stop)(void),
		void (*fct_draw)(void));

/**
  @ingroup NEW_CPP_MODULE
 * \fn int p3d_run_vis_prm(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
 * \brief fonction de lancement de l'algorithme Vis_PRM
 * @param Graph_Pt le graphPt affiché
 * @param fail in/out la trajectoire peut être générée
 * @param (*fct_stop)(void) la fonction d'arret
 * @param (*fct_draw)(void) la fonction d'affichage
 * @return le nombre de Node ajoutés au Graph
 */
int p3d_run_vis_prm(
		p3d_graph* Graph_Pt,
		int* fail,
		int (*fct_stop)(void),
		void (*fct_draw)(void));

/**
  @ingroup NEW_CPP_MODULE
 * \fn bool p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void));
 * \brief fonction de lancement de l'algorithme RRT
 * @param GraphPt le graphPt affiché
 * @param (*fct_stop)(void) la fonction d'arret
 * @param (*fct_draw)(void) la fonction d'affichage
 * @return la trajectoire peut être générée
 */
int p3d_run_rrt(
		p3d_graph* GraphPt,
		int (*fct_stop)(void),
		void (*fct_draw)(void));

/**
  @ingroup NEW_CPP_MODULE
 * \fn bool p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void));
 * \brief function running the EST algorithm
 * @param GraphPt the graph
 * @param (*fct_stop)(void) stop function
 * @param (*fct_draw)(void) displau function
 * @return if there's a trajectory
 */
bool p3d_run_est(
		p3d_graph* GraphPt,
		int (*fct_stop)(void),
		void (*fct_draw)(void));

/**
 * @ingroup NEW_CPP_MODULE
 * @brief LEARN FUNCTION to use with C++ Planner API
 */
void p3d_learn_cxx(int NMAX,
		int (*fct_stop)(void), void (*fct_draw)(void));

/**
 * @ingroup NEW_CPP_MODULE
 * @brief SPECIFIC LEARN FUNCTION to use with C++ Planner API
 */
int p3d_specific_learn_cxx(double *qs, double *qg, int *iksols, int *iksolg,
		int (*fct_stop)(void), void (*fct_draw)(void));

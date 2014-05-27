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
#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "API/ConfigSpace/configuration.hpp"

namespace Move3D {

#ifndef GRAPH_HPP
class Graph;
#endif

#ifndef NODE_HPP
class Node;
#endif

/**
 * @defgroup NEW_CPP_MODULE C++ Module
 * This Module takes in all that has been done with the new C++ API
 */

/**
 * @ingroup NEW_CPP_MODULE
 * @brief Base class for planning algorithms
 * @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Planner 
{
public:
    /**
   * Plain Constructor of the class
   */
    Planner();

    /**
   * Constructor of the class
   *
   * @param rob a robot robot
   * @param graph a graph belonging to that robot
   */
    Planner(Robot* rob, Graph* graph);

    /**
   * Destructeur de la classe
   */
    virtual ~Planner();

    /**
   * test de trajectoire
   * @return la trajectoire entre les Node Start et Goal existe
   */
    bool trajFound();

    /**
   * retourne le Robot activ
   * @return Le Robot activ
   */
    Robot* getActivRobot();

    /**
   * place le Robot utilisé pour la planification
   * @param R le Robot pour lequel la planification va se faire
   */
    void setRobot(Robot* R);

    /**
   * obtient le Graph actif pour la planification
   * @return le Graph actif pour la planification
   */
    Graph* getActivGraph();

    /**
   * modifie le Graph actif pour la planification
   * @param G le nouveau Graph activ
   */
    void setGraph(Graph* G);

    /**
   * place le Node initial de la planification
   * @param Cs la Configuration initiale du robot pour la planification
   * @return un Node a été ajouté au graph
   */
    bool setInit(confPtr_t Cs);

    /**
   * place le Node final de la planification
   * @param Cg la Configuration finale du robot pour la planification
   * @return un Node a été ajouté au graph
   */
    bool setGoal(confPtr_t Cg);

    /**
   * obtient le Node intial de la planification
   * @return le Node intial de la planification
   */
    Node* getInit();

    /**
   * obtient le Node final de la planification
   * @return le Node final de la planification
   */
    Node* getGoal();

    /**
   * Get init configuration
   */
    confPtr_t getInitConf() { return _q_start; }

    /**
   * Get goal configuration
   */
    confPtr_t getGoalConf() { return _q_goal; }

    /**
   * test si le Planner est initialisé pour la planification
   * @return le Planner est initialisé
   */
    bool getInitialized();

    /**
   * modifie la valeur du Booleen de test d'initialisation
   * @param b la valeur entrée
   */
    void setInitialized(bool b);

    /**
   * Méthode d'initialisation du Planner
   */
    virtual unsigned init();

    /**
   * Run function
   */
    virtual unsigned int run() = 0;

    /**
   * Get the run Id
   */
    int getRunId() { return m_runId; }

    /**
   * Set the run Id
   */
    void setRunId(int id) { m_runId = id; }

    /**
   * return time in algorithm
   * this function must be called after ChronoTimeOfDayOn()
   */
    double getTime();

protected:

    int (*_stop_func)();
    void (*_draw_func)();

    // Store configurations for cases where the
    // goal configuration is not inserted in the graph
    confPtr_t _q_start;
    confPtr_t _q_goal;

    Node* _Start; /*!< Le Node initial de la planification*/
    Node* _Goal; /*!< Le Node final de la planification*/

    Robot* _Robot;/*!< Le Robot pour lequel la recherche va se faire*/
    Graph* _Graph;/*!< Le Graph qui va être utilisé*/

    bool _Init;/*!< Le Planner a été initialisé*/

    bool m_fail;

    int m_runId;

    double m_time;
};

extern Planner* global_Move3DPlanner;

}

#endif

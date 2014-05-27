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
#ifndef PRM_HPP
#define PRM_HPP

#include "planner/planner.hpp"
/**
 @ingroup NEW_CPP_MODULE
 @defgroup PRM Probabilistic-RM
 @brief Probacilistic roadmap module
 \image html prm.jpg
 */

/**
 @ingroup PRM
 @brief Classe représentant l'algorithme PRM
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */

namespace Move3D
{

class PRM : public Planner
{
public:
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    PRM( Robot* R, Graph* G);

    /**
     * Destructeur de la classe
     */
    ~PRM();

    /**
     * initialise le Planner
     * @return le nombre de Node ajoutés lors de l'initialisation
     */
    virtual unsigned init();

    /**
     * test les conditions d'arret
     * @param (*fct_stop)(void) la fonction d'arret
     * @return l'algorithme doit s'arreter
     */
    bool checkStopConditions();

    /**
     * Checks out the preconditions
     */
    bool preConditions();

    /**
     * Function that adds nodes to Graph
     */
    virtual void expandOneStep();

    /**
     * Post process
     */
    virtual void postPocess() { }

    /**
     * fonction principale de l'algorithme PRM
     * @return le nombre de Node ajoutés au Graph
     */
    unsigned int run();

protected:
    /**
     * Members
     */
    unsigned int m_nbAddedNode;
    int m_nbConscutiveFailures; /*!< nombre d'échecs consécutifs*/
    int m_nbExpansions;
};

}

#endif

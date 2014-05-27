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
#ifndef ACR_HPP
#define ACR_HPP

#include "planner/PRM/PRM.hpp"
/**
  @ingroup PRM

    \brief Classe représentant l'algorithme ACR
    @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
*/

namespace Move3D
{

class ACR : public PRM
{
public:
    /**
     * Constructeur de la classe
     * @param WS Le WorkSpace de l'application
     */
    ACR(Robot* R, Graph* G);

    /**
     * Destructeur de la classe
     */
    ~ACR();

    /**
     * fonction principale de l'algorithme ACR
     * @param Graph_Pt le graphPt affiché
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     * @return le nombre de Node ajoutés au Graph
     */
    void expandOneStep();
};

}

#endif

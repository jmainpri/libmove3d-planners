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
/*
 * ManhattanLike-RRT.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef MANHATTANLIKERRT_HPP_
#define MANHATTANLIKERRT_HPP_

#include "planner/Diffusion/RRT.hpp"

#include <libmove3d/include/device.h>

namespace Move3D
{

/**
@ingroup Diffusion
*/
class ManhattanLikeRRT : public RRT {

public:
    /** Constructor from a WorkSpace object
     * @param WS the WorkSpace
     */
    ManhattanLikeRRT(Robot* R, Graph* G);

    /**
     * Destructor
     */
    virtual ~ManhattanLikeRRT() {}


    /**
     * expansion of one Node from one Component to an other
     * In the ML case
     * @param fromComp la composante connexe de départ
     * @param toComp la composante connexe d'arrivée
     * @return le nombre de Node créés
     */
    int expandOneStep(Node* fromComp, Node* toComp);

    bool getCurrentInvalidConf(Configuration& q);

    /**
     * expansion des joints passifs dans le cas ML_RRT
     * @param expansionNode
     * @param NbActiveNodesCreated le nombre de Node créés lors de l'expansion de joints actifs
     * @param directionNode la direction de l'expansion
     * @return le nombre de Node Créés
     */
    int passiveExpandProcess(Node* expansionNode, int NbActiveNodesCreated, Node* directionNode);

    /**
     * choisie si l'expansion sera de type Manhattan
     * @return l'expansion sera de type Manhattan
     */
    bool manhattanSamplePassive();

    int selectNewJntInList( std::vector<jnt*>& joints, std::vector<jnt*>& oldJoints, std::vector<p3d_jnt*>& newJoints );

    int getCollidingPassiveJntList( Robot* R, Configuration& qinv, std::vector<p3d_jnt*>& joints );

    void shoot_jnt_list_and_copy_into_conf( Configuration& qrand, std::vector<p3d_jnt*>& joints );

};

}

#endif /* MANHATTANLIKERRT_HPP_ */

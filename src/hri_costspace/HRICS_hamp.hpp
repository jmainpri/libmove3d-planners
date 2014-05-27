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
 * HriTaskSpaceCost.hpp
 *
 *  Created on: Sep 8, 2009
 *      Author: jmainpri
 */

#ifndef HRITASKSPACECOST_HPP_
#define HRITASKSPACECOST_HPP_

#include "API/Device/robot.hpp"

#include <libmove3d/hri/hri.h>

/**
  @ingroup HRICS
 * Human Interaction Cost definition
 */
namespace HRICS
{
    class HriSpaceCost {

    public:

            HriSpaceCost(p3d_rob* rob,int jnt);

            ~HriSpaceCost();

            /**
             *
             */
            std::vector<int> getTaskPosition();

            /**
             *
             */
            void changeTask(int idJnt);

            /**
             *
             */
            int test();

            /**
             *
             */
            void changeTest(int i);

            /**
             *
             */
            int getTask();

            hri_bitmapset* initialize();
		
            /**
             * Computes the Cost implied by the distance
             */
            double distanceCost();

            /**
              * Computes the Cost implied by the distance
              */
            double visibilityCost();

            /**
             *
             */
            double comfortCost();

            /**
             * Computes the combined cost
             */
            double combinedCost();

            /**
             *
             */
            double switchCost();

            void computeWorkspacePath();

            void computeHoleManipulationPath();

            void computingAStarOnGraph();


    private:
            /**
             *
             */
            Move3D::Robot* _Robot;

            hri_bitmapset* _Bitmap;

            std::vector<int> pos;

            int _JntId;

            int _test;

    };
}

#endif /* HRITASKSPACECOST_HPP_ */

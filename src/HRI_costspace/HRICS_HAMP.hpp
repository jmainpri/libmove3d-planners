/*
 * HriTaskSpaceCost.hpp
 *
 *  Created on: Sep 8, 2009
 *      Author: jmainpri
 */

#ifndef HRITASKSPACECOST_HPP_
#define HRITASKSPACECOST_HPP_


#include "Hri_planner-pkg.h"
#include "planningAPI.hpp"

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
            Robot* _Robot;

            hri_bitmapset* _Bitmap;

            std::vector<int> pos;

            int _JntId;

            int _test;

    };
}

#endif /* HRITASKSPACECOST_HPP_ */

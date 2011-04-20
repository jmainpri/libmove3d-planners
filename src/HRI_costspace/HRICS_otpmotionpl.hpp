#ifndef HRICS_OTPMOTIONPL_HPP
#define HRICS_OTPMOTIONPL_HPP

/*
 *  HRICS_CostSpace.h
 *  BioMove3D
 *
 *  Created by Mamoun Gharbi on 20/04/11.
 *  Copyright 2009 magharbi@laas.fr All rights reserved.
 *
 */
#include "API/planningAPI.hpp"
#include "planner/planner.hpp"

#include "HRICS_Workspace.hpp"

#include "Grid/HRICS_EnvGrid.hpp"

/**
 @defgroup HRICS Hri Cost space
 */

/**
 @ingroup HRICS
 */
namespace HRICS
{

    /**
      * Motion Planing for an object transfert point
      */
    class OTPMotionPl : public HumanAwareMotionPlanner
    {
    public:
        OTPMotionPl();
        OTPMotionPl(Robot* R, Robot* H);

        ~OTPMotionPl();

        EnvGrid* getPlanGrid() { return m2DGrid; }



    private:
        void initCostSpace();

        Robot* mHuman;
        EnvGrid* m2DGrid;

        std::vector<double> mEnvSize;
    };

}

#endif // HRICS_OTPMOTIONPL_HPP

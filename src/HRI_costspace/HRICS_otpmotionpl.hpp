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
#include "Grid/HRICS_TwoDGrid.hpp"

#include <Eigen/StdVector>

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


        bool computeAStarIn2DGrid();

        void solveAStar(EnvState* start, EnvState* goal, bool isHuman);
        void draw2dPath();

    private:
        void initCostSpace();

        Robot* mHuman;
        EnvGrid* m2DGrid;

        std::vector<double> mEnvSize;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m2DPath;
        std::vector<API::TwoDCell*> m2DCellPath;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m2DHumanPath;
        std::vector<API::TwoDCell*> m2DHumanCellPath;

        bool mPathExist;
        bool mHumanPathExist;
    };

}

#endif // HRICS_OTPMOTIONPL_HPP

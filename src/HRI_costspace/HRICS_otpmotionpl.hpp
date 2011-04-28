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

        EnvGrid* getPlanGrid() { return m_2DGrid; }


        bool computeAStarIn2DGrid();

        void solveAStar(EnvState* start, EnvState* goal, bool isHuman);
        void draw2dPath();

        bool simpleComputeBaseAndOTP();
        bool computeObjectTransfertPoint();

        bool FindTraj(Eigen::Vector2d startPos, Eigen::Vector2d goalPos, bool isHuman);
        bool computeUpBodyOpt();

        void initDistance();

        bool moveToNextPos();
        void SetPathIndexNull(){ m_pathIndex = -1; }


    private:
        void initCostSpace();
        void initHumanCenteredGrid();

        Robot* m_Human;
        EnvGrid* m_2DGrid;
        EnvGrid* m_2DHumanCenteredGrid;

        std::vector<double> m_EnvSize;
        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DPath;
        std::vector<API::TwoDCell*> m_2DCellPath;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m_2DHumanPath;
        std::vector<API::TwoDCell*> m_2DHumanCellPath;

        bool m_PathExist;
        bool m_HumanPathExist;

        unsigned int m_pathIndex;
    };

}

#endif // HRICS_OTPMOTIONPL_HPP

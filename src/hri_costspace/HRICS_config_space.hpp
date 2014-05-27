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
#ifndef HRICS_CSPACE_HPP
#define HRICS_CSPACE_HPP

#include "planner/planner.hpp"

#include "HRICS_workspace.hpp"

#include "grid/HRICS_grid.hpp"
#include "grid/HRICS_two_d_grid.hpp"


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
      * Configuration space
      */
    class ConfigSpace : public HumanAwareMotionPlanner
    {
    public:
        ConfigSpace();
        ConfigSpace( Move3D::Robot* R, Move3D::Robot* H );

        ~ConfigSpace();

        /**
          * General config cost
          */
        double getConfigCost();
		
		/**
		 * Elemetary cost functions
		 */
        double getDistanceCost();
        double getVisibilityCost(const Eigen::Vector3d& WSPoint) 
		{
			return m_VisibilitySpace->getWorkspaceCost(WSPoint);
		}

        void computeVisibilityGrid();
        void computeDistanceGrid();
		
		
        Grid* getGrid() { return m3DGrid; }
        PlanGrid* getPlanGrid() { return m2DGrid; }
        std::vector<Move3D::TwoDCell*> getCellPath() { return m2DCellPath; }

        double getLastDistanceCost() {return mDistCost; }
        double getLastVisibiliCost() {return mVisiCost; }

        bool computeAStarIn2DGrid();
        void solveAStar(PlanState* start,PlanState* goal);
        void draw2dPath();
        double pathCost();

//        bool runHriRRT();
        bool initHriRRT();

    private:
        void initCostSpace();

        //        Robot* mRobot;
        Move3D::Robot* mHuman;
        Grid* m3DGrid;
        PlanGrid* m2DGrid;

        std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> >   m2DPath;
        std::vector<Move3D::TwoDCell*> m2DCellPath;

        int mIndexObjectDof;

        Eigen::Vector3d mVisibilityPoint;

        double mDistCost;
        double mVisiCost;

        bool mPathExist;

        std::vector<double> mEnvSize;
    };
}

#endif // HRICS_ConfigSpace_HPP

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

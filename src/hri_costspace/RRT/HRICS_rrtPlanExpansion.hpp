#ifndef HRICS_RRTPLANEXPANSION_H
#define HRICS_RRTPLANEXPANSION_H

#include "planner/Diffusion/Variants/Transition-RRT.hpp"
#include "hri_costspace/Grid/HRICS_TwoDGrid.hpp"

namespace HRICS
{

/**
  @ingroup HRICS
  @brief Special RRT Expansion method for the HRICS
  */
class HRICS_rrtPlanExpansion : public Move3D::TransitionExpansion
{
public:
    HRICS_rrtPlanExpansion();
    HRICS_rrtPlanExpansion(Move3D::Graph* G);

    /**
      * Initializes some variables for the expansion
      * method
      */
    void init();

    /**
      * Sets the grid
      */
    void setGrid( Move3D::TwoDGrid* grid ) { m2DGrid = dynamic_cast<HRICS::PlanGrid*>(grid); }

    /**
      * Sets the cell path
      */
    void setCellPath( std::vector<Move3D::TwoDCell*> cellPath );

    /**
      * Direction used in RRT one step
      */
    Move3D::confPtr_t getExpansionDirection( Move3D::Node* expandComp, Move3D::Node* goalComp, bool samplePassive, Move3D::Node*& directionNode);

    /**
      * Configuration from the next cell along the 3dPath
      */
    Move3D::confPtr_t getConfigurationInNextCell( Move3D::Node* node );

    /**
      * Adds a node to a conected component
      */
    Move3D::Node* addNode( Move3D::Node* currentNode, Move3D::LocalPath& path, double pathDelta, Move3D::Node* directionNode, int& nbCreatedNodes );

    /**
      * Checks it the cell is after the given cell on the
      * 2D path
      */
    bool on2DPathAndAfter( Move3D::TwoDCell* cell );

private:
    HRICS::PlanGrid*                m2DGrid;
    std::vector<Move3D::TwoDCell*>  m2DCellPath;

    Move3D::TwoDCell*               mLastForward;
    Move3D::TwoDCell*               mLastBackward;
    Move3D::TwoDCell*               mBiasedPlanCell;

    bool                            mForward;
    bool                            mBiasing;

    double*                         mBox;

};

}

#endif // HRICS_RRTEXPANSION_H

#ifndef HRICS_RRTPLANEXPANSION_H
#define HRICS_RRTPLANEXPANSION_H

#include "API/planningAPI.hpp"
#include "planner/Diffusion/Variants/Transition-RRT.hpp"
#include "HRI_costspace/Grid/HRICS_TwoDGrid.hpp"

/**
  @ingroup HRICS
  @brief Special RRT Expansion method for the HRICS
  */
class HRICS_rrtPlanExpansion : public TransitionExpansion
{
public:
    HRICS_rrtPlanExpansion();
    HRICS_rrtPlanExpansion(Graph* G);

    /**
      * Initializes some variables for the expansion
      * method
      */
    void init();

     /**
      * Sets the grid
      */
    void setGrid(API::TwoDGrid* grid) { m2DGrid = dynamic_cast<HRICS::PlanGrid*>(grid); }

     /**
      * Sets the cell path
      */
    void setCellPath(std::vector<API::TwoDCell*> cellPath);

    /**
      * Direction used in RRT one step
      */
    std::tr1::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive, Node*& directionNode);

    /**
      * Configuration from the next cell along the 3dPath
      */
    std::tr1::shared_ptr<Configuration> getConfigurationInNextCell(Node* node);

    /**
      * Adds a node to a conected component
      */
    Node* addNode(Node* currentNode, LocalPath& path, double pathDelta,
                  Node* directionNode, int& nbCreatedNodes);

    /**
      * Checks it the cell is after the given cell on the
      * 2D path
      */
    bool on2DPathAndAfter(API::TwoDCell* cell);

private:
    HRICS::PlanGrid*             m2DGrid;
    std::vector<API::TwoDCell*>  m2DCellPath;

    API::TwoDCell*               mLastForward;
    API::TwoDCell*               mLastBackward;
    API::TwoDCell*               mBiasedPlanCell;

    bool                     mForward;
    bool                     mBiasing;

    double*                  mBox;

};

#endif // HRICS_RRTEXPANSION_H

#ifndef HRICS_RRTEXPANSION_H
#define HRICS_RRTEXPANSION_H

#include "Diffusion/Variants/Transition-RRT.hpp"
#include "hri_costspace/grid/HRICS_grid.hpp"

namespace HRICS
{

/**
  @ingroup HRICS
  @brief Special RRT Expansion method for the HRICS
  */
class HRICS_rrtExpansion : public Move3D::TransitionExpansion
{
public:
    HRICS_rrtExpansion();
    HRICS_rrtExpansion(Move3D::Graph* G);

    /**
      * Initializes some variables for the expansion
      * method
      */
    void init();

     /**
      * Sets the grid
      */
    void setGrid(Move3D::ThreeDGrid* grid) { _3DGrid = dynamic_cast<HRICS::Grid*>(grid); }

     /**
      * Sets the cell path
      */
    void setCellPath(std::vector<Move3D::ThreeDCell*> cellPath);

    /**
      * Direction used in RRT one step
      */
    Move3D::confPtr_t getExpansionDirection( Move3D::Node* expandComp, Move3D::Node* goalComp, bool samplePassive, Move3D::Node*& directionNode);

    /**
      * Configuration from the next cell along the 3dPath
      */
    Move3D::confPtr_t getConfigurationInNextCell(Move3D::Node* node);

    /**
      * Adds a node to a conected component
      */
    Move3D::Node* addNode(Move3D::Node* currentNode, Move3D::LocalPath& path, double pathDelta, Move3D::Node* directionNode, int& nbCreatedNodes);

    /**
      * Checks it the cell is after the given cell on the
      * 3D path
      */
    bool on3DPathAndAfter(Move3D::ThreeDCell* cell);

private:
    HRICS::Grid*             _3DGrid;
    std::vector<Move3D::ThreeDCell*>  _3DCellPath;

    Move3D::ThreeDCell*               _LastForward;
    Move3D::ThreeDCell*               _LastBackward;

    int         mIndexObjectDof;

    bool                     _forward;
    bool                     _biasing;

    double*                  _Box;

};

}

#endif // HRICS_RRTEXPANSION_H

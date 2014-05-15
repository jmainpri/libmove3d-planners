#ifndef HRICS_RRT_H
#define HRICS_RRT_H

#include "planner/Diffusion/RRT.hpp"
#include "../grid/HRICS_grid.hpp"

/**
  @ingroup HRICS
  @brief Special RRT implentation for the HRICS
  */
namespace HRICS
{

class HRICS_RRT : public Move3D::RRT
{

public:
    HRICS_RRT(Move3D::Robot* R, Move3D::Graph* G);

    /**
      * Sets the grid pointer
      */
    void setGrid(HRICS::Grid* G);

    /**
      * Sets the cell path
      */
    void setCellPath(std::vector<Move3D::ThreeDCell*> cellPath);

   /**
     * Initialzation of the plannificator
     * @return the number of node added during the init phase
     */
    virtual unsigned init();

    /**
      * Intents to connect a node to the compco
      */
    bool connectNodeToCompco(Move3D::Node* node, Move3D::Node* compNode);

    /**
      * Finds the nearest neighbour in the cell
      * @return Other wise returns NULL
      */
    Move3D::Node* nearestNeighbourInCell(Move3D::Node* node, std::vector<Move3D::Node*> neigbour);

    /**
      * @return the cell in which is the given node
      */
    Move3D::ThreeDCell* getCellFromNode(Move3D::Node* node);

 private:
    HRICS::Grid* _Grid;

};

}

#endif // HRICS_RRT_H

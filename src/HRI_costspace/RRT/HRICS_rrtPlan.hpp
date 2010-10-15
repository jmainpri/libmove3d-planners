#ifndef HRICS_RRTPLAN_H
#define HRICS_RRTPLAN_H

#include "../../Diffusion/RRT.hpp"
#include "../Grid/HRICS_TwoDGrid.hpp"

/**
  @ingroup HRICS
  @brief Special RRT implentation for the HRICS
  */
class HRICS_RRTPlan : public RRT
{
public:
    HRICS_RRTPlan(Robot* R, Graph* G);

    /**
      * Sets the grid pointer
      */
    void setGrid(HRICS::PlanGrid* G);

    /**
      * Sets the cell path
      */
    void setCellPath(std::vector<API::TwoDCell*> cellPath);

   /**
     * Initialzation of the plannificator
     * @return the number of node added during the init phase
     */
    virtual int init();

    /**
      * Intents to connect a node to the compco
      */
    bool connectNodeToCompco(Node* node, Node* compNode);

    /**
      * Finds the nearest neighbour in the cell
      * @return Other wise returns NULL
      */
    Node* nearestNeighbourInCell(Node* node, std::vector<Node*> neigbour);

    /**
      * @return the cell in which is the given node
      */
    API::TwoDCell* getCellFromNode(Node* node);

 private:
    HRICS::PlanGrid* mGrid;

};

#endif // HRICS_RRT_H

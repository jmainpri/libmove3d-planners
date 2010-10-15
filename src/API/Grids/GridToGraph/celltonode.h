#ifndef CELLTONODE_H
#define CELLTONODE_H

#include "../ThreeDCell.h"
#include "gridtograph.h"

/**
  @ingroup GRID
  */
class CellToNode : public API::ThreeDCell
{
public:
    CellToNode();
    CellToNode(int i, Eigen::Vector3d corner, GridToGraph* grid);

    ~CellToNode();

    bool cellHasNode() { return _CellHasNode; }

    void setNode(Node* N) {_Node = N; _CellHasNode = true; }

    Node* getNode() { return _Node; }


private:
    bool _CellHasNode;
    Node* _Node;
};

#endif // CELLTONODE_H

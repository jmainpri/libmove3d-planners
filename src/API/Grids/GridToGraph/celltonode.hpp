#ifndef CELLTONODE_H
#define CELLTONODE_H

#include "API/Grids/ThreeDGrid.hpp"
#include "API/Grids/GridToGraph/gridtograph.hpp"

namespace Move3D {

/**
  @ingroup GRID
  */
class CellToNode : public Move3D::ThreeDCell
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

}

#endif // CELLTONODE_H

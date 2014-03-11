#include "celltonode.hpp"

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;
using namespace Move3D;

CellToNode::CellToNode()
{

}

CellToNode::CellToNode(int i, Vector3d corner, GridToGraph* grid) :
        ThreeDCell(i,corner,grid),
        _CellHasNode(false)
{

}

CellToNode::~CellToNode()
{

}

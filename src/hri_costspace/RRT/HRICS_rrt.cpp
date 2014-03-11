#include "HRICS_rrt.hpp"
#include "HRICS_rrtExpansion.hpp"
#include "HRICS_costspace.hpp"

#include "Roadmap/compco.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace HRICS;
using namespace Eigen;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

/**
  * Basic constructor
  */
HRICS_RRT::HRICS_RRT(Robot* R, Graph* G) : RRT(R,G)
{

}



/**
 * Initializes an RRT Planner
 */
unsigned  HRICS_RRT::init()
{
    int added = TreePlanner::init();

    _expan = new HRICS_rrtExpansion(_Graph);

    p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
                           this->getInit()->getNodeStruct(),
                           this->getGoal()->getNodeStruct());

    setGrid(dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid());
    setCellPath(dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getCellPath());

    cout << "End Init HRICS_RRT" << endl;

    return added;
}

/**
  * Sets the grid pointer
  */
void HRICS_RRT::setGrid(HRICS::Grid* G)
{
    _Grid = G;
    dynamic_cast<HRICS_rrtExpansion*>(_expan)->setGrid(G);

}

/**
 * Sets the cell path
 */
void HRICS_RRT::setCellPath(std::vector<Move3D::ThreeDCell*> cellPath)
{
    dynamic_cast<HRICS_rrtExpansion*>(_expan)->setCellPath(cellPath);
}

/**
 * Tries to connect a node to a given component
 * First checks that the node is note in the compco
 * and the if their is a neighbour in the same cell
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool HRICS_RRT::connectNodeToCompco(Node* node, Node* compNode)
{
    vector<Node*> nodes = _Graph->getNodesInTheCompCo(compNode);

    for(unsigned int i=0;i<nodes.size();i++)
    {
        if( *nodes[i] == *node )
        {
            cout << "HRICS_RRT::Error => Node is allready in the Connected Comp" << endl;
        }
    }

    Node* neighbour = nearestNeighbourInCell(node,nodes);

    if( neighbour )
    {
        cout << "HRICS_RRT:: Tries to connect to the neihbour in Cell" << endl;

        return p3d_ConnectNodeToComp(
                node->getGraph()->getGraphStruct(),
                node->getNodeStruct(),
                neighbour->getConnectedComponent()->getCompcoStruct());
    }

    return false;
}

/**
  * Gets the cell in which the Node is
  * Then from a vector of all nodes in the connected component
  * makes a vector of nodes in the cell, returns nearest
  */
Node* HRICS_RRT::nearestNeighbourInCell(Node* node, std::vector<Node*> neigbour)
{

    Move3D::ThreeDCell* cell = getCellFromNode(node);
    vector<Node*> nodesInCell;

    for(unsigned int i=0;i<neigbour.size();i++)
    {
        if((*cell) == (*getCellFromNode(neigbour[i])))
        {
            nodesInCell.push_back(neigbour[i]);
        }
    }

    double minDist = numeric_limits<double>::max();
    Node* nearest = 0x00;

    for(unsigned int i=0;i<nodesInCell.size();i++)
    {
        shared_ptr<Configuration> config = nodesInCell[i]->getConfiguration();
        double dist = node->getConfiguration()->dist(*config);
        if(minDist>dist)
        {
            minDist = dist;
            nearest = nodesInCell[i];
        }
    }

    return nearest;
}

/**
  * Gets Cell in grid from a given node
  */
Move3D::ThreeDCell* HRICS_RRT::getCellFromNode(Node* node)
{
    shared_ptr<Configuration> config = node->getConfiguration();

    Vector3d pos;

#ifdef LIGHT_PLANNER
    int IndexObjectDof = config->getRobot()->getObjectDof();
	
    pos[0] = config->at(IndexObjectDof+0);
    pos[1] = config->at(IndexObjectDof+1);
		pos[2] = config->at(IndexObjectDof+2);
#endif
    //        cout << "pos = " << endl << pos << endl;

    return _Grid->getCell(pos);
}

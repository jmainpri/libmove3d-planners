/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "HRICS_rrtPlan.hpp"
#include "HRICS_rrtPlanExpansion.hpp"
#include "HRICS_costspace.hpp"

#include "Roadmap/compco.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;
using namespace HRICS;
using namespace Eigen;

MOVE3D_USING_SHARED_PTR_NAMESPACE

/**
  * Basic constructor
  */
HRICS_RRTPlan::HRICS_RRTPlan(Robot* R, Graph* G) : RRT(R,G)
{

}



/**
 * Initializes an RRT Planner
 */
unsigned  HRICS_RRTPlan::init()
{
    int added = TreePlanner::init();

    _expan = new HRICS_rrtPlanExpansion(_Graph);

    p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
                           this->getInit()->getNodeStruct(),
                           this->getGoal()->getNodeStruct());

    this->setGrid(dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getPlanGrid());
    this->setCellPath(dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getCellPath());

    cout << "End Init HRICS_RRTPlan" << endl;

    return added;
}

/**
  * Sets the grid pointer
  */
void HRICS_RRTPlan::setGrid(HRICS::PlanGrid* G)
{
    mGrid = G;
    dynamic_cast<HRICS_rrtPlanExpansion*>(_expan)->setGrid(G);

}

/**
 * Sets the cell path
 */
void HRICS_RRTPlan::setCellPath(std::vector<Move3D::TwoDCell*> cellPath)
{
    dynamic_cast<HRICS_rrtPlanExpansion*>(_expan)->setCellPath(cellPath);
}

/**
 * Tries to connect a node to a given component
 * First checks that the node is note in the compco
 * and the if their is a neighbour in the same cell
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool HRICS_RRTPlan::connectNodeToCompco(Node* node, Node* compNode)
{
    vector<Node*> nodes = _Graph->getNodesInTheCompCo(compNode);

    for( unsigned int i=0;i<nodes.size();i++ )
    {
        if( *nodes[i] == *node )
        {
            cout << "HRICS_RRTPlan::Error => Node is allready in the Connected Comp" << endl;
        }
    }

    Node* neighbour = nearestNeighbourInCell( node,nodes );

    if( neighbour )
    {
        cout << "HRICS_RRTPlan:: Tries to connect to the neihbour in Cell" << endl;

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
Node* HRICS_RRTPlan::nearestNeighbourInCell(Node* node, vector<Node*> neigbour)
{

    Move3D::TwoDCell* cell = getCellFromNode(node);
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
        confPtr_t config = nodesInCell[i]->getConfiguration();
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
Move3D::TwoDCell* HRICS_RRTPlan::getCellFromNode(Node* node)
{
    confPtr_t config = node->getConfiguration();

    Vector2d pos;

//    int IndexObjectDof = config->getRobot()->getObjectDof();
    pos[0] = config->at(6);
    pos[1] = config->at(7);
//    pos[2] = config->at(IndexObjectDof+2);
    //        cout << "pos = " << endl << pos << endl;

    return mGrid->getCell(pos);
}

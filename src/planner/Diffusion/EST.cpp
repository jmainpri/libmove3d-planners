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
/*
 * EST.cpp
 *
 *  Created on: Sep 16, 2009
 *      Author: jmainpri
 */

#include "EST.hpp"

#include "Planner-pkg.h" 

#include "planEnvironment.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

EST::EST(Robot* R,Graph* G) :
    TreePlanner(R,G)
{

}

EST::~EST()
{

}

/**
 * Ckecks Stop Conditions
 */
bool EST::checkStopConditions()
{
    if(_Graph->getNumberOfNodes() > 100)
    {
        //		cout << "Number of node reached 100" << endl;
        //		return true;
    }

    if(TreePlanner::checkStopConditions())
    {

        return true;
    }

    return false;

}

/**
 * Checks out the preconditions
 */
bool EST::preConditions()
{
    if(TreePlanner::preConditions())
    {
        if (ENV.getBool(Env::expandToGoal))
        {
            LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
            if (direct.isValid())
            {
                connectNodeToCompco( getInit(), getGoal() );

#ifdef DEBUG_STATUS
                cout << "Direct connection" << endl;
#endif
                return true;
            }
        }

        if (ENV.getBool(Env::expandToGoal))
        {
            if( connectNodeToCompco( getInit(), getGoal()) )
                return false;


        }
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * Initializes an RRT Planner
 */
unsigned  EST::init()
{

    int added = TreePlanner::init();

    _SortedNodes = _Start->getSortedNodes();
    addNodeToSet(_Start);
    _Start->setSortedNodes(_SortedNodes);

    //	if(ENV.getBool(Env::biDir))
    //	{
    _SortedNodes = _Goal->getSortedNodes();
    addNodeToSet(_Goal);
    _Goal->setSortedNodes(_SortedNodes);
    //	}

    _Expan = new ESTExpansion(_Graph);

    return added;
}

class myNodeComparator {

public:
    EST* ptrNode;

    bool operator()(Node* ptrNode1,Node* ptrNode2) {

        return (
                    ptrNode1->getSelectCost() <
                    ptrNode2->getSelectCost() );
    }

} nodeCompObject;

/**
 * Adds a node to the sorted set
 */
void EST::addNodeToSet(Node* extentionNode)
{
    //	cout << "addNodeToSet" << endl;

    _SortedNodes.push_back(extentionNode);

    sort( _SortedNodes.begin(), _SortedNodes.end(), nodeCompObject);

    //	if(_SortedNodes.size()==0)
    //	{
    //		_SortedNodes.push_back(extentionNode);
    //		return;
    //	}
    //
    //	vector<Node*>::iterator it;
    //
    //	for( it=_SortedNodes.begin();
    //		 it != _SortedNodes.end();
    //		 it++ )
    //	{
    //		Node* node = *it;
    //		if( extentionNode->getCost() < node->getSelectCost() )
    //		{
    //			_SortedNodes.insert(it,extentionNode);
    //			return;
    //		}
    //	}
}

/**
 * Three phases One Step Expansion
 *  - Node
 *  - Direction
 *  - Process
 *
 *  @param fromComp the component which is expanded
 *  @param toComp the goal component
 */
int EST::expandOneStep(Node* fromComp, Node* toComp)
{
    Node* extentionNode;
    Node* expansionNode;
    confPtr_t directionConfig;

    _SortedNodes = fromComp->getSortedNodes();


    // get node for expansion
    expansionNode = _Expan->getExpansionNode(_SortedNodes);

    // get direction
    directionConfig = _Expan->getExpansionDirection(expansionNode,toComp);

    // expansion
    int nbOfNodesAdded=0;

    extentionNode = _Expan->expandProcessEST(expansionNode,directionConfig, nbOfNodesAdded);

    if( nbOfNodesAdded>0 )
    {
        addNodeToSet(extentionNode);
        fromComp->setSortedNodes(_SortedNodes);
    }
    else
    {
        expansionNode->setExpandFailed();
        m_last_node = _Expan->getLasAddedNode();
    }

    return nbOfNodesAdded;
    //	}
}

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool EST::connectNodeToCompco(Node* node, Node* compNode)
{
    int SavedIsMaxDis = FALSE;
    Node* node2(NULL);

    switch(p3d_GetNodeCompStrategy()) {
    case K_NEAREST_NODE_COMP:
        /*Connect randomly to one of the k nearest
       nodes of the componant */
        /*todo*/
        return false;
    case NEAREST_NODE_COMP:
    default:
        SavedIsMaxDis =  PlanEnv->getBool(PlanParam::isMaxDisNeigh);
        //p3d_SetIsMaxDistNeighbor(FALSE);

        PlanEnv->setBool(PlanParam::isMaxDisNeigh,false);

        node2 = _Graph->nearestWeightNeighbour(compNode, node->getConfiguration(), ENV.getInt(Env::DistConfigChoice));

        p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);

        LocalPath path(node->getConfiguration(),node2->getConfiguration());

        if(!ENV.getBool(Env::costBeforeColl))
        {
            if( path.isValid() )
            {
                if( path.getParamMax() <= _Expan->step() )
                {
                    int nbCreatedNodes=0;

                    _Expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                    cout << "Path Valid Connected" << endl;
                    return true;
                }

                if( _Expan->expandToGoal( node, node2->getConfiguration()))
                {
                    int nbCreatedNodes=0;

                    _Expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                    cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
                    return true;
                }
            }
            return false;
        }
        else
        {
            if( path.getParamMax() <= _Expan->step() )
            {
                int nbCreatedNodes=0;

                if( path.isValid() )
                {
                    _Expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                    cout << "Path Valid Connected" << endl;
                    return true;
                }
                else
                {
                    return false;
                }
            }
            if( _Expan->expandToGoal( node, node2->getConfiguration() ))
            {
                if( path.isValid() )
                {
                    int nbCreatedNodes=0;
                    _Expan->addNode(node,path,1.0,node2,nbCreatedNodes);
                    cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
                    return true;
                }
                else
                {
                    return false;
                }
            }

            return(false);
        }
    }
}

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
 * ESTExpansion.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */

#include "ESTExpansion.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include "Planner-pkg.h"

using namespace std;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

ESTExpansion::ESTExpansion() :
    RRTExpansion()
{
}

ESTExpansion::ESTExpansion(Graph* ptrGraph) :
    RRTExpansion(ptrGraph)
{
}

ESTExpansion::~ESTExpansion()
{
}

/**
 * EST
 */
Node* ESTExpansion::getExpansionNode(vector<Node*>& nodes)
{
    // Get id biased to high values
    double x = (double) (pow(p3d_random(0,1),3));
    unsigned int size = nodes.size();
    unsigned int id = (unsigned int) (x * (double) (size-1));
    //	cout << "id = " <<  id << endl;
    //	printAllNodes(nodes);

    int	MaxFail = 5;

    for(;;)
    {
        unsigned int i = 0;
        vector<Node*>::iterator it = nodes.begin();
        // Get member of the map
        for (; it != nodes.end(); it++, i++)
        {
            //		if (i == id)
            //		{
            Node* thisNode(*it);

            if( thisNode->getNbExpandFailed() < MaxFail )
            {
                //					cout << "id = " << i << endl;
                return thisNode;
            }

            //			if( thisNode->getNumberOfNeighbors() > 1 )
            //			{
            //				if( nodes.size() > 10)
            //				{
            //					nodes.erase(it);
            //				}
            //				return getExpansionNode(nodes);

            //				nodes.erase(it);
            //				return thisNode;
            //			}
            //			cout << "Size = " << nodes.size() << endl;

            //			printAllNodes(nodes);
            //		}
        }
        MaxFail++;
    }


    cout << "Problem not finding node" << endl;
    //	Node* expandNode;
    return nodes[id];

}

/**
 * EST
 */
confPtr_t ESTExpansion::getExpansionDirection(
        Node* expansionNode, Node* toComp)
{
    confPtr_t q;

    q = m_Graph->getRobot()->shoot(true);
    return q;

}

/**
 * EST
 */
Node* ESTExpansion::expandProcessEST( Node* expansionNode,
                                      confPtr_t directionConfig,
                                      int& nbCreatedNodes)
{
    confPtr_t fromConfig = expansionNode->getConfiguration();
    confPtr_t toConfig;

    LocalPath directionLocalpath(fromConfig, directionConfig);

    double pathDelta = directionLocalpath.getParamMax() == 0. ? 1.
                                                              : MIN(1., step() / directionLocalpath.getParamMax() );

    // Expansion control
    // Discards potential nodes that are to close to the graph
    if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,*expansionNode))
    {
        nbCreatedNodes=0;
        return 0;
    }

    double param(0);
    double paramMax = directionLocalpath.getParamMax();

    // Perform extension toward directionConfig
    param = step();


    if (param > paramMax)
    {
        toConfig = directionConfig;
    }
    else
    {
        toConfig = directionLocalpath.configAtParam(param);
    }

    LocalPath extensionLocalpath(fromConfig,toConfig);

    Node* ptrToNode(NULL);

    Node* neigh = m_Graph->nearestWeightNeighbour(expansionNode,toConfig,
                                                  ENV.getInt(Env::DistConfigChoice));

    if( neigh->getConfiguration()->dist(*toConfig) < 2*step()/3 )
    {
        return ptrToNode;
    }

    //	if (transitionTest(*expansionNode, extensionLocalpath))
    //	{
    if( extensionLocalpath.isValid() )
    {
        //		cout << "Adding node" << endl;

        Node* newNode = addNode(expansionNode, extensionLocalpath,
                                pathDelta, ptrToNode,
                                nbCreatedNodes);

        //			double sum = newNode->sumCost();
        //
        //			cout << "Sum = " << newNode->sumCost() << endl;
        //			cout << "Cost = " << newNode->getCost() << endl;
        newNode->setSelectCost( newNode->sumCost() );

        //		cout << "nbCreatedNodes = " << nbCreatedNodes << endl;
        return newNode;
    }
    //	}
    else
    {
        return ptrToNode;
    }
}

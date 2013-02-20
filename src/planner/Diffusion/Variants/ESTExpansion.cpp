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
shared_ptr<Configuration> ESTExpansion::getExpansionDirection(
		Node* expansionNode, Node* toComp)
{
	shared_ptr<Configuration> q;

	q = m_Graph->getRobot()->shoot(true);
	return q;

}

/**
 * EST
 */
Node* ESTExpansion::expandProcessEST( Node* expansionNode,
                                             MOVE3D_PTR_NAMESPACE::shared_ptr<Configuration> directionConfig,
                                             int& nbCreatedNodes)
{
    shared_ptr<Configuration> fromConfig = expansionNode->getConfiguration();
    shared_ptr<Configuration> toConfig;

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

    Node* neigh = m_Graph->nearestWeightNeighbour(expansionNode,toConfig,false,
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

/*
 *  StartExpansion.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 31/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "StarExpansion.hpp"

#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

using namespace std;
using namespace tr1;

//using namespace Eigen;

/*!
 * Constructors with default values
 */
StarExpansion::StarExpansion() : RRTExpansion()
{
	m_K_Nearest = 10;
}

/*!
 * Constructors with default values
 */
StarExpansion::StarExpansion(Graph* G) : RRTExpansion(G)
{
	m_K_Nearest = 10;
}

/*!
 * Destructor
 */
StarExpansion::~StarExpansion()
{
	
}

/*!
 * Expand the localpath
 */
bool StarExpansion::expandToGoal(Node* expansionNode, 
																 shared_ptr<Configuration> directionConfig)
{
	return false;
}

/*!
 * Connect expansion method
 */
int StarExpansion::connectExpandProcess(Node* expansionNode, 
																				std::tr1::shared_ptr<Configuration> directionConfig, 
																				Node* directionNode)
{
	cout << "StarExpansion::connectExpandProcess Not implemented" << endl;
	return 0;
}

/*!
 * Extend expansion method
 *
 * The expansion node is the node from the 
 * graph that has been selected by the voronoi bias
 */
int StarExpansion::extendExpandProcess(Node* expansionNode, 
																			 std::tr1::shared_ptr<Configuration> directionConfig, 
																			 Node* directionNode)
{
	bool failed(false);
	int nbCreatedNodes(0);
	
	LocalPath directionLocalpath(expansionNode->getConfiguration(), 
															 directionConfig);
	
	/*
	// Expansion control
	// Discards potential nodes that are to close to the graph
	if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,
																												pathDelta(directionLocalpath), 
																												*expansionNode))
	{
		cout << "Failed expandControl test in " << __func__ << endl;
		return 0;
	}*/
	
	// Perform extension toward directionConfig
	// Construct a smaller path from direction path
	LocalPath extensionLocalpath = getExtensiontPath (expansionNode->getConfiguration(), 
																										directionConfig );
	
	Node* NodeNew = NULL;
	
	if ( extensionLocalpath.isValid() )
	{
		// Create new node and add it to the graph
		NodeNew = new Node(mGraph,extensionLocalpath.getEnd());
		nbCreatedNodes++;
		cout << "NodeNew : " << NodeNew << endl;
		cout << "Dist From expansionNode" << extensionLocalpath.getParamMax() <<  endl ;
		
		vector<Node*> NodesNear = mGraph->KNearestWeightNeighbour(NodeNew->getConfiguration(),
																															m_K_Nearest,
																															3*step(),
																															false,
																															ENV.getInt(Env::DistConfigChoice));
		// The normal RRT procedure
		Node* NodeMin = expansionNode;
		
		// Compute the node that minimizes the Sum of Cost
		for (unsigned int i=0; i<NodesNear.size(); i++) 
		{
			cout << "NodesNear[i] : " << NodesNear[i] 
			<<  " , d : " << NodesNear[i]->dist(NodeNew) << endl;
			
			LocalPath path(NodesNear[i]->getConfiguration(),NodeNew->getConfiguration());
			
			if ( path.isValid()  &&
				(	NodeNew->sumCost()  >  NodesNear[i]->sumCost() + path.cost() ) )
			{
				NodeMin = NodesNear[i];
			}
		}
		
		// Add edge in graph between the node of minimial
		// integral of cost with the new node
//		cout << "NodeMin : " << NodeMin << endl;
//		cout << "NodeNew : " << NodeNew << endl;
//		NodeMin->getConfiguration()->print();
//		NodeNew->getConfiguration()->print();
		
		mGraph->addNode(NodeNew);
		
		mGraph->addEdges(NodeMin,NodeNew,NULL,true);
		NodeNew->merge(NodeMin);
		
		LocalPath path(NodeMin->getConfiguration(),
									 NodeNew->getConfiguration());
		
		NodeNew->sumCost() = NodeMin->sumCost() + path.cost();
		
		// Remove the edge
		for (unsigned int i=0; i<NodesNear.size(); i++) 
		{
			if ( NodesNear[i] == NodeMin ) continue;
			
			LocalPath path(NodesNear[i]->getConfiguration(),
										 NodeNew->getConfiguration());
			
			// Compute the cost to get to the node
			// by each neighbor to the new node and change edge if superior
			if ( path.isValid()  && 
					( NodesNear[i]->sumCost()  >  NodeNew->sumCost() + path.cost() ) )
			{
				cout << "Removing edge" << endl;
				mGraph->removeEdges(NodesNear[i],NodesNear[i]->parent());
				mGraph->addEdges(NodesNear[i],NodeNew,path.getParamMax());
				NodesNear[i]->sumCost() = NodeNew->sumCost() + path.cost();
			}
		}
	}
	else 
	{
		failed = true;
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		// Components were merged
		if(( directionNode != NULL )&&( NodeNew == directionNode ))
		{
			cout << "Connected in Transition" << __func__ << endl;
			return 0;
		}
	}
	else
	{
		expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
	
}

/*!
 * expandProcess
 */
int StarExpansion::expandProcess(Node* expansionNode,
																 shared_ptr<Configuration> directionConfig,
																 Node* directionNode,
																 Env::expansionMethod method)
{
	switch (method) 
	{
		case Env::Connect:
			
			return connectExpandProcess(expansionNode,
																	directionConfig,
																	directionNode);
			
		case Env::Extend:
			
			return extendExpandProcess(expansionNode,
																 directionConfig,
																 directionNode);
		default:
			cerr << "Error : expand process not implemented" << endl;
			return 0;
	} 
}
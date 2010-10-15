/*
 *  CostmapExpansion.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CostmapExpansion.hpp"

#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/node.hpp"

using namespace std;
using namespace tr1;

//using namespace Eigen;

/*!
 * Constructors with default values
 */
CostmapExpansion::CostmapExpansion() : RRTExpansion()
{
	
}

/*!
 * Constructors with default values
 */
CostmapExpansion::CostmapExpansion(Graph* G) : RRTExpansion(G)
{
	
}

/*!
 * Destructor
 */
CostmapExpansion::~CostmapExpansion()
{
	
}

/*!
 * Expand the localpath
 */
bool CostmapExpansion::expandToGoal(Node* expansionNode, shared_ptr<Configuration> directionConfig)
{
	LocalPath extensionLocalpath(expansionNode->getConfiguration(), 
															 directionConfig);
		
	double costOfNode = expansionNode->sumCost() + extensionLocalpath.cost();
	
	if ( costOfNode > m_lastBestPath )
	{
//		cout << "expansionNode->sumCost = "						<< expansionNode->sumCost() << endl;
//		cout << "extensionLocalpath.cost() = "				<< extensionLocalpath.cost() << endl;
//		cout << "Failed because of cost sup to : ( "	<< m_lastBestPath << " ) " << endl;
		return false;
	}
	
	return true;
}

/*!
 * Connect expansion method
 */
int CostmapExpansion::connectExpandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode)
{
	bool failed(false);
	int nbCreatedNodes(0);
	Node* extensionNode(NULL);
	
	// Perform extension toward directionConfig
	LocalPath directionLocalpath(expansionNode->getConfiguration(), directionConfig);
	
	double delta = this->pathDelta(directionLocalpath);
	
	shared_ptr<Configuration> ptrConf;
	
	double param = directionLocalpath.whenCostIntegralPasses( m_lastBestPath*0.80 );
	
	LocalPath extensionLocalpath(directionLocalpath.getBegin(),
															 directionLocalpath.configAtParam(param));
	
	// Expansion control
	// Discards potential nodes that are to close to the graph
	if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,
																												delta, *expansionNode))
	{
		//cout << "Failed expandControl test in " << __func__ << endl;
		return 0;
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		extensionNode = addNode(expansionNode, 
														extensionLocalpath, 
														delta,directionNode, nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ) )
		{
			// Components were merged
			return 0;
		}
	}
	else
	{
		//cout << "Expansion failed in " << __func__ << endl;
		this->expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
}

/*!
 * Extend expansion method
 */
int CostmapExpansion::extendExpandProcess(Node* expansionNode, 
																					std::tr1::shared_ptr<Configuration> directionConfig,
																					Node* directionNode)
{
	bool failed(false);
	int nbCreatedNodes(0);
	Node* extensionNode(NULL);
	
	// Perform extension toward directionConfig
	LocalPath directionLocalpath(expansionNode->getConfiguration(), directionConfig);
	
	double delta = this->pathDelta(directionLocalpath);
	
	shared_ptr<Configuration> ptrConf;
	
	if( (delta==1)&& directionNode )
	{
		ptrConf = directionNode->getConfiguration();
	}
	else
	{
		ptrConf = directionLocalpath.configAtParam( delta * directionLocalpath.getParamMax() );
	}

	
	LocalPath extensionLocalpath(directionLocalpath.getBegin(),ptrConf);
	
	// Expansion control
	// Discards potential nodes that are to close to the graph
	if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,
																												delta, *expansionNode))
	{
		//cout << "Failed expandControl test in " << __func__ << endl;
		return 0;
	}
	
	// Transition test and collision check
	if (ENV.getBool(Env::costBeforeColl))
	{
		double costOfNode = expansionNode->sumCost() + extensionLocalpath.cost();
		
		if( costOfNode  > m_lastBestPath )
		{
//			cout << "expansionNode->sumCost = "						<< expansionNode->sumCost() << endl;
//			cout << "extensionLocalpath.cost() = "				<< extensionLocalpath.cost() << endl;
//			cout << "Failed because of cost sup to : ( "	<< m_lastBestPath << " ) " << endl;
			
			failed = true;
		}
		
		if (!failed)
		{
			if (!extensionLocalpath.isValid())
			{
				failed = true;
			}
		}
	}
	else
	{
		if (!extensionLocalpath.isValid())
		{
			failed = true;
		}
		if (!failed)
		{
			double costOfNode = expansionNode->sumCost() + extensionLocalpath.cost();
			
			if( costOfNode > m_lastBestPath )
			{
				failed = true;
			}
		}
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		//cout << "Expansion succeded in " << __func__ << endl;
		extensionNode = addNode(expansionNode, 
														extensionLocalpath, 
														delta,directionNode, nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ) )
		{
			// Components were merged
			//cout << "Connected in Transition" << __func__ << endl;
			return 0;
		}
	}
	else
	{
		//cout << "Expansion failed in " << __func__ << endl;
		this->expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
}

/*!
 * expandProcess
 */
int CostmapExpansion::expandProcess(Node* expansionNode,
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

/*
 *  ThresholdExpansion.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "ThresholdExpansion.hpp"

#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/node.hpp"

using namespace std;
using namespace tr1;

//using namespace Eigen;

/*!
 * Constructors with default values
 */
ThresholdExpansion::ThresholdExpansion() : RRTExpansion()
{
	
}

/*!
 * Constructors with default values
 */
ThresholdExpansion::ThresholdExpansion(Graph* G) : RRTExpansion(G)
{
	
}

/*!
 * Destructor
 */
ThresholdExpansion::~ThresholdExpansion()
{
	
}

/*!
 * Expand the localpath
 */
bool ThresholdExpansion::expandToGoal(Node* expansionNode, shared_ptr<Configuration> directionConfig)
{
    LocalPath directionLocalPath(expansionNode->getConfiguration(), directionConfig);
	const double paramMax = directionLocalPath.getParamMax();
	
	double param = 0;
    for ( int i = 1; param < paramMax; i++)
    {
		shared_ptr<Configuration> q;
        param = ((double) i) * step();
		
        if (param > paramMax)
        {
            q = directionConfig;
        }
        else
        {
            q = directionLocalPath.configAtParam(param);
        }
			
		if ( q->cost() > m_threshold )
		{
                return false;
		}
    }
	
    return true;
}

/*!
 * Connect expansion method
 */
int ThresholdExpansion::connectExpandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode)
{
	cout << "ThresholdExpansion::connectExpandProcess Not implemented" << endl;
	return 0;
}

/*!
 * Extend expansion method
 */
int ThresholdExpansion::extendExpandProcess(Node* expansionNode, std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode)
{
	bool failed(false);
    int nbCreatedNodes(0);
    Node* extensionNode(NULL);
	
	// Perform extension toward directionConfig
    LocalPath directionLocalpath(expansionNode->getConfiguration(), directionConfig);
	
    double pathDelta = directionLocalpath.getParamMax() == 0. ? 1 : MIN(1., step() / directionLocalpath.getParamMax() );
	
    LocalPath extensionLocalpath(directionLocalpath.getBegin(), pathDelta == 1.
                                 && directionNode ? directionNode->getConfiguration()
								 : directionLocalpath.configAtParam(pathDelta * directionLocalpath.getParamMax()));
	
    // Expansion control
    // Discards potential nodes that are to close to the graph
    if (ENV.getBool(Env::expandControl) && !expandControl(directionLocalpath,
                                                          pathDelta, *expansionNode))
    {
		//		cout << "Failed expandControl test in " << __func__ << endl;
        return 0;
    }
	
    // Transition test and collision check
    if (ENV.getBool(Env::costBeforeColl))
    {
		if( extensionLocalpath.getEnd()->cost() > m_threshold )
		{
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
			if( extensionLocalpath.getEnd()->cost() > m_threshold )
			{
				failed = true;
			}
		}
	}
	
	// Add node to graph if everything succeeded
	if (!failed)
	{
		extensionNode = addNode(expansionNode, 
								extensionLocalpath, 
								pathDelta,
								directionNode, nbCreatedNodes);
		
		if ( ( directionNode != NULL )&&( extensionNode == directionNode ))
		{
			// Components were merged
			cout << "Connected in Transition" << __func__ << endl;
			return 0;
		}
	}
	else
	{
		this->expansionFailed(*expansionNode);
	}
	
	return nbCreatedNodes;
	
}

/*!
 * expandProcess
 */
int ThresholdExpansion::expandProcess(Node* expansionNode,
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
			cerr << "Error : expand process not omplemented" << endl;
			return 0;
	} 
}

/*
 *  CostGraph.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "CostGraph.hpp"

CostGraph::CostGraph() : Graph()
{
	
}

CostGraph::CostGraph(const Graph& G) : Graph(G)
{
	
}

CostGraph::~CostGraph()
{
	
}

Graph* getGraphUnderThreshold(double thresh)
{
	for (unsigned int i=0; i<_Edges.size(); i++) 
	{
		if( isEdgeUnderThreshold(*_Edges[i]) )
		{
			
		}
	}
}

vector<LocalPath*> getLocalPathsUnderThreshold(const LocalPath& LP)
{

}

bool CostGraph::isEdgeUnderThreshold(const Edge& E)
{
	shared_ptr<LocalPath> path = E.getLocalPath();
	const double DeltaStep = path.getResolution();
	const unsigned int nStep = path.getParamMax() / DeltaStep;
	
	double currentParam = 0;
	
	for (unsigned int i = 0; i < nStep; i++)
	{
		currentParam += DeltaStep;
		if( path->configAtParam(currentParam)->cost() > m_thresh )
		{
			return false;
		}
	}
}

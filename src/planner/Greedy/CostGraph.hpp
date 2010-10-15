/*
 *  CostGraph.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef COST_GRAPH_H_
#define COST_GRAPH_H_

#include "API/Device/robot.hpp"
#include "API/ConfigSpace/localpath.hpp"
#include "API/Roadmap/Graph.hpp"

class CostGraph : class Graph 
{
public:
	CostGraph(Robot* R);
	CostGraph(Robot* R, p3d_graph* G);
	CostGraph(const Graph& G);
	~CostGraph();
	
	void setThreshold(const double& thresh) { m_thresh = thresh; }
	double getThreshold() { return m_thresh; }
	
	Graph* getGraphUnderThreshold();
	vector<LocalPath*> getLocalPathsUnderThreshold(const LocalPath& LP);
	
private:
	double m_thresh;
};

#endif
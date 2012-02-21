/*
 *  compco.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"

#include "../p3d/env.hpp"

#include "Planner-pkg.h"

#include <iterator>

using namespace std;
using namespace tr1;

ConnectedComponent::ConnectedComponent(Graph* G, p3d_compco* C) :
m_Compco(C),
m_Graph(G),
m_Id(-1)
{
	cout << "Warning : constructor out of use ConnectedComponent::ConnectedComponent(Graph* G, p3d_compco* C)" << endl;
	//throw string("Constructor out of use ConnectedComponent::ConnectedComponent(Graph* G, p3d_compco* C)");
}


ConnectedComponent::ConnectedComponent(Graph* G, Node* N) :
m_Graph(G),
m_Id(-1)
{
#ifdef P3D_PLANNER
  m_Compco = MY_ALLOC(p3d_compco, 1);
  m_Compco->num = G->getNumberOfCompco();
  
  // Warning the compco are handled in the boost graph
	// p3d_create_compco(m_Graph->getGraphStruct(), N->getNodeStruct());
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
	// m_Compco = m_Graph->getGraphStruct()->last_comp;
	// cout << "new compco : " << m_Compco << endl;
	addNode(N);
	//cerr << "Add Compco for Node " << N->getNodeStruct() << endl;
}

ConnectedComponent::~ConnectedComponent()
{
  //cout << "Delete compoc num : " <<  m_Compco->num << endl;
  free( m_Compco );
}

p3d_compco* ConnectedComponent::getCompcoStruct()
{
	//cout << "ConnectedComponent is : " << m_Compco << endl;
	return m_Compco;
}

unsigned int ConnectedComponent::getId() const
{
  return m_Compco->num;
}

void ConnectedComponent::addNode(Node* N)
{
	m_Nodes.push_back(N);
	N->setConnectedComponent(this);
}

unsigned int ConnectedComponent::getNumberOfNodes() const
{
	return m_Nodes.size();
}


std::vector<Node*>& ConnectedComponent::getNodes()
{
	/*cout << "--------------------------------------" << endl;
   vector<Node*>::iterator it;
   for (it = m_Nodes.begin(); it != m_Nodes.end(); ++it) 
   {
   cout << "Node num : " << (*it)->getId() << endl;
   }*/
	return m_Nodes;
}

void ConnectedComponent::addToReachableList(ConnectedComponent* Comp)
{
	// This function is a copy of
	// p3d_add_compco_to_reachable_list
	m_CanReach.push_back(Comp);
}

//! This function is a copy of p3d_compco_linked_to_compco
bool ConnectedComponent::isLinkedToCompco(ConnectedComponent* Comp)
{
	if( find (m_CanReach.begin(), m_CanReach.end(), Comp ) == m_CanReach.end() )
		return false;
	else
		return true;
}

void ConnectedComponent::addToReachableListAndUpdatePredecessors(ConnectedComponent* compco_add)
{
	// This function is a copy of
	// p3d_add_compco_to_reachable_list_and_update_predecessors
	
	m_CanReach.push_back(compco_add);
	
	vector<ConnectedComponent*> v = m_Graph->getConnectedComponents();
	vector<ConnectedComponent*>::iterator compco;
	
	for (compco = v.begin(); compco!=v.end(); ++compco) 
	{
		if( (*compco)->isLinkedToCompco( this ) )
		{
			(*compco)->addToReachableList( compco_add );
		}
	}
}

//! This function is a copy of p3d_merge_comp which updates the 
//! graph and the connected components
void ConnectedComponent::mergeWith(ConnectedComponent* compco)
{
	// The nodes of C2 are now in C1 
	for(int i=0;i<int(compco->m_Nodes.size());i++)
	{
		Node* tmpNode = compco->m_Nodes[i];
		tmpNode->setConnectedComponent( this ); 
    addNode( tmpNode );
	}
	
	// All the compcos that can reach C2 can now reach C1 
	if (m_Graph->getGraphStruct()->oriented) 
	{
		cout << "Oriented" << endl;
    p3d_list_compco * ListCompcoScan;
		ListCompcoScan = compco->getCompcoStruct()->canreach;
		while (ListCompcoScan != NULL) 
		{
			p3d_add_compco_to_reachable_list_and_update_predecessors(m_Graph->getGraphStruct(), getCompcoStruct(), ListCompcoScan->comp);
			ListCompcoScan = ListCompcoScan->next;
		}
	}
	
	// C2 is deleted from the graph 
  // and throw an exception if the p3d and C++ are not the same
	m_Graph->removeCompco( compco );
	getNumberOfNodes();
	
	if (m_Graph->getGraphStruct()->oriented)
	{
		m_Graph->mergeCheck();
	}
}

Node* ConnectedComponent::nearestWeightNeighbour(shared_ptr<Configuration> q, bool weighted, int distConfigChoice)
{
	double current_dist, current_score;
	double best_score = numeric_limits<double>::max();
	Node* BestNodePt = NULL;
	Node* node = NULL;
  
	for (int i=0; i<int(m_Nodes.size()); i++) 
  {
    node = m_Nodes[i];
    
		// We take into account only the undiscarded nodes  
		if (!node->getNodeStruct()->IsDiscarded)
		{
			current_dist = q->dist( *node->getConfiguration(), distConfigChoice );
			current_score = current_dist * (weighted ? p3d_GetNodeWeight(node->getNodeStruct()) : 1.0);
			
			if (current_score < best_score)
			{
				best_score = current_score;
				BestNodePt = node;
			}
		}
	}
	
	return BestNodePt;
}

Node* ConnectedComponent::searchConf(Configuration& q) {
  
  for(int i=0;i<int(m_Nodes.size());i++)
  {
    if(m_Nodes[i]->getConfiguration()->equal(q))
    {
      return m_Nodes[i];
    }
  }
  return NULL;
}

//
//  p3dGraphConverter.cpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/01/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#include "graphConverter.hpp"
#include "API/Roadmap/compco.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

#include <iostream>

using namespace std;

const bool graph_debug_import_export = false;

/**
 * Fonction that imports a Graph
 */
Graph* GraphConverter::convert(p3d_graph* G) const
{
  /*
	if( graph_debug_import_export )
	{
		cout << "Importing the graph" << endl;
	}
	
	robot = global_Project->getActiveScene()->getRobotByName(G->rob->name);
	m_Graph = G;
	
	if (G->nodes)
	{
		p3d_list_node* l = G->nodes;
		while (l)
		{
			nodes.push_back(new Node(this, l->N));
			l = l->next;
		}
	}
	if (G->edges)
	{
		p3d_list_edge* l = G->edges;
		while (l)
		{
			edges.push_back(new Edge(this, 
																 static_cast<unsigned int>(l->E->Ni->num - 1 ), 
																 static_cast<unsigned int>(l->E->Nf->num - 1 ) ));
			
			//cout << "importGraphStruct : Edge = " << l->E << endl;
			
			l = l->next;
		}
	}
	if (G->comp)
	{
		p3d_compco* l = G->comp;
		while (l)
		{
			compcos.push_back(new ConnectedComponent(this, l));
			l = l->suiv;
		}
	}
	
	this->setName();*/
  return NULL;
}

/**
 * Fonction exports a copy of the p3d_graph contained in
 * the Graph class suposing that it is valid regarding connexity
 */
p3d_graph* GraphConverter::exportGraphStruct(const Graph& g) const
{
	if( graph_debug_import_export )
	{
		cout << "Exporting the Graph structure" <<  endl;
	}
  
  Robot* robot = g.getRobot();
  vector<Node*> nodes = g.getNodes();
  vector<Edge*> edges = g.getEdges();
  vector<ConnectedComponent*> compcos = g.getConnectedComponents();
  
	p3d_graph* G = new p3d_graph(*g.getGraphStruct());
	
	G->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	G->rob = robot->getRobotStruct();
	
	robot->getRobotStruct()->GRAPH = G;
	
	G->nnode = static_cast<int>(nodes.size());
	G->nedge = static_cast<int>(edges.size());
	
#if P3D_PLANNER
	if (STAT)
	{
		G->stat = createStat();
	}
	else
	{
		G->stat = NULL;
	}
#endif
	
	//---------------------------------------------------------------------
	// Copy all nodes in the nodes list
	if( graph_debug_import_export )
	{
		//cout << "Graph G : " << G <<  endl;
		cout << "nodes.size() = " << nodes.size() << endl;
	}
	
	// New table of nodes
	// Old node to new nodes
	map<p3d_node*,p3d_node*> NodeMap;
	
	p3d_list_node* ln = NULL;
	p3d_list_node* prevNode = NULL;
	
	for (unsigned int i = 0; i < nodes.size(); i++) 
	{
		// Creates a new copied p3d_node
		ln = new p3d_list_node;
		ln->N = new p3d_node(*nodes[i]->getNodeStruct());
		ln->N->q = p3d_copy_config(robot->getRobotStruct(),ln->N->q);
		cout << "Node( " << ln->N->num << " ) = " << ln->N->numcomp << endl;
		
		//cout <<  "ln->N->num = " << ln->N->num << endl;
		
		// Adds the new node to the map
		NodeMap.insert(pair<p3d_node*,p3d_node*>(nodes[i]->getNodeStruct(),ln->N));
		
		if (i==0) {
			G->nodes = ln;
		}
		
		// Link the new nodes
		if (prevNode) {
			prevNode->next = ln;
		}
		ln->prev = prevNode;
		prevNode = ln;
	}
	if (ln) {
		ln->next = NULL;
	}
	G->last_node = ln;
	
	
	//---------------------------------------------------------------------
	// Copy all edges
	if( graph_debug_import_export )
	{
		cout << "Graph G : " << G <<  endl;
		cout << "medges.size() = " << edges.size() << endl;
	}
	
	// New table of edges
	// Old edges to new edges
	map<p3d_edge*,p3d_edge*> EdgeMap;
	
	p3d_list_edge* le = NULL;
	p3d_list_edge* prevEdge = NULL;
	
	for (unsigned int i = 0; i < edges.size(); i++) 
	{
		// Creates new edge
		le = new p3d_list_edge;
		le->E = new p3d_edge(*edges[i]->getEdgeStruct());
		
		// Change the p3d_node pointers
		le->E->Ni = NodeMap[edges[i]->getSource()->getNodeStruct()];
		le->E->Nf = NodeMap[edges[i]->getTarget()->getNodeStruct()];
		
		// Adds the new Edge to the map
		EdgeMap.insert(pair<p3d_edge*,p3d_edge*>(edges[i]->getEdgeStruct(),le->E));
		
		if (i==0) {
			G->edges = le;
		}
		
		if (prevEdge) {
			prevEdge->next = le;
		}
		le->prev = prevEdge;
		prevEdge = le;
		
		//cout << "New Edges in list" << i << endl;
	}
	if (le) {
		le->next = NULL;
	}
	G->last_edge = le;
	
	//---------------------------------------------------------------------
	// Copy connected components
	if( graph_debug_import_export || true )
	{
		//cout << "Graph G : " << G <<  endl;
		cout << "compcos.size() = " << compcos.size() << endl;
	}
	
	// New table of compco
	// Old compco to new compco
	map<p3d_compco*,p3d_compco*> CompMap;
	
	if( ! compcos.empty() )
	{
		p3d_compco* lc;
		p3d_compco* prevComp = NULL;
		
		for (unsigned int i = 0; i < compcos.size(); i++) 
		{
			lc = new p3d_compco(*compcos[i]->getCompcoStruct());
			
			// Adds the new Compco to the map
			CompMap.insert(pair<p3d_compco*,p3d_compco*>(compcos[i]->getCompcoStruct(),lc));
			
			// Copy node list of the compco
			lc->nodes					= copyNodeList(NodeMap,lc->nodes);
			lc->dist_nodes		= copyNodeList(NodeMap,lc->dist_nodes);
			
			//cout << "Nb : Can reach = " << lc->ncanreach <<  endl;
			
			// Warning size may change!!! see p3d file
			lc->AnaSuccessTab = new int[100];
			memcpy((void*)lc->AnaSuccessTab,(void*)compcos[i]->getCompcoStruct()->AnaSuccessTab,100*sizeof(int));
			
			if (i==0){
				G->comp = lc;
			}
			
			if (prevComp){
				prevComp->suiv = lc;
			}
			lc->prec = prevComp;
			prevComp = lc;
		}
		lc->suiv = NULL;
		G->last_comp = lc;
	}
	
	//---------------------------------------------------------------------
	// Copy all nodes neighbours, last node, last edge
	ln = G->nodes;
	for (unsigned int i = 0; i < nodes.size(); i++) 
	{
		ln->N->neighb =				copyNodeList( NodeMap, ln->N->neighb , ln->N->last_neighb );
		ln->N->edges =				copyEdgeList( EdgeMap, ln->N->edges , ln->N->last_edge );
		
		ln->N->parent =				NodeMap[ln->N->parent];
		ln->N->search_from =	NodeMap[ln->N->search_from];
		ln->N->search_from =	NodeMap[ln->N->search_to];
		ln->N->edge_from =		EdgeMap[ln->N->edge_from];
		ln->N->comp	=					CompMap[ln->N->comp];
		
		ln = ln->next;
	}
	//G->lastm_Edge = lc
	
	return G;
}

//! copy a node list
//! @param the map from old nodes to new nodes
p3d_list_node* GraphConverter::copyNodeList(map<p3d_node*,p3d_node*>& NodeMap, p3d_list_node* ln, p3d_list_node* end) const
{	
	if (!ln) {
		if (end) {
			cout << "ERROR in  Graph::copyNodeList" << endl;
		}
		return NULL;
	}
	
	// Assume the first pointer is the first element
	p3d_list_node* newList = new p3d_list_node;
	newList->N = NodeMap[ln->N];
	newList->prev = NULL;
	
	p3d_list_node* tmpNodeListItem1 = newList;
	
	while (ln) 
	{
		if( ln->next )
		{
			p3d_list_node* tmpNodeListItem2 = new p3d_list_node;
			tmpNodeListItem2->N = NodeMap[ln->next->N];
			tmpNodeListItem2->prev = tmpNodeListItem1;
			tmpNodeListItem1->next = tmpNodeListItem2;
			tmpNodeListItem1 = tmpNodeListItem2;
		}
		else 
		{
			tmpNodeListItem1->next = NULL ;
			
			if( end )
			{
				end = tmpNodeListItem1;
			}
		}
		
		ln = ln->next;
	}
	
	// Test
	tmpNodeListItem1 = newList;
	
	while (tmpNodeListItem1) {
		//cout << "Node = " << tmpNodeListItem1->N << endl;
		tmpNodeListItem1 = tmpNodeListItem1->next;
	}
	
	return newList;
}

//! copy a edge lisyt
//! @param the map from old edges to new edges
p3d_list_edge* GraphConverter::copyEdgeList(map<p3d_edge*,p3d_edge*>& EdgeMap, p3d_list_edge* le, p3d_list_edge* end) const 
{
	if (!le) {
		if (end) {
			cout << "ERROR in  Graph::copyEdgeList" << endl;
		}
		return NULL;
	}
	
	// Assume the first pointer is the first element
	p3d_list_edge* newList = new p3d_list_edge;
	newList->E = EdgeMap[le->E];
	newList->prev = NULL;
	
	p3d_list_edge* tmpEdgeListItem1 = newList;
	
	while (le) 
	{
		if( le->next )
		{
			p3d_list_edge* tmpEdgeListItem2 = new p3d_list_edge;
			tmpEdgeListItem2->E = EdgeMap[le->next->E];
			tmpEdgeListItem2->prev = tmpEdgeListItem1;
			tmpEdgeListItem1->next = tmpEdgeListItem2;
			tmpEdgeListItem1 = tmpEdgeListItem2;
		}
		else 
		{
			tmpEdgeListItem1->next = NULL;
			
			if( end )
			{
				end = tmpEdgeListItem1;
			}
		}
		
		le = le->next;
	}
	
	// Test
	tmpEdgeListItem1 = newList;
	
	while (tmpEdgeListItem1) {
		//cout << "Edge = " << tmpEdgeListItem1->E << endl;
		tmpEdgeListItem1 = tmpEdgeListItem1->next;
	}
	
	return newList;
}

//! Updates the list of connected
//! components from the p3d_graph structure
/*void GraphConverter::updateCompcoFromStruct()
{
	compcos.clear();
	
	if (m_Graph->comp)
	{
		p3d_compco* l = m_Graph->comp;
		while (l)
		{
			compcos.push_back(new ConnectedComponent(this, l));
			l = l->suiv;
		}
	}
}*/


/**
 * This function creates a graph from the Cpp graph
 */
p3d_graph* GraphConverter::convert(const Graph& g, bool deleteGraphStruct) const
{
	if( graph_debug_import_export )
	{
		cout << "Exporting the Graph structure" <<  endl;
	}

	p3d_graph* G = new p3d_graph(*g.getGraphStruct());
  
  vector<Node*> nodes = g.getNodes();
  vector<Edge*> edges = g.getEdges();
  vector<ConnectedComponent*> compcos = g.getConnectedComponents();
  
  Robot* robot = g.getRobot();
  
	
	if (deleteGraphStruct) 
	{
		//p3d_del_graph(m_Graph);
	}
	
	G->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	G->rob = robot->getRobotStruct();
	
	robot->getRobotStruct()->GRAPH = G;
	
	G->nnode = static_cast<int>(nodes.size());
	G->nedge = static_cast<int>(edges.size());
	
#if P3D_PLANNER
	if (STAT)
	{
		G->stat = createStat();
	}
	else
	{
		G->stat = NULL;
	}
#endif
	
	unsigned int i;
	//---------------------------------------------------------------------
	// Copy all nodes in the nodes list
	if( graph_debug_import_export )
	{
		//cout << "Graph G : " << G <<  endl;
		cout << "nodes.size() = " << nodes.size() << endl;
	}
	
	// New table of nodes
	// Old node to new nodes
	map<Node*,p3d_node*> NodeMap;
	
	p3d_list_node* ln = NULL;
	p3d_list_node* prevNode = NULL;
	
	i=0;
	for (vector<Node*>::iterator it = nodes.begin() ; it != nodes.end(); ++it) 
	{
		// Creates a new copied p3d_node
		ln = new p3d_list_node;
		ln->N = new p3d_node(*(*it)->getNodeStruct()); i++;
		
		ln->N->q = p3d_copy_config(robot->getRobotStruct(),
															 (*it)->getConfiguration()->getConfigStruct());
		
		//cout << "Node( " << ln->N->num << " ) = " << ln->N->numcomp << endl;
		
		//cout <<  "ln->N->num = " << ln->N->num << endl;
		
		// Adds the new node to the map
		NodeMap.insert(pair<Node*,p3d_node*>((*it),ln->N));
		
		if (it==nodes.begin()) {
			G->nodes = ln;
		}
		
		// Link the new nodes
		if (prevNode) {
			prevNode->next = ln;
		}
		ln->prev = prevNode;
		prevNode = ln;
	}
	if (ln) {
		ln->next = NULL;
	}
	G->last_node = ln;
	
	if (i != nodes.size()) 
	{
		throw string("The number of nodes exporting is wrong");
	}
	
	//---------------------------------------------------------------------
	// Copy all edges
	if( graph_debug_import_export )
	{
		cout << "Graph G : " << G <<  endl;
		cout << "edges.size() = " << edges.size() << endl;
	}
	
	// New table of edges
	// Old edges to new edges
	map<Edge*,p3d_edge*> EdgeMap;
	
	p3d_list_edge* le = NULL;
	p3d_list_edge* prevEdge = NULL;
	i=0;
	for (vector<Edge*>::iterator it = edges.begin() ; it != edges.end(); ++it) 
	{
		// Creates new edge
		le = new p3d_list_edge;
		//cout << "export edge : " << (*it) << endl;
		le->E = new p3d_edge(*(*it)->getEdgeStruct()); i++;
		
		// Change the p3d_node pointers
		le->E->Ni = NodeMap[(*it)->getSource()];
		le->E->Nf = NodeMap[(*it)->getTarget()];
		
		// Copy the localpath struct
		le->E->path = (*it)->getLocalPath()->getLocalpathStruct()->copy(
																																		(*it)->getLocalPath()->getRobot()->getRobotStruct(),
																																		(*it)->getLocalPath()->getLocalpathStruct());
		
		// Adds the new Edge to the map
		EdgeMap.insert(pair<Edge*,p3d_edge*>((*it),le->E));
		
		if (it == edges.begin()) 
		{
			G->edges = le;
		}
		
		if (prevEdge) {
			prevEdge->next = le;
		}
		le->prev = prevEdge;
		prevEdge = le;
		
		//cout << "New Edges in list" << i << endl;
	}
	if (le) {
		le->next = NULL;
	}
	G->last_edge = le;
	
	if (i != edges.size()) 
	{
		throw string("The number of edges exporting is wrong");
	}
	//---------------------------------------------------------------------
	// Copy connected components
	if( graph_debug_import_export  )
	{
		//cout << "Graph G : " << G <<  endl;
		cout << "compcos.size() = " << compcos.size() << endl;
	}
	
	// New table of compco
	// Old compco to new compco
	map<ConnectedComponent*,p3d_compco*> CompMap;
	
	G->ncomp = compcos.size();
	
	if( ! compcos.empty() )
	{
		p3d_compco* lc;
		p3d_compco* prevComp = NULL;
		
		for (unsigned int i = 0; i < compcos.size(); i++) 
		{
			// Copy all fields
			lc = new p3d_compco(*compcos[i]->getCompcoStruct());
			
			// Adds the new Compco to the map
			CompMap.insert(pair<ConnectedComponent*,p3d_compco*>(compcos[i],lc));
			
			// Copy node list of the compco
			lc->nodes					= createNodeList( NodeMap, compcos[i]->getNodes(), lc->last_node);
			lc->dist_nodes		= createNodeList (NodeMap, compcos[i]->getNodes() );
			lc->nnode					= compcos[i]->getNumberOfNodes();
			
			//cout << "Nb : Can reach = " << lc->ncanreach <<  endl;
			
			// Warning size may change!!! see p3d file
			lc->AnaSuccessTab = new int[100];
			//memcpy((void*)lc->AnaSuccessTab,(void*)compcos[i]->getCompcoStruct()->AnaSuccessTab,100*sizeof(int));
			
			if (i==0){
				G->comp = lc;
			}
			
			if (prevComp){
				prevComp->suiv = lc;
			}
			lc->prec = prevComp;
			prevComp = lc;
		}
		lc->suiv = NULL;
		G->last_comp = lc;
	}
	
	//---------------------------------------------------------------------
	// Copy all nodes neighbours, last node, last edge
	ln = G->nodes;
	for (vector<Node*>::iterator it = nodes.begin(); it != nodes.end(); ++it) 
	{
		ln->N->numcomp =			(*it)->getConnectedComponent()->getId();
		//cout << "Component num : " << ln->N->numcomp << endl;
		
		ln->N->neighb =				createNodeList( NodeMap, (*it)->getNeighbors(), ln->N->last_neighb );
		ln->N->edges =				createEdgeList(	EdgeMap, (*it)->getEdges(),  ln->N->last_edge );
		ln->N->nedge =				(*it)->getNumberOfEdges();
		//cout << "Number of Edges in node : " << ln->N->nedge << endl;
		
    // WARNING this part of the graph is not kept
//		ln->N->parent =				NodeMap[(*it)->getParent()];
//		ln->N->search_from =	NodeMap[(*it)->getSearchFrom()];
//		ln->N->search_to =		NodeMap[(*it)->getSearchTo()];
//		ln->N->edge_from =		EdgeMap[(*it)->getEdgeFrom()];
    ln->N->parent =				NULL;
		ln->N->search_from =	NULL;
		ln->N->search_to =		NULL;
		ln->N->edge_from =		NULL;
		
		ln->N->comp	=					CompMap[(*it)->getConnectedComponent()];
		
		ln = ln->next;
	}
	//G->lastm_Edge = lc;
	
	//m_graphChanged = false;
	
	if (graph_debug_import_export) 
	{
		cout << "Number of nodes : " << G->nnode << endl;
		cout << "Number of edges : " << G->nedge << endl;
		cout << "Number of compo : " << G->ncomp << endl;
	}
	return G;
}

//! create node list
p3d_list_node* GraphConverter::createNodeList(map<Node*,p3d_node*>& NodeMap, const vector<Node*>& nodes, p3d_list_node* end)  const
{
	if (nodes.empty()) 
		return NULL;
	
	vector<Node*>::const_iterator it = nodes.begin();
	vector<Node*>::const_iterator it_tmp;
	
	// Assume the first pointer is the first element
	p3d_list_node* newList = new p3d_list_node;
	newList->N = NodeMap[*it];
	newList->prev = NULL;
	
	p3d_list_node* tmpNodeListItem1 = newList;
	
	while ( it != nodes.end() ) 
	{
		it_tmp = it;
		++it_tmp;
		if( it_tmp != nodes.end() )
		{
			p3d_list_node* tmpNodeListItem2 = new p3d_list_node;
			tmpNodeListItem2->N = NodeMap[*it_tmp];
			tmpNodeListItem2->prev = tmpNodeListItem1;
			tmpNodeListItem1->next = tmpNodeListItem2;
			tmpNodeListItem1 = tmpNodeListItem2;
		}
		else 
		{
			tmpNodeListItem1->next = NULL ;
			
			if( end )
			{
				end = tmpNodeListItem1;
			}
		}
		++it;
	}
	
	// Test
	tmpNodeListItem1 = newList;
	
	while (tmpNodeListItem1) {
		//cout << "Node = " << tmpNodeListItem1->N << endl;
		tmpNodeListItem1 = tmpNodeListItem1->next;
	}
	
	return newList;
}


//! create edge list
p3d_list_edge* GraphConverter::createEdgeList(map<Edge*,p3d_edge*>& EdgeMap, const vector<Edge*>& edges, p3d_list_edge* end) const
{
	if ( edges.empty() )
		return NULL;
	
	vector<Edge*>::const_iterator it = edges.begin();
	vector<Edge*>::const_iterator it_tmp;
	
	// Assume the first pointer is the first element
	p3d_list_edge* newList = new p3d_list_edge;
	newList->E = EdgeMap[*it];
	newList->prev = NULL;
	
	p3d_list_edge* tmpEdgeListItem1 = newList;
	
	while ( it != edges.end() ) 
	{
		it_tmp = it;
		++it_tmp;
		if( it_tmp != edges.end() )
		{
			p3d_list_edge* tmpEdgeListItem2 = new p3d_list_edge;
			tmpEdgeListItem2->E = EdgeMap[*it_tmp];
			tmpEdgeListItem2->prev = tmpEdgeListItem1;
			tmpEdgeListItem1->next = tmpEdgeListItem2;
			tmpEdgeListItem1 = tmpEdgeListItem2;
		}
		else 
		{
			tmpEdgeListItem1->next = NULL;
			
			if( end )
			{
				end = tmpEdgeListItem1;
			}
		}
		++it;
	}
	
	// Test
	tmpEdgeListItem1 = newList;
	
	while (tmpEdgeListItem1) {
		//cout << "Edge = " << tmpEdgeListItem1->E << endl;
		tmpEdgeListItem1 = tmpEdgeListItem1->next;
	}
	
	return newList;
}
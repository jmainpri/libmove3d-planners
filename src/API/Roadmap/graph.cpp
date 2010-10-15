//
// C++ Implementation: graph
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/project.hpp"

#include "planEnvironment.hpp"
#include "cost_space.hpp"

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Move3d-pkg.h"

#ifdef LIGHT_PLANNER
#include "lightPlanner/proto/lightPlannerApi.h"
#endif

#include <boost/config.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/graph_utility.hpp>

Graph* API_activeGraph = NULL;

using namespace std;
using namespace tr1;

const bool graph_debug_import_export = false;

// Constructors
//----------------------------------------------

Graph::Graph(Robot* R, p3d_graph* G)
{
	if (G)
	{
		m_Graph = new p3d_graph(*G);
	}
	else
	{
		m_Graph = p3d_create_graph();
	}
	m_Robot = R;
	m_Graph->rob->GRAPH = m_Graph;
	m_Traj = NULL;
	m_graphChanged = true;
	this->init();
	this->initBGL();
}

Graph::Graph(Robot* R)
{
	m_Robot = R;
	m_Graph = p3d_allocinit_graph();
	
	m_Graph->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	m_Graph->rob = m_Robot->getRobotStruct();
	if (m_Robot->getRobotStruct() != NULL)
	{
		m_Robot->getRobotStruct()->GRAPH = m_Graph;
		XYZ_GRAPH = m_Graph;
	}
#if P3D_PLANNER
	if (STAT)
	{
		m_Graph->stat = createStat();
	}
	else
	{
		m_Graph->stat = NULL;
	}
#endif
	m_Traj = NULL;
	m_graphChanged = true;
	this->init();
	this->initBGL();
}

Graph::Graph(p3d_graph* G)
{
	if (G)
	{
		/*_Graph = MY_ALLOC(p3d_graph, 1);
		 *_Graph = *G;*/
		m_Graph = G;
	}
	else
	{
		m_Graph = p3d_create_graph();
	}
	m_Robot = global_Project->getActiveScene()->getRobotByName(G->rob->name);
	m_Traj = NULL;
	m_graphChanged = true;
	this->init();
	this->initBGL();
}

/**
 * Graph copy constructor : be sure that the connected componants are updated
 * @param G the graph
 */
Graph::Graph(const Graph& G)
{
	m_Graph = G.exportGraphStruct();
	m_Robot = G.m_Robot;
	
	shared_ptr<Configuration> q_start = G.m_Start->getConfiguration();
	shared_ptr<Configuration> q_goal =  G.m_Goal->getConfiguration();
	
	m_Start = searchConf(*q_start);
	m_Goal =  searchConf(*q_goal);
	
	m_graphChanged = true;
	
	this->init();
	this->initBGL();
}

/**
 * Fonction creant un Objet Graph a partir d'une structure de p3d_Graph
 */
void Graph::init()
{
	if (m_Graph->nodes)
	{
		p3d_list_node* l = m_Graph->nodes;
		m_Nodes.clear();
		while (l)
		{
			Node* node = new Node(this, l->N);
			m_NodesTable.insert(pair<p3d_node*, Node*>(l->N, node));
			m_Nodes.push_back(node);
			l = l->next;
		}
	}
	if (m_Graph->edges)
	{
		p3d_list_edge* l = m_Graph->edges;
		m_Edges.clear();
		while (l)
		{
			//cout << "Init : Edge = " << l->E << endl;
			Edge* edge = new Edge(this,l->E);
			m_Edges.push_back(edge);
			l = l->next;
		}
	}
	this->setName();
}

// BGL Functions
//----------------------------------------------
void Graph::initBGL()
{
	try 
	{
		BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , m_BoostGraph );
		
		for(unsigned int i = 0; i < m_Nodes.size(); ++i)
		{
			BGL_Vertex v	= boost::add_vertex(m_BoostGraph);
			NodeData[v]		= m_Nodes[i];
			m_Nodes[i]->setDescriptor(v);
		}
		
		BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , m_BoostGraph );
		
		for(unsigned int i = 0; i < m_Edges.size(); ++i)
		{
			// TODO add vertex does'nt need to find vertex
			//BGL_Vertex s = findVertexDescriptor(m_Edges[i]->getSource());
//			BGL_Vertex t = findVertexDescriptor(m_Edges[i]->getTarget());
			
			BGL_Vertex s = m_Edges[i]->getSource()->getDescriptor();
			BGL_Vertex t = m_Edges[i]->getTarget()->getDescriptor();
			
			BGL_Edge e; bool found;
			
			tie(e, found) = boost::add_edge(s,t,m_BoostGraph);
			
			if (!found) 
			{
				throw string("Erreur: BGL edge exists allready or nodes inexistant");
			}
			
			EdgeData[e] = m_Edges[i];
			m_Edges[i]->setDescriptor(e);
		}
	}
	catch (string str) 
	{
		cerr << str << endl;
		return;
	}
	catch (...) 
	{
		cerr << "Unkomwn error building BGL Graph" << endl;
		return;
	}
	
	saveBGLGraphToDotFile("/Users/jmainpri/Desktop/Graph/myGraph.dot");
	cout << "Building BGL_Graph OK!!!" << endl;
}

BGL_Vertex Graph::findVertexDescriptor(Node* N)
{
	BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , m_BoostGraph );
	
	typedef boost::graph_traits<BGL_Graph>::vertex_iterator vertex_iter;
	pair<vertex_iter, vertex_iter> vp;
	
//	cout << "------------------------------" << endl;
//	cout << "Node : " << N << endl;
	unsigned int i=0;
	for (vp = vertices(m_BoostGraph); vp.first != vp.second; ++vp.first)
	{		
		//cout << "NodeData[*vp.first] : " << i << " = " << NodeData[*vp.first] << endl;
		//		cout << "Node : " << i << " = " << m_Nodes[i] << endl;
		i++;
		if( NodeData[*vp.first]	== N )
		{
			return *vp.first;
		}
	}
	//cout << i << endl;
	//	cout << m_Nodes.size() << endl;
	throw string("Erreur: BGL vertex not found");
}

BGL_Edge Graph::findEdgeDescriptor(Edge* E)
{
	BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , m_BoostGraph );
	
	typedef boost::graph_traits<BGL_Graph>::edge_iterator edge_iter;
	pair<edge_iter, edge_iter> ep;
	
//	cout << "------------------------------" << endl;
//	cout << "Edge : " << E << endl;
	unsigned int i=0;
	for (ep = edges(m_BoostGraph); ep.first != ep.second; ++ep.first)
	{		
		//cout << "EdgeData[*ep.first] : " << i << " = " << EdgeData[*ep.first] << endl;
		i++;
		if( EdgeData[*ep.first]	== E )
		{
			return *ep.first;
		}
	}
	
	throw string("Erreur: BGL edge not found");
}

void Graph::setAllDescriptorsInvalid()
{
	// Unset descriptors
	for(unsigned int i=0; i<m_Nodes.size(); i++ )
	{
		m_Nodes[i]->unSetDescriptor();
	}
	
	for(unsigned int i=0; i<m_Edges.size(); i++ )
	{
		m_Edges[i]->unSetDescriptor();
	}
}

void Graph::saveBGLGraphToDotFile(const std::string& filename)
{
	std::ofstream out(filename.c_str());
	boost::write_graphviz(out,m_BoostGraph);
}

// Destructors
//----------------------------------------------
Graph::~Graph()
{
	if( graph_debug_import_export )
	{
		cout << "Graph::~Graph() => Deleting graph" << endl;
	}
	
	// Deletes the old graph
	if( ENV.getBool(Env::use_p3d_structures) )
	{
		deleteGraphStruct();
	}
	else 
	{
		cout << "Mark the XYZ_GRAPH as deleted" << endl;
		m_Robot->getRobotStruct()->GRAPH = XYZ_GRAPH = NULL;
		delete m_Graph;
	}
	
	// The configuration arrays have been deleted in the old API
	freeResources();
	
	cout << "XYZ_GRAPH = " << XYZ_GRAPH << endl;
}

void Graph::deleteGraphStruct()
{	
	/* verification of the existence of the graph */
	if (m_Graph)
	{
		// Get the compoc mirroring the P3d
		updateCompcoFromStruct();
		
		p3d_list_node *NLS, *NLD;  /* Node List Scanner, Node List pointer for Deletion */
		p3d_list_node *NeLS, *NeLD; /* Neighbour List Scanner, Neighbour List pointer for Deletion */
		p3d_list_edge *ELS, *ELD;  /* Edge List Scanner, Edge pointer for Deletion */
		
		/*nodes desallocation*/
		NLS = m_Graph->nodes;
		
		while (NLS) {
			NLD = NLS;
			NLS = NLS->next;
			
			/* neighbour list desallocation */
			NeLS = NLD->N->neighb;
			while (NeLS) {
				NeLD = NeLS;
				NeLS = NeLS->next;
				delete NeLD;
			}
			/* node's edge list desallocation */
			ELS = NLD->N->edges;
			while (ELS) {
				ELD = ELS;
				ELS = ELS->next;
				delete ELD;
			}
			delete NLD;
		}
		
#if P3D_PLANNER
		if((m_Graph->stat != NULL))
		{
			destroyStat(&(m_Graph->stat));
		}
		
		/* Delete references to the graph */
		if (m_Graph->rob != NULL) 
		{
			m_Graph->rob->GRAPH = NULL;
		}
#endif
		cout << "m_Graph = " << m_Graph << endl;
		cout << "XYZ_GRAPH = " << XYZ_GRAPH << endl;
		
		if (XYZ_GRAPH == m_Graph) 
		{
			m_Robot->getRobotStruct()->GRAPH = XYZ_GRAPH = NULL;
		}
		
		delete m_Graph;
	}
}

/**
 * Frees the Nodes and Edges
 */
void Graph::freeResources()
{
	for(unsigned int i=0;i<m_Comp.size();i++)
	{
		delete m_Comp[i];
	}
	
	for(unsigned int i=0;i<m_Edges.size();i++)
	{
		delete m_Edges[i];
	}
	
	for(unsigned int i=0;i<m_Nodes.size();i++)
	{
		delete m_Nodes[i];
	}
}

// Import and Export function to p3d
//----------------------------------------------
/**
 * Fonction that imports a Graph
 */
void Graph::importGraphStruct(p3d_graph* G)
{
	if( graph_debug_import_export )
	{
		cout << "Importing the graph" << endl;
	}
	
	m_Robot = global_Project->getActiveScene()->getRobotByName(G->rob->name);
	m_Graph = G;
	
	if (G->nodes)
	{
		p3d_list_node* l = G->nodes;
		while (l)
		{
			m_Nodes.push_back(new Node(this, l->N));
			l = l->next;
		}
	}
	if (G->edges)
	{
		p3d_list_edge* l = G->edges;
		while (l)
		{
			m_Edges.push_back(new Edge(this, 
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
			m_Comp.push_back(new ConnectedComponent(this, l));
			l = l->suiv;
		}
	}
	
	this->setName();
}

/**
 * Fonction exports a copy of the p3d_graph contained in
 * the Graph class suposing that it is valid regarding connexity
 */
p3d_graph* Graph::exportGraphStruct() const
{
	if( graph_debug_import_export )
	{
		cout << "Exporting the Graph structure" <<  endl;
	}
	//this->updateCompcoFromStruct();
	
	//p3d_graph* G = p3d_allocinit_graph();
	p3d_graph* G = new p3d_graph(*m_Graph);
	
	G->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	G->rob = m_Robot->getRobotStruct();
	
	m_Robot->getRobotStruct()->GRAPH = G;
	
	G->nnode = static_cast<int>(m_Nodes.size());
	G->nedge = static_cast<int>(m_Edges.size());
	
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
		cout << "m_Nodes.size() = " << m_Nodes.size() << endl;
	}
	
	// New table of nodes
	// Old node to new nodes
	map<p3d_node*,p3d_node*> NodeMap;
	
	p3d_list_node* ln = NULL;
	p3d_list_node* prevNode = NULL;
	
	for (unsigned int i = 0; i < m_Nodes.size(); i++) 
	{
		// Creates a new copied p3d_node
		ln = new p3d_list_node;
		ln->N = new p3d_node(*m_Nodes[i]->getNodeStruct());
		ln->N->q = p3d_copy_config(m_Robot->getRobotStruct(),ln->N->q);
		cout << "Node( " << ln->N->num << " ) = " << ln->N->numcomp << endl;
		
		//cout <<  "ln->N->num = " << ln->N->num << endl;
		
		// Adds the new node to the map
		NodeMap.insert(pair<p3d_node*,p3d_node*>(m_Nodes[i]->getNodeStruct(),ln->N));
		
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
		cout << "mm_Edges.size() = " << m_Edges.size() << endl;
	}
	
	// New table of edges
	// Old edges to new edges
	map<p3d_edge*,p3d_edge*> EdgeMap;
	
	p3d_list_edge* le = NULL;
	p3d_list_edge* prevEdge = NULL;
	
	for (unsigned int i = 0; i < m_Edges.size(); i++) 
	{
		// Creates new edge
		le = new p3d_list_edge;
		le->E = new p3d_edge(*m_Edges[i]->getEdgeStruct());
		
		// Change the p3d_node pointers
		le->E->Ni = NodeMap[m_Edges[i]->getSource()->getNodeStruct()];
		le->E->Nf = NodeMap[m_Edges[i]->getTarget()->getNodeStruct()];
		
		// Adds the new Edge to the map
		EdgeMap.insert(pair<p3d_edge*,p3d_edge*>(m_Edges[i]->getEdgeStruct(),le->E));
		
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
		cout << "m_Comp.size() = " << m_Comp.size() << endl;
	}
	
	// New table of compco
	// Old compco to new compco
	map<p3d_compco*,p3d_compco*> CompMap;
	
	if( ! m_Comp.empty() )
	{
		p3d_compco* lc;
		p3d_compco* prevComp = NULL;
		
		for (unsigned int i = 0; i < m_Comp.size(); i++) 
		{
			lc = new p3d_compco(*m_Comp[i]->getCompcoStruct());
			
			// Adds the new Compco to the map
			CompMap.insert(pair<p3d_compco*,p3d_compco*>(m_Comp[i]->getCompcoStruct(),lc));
			
			// Copy node list of the compco
			lc->nodes					= copyNodeList(NodeMap,lc->nodes);
			lc->dist_nodes		= copyNodeList(NodeMap,lc->dist_nodes);
			
			//cout << "Nb : Can reach = " << lc->ncanreach <<  endl;
			
			// Warning size may change!!! see p3d file
			lc->AnaSuccessTab = new int[100];
			memcpy((void*)lc->AnaSuccessTab,(void*)m_Comp[i]->getCompcoStruct()->AnaSuccessTab,100*sizeof(int));
			
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
	for (unsigned int i = 0; i < m_Nodes.size(); i++) 
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

p3d_list_node* Graph::copyNodeList(map<p3d_node*,p3d_node*>& NodeMap, 
																	 p3d_list_node* ln, 
																	 p3d_list_node* end) const
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

p3d_list_edge* Graph::copyEdgeList(map<p3d_edge*,p3d_edge*>& EdgeMap, p3d_list_edge* le, p3d_list_edge* end) const 
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

/**
 * Updates the list of connected
 * components from the p3d_graph structure
 */ 
void Graph::updateCompcoFromStruct()
{
	m_Comp.clear();
	
	if (m_Graph->comp)
	{
		p3d_compco* l = m_Graph->comp;
		while (l)
		{
			m_Comp.push_back(new ConnectedComponent(this, l));
			l = l->suiv;
		}
	}
}

//--------------------------------------------------------------------------------
// Exporting the Cpp Graph to a p3d_graph structure
//--------------------------------------------------------------------------------
/**
 * This function creates a graph from the Cpp graph
 */
p3d_graph* Graph::exportCppToGraphStruct(bool deleteGraphStruct)
{
	//cout << "Graph::exportCppToGraphStruct  " << endl;
	
	if( ENV.getBool(Env::use_p3d_structures) )
	{
		cerr << "No graph exportation when it maps the c graph" << endl;
		return m_Graph;
	}
	
	if( graph_debug_import_export )
	{
		cout << "Exporting the Graph structure" <<  endl;
	}
	//saveBGLGraphToDotFile("/Users/jmainpri/workspace/BioMove3D/video/graph_BGL.dot");
	
	//this->updateCompcoFromStruct();
	
	//p3d_graph* G = p3d_allocinit_graph();
	p3d_graph* G = new p3d_graph(*m_Graph);
	
	if (deleteGraphStruct) 
	{
		//p3d_del_graph(m_Graph);
	}
	
	G->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
	G->rob = m_Robot->getRobotStruct();
	
	m_Robot->getRobotStruct()->GRAPH = G;
	
	G->nnode = static_cast<int>(m_Nodes.size());
	G->nedge = static_cast<int>(m_Edges.size());
	
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
		cout << "m_Nodes.size() = " << m_Nodes.size() << endl;
	}
	
	// New table of nodes
	// Old node to new nodes
	map<Node*,p3d_node*> NodeMap;
	
	p3d_list_node* ln = NULL;
	p3d_list_node* prevNode = NULL;
	
	i=0;
	for (vector<Node*>::iterator it = m_Nodes.begin() ; it != m_Nodes.end(); ++it) 
	{
		// Creates a new copied p3d_node
		ln = new p3d_list_node;
		ln->N = new p3d_node(*(*it)->getNodeStruct()); i++;
		
		ln->N->q = p3d_copy_config(m_Robot->getRobotStruct(),
															 (*it)->getConfiguration()->getConfigStruct());
		
		//cout << "Node( " << ln->N->num << " ) = " << ln->N->numcomp << endl;
		
		//cout <<  "ln->N->num = " << ln->N->num << endl;
		
		// Adds the new node to the map
		NodeMap.insert(pair<Node*,p3d_node*>((*it),ln->N));
		
		if (it==m_Nodes.begin()) {
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
	
	if (i != m_Nodes.size()) 
	{
		throw string("The number of nodes exporting is wrong");
	}
	
	//---------------------------------------------------------------------
	// Copy all edges
	if( graph_debug_import_export )
	{
		cout << "Graph G : " << G <<  endl;
		cout << "m_Edges.size() = " << m_Edges.size() << endl;
	}
	
	// New table of edges
	// Old edges to new edges
	map<Edge*,p3d_edge*> EdgeMap;
	
	p3d_list_edge* le = NULL;
	p3d_list_edge* prevEdge = NULL;
	i=0;
	for (vector<Edge*>::iterator it = m_Edges.begin() ; it != m_Edges.end(); ++it) 
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
		
		if (it == m_Edges.begin()) 
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
	
	if (i != m_Edges.size()) 
	{
		throw string("The number of edges exporting is wrong");
	}
	//---------------------------------------------------------------------
	// Copy connected components
	if( graph_debug_import_export  )
	{
		//cout << "Graph G : " << G <<  endl;
		cout << "m_Comp.size() = " << m_Comp.size() << endl;
	}
	
	// New table of compco
	// Old compco to new compco
	map<ConnectedComponent*,p3d_compco*> CompMap;
	
	G->ncomp = m_Comp.size();
	
	if( ! m_Comp.empty() )
	{
		p3d_compco* lc;
		p3d_compco* prevComp = NULL;
		
		for (unsigned int i = 0; i < m_Comp.size(); i++) 
		{
			// Copy all fields
			lc = new p3d_compco(*m_Comp[i]->getCompcoStruct());
			
			// Adds the new Compco to the map
			CompMap.insert(pair<ConnectedComponent*,p3d_compco*>(m_Comp[i],lc));
			
			// Copy node list of the compco
			lc->nodes					= createNodeList(NodeMap,m_Comp[i]->getNodes(),lc->last_node);
			lc->dist_nodes		= createNodeList(NodeMap,m_Comp[i]->getNodes());
			lc->nnode					= m_Comp[i]->getNumberOfNodes();
			
			//cout << "Nb : Can reach = " << lc->ncanreach <<  endl;
			
			// Warning size may change!!! see p3d file
			lc->AnaSuccessTab = new int[100];
			//memcpy((void*)lc->AnaSuccessTab,(void*)m_Comp[i]->getCompcoStruct()->AnaSuccessTab,100*sizeof(int));
			
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
	for (vector<Node*>::iterator it = m_Nodes.begin(); it != m_Nodes.end(); ++it) 
	{
		ln->N->numcomp =			(*it)->getConnectedComponent()->getId();
		//cout << "Component num : " << ln->N->numcomp << endl;
		
		ln->N->neighb =				createNodeList( NodeMap, (*it)->getNeighbors(), ln->N->last_neighb );
		ln->N->edges =				createEdgeList(	EdgeMap, (*it)->getEdges(),  ln->N->last_edge );
		ln->N->nedge =				(*it)->getNumberOfEdges();
		//cout << "Number of Edges in node : " << ln->N->nedge << endl;
		
		ln->N->parent =				NodeMap[(*it)->getParent()];
		ln->N->search_from =	NodeMap[(*it)->getSearchFrom()];
		ln->N->search_to =		NodeMap[(*it)->getSearchTo()];
		
		ln->N->edge_from =		EdgeMap[(*it)->getEdgeFrom()];
		
		ln->N->comp	=					CompMap[(*it)->getConnectedComponent()];
		
		ln = ln->next;
	}
	//G->lastm_Edge = lc;
	
	m_graphChanged = false;
	
	if (graph_debug_import_export) 
	{
		cout << "Number of nodes : " << G->nnode << endl;
		cout << "Number of edges : " << G->nedge << endl;
		cout << "Number of compo : " << G->ncomp << endl;
	}
	return G;
}

/**
 * create node list
 */
p3d_list_node* Graph::createNodeList(map<Node*,p3d_node*>& NodeMap, const vector<Node*>& nodes, p3d_list_node* end) 
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


/**
 * create edge list
 */
p3d_list_edge* Graph::createEdgeList(map<Edge*,p3d_edge*>& EdgeMap, const vector<Edge*>& edges, p3d_list_edge* end) 
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


// Accessors
//----------------------------------------------
p3d_graph* Graph::getGraphStruct()
{
	return m_Graph;
}

void Graph::setGraph(p3d_graph* G)
{
	*m_Graph = *G;
}

Robot* Graph::getRobot()
{
	return m_Robot;
}

/*void Graph::setRobot(Robot* R)
 {
 m_Robot = R;
 }*/

void Graph::setTraj(p3d_traj* Traj)
{
	m_Traj = Traj;
}

p3d_traj* Graph::getTrajStruct()
{
	return m_Traj;
}

unsigned int Graph::getNumberOfNodes()
{
	if ( ENV.getBool(Env::use_p3d_structures) ) 
	{
		return m_Graph->nnode;
	}
	else 
	{
		unsigned int nbNodes = num_vertices(m_BoostGraph);
		if ( nbNodes != m_Nodes.size() ) 
		{
			throw string("Verticies in boost graph don't reflect the C++ graph");
		}
		
		return nbNodes;
	}
}

unsigned int Graph::getNumberOfEdges()
{
	if ( ENV.getBool(Env::use_p3d_structures) ) 
	{
		return m_Graph->nedge;
	}
	else 
	{
		unsigned int nbEdges = num_edges(m_BoostGraph);
		if ( nbEdges != m_Edges.size() ) 
		{
			throw string("Edges in the boost graph don't reflect the C++ graph");
		}
		
		return nbEdges;
	}
}

vector<Node*> Graph::getNodes()
{
	return m_Nodes;
}

vector<Edge*> Graph::getEdges()
{
	return m_Edges;
}

map<p3d_node*, Node*> Graph::getNodesTable()
{
	return m_NodesTable;
}

Node* Graph::getNode(p3d_node* N)
{
	map<p3d_node*, Node*>::iterator it = m_NodesTable.find(N);
	if (it != m_NodesTable.end())
		return (it->second);
	else
	{
		cout << "node " << N << " not found: num: " << N->num << endl;
		return (NULL);
	}
}

Node* Graph::getLastNode()
{
	return (m_Nodes.back());
}

string Graph::getName()
{
	return m_Name;
}

void Graph::setName()
{
	m_Name = "graph";
}

void Graph::setName(string Name)
{
	m_Name = Name;
}

bool Graph::equal(Graph* G)
{
	return (m_Graph->nodes == G->getGraphStruct()->nodes);
}

bool Graph::equalName(Graph* G)
{
	return (m_Name == G->getName());
}

// Graph operations
//----------------------------------------------

/**
 * Search configuration in graph
 */
Node* Graph::searchConf(Configuration& q)
{
	if( ENV.getBool(Env::use_p3d_structures) )
	{
    p3d_node* node(p3d_TestConfInGraph(m_Graph,q.getConfigStruct()));
    return (node ? m_NodesTable[node] : NULL);
	}
	else 
	{
		for(unsigned int i=0;i<m_Nodes.size();i++)
		{
			if(m_Nodes[i]->getConfiguration()->equal(q))
				return m_Nodes[i];
		}
		return 0x00;
	}
}

/**
 * Insert node in graph
 */
Node* Graph::insertNode(Node* node)
{
	addNode(node);
	
	if ( ENV.getBool(Env::use_p3d_structures) )
	{
#ifdef P3D_PLANNER
    p3d_insert_node(m_Graph, node->getNodeStruct());
#else
		printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
		
    m_Graph->dist_nodes = p3d_add_node_to_list(
																							 node->getNodeStruct(),
																							 this->getGraphStruct()->dist_nodes);
	}
	
	return (node);
}

/**
 * Insert Extremal Node in Graph
 */
Node* Graph::insertExtremalNode(Node* node)
{
	insertNode(node);
	
	if ( ENV.getBool(Env::use_p3d_structures) ) 
	{
		node->getNodeStruct()->type = ISOLATED;
	}
	return (node);
}

/**
 * Insert node for RRT
 */
Node* Graph::insertNode(Node* expansionNode,LocalPath& path)

/*shared_ptr<Configuration> q,
 Node* expansionNode,
 double currentCost, double step)*/
{
	double step = path.getParamMax();
	
	// Precondition
	// Check that the expansion node is in graph
	if( !isInGraph(expansionNode) )
	{
		Node* newExpansion = searchConf( *expansionNode->getConfiguration() );
		
		if ( newExpansion ) 
		{
			expansionNode = newExpansion;
		}
		else 
		{
			throw string("expansionNode is not in the graph");
		}
	}
	
	// Insert the end configuration of the inpout localpath
	// as a node in the graph
	Node* newNode = insertConfigurationAsNode(path.getEnd(), expansionNode, step);
	
	// set expansion node as the node parent
	newNode->parent() = expansionNode;
	
	// Cost space special updates
	if (ENV.getBool(Env::isCostSpace))
	{
		double currentCost = path.getEnd()->cost();
		
		if (ENV.getBool(Env::use_p3d_structures)) 
		{
			p3d_SetNodeCost(m_Graph, newNode->getNodeStruct(), currentCost );
		}
		else 
		{
			global_costSpace->setNodeCost( newNode , newNode->parent() );
		}
		
		//for adaptive variant, new temp is refreshed except if it is going down.
		if (currentCost < expansionNode->getNodeStruct()->cost)
		{
			newNode->getNodeStruct()->temp = expansionNode->getNodeStruct()->temp;
		}
		else
		{
			newNode->getNodeStruct()->temp = expansionNode->getNodeStruct()->temp / 2.;
		}
	}
	
	//weight updates
	if (p3d_GetIsWeightedChoice())
	{
		p3d_SetNodeWeight(m_Graph, newNode->getNodeStruct());
	}
	
	//check stop conditions
	if (p3d_GetIsWeightStopCondition())
	{
		newNode->checkStopByWeight();
	}
	
	// Graph updates for RANDOM_IN_SHELL method
	if (ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH)
	{
		p3d_SetNGood(p3d_GetNGood() + 1);
		
		if (newNode->getNodeStruct()->weight > m_Graph->CurPbLevel)
		{
			m_Graph->CurPbLevel = newNode->getNodeStruct()->weight;
			m_Graph->n_consec_pb_level = 0;
			m_Graph->n_consec_fail_pb_level = 0;
			
			if (p3d_GetNGood() > 2)
			{
				m_Graph->critic_cur_pb_level = m_Graph->CurPbLevel;
			}
		}
		else
		{
			m_Graph->n_consec_pb_level++;
			m_Graph->n_consec_fail_pb_level = 0;
		}
	}
	
	//Additional cycles through edges addition if the flag is active
	if (ENV.getBool(Env::addCycles))
	{
		this->addCycles(newNode, step);
	}
	
	return (newNode);
}

/**
 * Insert Lining Node for RRT
 */
Node* Graph::insertConfigurationAsNode(shared_ptr<Configuration> q, Node* from,double step)
{
	//cout << "Insert configuration as node" << endl;
	Node* node = new Node( this , q );
	
	insertNode(node);
	
	mergeComp( from, node, step );
	
	node->getNodeStruct()->rankFromRoot = from->getNodeStruct()->rankFromRoot +1;
	node->getNodeStruct()->type = LINKING;
	
	return(node);
}

/**
 * Compare Length Edges
 */
bool Graph::compareEdges(Edge* E1, Edge* E2)
{
	return (E1->getEdgeStruct()->longueur < E2->getEdgeStruct()->longueur);
}

/**
 * Compare Nodes
 */
bool Graph::compareNodes(Node* N1, Node* N2)
{
	return (N1->getNodeStruct()->dist_Nnew < N2->getNodeStruct()->dist_Nnew);
	//double dist = N1->dist(N2);
	//return dist;
}

/**
 * Is Edge in graph
 */
Edge* Graph::isEdgeInGraph(Node* N1, Node* N2)
{
	bool flag = false;
	
	for (unsigned int i = 0; i < m_Edges.size(); i++)
	{
		if( ENV.getBool(Env::use_p3d_structures) )
		{
			flag =
			( m_Edges[i]->getEdgeStruct()->Ni == N1->getNodeStruct() ) &&
			( m_Edges[i]->getEdgeStruct()->Nf == N2->getNodeStruct() );
			
			if (flag)
			{
				return (Edge*)flag;
			}
			
		}
		else 
		{
			flag =	
			( m_Edges[i]->getSource() == N1 ) &&
			( m_Edges[i]->getTarget()   == N2 );
			
			if (flag)
			{
				return m_Edges[i];
			}
		}
	}
	return 0x0;
}

/**
 * Sort Edges
 */
void Graph::sortEdges()
{
	if (m_Edges.size() > 0)
	{
		sort(m_Edges.begin(), m_Edges.end(), &compareEdges);
	}
}

/**
 * Sort Node by distance
 */
void Graph::sortNodesByDist(Node* N)
{
	if (m_Nodes.size() > 1)
	{
		for (unsigned int i=0; i < m_Nodes.size(); i++)
		{
			m_Nodes[i]->dist(N);
		}
		sort(m_Nodes.begin(), m_Nodes.end(), &compareNodes);
	}
}

/**
 * Add Edge to Graph
 */
void Graph::addEdge(Node* source, Node* target, double Length )
{	
	if (!this->isEdgeInGraph(source, target))
	{
		Edge* E = new Edge(this, source, target, Length);
		m_Edges.push_back(E);
		
		//cout << "new edge : " << E << endl;
		
		BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , m_BoostGraph );
		BGL_Edge e; bool found;
		
		boost::tie(e, found) = boost::add_edge( source->getDescriptor(), 
																					 target->getDescriptor(), m_BoostGraph);
		
		if (found) 
		{
			//cout << "descriptor	: " << e << endl;
			EdgeData[e] = E;
			E->setDescriptor(e);
		}
		
		if( ENV.getBool(Env::use_p3d_structures) )
		{
			p3d_create_one_edge(m_Graph,
													source->getNodeStruct(),
													target->getNodeStruct(),
													Length);
		}
		
		m_graphChanged = true;
	}
}

/**
 * Add Edges
 * WARNING only in graph API doesn't work yet not to be used
 */
void Graph::addEdges(Node* N1, Node* N2, double Length , bool computeLength )
{
	if (computeLength) 
	{
		LocalPath path(N1->getConfiguration(), 
									 N2->getConfiguration());
		
		Length = path.getParamMax();
	}
	
	addEdge(N1, N2, Length);
	addEdge(N2, N1, Length);
		
	if( ENV.getBool(Env::use_p3d_structures) )
	{
		// This is the code copied from 
		
		//		p3d_create_edges(m_Graph,
		//									 N1->getNodeStruct(),
		//									 N2->getNodeStruct(),
		//									 DistNodes);
		
		N2->getNodeStruct()->parent = N1->getNodeStruct();  // modif Juan (WARNING : suppose that N1 is always the parent node)
		
		if(p3d_get_costComputation())
		{
			N1->getNodeStruct()->cost += N1->getNodeStruct()->last_edge->E->cost;
			N2->getNodeStruct()->cost += N2->getNodeStruct()->last_edge->E->cost;
		}
	}
}

/**
 * Remove edge from graph
 */
void Graph::removeEdge(Edge* E)
{
	m_Edges.erase( find( m_Edges.begin() , m_Edges.end(), E ) );
}

/**
 * Remove edges between source node and target node
 */
void Graph::removeEdge( Node* source, Node* target )
{
	if( ENV.getBool(Env::use_p3d_structures))
	{
		throw string("Remove edge only works without the p3d structures");
	}
	
	// Get out edges
	vector<Edge*> outEdges = source->getEdges();
	
	// Remove the ones going to target from graph
	for(unsigned int i=0;i<outEdges.size();i++)
	{
		if ( outEdges[i]->getSource() != source ) 
		{
			throw string("Error : Outedge has not source as start in remove edge");
		}
		
		if (outEdges[i]->getTarget() == target  ) 
		{
			boost::remove_edge( outEdges[i]->getDescriptor() ,  m_BoostGraph );
			this->removeEdge( outEdges[i] );
			delete outEdges[i];
		}
	}
}

void Graph::removeEdges( Node* N1, Node* N2 )
{
	removeEdge( N1, N2 );
	removeEdge( N2, N1 );
}

/**
 * Remove an edge from the graph
 */
//void Graph::removeEdge(Edge* E)
//{
//	removeNodeFromGraph(E->getSource());
//	removeNodeFromGraph(E->getTarget());
//	
//	p3d_removem_Edge_from_list(E->getEdgeStruct(),m_Graph->edges);
//}

/**
 * Add basically a node to the graph
 */
void Graph::addNode(Node* N)
{
	m_NodesTable.insert(pair<p3d_node*, Node*> (N->getNodeStruct(), N));
	m_Nodes.push_back(N);
	
	BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , m_BoostGraph );
	BGL_Vertex v								= boost::add_vertex( m_BoostGraph );
	NodeData[v]									= N;
	N->setDescriptor(v);
	
	m_graphChanged							= true;
}


/**
 * Add Edge to all node inferior to max Dist form N
 */
void Graph::addNode(Node* N, double maxDist)
{
	double d;
	for (unsigned int i = 0; i < m_Nodes.size(); i = i + 1)
	{
		if ((d = m_Nodes[i]->getConfiguration()->dist(*N->getConfiguration()))< maxDist)
		{
			LocalPath path(m_Nodes[i]->getConfiguration(), N->getConfiguration());
			
			if ( path.isValid() )
			{
				this->addEdges(m_Nodes[i], N, d);
			}
		}
	}
	
	addNode(N);
}

/**
 * Add  vector of node
 */
void Graph::addNodes(vector<Node*> N, double maxDist)
{
	for (vector<Node*>::iterator it = N.begin(), end = N.end(); it != end; it++)
	{
		this->addNode(*it, maxDist);
	}
}

/**
 * Remove a node from compco
 * TODO 
 */
void Graph::removeNode(Node* N)
{	
	if( !isInGraph(N) )
	{
		throw string("Node, N is not graph");
	}
	if ( ENV.getBool(Env::use_p3d_structures) ) 
	{
		throw string("Remove node only works without the p3d structures");
	}
	
	// Removes the node from the BGL graph
	BGL_Vertex u = N->getDescriptor();
	
	boost::clear_vertex(  u,  m_BoostGraph );
	boost::remove_vertex( u,  m_BoostGraph );
	
	cout << "Node vector is " << m_Nodes.size() << endl;
	
	vector<Edge*> toDelete;
	
	// Find edges to delete and suppres from graph
	for(vector<Edge*>::iterator itEdge = m_Edges.begin(); 
			itEdge != m_Edges.end(); ++itEdge )
	{
		if ( ( (*itEdge)->getSource() == N )||( (*itEdge)->getTarget() == N ) ) 
		{
			toDelete.push_back( *itEdge );
		}
	}
	cout << "Edges : " << m_Edges.size() << endl;
	
	for(unsigned int i=0; i<toDelete.size(); ++i )
	{
		cout << "Edges to delete : " << toDelete[i] << endl;
		m_Edges.erase( find( m_Edges.begin(), m_Edges.end() ,  toDelete[i]  )); 
		delete toDelete[i];
	}
	cout << "Edges : " << m_Edges.size() << endl;
	
	// Erase from node vector
	m_Nodes.erase( find( m_Nodes.begin() , m_Nodes.end() , N ) );
	
	// Rebuild compco
	this->rebuildCompcoFromBoostGraph();
	
	// All descriptors are now invalid
	this->setAllDescriptorsInvalid();
	
	// Save to file
	cout << "Saving graph" << endl;
	this->saveBGLGraphToDotFile( "/Users/jmainpri/Desktop/Graph/myGraph.dot" );
}

/**
 * Test if node is in graph
 */
bool Graph::isInGraph(Node* N)
{
	for (unsigned int i = 0; i < m_Nodes.size(); i++ )
	{
		if ( N == m_Nodes[i] ) 
			return true;
	}
	return false;
}

/**
 * Link node to graph
 */
bool Graph::linkNode(Node* N)
{
	if (ENV.getBool((Env::useDist)))
	{
		// Warning : TODO does not work with the C++ API
		return this->linkNodeAtDist(N);
	}
	else
	{
		return this->linkNodeWithoutDist(N);
	}
}

/**
 * Links node to graph Without distance
 */
bool Graph::linkNodeWithoutDist(Node* N)
{
	bool b = false;
	for (int j = 1; j <= m_Graph->ncomp; j = j + 1)
	{
		if (N->getNodeStruct()->numcomp != j)
		{
			for (unsigned int i = 0; i < m_Nodes.size(); i = i + 1)
			{
				if (m_Nodes[i]->getNodeStruct()->numcomp == j)
				{
					if (m_Nodes[i]->connectNodeToCompco(N, 0))
					{
						N->getNodeStruct()->search_to = N->getNodeStruct()->last_neighb->N;
						N->getNodeStruct()->last_neighb->N->search_from = N->getNodeStruct();
						N->getNodeStruct()->last_neighb->N->edge_from = N->getNodeStruct()->last_neighb->N->last_edge->E;
						b = true;
					}
					break;
				}
			}
		}
	}
	return b;
}

/**
 * Links node at distance
 */
bool Graph::linkNodeAtDist(Node* N)
{
	int nbLinkedComponents = 0;
	
#ifdef P3D_PLANNER
	// Warning TODO:  this function does not work with the C++ API
  nbLinkedComponents = p3d_link_node_graph_multisol(N->getNodeStruct(), m_Graph);
#else
	printf("P3D_PLANNER not compiled in %s in %s",__func__,__FILE__);
#endif
	
  this->mergeCheck();
  return(nbLinkedComponents > 0);
}

/**
 * Links Node to all nodes
 */
bool Graph::linkToAllNodes(Node* newN)
{
	// Warning TODO:  this function does not work with the C++ API
	return p3d_all_link_node(newN->getNodeStruct(), m_Graph);
}

bool Graph::areNodesLinked(Node* node1, Node* node2, double & dist)
{	
	// copy of the function p3d_APInode_linked
	// that computes also the IK solution
	
  /* current position of robot is saved */
  shared_ptr<Configuration> qSave = m_Robot->getCurrentPos();
	
  LocalPath* LP = new LocalPath(
																node1->getConfiguration(), 
																node2->getConfiguration() );
	
	
  if (LP->getParamMax() != 0.0 )
		dist = LP->getParamMax();
  else{
		PrintInfo(("Warning: created an edge with \
							 a 0 distance: no localpathPt->length \n"));
		dist = 0;
	}
  if((p3d_get_SORTING()==P3D_NB_CONNECT)&&
     (p3d_get_MOTION_PLANNER()==P3D_BASIC)) 
	{
    if((dist > p3d_get_DMAX())&&(LEQ(0.,p3d_get_DMAX())))
		{ 
			/* ecremage deja fait dans le cas tri par distance... */
      /* the local path is destroyed */
      delete LP;
      return false;
    }
  }
	
  bool isNoCol = LP->isValid();   // <- modif Juan
	//   isNoCol = 1;
	
	int ntest = LP->getNbColTest();
	
	delete LP;
	
  if(m_Graph){
    m_Graph->nb_local_call = m_Graph->nb_local_call + 1;
    m_Graph->nb_test_coll = m_Graph->nb_test_coll + ntest;
  }
	
  return(isNoCol);
}


/**
 * Link Node and Merge
 */
bool Graph::linkNodeAndMerge(Node* node1, Node* node2)
{
	double DistNodes = 0.;
	bool  IsLinkedAndMerged = false;
	
	if ( ENV.getBool(Env::use_p3d_structures) ) 
	{
		p3d_node* Node1Pt = node1->getNodeStruct();
		p3d_node* Node2Pt = node2->getNodeStruct();
		
		if ((Node1Pt == NULL) || (Node2Pt == NULL)) 
		{
			cerr << "Warning: Try to link two nodes with NULL structures \n";
			return false;
		}
		
		if (p3d_APInode_linked(m_Graph, Node1Pt, Node2Pt, &DistNodes)) 
		{
			/*p3d_compco* CompNode1Pt = Node1Pt->comp;
			 p3d_compco* CompNode2Pt = Node2Pt->comp;
			 
			 if (CompNode1Pt->num < CompNode2Pt->num)
			 p3d_merge_comp(m_Graph, CompNode1Pt, &CompNode2Pt);
			 else if (CompNode1Pt->num > CompNode2Pt->num)
			 p3d_merge_comp(m_Graph, CompNode2Pt, &CompNode1Pt);
			 
			 //p3d_create_edges(m_Graph, Node1Pt, Node2Pt, DistNodes);
			 //    PrintInfo(("dist: %f\n",DistNodes));
			 
			 addEdges(node1, node2, DistNodes);*/
			
			mergeComp(node1, node2, DistNodes);
			
			IsLinkedAndMerged = true;
		}
	}
	else 
	{
		if (areNodesLinked(node1, node2, DistNodes)) 
		{
			mergeComp(node1, node2, DistNodes); 
			
			IsLinkedAndMerged = true;
		}
	}
	
	return IsLinkedAndMerged;
}



/**
 * Create Random Configurations
 */
void Graph::createRandConfs(int NMAX)
{
	int inode;
	double tu, ts;
	
	ChronoOn();
	
	inode = 0;
	
	shared_ptr<Configuration> Cs = shared_ptr<Configuration> (
																														new Configuration(m_Robot, m_Robot->getRobotStruct()->ROBOT_POS));
	Node* Ns = new Node(this, Cs);
	this->insertNode(Ns);
	
	while (inode < NMAX)
	{
		shared_ptr<Configuration> C = m_Robot->shoot();
		if (!C->isInCollision())
		{
			this->insertNode(new Node(this, C));
			inode = inode + 1;
			if (fct_draw!=NULL)
				(*fct_draw)();
			else
			{
				PrintInfo(("Random conf. generation in not possible\n"));
				break;
			}
			
			if (fct_stop)
			{
				if (!(*fct_stop)())
				{
					PrintInfo(("Random confs. generation canceled\n"));
					break;
				}
			}
		}
	}
	
	PrintInfo(("For the generation of %d configurations : ", inode));
	ChronoTimes(&tu, &ts);
	m_Graph->time = m_Graph->time + tu;
	ChronoPrint("");
	ChronoOff();
	p3d_print_info_graph(m_Graph);
}

/**
 * Random Nodes From Component
 */
Node* Graph::randomNodeFromComp(Node* comp)
{
	return (this->getNode(p3d_RandomNodeFromComp(comp->getConnectedComponent()->getCompcoStruct())));
}

/**
 * Intempts to connect a node to a compco
 */
bool Graph::connectNodeToCompco(Node* node1, Node* compco)
{
	//This is a copy of the old p3d_ConnectNodeToComp
	Node* node2 = NULL;
	
	bool IsConnectSuccess = false;
	
	//	p3d_compco* CompToConnectPt = compco->getCompcoStruct();
	
	bool SavedIsMaxDis = false;
	bool SavedIsWeightChoice = FALSE;
	//  double ratio = 1./5.;
	
	if ((node1 == NULL) || (compco->getConnectedComponent()->getCompcoStruct() == NULL)) 
	{
		cout << "Warning: Try to connect a node to a comp \
		with NULL structures" << endl;
		return false;
	}
	
	if (node1->getConnectedComponent()->getCompcoStruct()->num == 
			compco->getConnectedComponent()->getCompcoStruct()->num) 
	{
		cout << "Warning: Try to connect a Node to its own componant" << endl;
		return true;
	}
	
	switch (p3d_GetNodeCompStrategy()) 
	{
		case NEAREST_NODE_COMP:
			/* Connect the node to the nearest node of the comp */
			
			SavedIsMaxDis = PlanEnv->getBool(PlanParam::isMaxDisNeigh);
			SavedIsWeightChoice = PlanEnv->getBool(PlanParam::isWeightedChoice);
//			SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
//			SavedIsWeightChoice = p3d_GetIsWeightedChoice();
//			p3d_SetIsMaxDistNeighbor(false);
//			p3d_SetIsWeightedChoice(false);
			
			PlanEnv->setBool(PlanParam::isMaxDisNeigh,false);
			PlanEnv->setBool(PlanParam::isWeightedChoice,false);
			
			node2 =  nearestWeightNeighbour(compco,
																			node1->getConfiguration(),
																			false,
																			GENERAL_CSPACE_DIST);
			
			p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
			p3d_SetIsWeightedChoice(SavedIsWeightChoice);
			
			if (node2->getConnectedComponent()->getCompcoStruct() == NULL) 
			{
				cout << "Warning: Failed to find a nearest node in \
				the Componant to connect\n" << endl;
				return false;
			}
			
			/*     if(SelectedDistConfig(GraphPt->rob, Node1ToConnectPt->q, Node2ToConnectPt->q) >  */
			/*        ratio*SelectedDistConfig(GraphPt->rob, GraphPt->search_start->q, GraphPt->search_goal->q)){ */
			/*       return FALSE; */
			/*     } */
			IsConnectSuccess = linkNodeAndMerge(node2,node1);
			break;
			
		case K_NEAREST_NODE_COMP:
			/*Connect randomly to one of the k nearest
			 nodes of the componant */
			/*todo*/
			break;
		default:
			/* By default  try to
			 connect to the node to the nearest node of the componant */
//			SavedIsMaxDis =  PlanEnv->getBool(PlanParam::isMaxDis,false);;
//			SavedIsWeightChoice = p3d_GetIsWeightedChoice();
			
			SavedIsMaxDis =  PlanEnv->getBool(PlanParam::isMaxDisNeigh);
			SavedIsWeightChoice = p3d_GetIsWeightedChoice();
			
			PlanEnv->setBool(PlanParam::isMaxDisNeigh,false);
			
			p3d_SetIsMaxDistNeighbor(false);
			p3d_SetIsWeightedChoice(false);
			node2 =  nearestWeightNeighbour(compco,
																			node1->getConfiguration(),
																			false,
																			GENERAL_CSPACE_DIST);
			
			p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);
			p3d_SetIsWeightedChoice(SavedIsWeightChoice);
			
			if (node2->getConnectedComponent()->getCompcoStruct() == NULL) 
			{
				cout << "Warning: Failed to find a nearest node in \
				the Componant to connect" << endl;
				return false;
			}
			/*    if(SelectedDistConfig(GraphPt->rob, Node1ToConnectPt->q, Node2ToConnectPt->q) >  */
			/*        ratio*SelectedDistConfig(GraphPt->rob, GraphPt->search_start->q, GraphPt->search_goal->q)){ */
			/*       return FALSE; */
			/*    } */
			IsConnectSuccess = linkNodeAndMerge(node2,node1);
	}
	return IsConnectSuccess;
}

/**
 * Nearest Weighted Neighbout in graph
 *  
 * returns the KNearest neighbours of the configuration config
 * in the entire graph within a minimum radius
 */
vector<Node*> Graph::KNearestWeightNeighbour(shared_ptr<Configuration> q,
																						 int K,
																						 double radius,
																						 bool weighted,
																						 int distConfigChoice)
{
	double CurrentScore;
	
	
	vector< pair<double,Node*> > nearNodesQueue;
	
	// The null node has to be remouved
	Node* nullNode(NULL);
	nearNodesQueue.push_back( make_pair(numeric_limits<double>::max(),nullNode) );
	bool firstNode = true;
	
	cout << "Number of nodes in the graph : " << m_Nodes.size() << endl;
	
	for (vector<Node*>::iterator it = m_Nodes.begin();
			 it != m_Nodes.end() ; ++it)
	{
		// We take into account only the nodes undiscarded 
		//if (!(*it)->getNodeStruct()->IsDiscarded)
		//{
			// Compute the distance for all nodes
			CurrentScore = q->dist(*(*it)->getConfiguration(),distConfigChoice)
			
			// And weigth if flag is on
			* (weighted ? p3d_GetNodeWeight((*it)->getNodeStruct()) : 1.0);
			
//			cout << "CurrentScore : " << CurrentScore << endl;
//			cout << "radius : " << radius << endl;
		
			//cout << "nearNodesQueue.back().first" << nearNodesQueue.back().first << endl;
		
			// Do not add the nodes outside of radius
			if ( CurrentScore > radius) continue;
			
			// If better than last node of the Queue
			// add to the Queue (rewrite better code)
			if (CurrentScore < nearNodesQueue.back().first)
			{
				if ( firstNode ) 
				{
					nearNodesQueue.clear(); firstNode = false;
				}
				nearNodesQueue.push_back( make_pair( CurrentScore, *it ) );
				
				sort( nearNodesQueue.begin(), nearNodesQueue.end() );
				
				if( nearNodesQueue.size() > (unsigned int)K )
				{
					nearNodesQueue.resize( K );
				}
			}
		//}
	}
	
	// Put queue in a vector
	vector<Node*> nearNodes;
	
	for(unsigned int i=0;i<nearNodesQueue.size();i++)
		nearNodes.push_back( nearNodesQueue[i].second );
	
	return nearNodes;	
}


/**
 * Nearest Weighted Neighbour in graph
 *
 * finds the closest node from configuration in the 
 * connected component of node compco
 */
Node* Graph::nearestWeightNeighbour(Node* compco, shared_ptr<Configuration> config,
                                    bool weighted, int distConfigChoice)
{
	p3d_node* BestNodePt = NULL;
	double BestScore = P3D_HUGE;
	double CurrentDist = -1.;
	double CurrentScore = -1.;
	double DistOfBestNode = -1.;
	p3d_matrix4 *RefFramePt = NULL, *MobFramePt = NULL;
	p3d_matrix4 MobFrameRef, invT;
	
	Node* BestNode = NULL;
	
#if P3D_PLANNER
	// When retrieving statistics
	if (getStatStatus())
	{
		m_Graph->stat->planNeigCall++;
	}
#endif
	
	//computation of the mobFrameRef of the Config
	if (distConfigChoice == MOBILE_FRAME_DIST && p3d_GetRefAndMobFrames(
																																			m_Graph->rob, &RefFramePt, &MobFramePt))
	{
		p3d_set_robot_config(m_Graph->rob, config->getConfigStruct());
		p3d_update_this_robot_pos_without_cntrt_and_obj(m_Graph->rob);
		p3d_GetRefAndMobFrames(m_Graph->rob, &RefFramePt, &MobFramePt);
		if (RefFramePt == NULL)
		{
			p3d_mat4Copy(*MobFramePt, MobFrameRef);
		}
		else
		{
			p3d_matInvertXform(*RefFramePt, invT);
			p3d_matMultXform(invT, *MobFramePt, MobFrameRef);
		}
	}
	
#ifdef LIGHT_PLANNER
	if(ENV.getBool(Env::FKDistance))
	{
		
		if( ! config->setConstraints() )
		{
			cout << "Graph CPP API :: FK Constraint could not be satistfied"  << endl;
		}
		
		activateCcCntrts(m_Robot->getRobotStruct(),-1,true);
	}
#endif
	
	//    cout << "distConfigChoice = " << distConfigChoice << endl;
	
	if (ENV.getBool(Env::use_p3d_structures)) 
	{
    p3d_list_node* nodes(compco->getConnectedComponent()->getCompcoStruct()->dist_nodes);
		//    cout << "nearestWeightNeighbour "  << endl;
    while (nodes)
    {
			/* We take into account only the nodes undiscarded */
			if (!nodes->N->IsDiscarded)
			{
				if (distConfigChoice == MOBILE_FRAME_DIST)
				{
					CurrentDist = p3d_GetSe3DistanceFrames(m_Graph->rob,
																								 MobFrameRef, nodes->N->RelMobFrame);
				}
				else
				{
					
					CurrentDist = config->dist(
																		 *m_NodesTable[nodes->N]->getConfiguration(),
																		 distConfigChoice);
				}
				
				CurrentScore = CurrentDist
				* (weighted ? p3d_GetNodeWeight(nodes->N) : 1.0);
				
				if (CurrentScore < BestScore)
				{
					BestScore = CurrentScore;
					BestNodePt = nodes->N;
					DistOfBestNode = CurrentDist;
				}
			}
			nodes = nodes->next;
    }
		
		BestNode = m_NodesTable[BestNodePt];
	}
	else 
	{
		BestNode = compco->getConnectedComponent()->nearestWeightNeighbour(config,weighted,distConfigChoice);
	}
	
#ifdef LIGHT_PLANNER
	if(ENV.getBool(Env::FKDistance))
	{
		deactivateCcCntrts(m_Robot->getRobotStruct(),-1);
	}
#endif
	
	if ((p3d_GetIsMaxDistNeighbor() == TRUE) && (BestNodePt->boundary == TRUE)
			&& (BestNodePt->radius < DistOfBestNode))
	{
		/* There is a maximal distance allowed to get a node as neighbor */
		return NULL;
	}
	
	return BestNode;
}

unsigned int Graph::getNumberOfCompco()
{
	return m_Graph->ncomp;
}

//-----------------------------------------------------------
// Connected componnents
//-----------------------------------------------------------

/**
 * Merge Component
 */
int Graph::mergeComp(Node* CompCo1, Node* CompCo2, double DistNodes)
{
	if ((CompCo1 == NULL) || (CompCo2 == NULL))
	{
		cerr << "Warning: Try to link two nodes with NULL structures \n";
		return false;
	}
	
	if (CompCo1->getConnectedComponent()->getId()  > 
			CompCo2->getConnectedComponent()->getId() )
	{
		std::swap( CompCo1, CompCo2 );
	}
	
	//	cout << "Graph::merge compco " 
	//	<< CompCo1->getConnectedComponent()->getId()  << " with " 
	//	<< CompCo2->getConnectedComponent()->getId()  << endl;
	
	CompCo1->getConnectedComponent()->mergeWith( 
																							CompCo2->getConnectedComponent() );
	
	addEdges(CompCo1, CompCo2, DistNodes);
	
	return true;
}

/**
 * Create a Compco
 */
void Graph::createCompco(Node* N)
{
	m_Comp.push_back(new ConnectedComponent(this,N));
}

/**
 * Returns the ith compco
 */
Node* Graph::getCompco(unsigned int ith)
{
	if( ENV.getBool(Env::use_p3d_structures) )
	{
		p3d_compco* component = m_Graph->comp;
		
		for(unsigned int i=0;i<getNumberOfCompco();i++)
		{
			if (i==ith) 
			{
				//			cout << "component = " << component << endl;
				return getNode(component->nodes->N);
			}
			
			component = component->suiv;
		}
	}
	else 
	{
		return m_Comp[ith]->getNodes().at(0);
	}
	
	cout << "Error in Graph::" << __func__ << endl;
	return NULL;
}

/**
 * Get Nodes in the same compco
 */
std::vector<Node*> Graph::getNodesInTheCompCo(Node* node)
{
	p3d_list_node* ListNode = node->getConnectedComponent()->getCompcoStruct()->dist_nodes;
	std::vector<Node*> Nodes;
	
	while (ListNode!=NULL)
	{
		Nodes.push_back( m_NodesTable[ListNode->N] );
		ListNode = ListNode->next;
	}
	return Nodes;
}

void Graph::rebuildCompcoFromBoostGraph()
{
	// Find new compco with the BGL strongly connected components
	vector<int>		component(			boost::num_vertices(m_BoostGraph) );
	vector<int>		discover_time(	boost::num_vertices(m_BoostGraph) );
	
	vector<boost::default_color_type> color(boost::num_vertices(m_BoostGraph));
	vector<BGL_Vertex> root(	boost::num_vertices(m_BoostGraph));
	
	if (m_Nodes.size() != component.size()) 
	{
		throw string("Error in the component size() before the strongly connected algo");
	}
	
  int num = boost::strong_components(m_BoostGraph, 
																		 &component[0], 
																		 boost::root_map(&root[0]).
																		 color_map(&color[0]).
																		 discover_time_map(&discover_time[0]));
	
  cout << "Total number of components: " << num << endl;
	
	BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , m_BoostGraph );
	
	// Delete all components
	while (!m_Comp.empty())
	{
		removeCompco(*m_Comp.begin());
	}
	
	// Make new compcos
	// TODO be carefull with the vertex function
	BGL_Vertex v;
	vector<int> newComponents;
	for (unsigned int i=0;i<component.size();i++ )
	{
		v = vertex(i,m_BoostGraph);
		
		cout	<< "Node id : "			<< NodeData[ v ]->getId() 
					<< " , Comp id : "	<< component[i] << endl;
		
		if( find( newComponents.begin(), newComponents.end(), component[i] ) 
			 != newComponents.end() )
		{
			m_Comp[ component[i] ]->addNode( NodeData[ v ] );
		}
		else 
		{
			newComponents.push_back( component[i] );
			this->createCompco( NodeData[ v ] );
		}
	}	
	
	// The graph has changed
	m_graphChanged = true;
}

/**
 * Detect the need of merging comp
 */
void Graph::mergeCheck()
{
	// This function is a copy of
	
	if( ENV.getBool(Env::use_p3d_structures) )
	{
		p3d_merge_check(m_Graph);
	}
	
	ConnectedComponent *c1, *c2;
	bool needMerge = false;
	
	vector<ConnectedComponent*>::iterator CompScan1 = m_Comp.begin();
	vector<ConnectedComponent*>::iterator CompScan2 = m_Comp.begin();
	
  while ( (CompScan1 != m_Comp.end()) && (needMerge == false)) 
	{
    /* For each other compco of th egraph */
    CompScan2 = m_Comp.begin();
		
    while ((CompScan2 != m_Comp.end()) && (needMerge == false))
		{
      if ((*CompScan1) != (*CompScan2)) 
			{
        /* If a forward and a backward path exists between the scanned compcos */
        if ((*CompScan1)->isLinkedToCompco(*CompScan2) &&
						(*CompScan2)->isLinkedToCompco(*CompScan1))
				{
					/* A merge should be done */
					c1 = (*CompScan1);
					c2 = (*CompScan2);
					needMerge = true;
					break;
				}
      }
      ++CompScan2;
    }
    ++CompScan1;
  }
  /* Do the merge if necessary */
  if (needMerge == true) 
	{
    if (c1->getId() < c2->getId() )
      c1->mergeWith(c2);
    else
      c2->mergeWith(c1);
  }
	
	// Checks C and C++ Compco
	checkConnectedComponents();
}

/**
 * Deletes a connected component
 */
void Graph::removeCompco(ConnectedComponent* CompCo)
{
	// Remove Compco from vector
	vector<ConnectedComponent*>::iterator itCompCo;
	itCompCo = find( m_Comp.begin(), m_Comp.end(), CompCo );
	m_Comp.erase( itCompCo );
	
	// Update graph counter
	if( m_Graph && (!ENV.getBool(Env::use_p3d_structures)) )
	{
		m_Graph->ncomp--;
		m_Graph->last_comp = CompCo->getCompcoStruct()->prec;
	}
	
	//cout << "m_Graph->last_comp : " << m_Graph->last_comp << endl;
	
	// Free memory Compco
	// p3d_remove_compco(m_Graph,CompCo->getCompcoStruct());
	delete CompCo;
	
	if ( m_Comp.empty() ) 
	{
		m_Graph->comp = NULL;
	}
	
	// Checks C and C++ Compco
	if (ENV.getBool(Env::use_p3d_structures)) 
	{
		checkConnectedComponents();
	}
	
	//m_graphChanged = true;
}

bool Graph::checkConnectedComponents()
{
	p3d_compco* lc = m_Graph->comp;
	vector<ConnectedComponent*>::iterator itCompCo;
	
	while (lc) 
	{
		for (itCompCo = m_Comp.begin(); itCompCo != m_Comp.end(); ++itCompCo ) 
		{
			if ( (*itCompCo)->getCompcoStruct() == lc ) 
			{
				break;
			}
			if ( itCompCo == ( m_Comp.end() - 1 ) )
			{
				throw string("The C compco and C++ are not the same");
			}
		}
		lc = lc->suiv;
	}
	//cout << "Compco OK!" << endl;
	return true;
}


//-----------------------------------------------------------
// Misc
//-----------------------------------------------------------
/**
 * Add Cycle in Graph
 */
void Graph::addCycles(Node* node, double step)
{
	double longStep = 3. * step;
	p3d_list_node* listDistNodePt = p3d_listNodeInDist(
																										 m_Robot->getRobotStruct(), node->getNodeStruct()->comp,
																										 node->getNodeStruct(), longStep);
	
	p3d_list_node* savedListDistNodePt = listDistNodePt;
	
	shared_ptr<LocalPath> LP;
	
	while (listDistNodePt)
	{
		if (!p3d_IsSmallDistInGraph(m_Graph, node->getNodeStruct(),
																listDistNodePt->N, 5, step))
		{
			LP = shared_ptr<LocalPath> (new LocalPath(node->getConfiguration(),
																								this->getNode(listDistNodePt->N)->getConfiguration()));
			if (LP->isValid()
					/*&& this->getNode(listDistNodePt->N)->getConfiguration()->costTestSucceeded(
					 node, step)
					 && node->getConfiguration()->costTestSucceeded(
					 this->getNode(listDistNodePt->N), step)*/)
				
			{
				addEdges(node, m_NodesTable[listDistNodePt->N], LP->length() );
				//                p3d_create_edges(m_Graph, node->getNodeStruct(),
				//                                 listDistNodePt->N, LP->length());
				
				node->getNodeStruct()->edges->E->for_cycle = true;
			}
		}
		listDistNodePt = listDistNodePt->next;
	}
	while (savedListDistNodePt)
	{
		p3d_list_node* destroyListNodePt = savedListDistNodePt;
		savedListDistNodePt = savedListDistNodePt->next;
		MY_FREE(destroyListNodePt, p3d_list_node, 1);
	}
}

/**
 * Recompute all node and edge
 * cost
 */
void Graph::recomputeCost()
{
	for(unsigned int i=0; i<m_Nodes.size();i++)
	{
		m_Nodes[i]->cost();
	}
	
	for(unsigned int i=0;i<m_Edges.size();i++)
	{
		m_Edges[i]->cost();
	}
}

/**
 * Check if all edges are valid
 */
bool Graph::checkAllEdgesValid()
{
	int collTest = 0;
	for(unsigned int i=0;i<m_Edges.size();i++)
	{
		shared_ptr<LocalPath> ptrLP = m_Edges[i]->getLocalPath();
		
		if(!(ptrLP->isValid()))
		{
			return false;
		}
		
		collTest += ptrLP->getNbColTest();
	}
	
	cout << "Graph Total Number of coll. test = " << collTest << endl;
	
	return true;
}

API::Trajectory* Graph::extractBestTraj(shared_ptr<Configuration> qi,
																				shared_ptr<Configuration> qf)
{
	//cout << "----------------------------------------------" << endl;
	cout << "Extracting the trajectory" << endl;
	
	Node  *Ns=NULL,*Ng=NULL;
	p3d_traj* trajPt = NULL;
	
	bool drawWarning = false;
	
	if(m_Graph == NULL) 
	{
		cout << "Warning: cannot extract the best path\
		as there is no current graph" << endl;
		return 0x00;
	}
	
	if(qf->isInCollision()) 
	{
		(m_Graph->nb_test_coll)++;
		cout << "Computation of approximated optimal cost stopped: \
		Goal configuration in collision" << endl;
		return 0x00;
	}
	
	if ( !ENV.getBool(Env::use_p3d_structures) ) 
	{
		//cout << "Exporting the C++ graph to a p3d_graph for the trajectory extraction" << endl;
		//m_Graph = exportCppToGraphStruct(true);
		//init();
		//updateCompcoFromStruct();
	}
	
	// start and goal nodes creation and initialisation
	Ns = searchConf(*qi);
	if(Ns == NULL) 
	{
		Ns = new Node(this,qi);
		insertExtremalNode(Ns);
		cout << "Create start node" <<  endl;
	}
	
	if (drawWarning) 
	{
		cout << "Start comp : " << Ns->getNodeStruct()->comp->num << endl;
	}
	
	Ng = searchConf(*qf);
	if(Ng == NULL) 
	{
		Ng = new Node(this,qf);
		insertExtremalNode(Ng);
		cout << "Create goal node" <<  endl;
	}
	
	if (drawWarning) 
	{
		cout << "Goal comp : " << Ng->getNodeStruct()->comp->num << endl;
	}
	
	// Set the graph variables for a 
	// motion planning querry
	initMotionPlanning(Ns,Ng);
	
	m_Robot->setAndUpdate(*Ns->getConfiguration());
	
	//search of the main constructed connected compoant
	p3d_compco* actualCompco = m_Graph->comp;
	p3d_compco* mainCompPt = actualCompco;
	cout << "Nb nb of comp : " << m_Graph->ncomp << endl;
	for(int i=0;i<m_Graph->ncomp;i++)
	{
		if (drawWarning) 
		{
			cout << "Id of actual comp : " << actualCompco->num << endl;
			cout << "Nb of nodes in actual comp : " << actualCompco->nnode << endl;
		}
		
		if (actualCompco->nnode > mainCompPt->nnode) 
		{
			mainCompPt = actualCompco;
		}
		actualCompco = actualCompco->suiv;
	}
	
	if (drawWarning) 
	{
		cout << "Number of connected component : " << m_Graph->ncomp << endl;
		cout << "Main connected component : " << mainCompPt->num << endl;
	}
	
	//connection of the the extremal nodes to the main componant
	bool ConnectRes = true;
	
	ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(m_Graph,  
																									 Ns->getNodeStruct(), mainCompPt));
	ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(m_Graph, 
																										Ng->getNodeStruct(), mainCompPt)) ;
	
	if(!ConnectRes) 
	{
		cout << "Connection of the extremal nodes failed" << endl;
	}
	else 
	{
		// on construit la trajectoire entre les points etapes
		(m_Graph->nb_test_coll)++;
		cout << "Connection of the extremal nodes succeeded" << endl;
		//cout << m_Graph->ncomp << endl;
		trajPt = p3d_graph_to_traj(m_Robot->getRobotStruct());
	}
	
	// time info
	//	m_Graph->time = m_Graph->time + tu;
	
	if( (ConnectRes == true) && trajPt) 
	{
		API::Trajectory* traj = new API::Trajectory(m_Robot,trajPt);
		//cout << "Trajectory cost  = " << traj->cost() << endl;
		return traj;
	}
	else 
	{
		cout << "Trajectory extraction failed" << endl;
		return 0x00;
	}
}

/**
 * Replaces p3d_InitRun
 */
void Graph::initMotionPlanning(Node* start,Node* goal)
{
#ifdef ENERGY
	int n_coldeg,icoldeg;
	double *coldeg_qs;
#endif
	m_Graph->search_start = start->getNodeStruct();
	if(ENV.getBool(Env::expandToGoal)== true) 
	{
		m_Graph->search_goal = goal->getNodeStruct();
	}
	
	m_Graph->search_done = FALSE;
	p3d_InitDynDomainParam(m_Graph,start->getNodeStruct(),goal->getNodeStruct());
	
	if(ENV.getBool(Env::isCostSpace) == true) 
	{
		global_costSpace->initMotionPlanning(this, start, goal);
	}
	if (p3d_GetIsWeightedChoice()== TRUE) 
	{
		p3d_init_root_weight(m_Graph);
	}
	if(ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH ) 
	{
		p3d_init_pb_variables(m_Graph);
	}
	if(start != NULL) 
	{
		start->getNodeStruct()->rankFromRoot = 1;
		start->getNodeStruct()->comp->nbTests = 0;
	}
	if(goal != NULL) 
	{
		goal->getNodeStruct()->rankFromRoot = 1;
		goal->getNodeStruct()->comp->nbTests = 0;
	}
#ifdef ENERGY
	if(p3d_get_MOTION_PLANNER() ==  BIO_COLDEG_RRT) 
	{
		n_coldeg = bio_get_num_collective_degrees();
		// init coldeg_q in Ns
		coldeg_qs = bio_alloc_coldeg_config(n_coldeg);
		for(icoldeg=0; icoldeg<n_coldeg; icoldeg++) 
		{
			coldeg_qs[icoldeg] = 0.0;
		}
		bio_copy_coldeg_q_in_N(start,coldeg_qs,n_coldeg);
		bio_destroy_coldeg_config(coldeg_qs,n_coldeg);
		// WARNING : currently Ng is not considered !!!
	}
#endif
}
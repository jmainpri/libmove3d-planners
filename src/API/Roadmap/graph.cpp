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
#include "API/Roadmap/graphConverter.hpp"

#include "planEnvironment.hpp"
#include "cost_space.hpp"

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "GroundHeight-pkg.h"

#include "move3d-headless.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

#undef None
#undef Upper
#undef Lower

#include <boost/config.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/foreach.hpp>

extern void* GroundCostObj;

Graph* API_activeGraph = NULL;

using namespace std;
using namespace tr1;

const bool graph_debug_import_export = false;

// Constructors
//----------------------------------------------

//! Constructor from graph and robot
//! @param R the robot for which the graph is created
//! @param G the old graph structure
Graph::Graph(Robot* R, p3d_graph* G)
{
	m_Robot = R;
  
  if (G)
	{
		m_Graph = new p3d_graph(*G);
	}
	else
	{
		m_Graph = p3d_create_graph(R->getRobotStruct());
    cout << "Allocate graph : " << m_Graph << endl;
	}
	
	m_Graph->rob->GRAPH = m_Graph;
	m_graphChanged = true;
  m_initBGL = false;
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

	if (STAT)
	{
		m_Graph->stat = createStat();
	}
	else
	{
		m_Graph->stat = NULL;
	}

	m_graphChanged = true;
  m_initBGL = false;
  
	this->init();
	this->initBGL();
}

//! Constructeur de la classe
//! 
//! @param G la structure p3d_graph qui sera stockée
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
	m_graphChanged = true;
  m_initBGL = false;
  
	this->init();
	this->initBGL();
}

//! Graph copy constructor : be sure that the connected componants are updated
//! @param G the graph
Graph::Graph(const Graph& G)
{
  cout << "Copy constructor broken" << endl;
  return;
  
	//m_Graph = G.exportGraphStruct();
  GraphConverter gc;
  m_Graph = gc.exportGraphStruct( G );
  
	m_Robot = G.m_Robot;
	
	m_graphChanged = true;
	m_initBGL = false;
  
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
    
    m_initBGL = true;
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
  API_activeGraph = NULL;
  
	if( graph_debug_import_export )
	{
		cout << "Graph::~Graph() => Deleting graph" << endl;
	}
	
	// Deletes the old graph
  cout << "Mark the XYZ_GRAPH as deleted" << endl;
  m_Robot->getRobotStruct()->GRAPH = XYZ_GRAPH = NULL;
  delete m_Graph;
	
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
		
		if((m_Graph->stat != NULL))
		{
			destroyStat(&(m_Graph->stat));
		}
		
		/* Delete references to the graph */
		if (m_Graph->rob != NULL) 
		{
			m_Graph->rob->GRAPH = NULL;
		}

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

// Accessors
//----------------------------------------------
p3d_graph* Graph::getGraphStruct() const
{
	return m_Graph;
}

void Graph::setGraph(p3d_graph* G)
{
	*m_Graph = *G;
}

Robot* Graph::getRobot() const
{
	return m_Robot;
}

/*void Graph::setRobot(Robot* R)
 {
 m_Robot = R;
 }*/

unsigned int Graph::getNumberOfNodes()
{
  unsigned int nbNodes = num_vertices(m_BoostGraph);
  
  if ( nbNodes != m_Nodes.size() ) 
  {
    throw string("Verticies in boost graph don't reflect the C++ graph");
  }
  
  return nbNodes;
}

unsigned int Graph::getNumberOfEdges()
{
  unsigned int nbEdges = num_edges(m_BoostGraph);
  
  if ( nbEdges != m_Edges.size() ) 
  {
    throw string("Edges in the boost graph don't reflect the C++ graph");
  }
  
  return nbEdges;
}

vector<Node*> Graph::getNodes() const
{
	return m_Nodes;
}

vector<Edge*> Graph::getEdges() const
{
	return m_Edges;
}

vector<ConnectedComponent*> Graph::getConnectedComponents() const
{
	return m_Comp;
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
  for(int i=0;i<int(m_Nodes.size());i++)
  {
    if(m_Nodes[i]->getConfiguration()->equal(q))
    {
      return m_Nodes[i];
    }
  }
  return NULL;
}

/**
 * Insert node in graph
 */
Node* Graph::insertNode(Node* node)
{
	addNode(node);
  return (node);
}

/**
 * Insert Extremal Node in Graph
 */
Node* Graph::insertExtremalNode(Node* node)
{
	insertNode(node);
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
	
	// Insert the end configuration of the input localpath
	// as a node in the graph
	Node* newNode = insertConfigurationAsNode(path.getEnd(), expansionNode, step);
	
	// Set expansion node as the node parent
	newNode->parent() = expansionNode;
	
	// Cost space special updates
	if (ENV.getBool(Env::isCostSpace))
	{
		double currentCost = path.getEnd()->cost();
		
    global_costSpace->setNodeCost( newNode, newNode->parent() );
    
		//for adaptive variant, new temp is refreshed except if it is going down.
		if (currentCost < expansionNode->cost() )
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
  
  m_graphChanged = true;
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
	for ( int i=0; i<int(m_Edges.size()); i++)
	{
    if((m_Edges[i]->getSource()==N1 )&&( m_Edges[i]->getTarget()==N2))
    {
      return m_Edges[i];
    }
	}
	return NULL;
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
		for ( int i=0; i<int(m_Nodes.size()); i++)
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
		LocalPath path(N1->getConfiguration(), N2->getConfiguration());
		Length = path.getParamMax();
	}
	
	addEdge(N1, N2, Length);
	addEdge(N2, N1, Length);
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
	
	for( int i=0; i<int(toDelete.size()); ++i )
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
	
	// Warning TODO:  this function does not work with the C++ API
  nbLinkedComponents = p3d_link_node_graph_multisol(N->getNodeStruct(), m_Graph);
	
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

/**
 *Check if two nodes are linked using the collision checking method
 */
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
	{
		dist = LP->getParamMax();
	}
  else
	{
		PrintInfo(("Warning: created an edge with a 0 distance: no localpathPt->length \n"));
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
	
  if(m_Graph)
	{
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
	
  if (areNodesLinked(node1, node2, DistNodes)) 
  {
    mergeComp(node1, node2, DistNodes); 
    IsLinkedAndMerged = true;
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
			
			if (fct_stop != NULL)
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
  // The null node has to be remouved
	Node* nullNode(NULL);
	double CurrentScore;
  bool firstNode = true;
  
	vector< pair<double,Node*> > nearNodesQueue;
	nearNodesQueue.push_back( make_pair(numeric_limits<double>::max(),nullNode) );
	
	//cout << "Number of nodes in the graph : " << m_Nodes.size() << endl;
	
	for (vector<Node*>::iterator it = m_Nodes.begin();it != m_Nodes.end() ; ++it)
	{
    // Compute the distance for all nodes
    CurrentScore = q->dist(*(*it)->getConfiguration(),distConfigChoice);
    //*(weighted ? p3d_GetNodeWeight((*it)->getNodeStruct()) : 1.0);

    // Do not add the nodes outside of radius
    if ( CurrentScore > radius) continue;
    
    // If better than last node of the Queue add to the Queue (rewrite better code)
    if (CurrentScore < nearNodesQueue.back().first)
    {
      if ( firstNode ) 
      {
        nearNodesQueue.clear(); firstNode = false;
      }
      nearNodesQueue.push_back( make_pair( CurrentScore, *it ) );
      sort( nearNodesQueue.begin(), nearNodesQueue.end() );
      
      if( int(nearNodesQueue.size()) > K )
      {
        nearNodesQueue.resize( K );
      }
    }
	}
	
	// Put queue in a vector
	vector<Node*> nearNodes;
	for( int i=0;i<int(nearNodesQueue.size());i++)
  {
    Node* node = nearNodesQueue[i].second;
    if (node!= NULL) {
      nearNodes.push_back( node );
    }
  }
	
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
//	double BestScore = P3D_HUGE;
//	double CurrentDist = -1.;
//	double CurrentScore = -1.;
	double DistOfBestNode = -1.;
	p3d_matrix4 *RefFramePt = NULL, *MobFramePt = NULL;
	p3d_matrix4 MobFrameRef, invT;
	
	Node* BestNode = NULL;
	
	// When retrieving statistics
	if (getStatStatus())
	{
		m_Graph->stat->planNeigCall++;
	}
	
	//computation of the mobFrameRef of the Config
	if (distConfigChoice == MOBILE_FRAME_DIST && p3d_GetRefAndMobFrames( m_Graph->rob, &RefFramePt, &MobFramePt))
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
	
  BestNode = compco->getConnectedComponent()->nearestWeightNeighbour( config, weighted, distConfigChoice );
	
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

//-----------------------------------------------------------
// Connected componnents
//-----------------------------------------------------------

unsigned int Graph::getNumberOfCompco()
{
	return m_Graph->ncomp;
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
  return m_Comp[ith]->getNodes().at(0);
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
	if( m_Graph )
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

// This function works for tree type of graphs
API::Trajectory* Graph::extractBestTrajSoFar( std::tr1::shared_ptr<Configuration> qi, shared_ptr<Configuration> qf )
{
  // Start needs to be in the graph
  Node* source = searchConf(*qi);
	if( source == NULL )
  {
    cout << "Start not in graph in " << __func__ << endl;
    return NULL;
  }
  
  Node* node = nearestWeightNeighbour(source, qf, false, ENV.getInt(Env::DistConfigChoice));
  if( node == NULL ) {
    return NULL;
  }
  
  // Extract the nodes in reverse order
  vector<Node*> traj_nodes;
  traj_nodes.push_back( node );
  while ( !node->getConfiguration()->equal(*qi) ) 
  {
    node = node->parent();
    traj_nodes.push_back( node );
    
    if (node == NULL) {
      cout << "Error finding nodes in " << __func__ << endl;
      return NULL;
    }
  }
  
  if( int(traj_nodes.size()) <= 1 ) {
    return NULL;
  }
  
  // Build the trajectory
  API::Trajectory* traj = new API::Trajectory(m_Robot);
  for (int i=int(traj_nodes.size())-1; i>=0; i--) {
    traj->push_back( traj_nodes[i]->getConfiguration() );
  }
  
  traj->replaceP3dTraj();
  return traj;
}


// This function extracts a trajectory
// it first converts the grapgh to a p3d_graph
API::Trajectory* Graph::extractBestTraj(shared_ptr<Configuration> qi,
																				shared_ptr<Configuration> qf)
{  	
//  cout << "----------------------------------------------" << endl;
//	cout << "Extracting the trajectory" << endl;
	
	Node  *Ns=NULL,*Ng=NULL;
	p3d_traj* trajPt = NULL;
	
	bool drawWarning = false;
	
	if( m_Graph == NULL ) 
	{
		cout << "Warning: cannot extract the best path\
		as there is no current graph" << endl;
		return NULL;
	}
	
	if(qf->isInCollision()) 
	{
		(m_Graph->nb_test_coll)++;
		cout << "Computation of approximated optimal cost stopped: \
		Goal configuration in collision" << endl;
		return NULL;
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
		cout << "Start comp : " << Ns->getConnectedComponent()->getId() << endl;
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
		cout << "Goal comp : " << Ng->getConnectedComponent()->getId() << endl;
	}
  
  GraphConverter gc;
  m_Graph = gc.convert(*this);
  
  // start and goal nodes creation and initialisation
  p3d_node* Ns_ = p3d_TestConfInGraph( m_Graph, qi->getConfigStruct() );
  p3d_node* Ng_ = p3d_TestConfInGraph( m_Graph, qf->getConfigStruct() );
  
  if( Ns_== NULL || Ng_ == NULL )
  {
    cout << "Error in " << __func__ << endl;
    return NULL;
  }

	// Set the graph variables for a motion planning querry
	initMotionPlanning(Ns_,Ng_);
	
	m_Robot->setAndUpdate(*Ns->getConfiguration());
	
	// Search for the main constructed connected compoant
	p3d_compco* actualCompco = m_Graph->comp;
	p3d_compco* mainCompPt = actualCompco;
	//cout << "Nb nb of comp : " << m_Graph->ncomp << endl;
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
	
	// Connection of the the extremal nodes to the main componant
	bool ConnectRes = true;
	
	ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(m_Graph, Ns_, mainCompPt));
	ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(m_Graph, Ng_, mainCompPt));
	
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
	// m_Graph->time = m_Graph->time + tu;
	
	if( ConnectRes && trajPt) 
	{
		API::Trajectory* traj = new API::Trajectory(m_Robot,trajPt);
		//cout << "Trajectory cost  = " << traj->cost() << endl;
		return traj;
	}
	else 
	{
		cout << "Trajectory extraction failed" << endl;
		return NULL;
	}
}

/**
 * Replaces p3d_InitRun
 */
void Graph::initMotionPlanning( p3d_node* start, p3d_node* goal )
{
#ifdef ENERGY
	int n_coldeg,icoldeg;
	double *coldeg_qs;
#endif
	m_Graph->search_start = start;
	if(ENV.getBool(Env::expandToGoal)== true) 
	{
		m_Graph->search_goal = goal;
	}
	
	m_Graph->search_done = FALSE;
	p3d_InitDynDomainParam(m_Graph,start,goal);
	
	if(ENV.getBool(Env::isCostSpace) == true) 
	{
		global_costSpace->initMotionPlanning( m_Graph, start, goal);
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
		start->rankFromRoot = 1;
		start->comp->nbTests = 0;
	}
	if(goal != NULL) 
	{
		goal->rankFromRoot = 1;
		goal->comp->nbTests = 0;
	}
}

void Graph::drawEdge(BGL_Vertex v1, BGL_Vertex v2)
{
  int color = 0;
  
  p3d_jnt* 	drawnjnt=NULL;
	int indexjnt = p3d_get_user_drawnjnt();
  if (indexjnt != -1 && indexjnt >= 0 && indexjnt <= m_Robot->getRobotStruct()->njoints ) 
  {
    drawnjnt = m_Robot->getRobotStruct()->joints[indexjnt];
  }
  
  if (drawnjnt == NULL) {
    return;
  }
  
  BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , m_BoostGraph );
  
  m_Robot->setAndUpdate(*NodeData[v1]->getConfiguration());
  p3d_vector3 source_pos;
  p3d_jnt_get_cur_vect_point(drawnjnt, source_pos);
  
  m_Robot->setAndUpdate(*NodeData[v2]->getConfiguration());
  p3d_vector3 target_pos;
  p3d_jnt_get_cur_vect_point(drawnjnt, target_pos);
  
  if(GroundCostObj)
  {
    double Cost1(0);
    GHintersectionVerticalLineWithGround(GroundCostObj, source_pos[0], source_pos[1], &Cost1);
    double Cost2(0);
    GHintersectionVerticalLineWithGround(GroundCostObj, target_pos[0], target_pos[1], &Cost2);
    g3d_drawOneLine(source_pos[0], source_pos[1], Cost1 + 0.5, target_pos[0], target_pos[1], Cost2 + 0.5, color, NULL);
  }
  else
  {
    g3d_drawOneLine(source_pos[0], source_pos[1], source_pos[2],
                    target_pos[0], target_pos[1], target_pos[2], color, NULL);
  }
}

void Graph::draw()
{
  cout << "Draw BGL Graph , nb of nodes : " << m_Nodes.size() << endl;
  
  shared_ptr<Configuration> q = m_Robot->getCurrentPos();
  
  BOOST_FOREACH(BGL_Edge e, boost::edges(m_BoostGraph))
  {
    BGL_Vertex v1(boost::source(e, m_BoostGraph));
    BGL_Vertex v2(boost::target(e, m_BoostGraph));
    drawEdge(v1, v2);
  }
  
  m_Robot->setAndUpdate(*q);
}

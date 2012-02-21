//
// C++ Implementation: node
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "API/ConfigSpace/configuration.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/BGL_Graph.hpp"

#include <tr1/memory>

#include "planEnvironment.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

Node::Node() :
m_Node(NULL),
m_parent(NULL),
_SelectCost(0.0),
_nbExpan(0),
m_specificNode(false)
{
	m_is_BGL_Descriptor_Valid = false;
  m_is_leaf = true;
}

//Constructor and destructor
Node::Node(const Node& N) :
m_Graph(N.m_Graph),
m_Robot(N.m_Robot),
m_parent(NULL),
m_Configuration(N.m_Configuration),
_activ(false),
_SelectCost(0.0),
_nbExpan(0),
m_specificNode(false)
{
	throw string("Copy constructor has to be removed");
	
	m_Node = new p3d_node(*N.m_Node);
	
	// BGL data
	m_is_BGL_Descriptor_Valid = false;
	m_BGL_Descriptor = N.m_BGL_Descriptor;
  m_is_leaf = true;
}

//Constructor and destructor
Node::Node(Graph* G, shared_ptr<Configuration> C, bool newCompco) :
m_parent(NULL),
_SelectCost(0.0),
_nbExpan(0),
m_specificNode(false)
{
	m_is_BGL_Descriptor_Valid = false;
	m_Graph = G;
	m_Robot = G->getRobot();
	m_Configuration = C;
	_activ = false;
  m_is_leaf = true;
  
	// The node has to be made
	// without using the old graph struck
	// this is a copy of the p3d_APInode_make_multisol
  m_Node = p3d_create_node( NULL );
  m_Node->q = C->getConfigStruct();
	m_Node->num = m_Graph->getNumberOfNodes();
	m_Node->parent = NULL;
	m_Node->sumCost = 0.0;
	
	int singularityCheck = 0;
  if(singularityCheck){
    m_Node->isSingularity = TRUE;
  }
  
	// Compco are now handled in the boost graph
	if (newCompco) 
	{
		m_Graph->createCompco(this);
	}
}

Node::Node(Graph* G, p3d_node* N) :
_SelectCost(0.0),
_nbExpan(0),
m_specificNode(false)
{
	m_Graph = G;
	m_Robot = G->getRobot();
	m_Configuration = shared_ptr<Configuration> (new Configuration(m_Robot, N->q));
  
	_activ = false;
	m_Node = N;
  m_is_leaf = true;
	
	if (m_Node->comp == NULL)
	{
		m_Graph->createCompco(this);
	}
}

bool Node::operator==(Node& N)
{
	return *m_Configuration == *N.m_Configuration;
}

void Node::deleteNode()
{
	if (m_Node) 
	{
		if (m_Node->list_closed_flex_sc != NULL) 
		{
			delete m_Node->list_closed_flex_sc;
		}
		if(m_Node->iksol)
		{
			p3d_destroy_specific_iksol(m_Robot->getRobotStruct()->cntrt_manager, m_Node->iksol);
			m_Node->iksol = NULL;
		}
		delete m_Node;
	}	
}

Node::~Node()
{
  if(m_specificNode)
    
  {
    // cout << "specific (node) = destroy" << endl; 
  }
	//this->deleteNode();
}

//Accessors

/**
 * Returns the temperature
 */
double Node::getTemp() 
{ 
	return(m_Node->temp); 
}

/**
 * modifie la temperature du Node
 * @param t la nouvelle temperature
 */
void Node::setTemp(double t) 
{ 
	m_Node->temp = t; 
}

/**
 * Returns the node id
 */
unsigned int Node::getId() const
{
	return m_Node->num;
}

/**
 * Sets the Id
 */
void Node::setId(unsigned int id)
{
	m_Node->num = id;
}

/**
 * Get Number of neighbors
 */
int Node::getNumberOfNeighbors()
{ 
  return getNeighbors().size();
}

//! Get number of neighbors
vector<Node*> Node::getNeighbors()
{
	vector<Node*> allNeighbors;
	
	boost::graph_traits<BGL_Graph>::adjacency_iterator ai;
	boost::graph_traits<BGL_Graph>::adjacency_iterator ai_end;
	
	BGL_Graph& g = m_Graph->get_BGL_Graph();
	BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , g );
  BGL_Vertex u = getDescriptor();
	
	for (tie(ai, ai_end) = adjacent_vertices(u, g); ai != ai_end; ++ai)
	{
		allNeighbors.push_back( NodeData[*ai] );
	}
	return allNeighbors;
}

void Node::printNeighbors()
{
  vector<Node*> allNeighbors = getNeighbors();
  vector<Node*>::iterator it;
  cout << "--------------------------------------" << endl;
  for (it = allNeighbors.begin(); it != allNeighbors.end(); ++it) 
  {
    cout << "Node num (neigh): " << (*it)->getId() << endl;
  }
}

//! Get Number of Edges
int Node::getNumberOfEdges()
{ 
  return getEdges().size();
}

vector<Edge*> Node::getEdges()
{
	vector<Edge*> allEdges; 
	
	BGL_Graph& g = m_Graph->get_BGL_Graph();
	BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , g );
  BGL_Vertex u = getDescriptor();
  
  // Get out edges
  typedef boost::graph_traits<BGL_Graph>::out_edge_iterator out_edge_iter;
  out_edge_iter beginOut, endOut;
  boost::tie(beginOut, endOut) = boost::out_edges( u, g );
  for( out_edge_iter it = beginOut; it != endOut; ++(it))
  {
    // allEdges.push_back( EdgeData[*it] );
    // allEdges.push_back( g[*ep.first] );
    allEdges.push_back( boost::get(EdgeData, *it) );
  }
  
  // Get in edges
  typedef boost::graph_traits<BGL_Graph>::in_edge_iterator in_edge_iter;
  in_edge_iter beginIn, endIn;
  boost::tie(beginIn, endIn) = boost::in_edges( u, g );
  for (in_edge_iter it = beginIn; it != endIn; ++(it))
  {
    allEdges.push_back( EdgeData[*it] );
  }

	return allEdges;
}

p3d_node* Node::getNodeStruct()
{
	return m_Node;
}

Graph* Node::getGraph()
{
	return m_Graph;
}

Robot* Node::getRobot()
{
	return m_Robot;
}

shared_ptr<Configuration> Node::getConfiguration()
{
	return m_Configuration;
}

void Node::activ(bool b)
{
	_activ = b;
}

bool Node::isActiv()
{
	return _activ;
}

double Node::cost()
{
	m_Node->cost = m_Configuration->cost();
	return (m_Node->cost);
}

//! When using a tree structure compute the sum of cost
//! to this node
double& Node::sumCost(bool recompute)
{
  if( recompute ) 
  {
    double sum_cost(0);
    Node* node = this;
    
    while(node->parent() != NULL) {
      
      Edge* edge = NULL;
      vector<Edge*> edges = node->getEdges();

      for (int i=0; i<int(edges.size()); i++) {

        if((edges[i]->getTarget() == node) && (edges[i]->getSource() == node->parent())) {
          edge = edges[i];
          break;
        }
      }
      sum_cost += edge->cost();
      node = node->parent();
    }
    m_Node->sumCost = sum_cost;
  }
  
	return (m_Node->sumCost);
}

double Node::dist(Node* node) const
{
	double d = m_Configuration->dist(*node->m_Configuration);
	m_Node->dist_Nnew = d;
	return d;
}

double Node::dist(confPtr_t q) const
{
	double d = m_Configuration->dist(*q);
	m_Node->dist_Nnew = d;
	return d;
}

//! Copy of p3d_APInode_dist_multisol
double Node::distMultisol(Node *node) const
{
  int *ikSol = NULL;
  double dist = P3D_HUGE;
  //if the two nodes are in the same solution class
  if (p3d_compare_iksol(m_Robot->getRobotStruct()->cntrt_manager, m_Node->iksol, node->m_Node->iksol))
  {
    p3d_get_non_sing_iksol(m_Robot->getRobotStruct()->cntrt_manager, m_Node->iksol, node->m_Node->iksol, &ikSol);
    dist = p3d_dist_q1_q2_multisol(m_Robot->getRobotStruct(), m_Configuration->getConfigStruct(), node->m_Configuration->getConfigStruct(), ikSol);
  }
  m_Node->dist_Nnew = dist;
  return dist;
}

bool Node::equal(Node* node) const
{
	return m_Configuration->equal(*node->m_Configuration);
}

//! Function that is the copy of p3d_APInode_linked
//! that relies on the old graph
//! Compute the localpath using the local method associated to the robot 
bool Node::isLinkable(Node* N, double* dist) const
{
  // current position of robot is saved 
  confPtr_t qsave = m_Robot->getCurrentPos();
	
	int DEBUGm_Graph_API = 0;
  if (DEBUGm_Graph_API){
    printf("API Node Linked :\n");
    p3d_print_iksol( m_Robot->getRobotStruct()->cntrt_manager,m_Node->iksol);
    p3d_print_iksol(m_Robot->getRobotStruct()->cntrt_manager,N->m_Node->iksol);
  }
  
  int isNoCol = 0, *ikSol = NULL;
  
  if(!p3d_compare_iksol(m_Robot->getRobotStruct()->cntrt_manager, m_Node->iksol, N->m_Node->iksol)) {
    return false;
  }
  
  p3d_get_non_sing_iksol(m_Robot->getRobotStruct()->cntrt_manager, m_Node->iksol, N->m_Node->iksol, &ikSol);
  
  LocalPath path(m_Configuration, N->m_Configuration);
  path.setIkSol( ikSol );
  if ( path.getLocalpathStruct(true) == NULL) { // With Ik Sol
    return false;
  }
	
  if (path.getLocalpathStruct()->length != NULL)
		*dist = path.getLocalpathStruct()->length(m_Robot->getRobotStruct(),path.getLocalpathStruct());
  else{
		PrintInfo(("Warning: created an edge with \
							 a 0 distance: no localpathPt->length \n"));
		*dist = 0;
	}
  
  if((p3d_get_SORTING()==P3D_NB_CONNECT)&&(p3d_get_MOTION_PLANNER()==P3D_BASIC)) {
    if((*dist > p3d_get_DMAX())&&(LEQ(0.,p3d_get_DMAX()))){ 
      // ecremage deja fait dans le cas tri par distance... 
      // The initial position of the robot is recovered 
      m_Robot->setAndUpdate(*qsave);
      return false;
    }
  }
  
  //start path deform
  if (p3d_get_cycles() == TRUE) {
    if (path.getLocalpathStruct()->length != NULL)
      *dist = path.getLocalpathStruct()->length(m_Robot->getRobotStruct(),path.getLocalpathStruct());
    else {
      PrintInfo(("linked: no distance function specified\n"));
      *dist = 0;
    }
  }
  
	// See this later (Add counts to local methods and collision tests)
  // The initial position of the robot is recovered 
  isNoCol = path.isValid();
  m_Robot->setAndUpdate(*qsave);
  return isNoCol;
}

// Copy of p3d_APInode_linked_multisol
bool Node::isLinkableMultisol(Node* node, double* dist) const
{
  if (p3d_compare_iksol(m_Robot->getRobotStruct()->cntrt_manager, m_Node->iksol, node->m_Node->iksol))
  {
    if ( m_Node->isSingularity || node->m_Node->isSingularity )
    {
      if (!p3d_test_singularity_connexion(m_Robot->getRobotStruct()->cntrt_manager, m_Node, node->m_Node))
      {
        return false;
      }
    }
    //return p3d_APInode_linked( m_Graph->getGraphStruct(), m_Node, node->m_Node, dist );
    return isLinkable( node, dist );
  }
  return false;
}

void Node::checkStopByWeight()
{
	double stopWeight;
	int signStopWeight;
	p3d_GetStopWeightAndSign(&stopWeight, &signStopWeight);
	if (signStopWeight * (m_Node->weight - stopWeight) > 0)
	{
		PlanEnv->setBool(PlanParam::stopPlanner,true);
    //		p3d_SetStopValue(true);
		p3d_SetDiffuStoppedByWeight(true);
	}
}

// fonctions sur les composantes connexes
void Node::deleteCompco()
{
	// this is now handled in the Boost Graph
	m_Graph->removeCompco(m_Compco);
}

bool Node::maximumNumberNodes()
{
	return ((int)m_Compco->getNumberOfNodes()) >= ENV.getInt(Env::maxNodeCompco);
}

void Node::setConnectedComponent(ConnectedComponent* Compco)
{
	m_Compco = Compco;
	m_Node->numcomp = m_Compco->getId();
}

bool Node::inSameComponent(Node* node) const
{
	return ( m_Compco == node->m_Compco );
}

bool Node::connectNodeToCompco(Node* node, double step)
{
	if (ENV.getBool(Env::isCostSpace))
	{
		cout << "WARNING: Using Cost Space with wrong algortihm" << endl;
		return false;
	}
	else
	{
		return m_Graph->connectNodeToCompco(node,this);
	}
}

//place la compco dans la CompCo presente
void Node::merge(Node* compco)
{
	m_Graph->mergeComp(this,compco,dist(compco));
}

bool Node::equalCompco(Node* compco) const
{
	return ( m_Compco == compco->m_Compco );
}

Node* Node::randomNodeFromComp() const
{
	return (m_Graph->getNode(p3d_RandomNodeFromComp(m_Node->comp)));
}

void Node::print() const
{
	m_Configuration->print();
}

//---------------------------------------------------
// BGL functions
//---------------------------------------------------
void	Node::setDescriptor(const BGL_Vertex& V) 
{ 
	m_is_BGL_Descriptor_Valid=true; 
	m_BGL_Descriptor=V; 
}

void	Node::unSetDescriptor() 
{ 
	m_is_BGL_Descriptor_Valid=false; 
}

BGL_Vertex Node::getDescriptor()
{
	if (  m_is_BGL_Descriptor_Valid  ) 
	{
		return m_BGL_Descriptor;
	}
	else 
	{
		m_is_BGL_Descriptor_Valid = true;
		m_BGL_Descriptor = m_Graph->findVertexDescriptor(this);
		return m_BGL_Descriptor;
	}
}


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
_Node(NULL),
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
_Graph(N._Graph),
_Robot(N._Robot),
m_parent(NULL),
_Configuration(N._Configuration),
_activ(false),
_SelectCost(0.0),
_nbExpan(0),
m_specificNode(false)
{
	throw string("Copy constructor has to be removed");
	
	_Node = new p3d_node(*N._Node);
	
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
	_Graph = G;
	_Robot = G->getRobot();
	_Configuration = C;
	_activ = false;
  m_is_leaf = true;
  
	// The node has to be made
	// without using the old graph struck
	// this is a copy of the p3d_APInode_make_multisol
  _Node = p3d_create_node( NULL );
  _Node->q = C->getConfigStruct();
	_Node->num = _Graph->getNumberOfNodes();
	_Node->parent = NULL;
	_Node->sumCost = 0.0;
	
	int singularityCheck = 0;
  if(singularityCheck){
    _Node->isSingularity = TRUE;
  }
  
	// Compco are now handled
	// in the boost graph
	if (newCompco) 
	{
		_Graph->createCompco(this);
	}
}

Node::Node(Graph* G, p3d_node* N) :
_SelectCost(0.0),
_nbExpan(0),
m_specificNode(false)
{
	_Graph = G;
	_Robot = G->getRobot();
	_Configuration = shared_ptr<Configuration> (new Configuration(_Robot, N->q));
	_activ = false;
	_Node = N;
  m_is_leaf = true;
	
	if (_Node->comp == NULL)
	{
		_Graph->createCompco(this);
	}
}

bool Node::operator==(Node& N)
{
	return *_Configuration == *N._Configuration;
}

void Node::deleteNode()
{
	if (_Node) 
	{
		if (_Node->list_closed_flex_sc != NULL) 
		{
			delete _Node->list_closed_flex_sc;
		}
		if(_Node->iksol)
		{
			p3d_destroy_specific_iksol(_Robot->getRobotStruct()->cntrt_manager, _Node->iksol);
			_Node->iksol = NULL;
		}
		delete _Node;
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
	return(_Node->temp); 
}

/**
 * modifie la temperature du Node
 * @param t la nouvelle temperature
 */
void Node::setTemp(double t) 
{ 
	_Node->temp = t; 
}

/**
 * Returns the node id
 */
unsigned int Node::getId()
{
	return _Node->num;
}

/**
 * Sets the Id
 */
void Node::setId(unsigned int id)
{
	_Node->num = id;
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
	
	BGL_Graph& g = _Graph->get_BGL_Graph();
	BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , g );
	
	for (tie(ai, ai_end) = adjacent_vertices(  getDescriptor() , g); ai != ai_end; ++ai)
	{
		Node* N = NodeData[*ai] ;
		allNeighbors.push_back( N );
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
	
	boost::graph_traits<BGL_Graph>::out_edge_iterator out_i, out_end;
	BGL_Graph& g = _Graph->get_BGL_Graph();
	
	BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , g );
	
	for (tie(out_i, out_end) = boost::out_edges( getDescriptor()  , g ); 
			 out_i != out_end; ++out_i) 
	{
		Edge * E = boost::get( EdgeData , *out_i );
		allEdges.push_back( E );
	}
	
	return allEdges;
}

p3d_node* Node::getNodeStruct()
{
	return _Node;
}

Graph* Node::getGraph()
{
	return _Graph;
}

Robot* Node::getRobot()
{
	return _Robot;
}

shared_ptr<Configuration> Node::getConfiguration()
{
	return _Configuration;
}

void Node::activ(bool b)
{
	_activ = b;
}

bool Node::isActiv()
{
	return _activ;
}

unsigned int Node::getNumberOfNodesInCompco()
{
	return _Node->comp->nnode;
}

double Node::cost()
{
	_Node->cost = _Configuration->cost();
	return (_Node->cost);
}

double& Node::sumCost()
{
	return (_Node->sumCost);
}

double Node::getDist()
{
	return (_Node->dist_Nnew);
}

double Node::dist(Node* N)
{
	double d = getConfiguration()->dist(*N->getConfiguration());
	_Node->dist_Nnew = d;
	return d;
}

bool Node::equal(Node* N)
{
	return getConfiguration()->equal(*N->getConfiguration());
}

void Node::setConnectedComponent(ConnectedComponent* Compco)
{
	m_Compco = Compco;
	_Node->numcomp = m_Compco->getId();
}

bool Node::inSameComponent(Node* N)
{
	bool isInSameComponent = false;
  isInSameComponent = ( m_Compco == N->getConnectedComponent() );
	return isInSameComponent;
}

bool Node::isLinkable(Node* N, double* dist)
{
	// Function that is the copy of p3d_APInode_linked
	// that relies on the old graph
	p3d_node *N1 = _Node;
	p3d_node *N2 = N->getNodeStruct();
	
	p3d_rob *robotPt = _Robot->getRobotStruct();
  p3d_localpath *localpathPt;
  int ntest = 0, isNoCol = 0, *ikSol = NULL;
  configPt qsave;
	
  /* current position of robot is saved */
  qsave = p3d_get_robot_config(robotPt);
	
  /* compute the local path using the local method associated to
	 the robot */
	int DEBUG_GRAPH_API = 0;
  if (DEBUG_GRAPH_API){
    printf("API Node Linked :\n");
    p3d_print_iksol(robotPt->cntrt_manager,N1->iksol);
    p3d_print_iksol(robotPt->cntrt_manager,N2->iksol);
  }
  if(!p3d_compare_iksol(robotPt->cntrt_manager, N1->iksol, N2->iksol)){
    p3d_destroy_config(robotPt, qsave);
    return(FALSE);
  }
  p3d_get_non_sing_iksol(robotPt->cntrt_manager, N1->iksol, N2->iksol, &ikSol);
  localpathPt = p3d_local_planner_multisol(robotPt, N1->q, N2->q, ikSol);
	
  if (localpathPt == NULL) { // Not valid localpath
    p3d_destroy_config(robotPt, qsave);
    return(FALSE);
  }
	
  if (localpathPt->length != NULL)
		*dist = localpathPt->length(robotPt,localpathPt);
  else{
		PrintInfo(("Warning: created an edge with \
							 a 0 distance: no localpathPt->length \n"));
		*dist = 0;
	}
  if((p3d_get_SORTING()==P3D_NB_CONNECT)&&
     (p3d_get_MOTION_PLANNER()==P3D_BASIC)) {
    if((*dist > p3d_get_DMAX())&&(LEQ(0.,p3d_get_DMAX()))){ /* ecremage deja fait dans le cas tri par distance... */
      /* the local path is destroyed */
      localpathPt->destroy(robotPt, localpathPt);
      localpathPt = NULL;
			
      /* The initial position of the robot is recovered */
      p3d_set_robot_config(robotPt, qsave);
      p3d_destroy_config(robotPt, qsave);
      return(FALSE);
    }
  }
  //start path deform
  if (p3d_get_cycles() == TRUE) {
    if (localpathPt->length != NULL)
      *dist = localpathPt->length(robotPt, localpathPt);
    else {
      PrintInfo(("linked: no distance function specified\n"));
      *dist = 0;
    }
  }
  //end path deform
  isNoCol = !p3d_unvalid_localpath_test(robotPt, localpathPt, &ntest);   // <- modif Juan
	//   isNoCol = 1;
  localpathPt->destroy(robotPt, localpathPt);
	
	// See this later (Add counts to local methods and collision tests)
	//  if(graphPt){
	//    graphPt->nb_local_call = graphPt->nb_local_call + 1;
	//    graphPt->nb_test_coll = graphPt->nb_test_coll + ntest;
	//  }
	
  /* The initial position of the robot is recovered */
  p3d_set_robot_config(robotPt, qsave);
  p3d_destroy_config(robotPt, qsave);
  return(isNoCol);
}

void Node::checkStopByWeight()
{
	double stopWeight;
	int signStopWeight;
	p3d_GetStopWeightAndSign(&stopWeight, &signStopWeight);
	if (signStopWeight * (_Node->weight - stopWeight) > 0)
	{
		PlanEnv->setBool(PlanParam::stopPlanner,true);
    //		p3d_SetStopValue(true);
		p3d_SetDiffuStoppedByWeight(true);
	}
}

//fonctions sur les composantes connexes
void Node::deleteCompco()
{
	// This is now handled in the Boost Graph
	_Graph->removeCompco(m_Compco);
}

bool Node::maximumNumberNodes()
{
	return ((int)m_Compco->getNumberOfNodes()) >= ENV.getInt(Env::maxNodeCompco);
}

bool Node::connectNodeToCompco(Node* N, double step)
{
	if (ENV.getBool(Env::isCostSpace))
	{
		cout << "WARNING: Using Cost Space with wrong algortihm" << endl;
		return false;
	}
	else
	{
		return _Graph->connectNodeToCompco(N,this);
	}
}

//place la compco dans la CompCo presente
void Node::merge(Node* compco)
{
	_Graph->mergeComp(this,compco,dist(compco));
}

bool Node::equalCompco(Node* compco)
{
	bool equalCompco = false;
  equalCompco = ( m_Compco == compco->getConnectedComponent() );
	return equalCompco;
}

Node* Node::randomNodeFromComp()
{
	return (_Graph->getNode(p3d_RandomNodeFromComp(_Node->comp)));
}

void Node::print()
{
	_Configuration->print();
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
		m_BGL_Descriptor  = _Graph->findVertexDescriptor(this);
		return m_BGL_Descriptor;
	}
}


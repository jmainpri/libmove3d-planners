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
#include <boost/foreach.hpp>
#include <boost/config.hpp>
#include <boost/random.hpp>
#include <boost/graph/vector_as_graph.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/strong_components.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>

#include "API/ConfigSpace/configuration.hpp"
#include "API/project.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/graphConverter.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include "move3d-headless.h"

#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "GroundHeight-pkg.h"

#ifdef LIGHT_PLANNER
#include "LightPlanner-pkg.h"
#endif

#undef None
#undef Upper
#undef Lower

extern void* GroundCostObj;

Graph* API_activeGraph = NULL;

using std::cout;
using std::cerr;
using std::endl;

MOVE3D_USING_SHARED_PTR_NAMESPACE

const bool graph_debug_import_export = false;

// Constructors
//----------------------------------------------

//! Constructor from graph and robot
//! @param R the robot for which the graph is created
//! @param G the old graph structure
Graph::Graph(Robot* R, p3d_graph* G)
{
    robot_ = R;

    if (G)
    {
        graph_ = new p3d_graph(*G);
    }
    else
    {
        graph_ = p3d_create_graph(R->getRobotStruct());
        cout << "Allocate graph : " << graph_ << endl;
    }

    graph_->rob->GRAPH = graph_;
    graph_is_modified_ = true;
    init_bgl_ = false;
    this->init();
    this->initBGL();
}

Graph::Graph(Robot* R)
{
    robot_ = R;

    graph_ = p3d_allocinit_graph();
    graph_->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
    graph_->rob = robot_->getRobotStruct();

    if (STAT)
    {
        graph_->stat = createStat();
    }
    else
    {
        graph_->stat = NULL;
    }

    graph_is_modified_ = true;
    init_bgl_ = false;

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
        graph_ = G;
    }
    else
    {
        graph_ = p3d_create_graph();
    }
    robot_ = global_Project->getActiveScene()->getRobotByName(G->rob->name);
    graph_is_modified_ = true;
    init_bgl_ = false;

    this->init();
    this->initBGL();
}

//! Graph copy constructor : be sure that the connected componants are updated
//! @param G the graph
Graph::Graph(const Graph& G)
{
    //	//graph_ = G.exportGraphStruct();
    //  GraphConverter gc;
    //  graph_ = gc.exportGraphStruct( G );

    robot_ = G.robot_;
    graph_ = p3d_create_graph(G.robot_->getRobotStruct());
    name_ = G.name_;

    graph_is_modified_ = true;
    init_bgl_ = false;

    try {
        // Copy all Nodes
        for(int i=0; i<int(G.nodes_.size()); i++)
        {
            Node* node = G.nodes_[i];
            nodes_.push_back( new Node( this, node->getConfiguration(), false ) );
        }

        // Make new Edges
        for(int i=0; i<int(G.edges_.size()); i++)
        {
            Node* Source = searchConf( *G.edges_[i]->getSource()->getConfiguration() );
            Node* Target = searchConf( *G.edges_[i]->getTarget()->getConfiguration() );

            if ( PlanEnv->getBool(PlanParam::orientedGraph) )
                addEdges( Source, Target, true, 0.0 , true, 0.0 );
            else
                addEdge( Source, Target, true, 0.0 , true, 0.0 );

        }

        int k=0;

        comp_.clear();

        // Make new Compco
        for(int i=0; i<int(G.comp_.size()); i++)
        {
            for(int j=0; j<int(G.comp_[i]->getNumberOfNodes()); j++)
            {
                Node* new_node = searchConf( *G.comp_[i]->getNode(j)->getConfiguration() );

                if( j == 0 ) {
                    comp_.push_back( new ConnectedComponent( this, new_node ) );
                }
                else {
                    comp_[i]->addNode( new_node );
                }

                k++;
            }
        }

        if( k != int(G.nodes_.size()) ) {
            cout << "Error in copying compco" << endl;
        }
    }
    catch (std::string str) {
        cout << str << endl;
    }
    catch (...) {
        cout << "Exeption in Graph copy "  << endl;
    }

    //  for(int i=0; i<int(nodes_.size()); i++)
    //  {
    //    cout << G.nodes_[i]->getConnectedComponent()->getId() << endl;
    //  }

    this->initBGL();
}

void Graph::setRobot(Robot* robot) {
    robot_ = robot;
}

/**
 * Fonction creant un Objet Graph a partir d'une structure de p3d_Graph
 */
void Graph::init()
{
    if (graph_->nodes)
    {
        p3d_list_node* l = graph_->nodes;
        nodes_.clear();
        while (l)
        {
            Node* node = new Node(this, l->N);
            nodes_Table.insert( std::pair<p3d_node*, Node*>(l->N, node));
            nodes_.push_back(node);
            l = l->next;
        }
    }
    if (graph_->edges)
    {
        p3d_list_edge* l = graph_->edges;
        edges_.clear();
        while (l)
        {
            //cout << "Init : Edge = " << l->E << endl;
            Edge* edge = new Edge(this,l->E);
            edges_.push_back(edge);
            l = l->next;
        }
    }
    if (graph_->comp)
    {
        compco* l=graph_->comp;
        comp_.clear();
        while (l)
        {
            p3d_list_node* ln = l->last_node;
            ConnectedComponent* current=new ConnectedComponent(this,this->getNode(ln->N));
            while (ln->next)
            {
                ln=ln->next;
                current->addNode(this->getNode(ln->N));
            }
            comp_.push_back(current);
            l = l->suiv;
        }


    }
    this->setName();
    p3d_calc_DMAX(robot_->getRobotStruct());
}

// BGL Functions
//----------------------------------------------
void Graph::initBGL()
{
    try
    {
        BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , boost_graph_ );

        for(unsigned int i = 0; i < nodes_.size(); ++i)
        {
            BGL_Vertex v	= boost::add_vertex(boost_graph_);
            NodeData[v]		= nodes_[i];
            nodes_[i]->setDescriptor(v);
        }

        BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , boost_graph_ );

        for(unsigned int i = 0; i < edges_.size(); ++i)
        {
            // TODO add vertex does'nt need to find vertex
            //BGL_Vertex s = findVertexDescriptor(edges_[i]->getSource());
            //BGL_Vertex t = findVertexDescriptor(edges_[i]->getTarget());

            BGL_Vertex s = edges_[i]->getSource()->getDescriptor();
            BGL_Vertex t = edges_[i]->getTarget()->getDescriptor();

            BGL_Edge e; bool found;

            tie(e, found) = boost::add_edge(s,t,boost_graph_);

            if (!found)
            {
                throw std::string("Erreur: BGL edge exists allready or nodes inexistant");
            }

            EdgeData[e] = edges_[i];
            edges_[i]->setDescriptor(e);
        }

        init_bgl_ = true;
    }
    catch (std::string str)
    {
        std::cerr << str << endl;
        return;
    }
    catch (...)
    {
        std::cerr << "Unkomwn error building BGL Graph" << endl;
        return;
    }
    //	saveBGLGraphToDotFile("/Users/jmainpri/Desktop/Graph/myGraph.dot");
    //	cout << "Building BGL_Graph OK!!!" << endl;
}

BGL_Vertex Graph::findVertexDescriptor(Node* N)
{
    BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , boost_graph_ );

    typedef boost::graph_traits<BGL_Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;

    //	cout << "------------------------------" << endl;
    //	cout << "Node : " << N << endl;
    unsigned int i=0;
    for (vp = vertices(boost_graph_); vp.first != vp.second; ++vp.first)
    {
        //cout << "NodeData[*vp.first] : " << i << " = " << NodeData[*vp.first] << endl;
        //		cout << "Node : " << i << " = " << nodes_[i] << endl;
        i++;
        if( NodeData[*vp.first]	== N )
        {
            return *vp.first;
        }
    }
    //cout << i << endl;
    //	cout << nodes_.size() << endl;
    throw std::string("Erreur: BGL vertex not found");
}

BGL_Edge Graph::findEdgeDescriptor(Edge* E)
{
    BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , boost_graph_ );

    typedef boost::graph_traits<BGL_Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> ep;

    //	cout << "------------------------------" << endl;
    //	cout << "Edge : " << E << endl;
    unsigned int i=0;
    for (ep = edges(boost_graph_); ep.first != ep.second; ++ep.first)
    {
        //cout << "EdgeData[*ep.first] : " << i << " = " << EdgeData[*ep.first] << endl;
        i++;
        if( EdgeData[*ep.first]	== E )
        {
            return *ep.first;
        }
    }

    throw std::string("Erreur: BGL edge not found");
}

void Graph::setAllDescriptorsInvalid()
{
    // Unset descriptors
    for(int i=0; i<int(nodes_.size()); i++)
    {
        nodes_[i]->unSetDescriptor();
    }

    for(int i=0; i<int(edges_.size()); i++)
    {
        edges_[i]->unSetDescriptor();
    }
}

void Graph::saveBGLGraphToDotFile(const std::string& filename)
{
    std::ofstream out(filename.c_str());
    boost::write_graphviz(out,boost_graph_);
}

// Destructors
//----------------------------------------------
Graph::~Graph()
{
    //  API_activeGraph = NULL;

    if( graph_debug_import_export )
    {
        cout << "Graph::~Graph() => Deleting graph" << endl;
        cout << "Mark the XYZ_GRAPH as deleted" << endl;
        cout << "XYZ_GRAPH = " << XYZ_GRAPH << endl;
    }

    // Deletes the old graph

    //  robot_->getRobotStruct()->GRAPH = XYZ_GRAPH = NULL;
    deleteGraphStruct();

    // The configuration arrays have been deleted in the old API
    freeResources();
}

void Graph::deleteGraphStruct()
{	
    /* verification of the existence of the graph */
    if (graph_)
    {
        // Get the compoc mirroring the P3d
        updateCompcoFromStruct();

        p3d_list_node *NLS, *NLD;  /* Node List Scanner, Node List pointer for Deletion */
        p3d_list_node *NeLS, *NeLD; /* Neighbour List Scanner, Neighbour List pointer for Deletion */
        p3d_list_edge *ELS, *ELD;  /* Edge List Scanner, Edge pointer for Deletion */

        /*nodes desallocation*/
        NLS = graph_->nodes;

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

        if((graph_->stat != NULL))
        {
            destroyStat(&(graph_->stat));
        }

        /* Delete references to the graph */
        if (graph_->rob != NULL)
        {
            graph_->rob->GRAPH = NULL;
        }

        cout << "graph_ = " << graph_ << endl;
        cout << "XYZ_GRAPH = " << XYZ_GRAPH << endl;

        if (XYZ_GRAPH == graph_)
        {
            robot_->getRobotStruct()->GRAPH = XYZ_GRAPH = NULL;
        }

        delete graph_;
    }
}

/**
 * Frees the Nodes and Edges
 */
void Graph::freeResources()
{
    for(unsigned int i=0;i<comp_.size();i++)
    {
        delete comp_[i];
    }

    for(unsigned int i=0;i<edges_.size();i++)
    {
        delete edges_[i];
    }

    for(unsigned int i=0;i<nodes_.size();i++)
    {
        delete nodes_[i];
    }
}

// Import and Export function to p3d
//----------------------------------------------

// Accessors
//----------------------------------------------
p3d_graph* Graph::getGraphStruct() const
{
    return graph_;
}

void Graph::setGraph(p3d_graph* G)
{
    *graph_ = *G;
}

Robot* Graph::getRobot() const
{
    return robot_;
}

/*void Graph::setRobot(Robot* R)
 {
 robot_ = R;
 }*/

unsigned int Graph::getNumberOfNodes()
{
    if( init_bgl_ ) {

        unsigned int nbNodes = num_vertices(boost_graph_);

        if ( nbNodes != nodes_.size() )
        {
            throw std::string("Verticies in boost graph don't reflect the C++ graph");
        }
        return nbNodes;
    }
    else {
        return nodes_.size();
    }
}

unsigned int Graph::getNumberOfEdges() const
{
    if( init_bgl_ ) {

        unsigned int nbEdges = num_edges(boost_graph_);

        if ( nbEdges != edges_.size() )
        {
            throw std::string("Edges in the boost graph don't reflect the C++ graph");
        }
        return nbEdges;
    }
    else {
        return edges_.size();
    }
}

const std::vector<Node*>& Graph::getNodes() const
{
    return nodes_;
}

const std::vector<Edge*>& Graph::getEdges() const
{
    return edges_;
}

std::vector<ConnectedComponent*> Graph::getConnectedComponents() const
{
    return comp_;
}

std::map<p3d_node*, Node*> Graph::getNodesTable()
{
    return nodes_Table;
}

Node* Graph::getNode(p3d_node* N)
{
    std::map<p3d_node*, Node*>::iterator it = nodes_Table.find(N);
    if (it != nodes_Table.end())
        return (it->second);
    else
    {
        cout << "node " << N << " not found: num: " << N->num << endl;
        return (NULL);
    }
}

Node* Graph::getLastNode()
{
    return (nodes_.back());
}

std::string Graph::getName()
{
    return name_;
}

void Graph::setName()
{
    name_ = "graph";
}

void Graph::setName(std::string Name)
{
    name_ = Name;
}

bool Graph::equal(Graph* G)
{
    return (graph_->nodes == G->getGraphStruct()->nodes);
}

bool Graph::equalName(Graph* G)
{
    return (name_ == G->getName());
}

// Graph operations
//----------------------------------------------

/**
 * Search configuration in graph
 */
Node* Graph::searchConf(Configuration& q)
{
    for(int i=0;i<int(nodes_.size());i++)
    {
        if(nodes_[i]->getConfiguration()->equal(q))
        {
            return nodes_[i];
        }
    }
    return NULL;
}

/**
 * Insert node for RRT
 */
Node* Graph::insertNode( Node* expansionNode, LocalPath& path )

/*confPtr_t q,
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
            throw std::string("expansionNode is not in the graph");
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
        p3d_SetNodeWeight(graph_, newNode->getNodeStruct());
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

        if (newNode->getNodeStruct()->weight > graph_->CurPbLevel)
        {
            graph_->CurPbLevel = newNode->getNodeStruct()->weight;
            graph_->n_consec_pb_level = 0;
            graph_->n_consec_fail_pb_level = 0;

            if (p3d_GetNGood() > 2)
            {
                graph_->critic_cur_pb_level = graph_->CurPbLevel;
            }
        }
        else
        {
            graph_->n_consec_pb_level++;
            graph_->n_consec_fail_pb_level = 0;
        }
    }

    //Additional cycles through edges addition if the flag is active
    if (ENV.getBool(Env::addCycles))
    {
        this->addCycles(newNode, step);
    }

    graph_is_modified_ = true;
    return (newNode);
}

/**
 * Insert Lining Node for RRT
 */
Node* Graph::insertConfigurationAsNode(confPtr_t q, Node* from,double step)
{
    //cout << "Insert configuration as node" << endl;
    Node* node = new Node( this , q );

    addNode(node);

    mergeComp( from, node, step, false );

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
bool Graph::compareNodes(Node* node_1, Node* node_2)
{
    return (node_1->getNodeStruct()->dist_Nnew < node_2->getNodeStruct()->dist_Nnew);
    //double dist = N1->dist(N2);
    //return dist;
}


/**
 * Compare Nodes
 */
bool compareNodesInverse(Node* node_1, Node* node_2)
{
    return (node_1->getNodeStruct()->dist_Nnew > node_2->getNodeStruct()->dist_Nnew);
}

/**
 * Sort Nodes
 */
void Graph::sortNodesByDist(std::vector<Node*>& nodes, confPtr_t q)
{
    if (nodes.size() > 1)
    {
        for (int i=0; i<int(nodes.size()); i++)
        {
            nodes[i]->dist(q);
        }
        sort(nodes.begin(), nodes.end(), &compareNodes);
    }
}

/**
 * Sort Node by distance
 */
void Graph::sortNodesByDist(confPtr_t q)
{
    if (nodes_.size() > 1)
    {
        for (int i=0; i<int(nodes_.size()); i++)
        {
            nodes_[i]->dist(q);
        }
        sort(nodes_.begin(), nodes_.end(), &compareNodes);
    }
}

/**
 * Sort Node by distance
 */
void Graph::sortNodesByDist(Node* N)
{
    if (nodes_.size() > 1)
    {
        for (int i=0; i<int(nodes_.size()); i++)
        {
            nodes_[i]->dist(N);
        }
        sort(nodes_.begin(), nodes_.end(), &compareNodes);
    }
}

/**
 * Sort Edges
 */
void Graph::sortEdges()
{
    if (edges_.size() > 1)
    {
        sort(edges_.begin(), edges_.end(), &compareEdges);
    }
}

/**
 * Is Edge in graph
 */
Edge* Graph::isEdgeInGraph(Node* N1, Node* N2)
{
    if ( PlanEnv->getBool(PlanParam::orientedGraph) )
    {
        for ( int i=0; i<int(edges_.size()); i++)
        {
            if((edges_[i]->getSource()==N1 )&&( edges_[i]->getTarget()==N2))
            {
                return edges_[i];
            }
        }
    }
    else
    {
        for ( int i=0; i<int(edges_.size()); i++)
        {
            if( ((edges_[i]->getSource()==N1 )&&( edges_[i]->getTarget()==N2)) ||
                ((edges_[i]->getSource()==N2 )&&( edges_[i]->getTarget()==N1)) )
            {
                return edges_[i];
            }
        }
    }
    return NULL;
}

/**
 * Add Edge to Graph
 */
Edge* Graph::addEdge( Node* source, Node* target, bool compute_length, double length , bool compute_cost,  double cost )
{	
    Edge* E = isEdgeInGraph( source, target );

    // Edge is not in the graph
    if ( E == NULL )
    {
        E = new Edge(this, source, target, compute_length, length , compute_cost, cost );
        edges_.push_back(E);

        if (init_bgl_)
        {
            BGL_EdgeDataMapT EdgeData = boost::get( EdgeData_t() , boost_graph_ );
            BGL_Edge e; bool found;

            boost::tie(e, found) = boost::add_edge( source->getDescriptor(), target->getDescriptor(), boost_graph_);
            if (found)
            {
                // cout << "Add edge : " << e << endl;
                EdgeData[e] = E;
                E->setDescriptor(e);
            }
        }
        graph_is_modified_ = true;
    }
//    else
//    {
//        cout << "edge is already in graph" << endl;
//    }

    return E;
}

/**
 * Add Edges
 * WARNING only in graph API doesn't work yet not to be used
 */
std::vector<Edge*> Graph::addEdges(Node* N1, Node* N2, bool compute_length, double length , bool compute_cost,  double cost )
{
    std::vector<Edge*> edges(2);
    edges[0] = addEdge(N1, N2, compute_length, length, compute_cost, cost );
    edges[1] = addEdge(N2, N1, false, length, false, edges[0]->cost() );
    //cout << "Edge cost : " << edges[0]->cost() << endl;
    return edges;
}

/**
 * Remove edge from graph
 */
void Graph::removeEdge(Edge* E)
{
    boost::remove_edge( E->getDescriptor(),  boost_graph_ );
    edges_.erase( find( edges_.begin() , edges_.end(), E ) );
    delete E;
    setAllDescriptorsInvalid();
}

/**
 * Remove edges between source node and target node
 */
void Graph::removeEdge( Node* source, Node* target )
{
    // Get out edges
    std::vector<Edge*> edges = source->getEdges();

    if ( PlanEnv->getBool(PlanParam::orientedGraph) )
    {
        // Remove the ones going to target from graph
        for(int i=0;i<int(edges.size());i++)
        {
            if ((edges[i]->getSource() == source) && (edges[i]->getTarget() == target))
            {
                removeEdge( edges[i] );
            }
        }
    }
    else
    {
        for(int i=0;i<int(edges.size());i++)
        {
            if( ((edges[i]->getSource() == source) && (edges[i]->getTarget() == target)) ||
                ((edges[i]->getSource() == target) && (edges[i]->getTarget() == source)) )
            {
                removeEdge( edges[i] );
            }
        }
    }
}

void Graph::removeEdges( Node* N1, Node* N2 )
{
    // Get out edges
    std::vector<Edge*> edges = N1->getEdges();

    // Remove the ones going to target from graph
    for(int i=0;i<int(edges.size());i++)
    {
        if (((edges[i]->getSource() == N1) && (edges[i]->getTarget() == N2)) ||
                ((edges[i]->getTarget() == N1) && (edges[i]->getSource() == N2)))
        {
            removeEdge( edges[i] );
        }
    }
}

/**
 * Add basically a node to the graph
 */
void Graph::addNode(Node* N)
{
    nodes_Table.insert( std::pair<p3d_node*, Node*> (N->getNodeStruct(), N));
    nodes_.push_back(N);

    BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , boost_graph_ );
    BGL_Vertex v								= boost::add_vertex( boost_graph_ );
    NodeData[v]									= N;
    N->setDescriptor(v);

    graph_is_modified_							= true;
}


/**
 * Add Edge to all node inferior to max Dist form N
 */
void Graph::addNode(Node* N, double maxDist)
{
    double d;
    for (unsigned int i = 0; i < nodes_.size(); i = i + 1)
    {
        if ((d = nodes_[i]->getConfiguration()->dist(*N->getConfiguration()))< maxDist)
        {
            LocalPath path(nodes_[i]->getConfiguration(), N->getConfiguration());

            if ( path.isValid() )
            {
                if ( PlanEnv->getBool(PlanParam::orientedGraph) )
                    this->addEdges( nodes_[i], N, false, d, true, 0.0 );
                else
                    this->addEdge( nodes_[i], N, false, d, true, 0.0 );
            }
        }
    }

    addNode(N);
}

/**
 * Add  vector of node
 */
void Graph::addNodes( std::vector<Node*> N, double maxDist)
{
    for (std::vector<Node*>::iterator it = N.begin(), end = N.end(); it != end; it++)
    {
        this->addNode(*it, maxDist);
    }
}

const bool print_remove_node = false;

/**
 * Remove a node from compco
 * TODO
 */
void Graph::removeNode(Node* to_del_node, bool rebuild_compco)
{	
    if( !isInGraph( to_del_node ) )
    {
        throw std::string("Node, N is not graph");
    }

    // Get edges to remove
    std::vector<Edge*> to_del_edges = to_del_node->getEdges();

    // Removes the node from the BGL graph
    BGL_Vertex u = to_del_node->getDescriptor();
    boost::clear_vertex( u, boost_graph_ );
    boost::remove_vertex( u, boost_graph_ );

    if( print_remove_node ) {
        cout << "node vector size : " << nodes_.size() << endl;
        cout << "edge vector size : " << edges_.size() << endl;
        cout << "edges to delete size : " << to_del_edges.size() << endl;
    }

    // Erase from edges vector
    for( int i=0; i<int(to_del_edges.size()); ++i )
    {
        edges_.erase( find( edges_.begin(), edges_.end(), to_del_edges[i]));
        delete to_del_edges[i];
    }

    if( print_remove_node ) {
        cout << "nb of edges in graph : " << edges_.size() << endl;
    }

    // Erase from node vector
    nodes_.erase( find( nodes_.begin() , nodes_.end() , to_del_node ) );
    delete to_del_node;

    // Rebuild compco
    if( rebuild_compco )
    {
        rebuildCompcoFromBoostGraph();
    }

    // Set all nodes and edges descriptors are invalid now
    setAllDescriptorsInvalid();

    // Check that the number of verticies and edges are the same
    // in the vectors and the graph
    if( boost::num_vertices(boost_graph_) != nodes_.size() || boost::num_edges(boost_graph_) != edges_.size() )
    {
        cout << "boost::num_vertices(boost_graph_) : " << boost::num_vertices(boost_graph_) ;
        cout << ", nodes_.size() : " << nodes_.size() << endl;
        cout << "boost::nuedges_(boost_graph_) : " << boost::num_edges(boost_graph_) ;
        cout << " , edges_.size() : " << edges_.size() << endl;
        throw std::string("Boost Graph differs in the number of verticies or edges");
    }

    // Save to file
    if( print_remove_node )
    {
        cout << "Saving graph" << endl;
        saveBGLGraphToDotFile( "/Users/jmainpri/Desktop/Graph/myGraph.dot" );
    }
}

/**
 * Test if node is in graph
 */
bool Graph::isInGraph(Node* N)
{
    for (unsigned int i = 0; i < nodes_.size(); i++ )
    {
        if ( N == nodes_[i] )
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
        return this->linkNodeAtDist(N);
    }
    else
    {
        return this->linkNodeWithoutDist(N);
    }
}

/**
 * Links node to graph without distance
 * goes through all connected components and connects only once in each
 */
bool Graph::linkNodeWithoutDist(Node* N)
{
    int nof_link = 0;
    for (int i=0; i<int(comp_.size()); i++)
    {
        if( N->getConnectedComponent() != comp_[i] )
        {
            std::vector<Node*>& nodes = comp_[i]->getNodes();

            for (int j=0; j<int(nodes.size()); j++) {
                nodes[j]->distMultisol(N);
            }
            std::sort( nodes.begin(), nodes.end(), &compareNodes );

            for (int k=0; k<int(nodes.size()); k++)
            {
                if (linkNodeAndMerge(nodes[k],N,true))
                {
                    // cout << "Node linked in component : " << comp_[i]->getId() << endl;
                    nof_link++;
                    break;
                }
            }
        }
    }

    return(nof_link > 0);
}


//! Copy of p3d_link_node_comp_multisol
bool Graph::linkNodeCompMultisol( Node* N, ConnectedComponent* TargetComp ) 
{
    int ValidForward, ValidBackward;
    double dist = 0.;

    std::vector<Node*>& nodes = TargetComp->getNodes();

    // If the criteria for choosing the best node in the target compco is
    // the distance, then node lists must be ordered
    if (p3d_get_SORTING() == P3D_DIST_NODE)
    {
        for (int i=0; i<int(nodes.size()); i++) {
            nodes[i]->distMultisol(N);
        }
        std::sort(nodes.begin(), nodes.end(), &compareNodes);
    }

    ValidBackward = ValidForward = FALSE;

    for (int i=0; i<int(nodes.size()); i++)
    {
        Node* Nc = nodes[i];

        if (p3d_get_SORTING() == P3D_DIST_NODE) {
            if ((Nc->getNodeStruct()->dist_Nnew > p3d_get_DMAX()) && (p3d_get_DMAX() > 0.)) {
                return false;
            }
        }

        // Oriented case, forward and backward paths must be separately tested
        if (graph_->oriented) {
            if (ValidForward == FALSE) {
                if ( N->isLinkableMultisol( Nc, &dist) ) {
                    // A forward path is found
                    addEdge( N, Nc , false, dist, true, 0.0 );
                    ValidForward = TRUE;
                }
            }

            if (ValidBackward == FALSE) {
                if ( Nc->isLinkableMultisol( Nc,&dist)) {
                    // A bacward path is found
                    addEdge( Nc, N , false, dist, true, 0.0 );
                    ValidBackward = TRUE;
                }
            }

            if (ValidBackward && ValidForward) {
                if ( !N->getConnectedComponent() ) {
                    // A valid forward and backward path exist, and the node is still in none compco
                    // so the tested compco will now include the new node
                    TargetComp->addNode(N);
                } else {
                    // A valid forward and backward path exist, and the node is already included in a compco
                    // so the tested compco and the compco of the new node must merge
                    if( TargetComp->getId() > N->getConnectedComponent()->getId() )
                    {
                        TargetComp->mergeWith( N->getConnectedComponent() );
                    }
                    else {
                        N->getConnectedComponent()->mergeWith( TargetComp );
                    }
                }
                return true;
            }
        }
        // Non - oriented case, If the forward path is valid, the backward one is also valid.
        else
        {
            if ( N->isLinkableMultisol( Nc, &dist) )
            {
                if ( PlanEnv->getBool(PlanParam::orientedGraph) )
                    addEdges( N, Nc, false, dist, true, 0.0);
                else
                    addEdge( N, Nc, false, dist, true, 0.0);

                // If the node is still not included in a compco, it will be absorbed in the tested compco
                if ( N->getConnectedComponent() == NULL) {
                    TargetComp->addNode(N);
                }
                // Otherwise compcos merge
                else
                {
                    if( TargetComp->getId() > N->getConnectedComponent()->getId() )
                    {
                        TargetComp->mergeWith( N->getConnectedComponent() );
                    }
                    else {
                        N->getConnectedComponent()->mergeWith( TargetComp );
                    }
                }
                return true;
            }
        }
    }
    /* Non connexion has been found (in oriented case, arcs may have been created) */
    return false;
}

bool Graph::linkNodeAtDist(Node* N)
{
    int nof_link = 0;
    for (int i=0; i<int(comp_.size()); i++)
    {
        if( N->getConnectedComponent() != comp_[i] )
        {
            std::vector<Node*>& nodes = comp_[i]->getNodes();

            for (int j=0; j<int(nodes.size()); j++) {
                nodes[j]->distMultisol(N);
            }
            std::sort(nodes.begin(), nodes.end(), &compareNodes);

            for (int k=0; k<int(nodes.size()); k++)
            {
                if ((nodes[k]->getNodeStruct()->dist_Nnew < p3d_get_DMAX()) && (p3d_get_DMAX() > 0.) && linkNodeAndMerge(nodes[k],N,true))
                {
                    nof_link++;
                    break;
                }
            }
        }
    }

    return(nof_link > 0);
}

/**
 * Links node at distance
 */
//bool Graph::linkNodeAtDist(Node* N)
//{
//	int nbLinkedComponents = 0;
//	
//	// Warning TODO:  this function does not work with the C++ API
//  nbLinkedComponents = p3d_link_node_graph_multisol(N->getNodeStruct(), graph_);
//	
//  this->mergeCheck();
//  return(nbLinkedComponents > 0);
//}

/**
 * Links Node to all nodes
 */
bool Graph::linkToAllNodes(Node* newN)
{
    // Warning TODO:  this function does not work with the C++ API
    return p3d_all_link_node(newN->getNodeStruct(), graph_);
}

/**
 *Check if two nodes are linked using the collision checking method
 */
bool Graph::areNodesLinked( Node* node1, Node* node2, double & dist )
{	
    // copy of the function p3d_APInode_linked
    // that computes also the IK solution

    LocalPath path( node1->getConfiguration(), node2->getConfiguration() );

    if ( path.getParamMax() != 0.0 )
    {
        dist = path.getParamMax();
    }
    else
    {
        cerr << "Warning: created an edge with a 0 distance: no localpathPt->length \n";
        dist = 0;
    }
    if((p3d_get_SORTING()==P3D_NB_CONNECT)&& (p3d_get_MOTION_PLANNER()==P3D_BASIC))
    {
        if((dist > p3d_get_DMAX())&&(LEQ(0.,p3d_get_DMAX())))
        {
            /* ecremage deja fait dans le cas tri par distance... */
            /* the local path is destroyed */
            return false;
        }
    }

    bool isNoCol = path.isValid();

    if(graph_)
    {
        graph_->nb_local_call++;
        graph_->nb_test_coll += path.getNbColTest();
    }

    return(isNoCol);
}


/**
 * Link Node and Merge
 */
bool Graph::linkNodeAndMerge(Node* node1, Node* node2, bool compute_cost)
{
    double dist = 0.;
    bool  IsLinkedAndMerged = false;

    if( areNodesLinked( node1, node2, dist) )
    {
        mergeComp( node1, node2, dist, compute_cost );
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

    confPtr_t Cs = confPtr_t (new Configuration(robot_, robot_->getRobotStruct()->ROBOT_POS));
    Node* Ns = new Node(this, Cs);
    this->addNode(Ns);

    while (inode < NMAX)
    {
        confPtr_t C = robot_->shoot();
        if (!C->isInCollision())
        {
            this->addNode(new Node(this, C));
            inode = inode + 1;

            (*fct_draw)();

            if (!(*fct_stop)())
            {
                PrintInfo(("Random confs. generation canceled\n"));
                break;
            }
        }
    }

    PrintInfo(("For the generation of %d configurations : ", inode));
    ChronoTimes(&tu, &ts);
    graph_->time = graph_->time + tu;
    ChronoPrint("");
    ChronoOff();
    p3d_print_info_graph(graph_);
}

/**
 * Random Nodes From Component
 */
Node* Graph::randomNodeFromComp(Node* comp)
{
    //	return (this->getNode(p3d_RandomNodeFromComp(comp->getConnectedComponent()->getCompcoStruct())));

    int RandNodeNum = (int)floor(p3d_random(0.0, (double)comp->getConnectedComponent()->getNumberOfNodes() - EPS6));
    return  comp->getConnectedComponent()->getNode(RandNodeNum);
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

    if (node1->getConnectedComponent()->getId() == compco->getConnectedComponent()->getId())
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
        IsConnectSuccess = linkNodeAndMerge(node2,node1,false);
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
        IsConnectSuccess = linkNodeAndMerge(node2,node1,false);
    }
    return IsConnectSuccess;
}

//-----------------------------------------------------------
// Nearest Neighbour
//-----------------------------------------------------------

/**
 * Basic Nearest Neighbour
 */
Node* Graph::nearestNeighbour( confPtr_t q ) const
{
    double dist = std::numeric_limits<double>::max(), dist_tmp;
    Node* node = NULL;

    // for each node in the graph
    for ( std::vector<Node*>::const_iterator it = nodes_.begin(); it != nodes_.end(); ++it)
    {
        dist_tmp = (*it)->getConfiguration()->dist(*q);

        if( dist_tmp < dist )
        {
            node = (*it); dist = dist_tmp;
        }
    }

    return node;
}

/**
 * Nearest Weighted Neighbout in graph
 *
 * returns the KNearest neighbours of the configuration config
 * in the entire graph within a minimum radius
 */
std::vector<Node*> Graph::KNearestWeightNeighbour( confPtr_t q, int K, double radius, bool weighted, int distConfigChoice, Node* node_to_discard )
{
    return Graph::KNearestWeightNeighbour( nodes_, q, K, radius, weighted, distConfigChoice, node_to_discard );
}

/**
 * Nearest Weighted Neighbout in graph
 *
 * returns the KNearest neighbours of the configuration config
 * in the entire graph within a minimum radius
 */
std::vector<Node*> Graph::KNearestWeightNeighbour( const std::vector<Node*>& nodes, confPtr_t q, int K, double radius, bool weighted, int distConfigChoice, Node* node_to_discard )
{
    double score;
    std::vector< std::pair<double, Node*> > nearNodesQueue;
    std::vector< Node* > nearNodes;

    // for each node in the graph
    for ( std::vector<Node*>::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        if( *it != node_to_discard )
        {
            // compute its distance to the configuration q
            score = q->dist(*(*it)->getConfiguration(), distConfigChoice);
            //*(weighted ? p3d_GetNodeWeight((*it)->getNodeStruct()) : 1.0);

            // add it to the queue only if it is within radius
            if ( score <= radius )
                nearNodesQueue.push_back( std::make_pair(score, *it) );
        }
    }

    // sort the queue
    std::sort( nearNodesQueue.begin(), nearNodesQueue.end() );

    // put the first K nodes of the queue in a vector
    int bound = MIN(K, int(nearNodesQueue.size()));

    for (int i = 0; i < bound; ++i) {
        Node* node = nearNodesQueue[i].second;
        if (node)
            nearNodes.push_back(node);
    }

    return nearNodes;
}

/**
 * Nearest Weighted Neighbour in graph
 *
 * finds the closest node from configuration in the
 * connected component of node compco
 */
Node* Graph::nearestWeightNeighbour(Node* compco, confPtr_t q, bool weighted, int distConfigChoice)
{
    // When retrieving statistics
    if (getStatStatus())
    {
        graph_->stat->planNeigCall++;
    }

    return compco->getConnectedComponent()->nearestWeightNeighbour( q, weighted, distConfigChoice );
}

//-----------------------------------------------------------
// Connected componnents
//-----------------------------------------------------------

unsigned int Graph::getNumberOfCompco() const
{
    return comp_.size();
    //return graph_->ncomp;
}

/**
 * Updates the list of connected
 * components from the p3d_graph structure
 */
void Graph::updateCompcoFromStruct()
{
    comp_.clear();

    if (graph_->comp)
    {
        p3d_compco* l = graph_->comp;
        while (l)
        {
            comp_.push_back(new ConnectedComponent(this, l));
            l = l->suiv;
        }
    }
}

/**
 * Merge Component by adding an edge between node1 and node2
 * @param node1
 * @param node2
 */
bool Graph::mergeComp( Node* node1, Node* node2, double dist_nodes, bool compute_edge_cost )
{
    if ((node1 == NULL) || (node2 == NULL))
    {
        std::cerr << "Warning: Try to link two nodes with NULL structures \n";
        return false;
    }

    ConnectedComponent* compco1 = node1->getConnectedComponent();
    ConnectedComponent* compco2 = node2->getConnectedComponent();

    if( compco1->getId() > compco2->getId() ) // Make sure to merge highest id with lowest
    {
        std::swap( compco1, compco2 );
        std::swap( node1, node2 );
    }

    compco1->mergeWith( compco2 );

    if ( PlanEnv->getBool(PlanParam::orientedGraph) )
        addEdges( node1, node2, false, dist_nodes, compute_edge_cost, 0.0 );
    else
        addEdge( node1, node2, false, dist_nodes, compute_edge_cost, 0.0 );

    return true;
}

/**
 * Create a Compco
 */
void Graph::createCompco(Node* N)
{
    comp_.push_back(new ConnectedComponent(this,N));
}

/**
 * Returns a connected component
 */
ConnectedComponent* Graph::getConnectedComponent(int ith) const
{
    return comp_[ith];
}

/**
 * Returns the ith compco
 */
Node* Graph::getCompco(unsigned int ith)
{
    return comp_[ith]->getNodes().at(0);
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
        Nodes.push_back( nodes_Table[ListNode->N] );
        ListNode = ListNode->next;
    }
    return Nodes;
}

int Graph::rebuildCompcoFromBoostGraph()
{
    // Find new compco with the BGL strongly connected components
    std::vector<int>		component(			boost::num_vertices(boost_graph_) );
    std::vector<int>		discover_time(	boost::num_vertices(boost_graph_) );

    std::vector<boost::default_color_type> color(boost::num_vertices(boost_graph_));
    std::vector<BGL_Vertex> root(boost::num_vertices(boost_graph_));

    if ( nodes_.size() != component.size() )
    {
        throw std::string("Error in the component size() before the strongly connected algo");
    }

    // TODO BROKEN!!!
    int num=0;
    //  int num = boost::strong_components( boost_graph_, &component[0], boost::root_map(&root[0]).color_map(&color[0]).discover_time_map(&discover_time[0]) );

    //  std::cout << "Total number of components: " << num << std::endl;
    //  return num;

    BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , boost_graph_ );

    // Delete all components
    while (!comp_.empty())
    {
        removeCompco(*comp_.begin());
    }

    // Make new compcos
    // TODO be carefull with the vertex function
    BGL_Vertex v;
    std::vector<int> newComponents;
    for (unsigned int i=0;i<component.size();i++ )
    {
        v = vertex(i,boost_graph_);

        //cout	<< "Node id : "			<< NodeData[ v ]->getId() << " , Comp id : "	<< component[i] << endl;
        if( std::find( newComponents.begin(), newComponents.end(), component[i] ) != newComponents.end() )
        {
            comp_[ component[i] ]->addNode( NodeData[ v ] );
        }
        else
        {
            newComponents.push_back( component[i] );
            this->createCompco( NodeData[v] );
        }
    }

    // The graph has changed
    graph_is_modified_ = true;
    return num;
}

/**
 * Detect the need of merging comp
 */
void Graph::mergeCheck()
{
    // This function is a copy of
    ConnectedComponent *c1(NULL), *c2(NULL);
    bool needMerge = false;

    std::vector<ConnectedComponent*>::iterator CompScan1 = comp_.begin();
    std::vector<ConnectedComponent*>::iterator CompScan2 = comp_.begin();

    while ( (CompScan1 != comp_.end()) && (needMerge == false))
    {
        /* For each other compco of th egraph */
        CompScan2 = comp_.begin();

        while ((CompScan2 != comp_.end()) && (needMerge == false))
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
void Graph::removeCompco(ConnectedComponent* comcpo)
{
    // Remove compco from vector
    std::vector<ConnectedComponent*>::iterator itCompCo;
    itCompCo = find( comp_.begin(), comp_.end(), comcpo );
    comp_.erase( itCompCo );

    // Decrement the compco ids
    for(int i=0;i<int(comp_.size());i++)
    {
        if( comp_[i]->getId() > comcpo->getId() )
        {
            comp_[i]->getCompcoStruct()->num--;
        }
    }

    // Update graph counter
    if( graph_ )
    {
        graph_->ncomp--;
        graph_->last_comp = comcpo->getCompcoStruct()->prec;
    }
    if ( comp_.empty() )
    {
        graph_->comp = NULL;
    }

    // Free memory Compco
    // p3d_remove_compco(graph_,CompCo->getCompcoStruct());
    delete comcpo;
}

bool Graph::checkConnectedComponents()
{
    p3d_compco* lc = graph_->comp;
    std::vector<ConnectedComponent*>::iterator itCompCo;

    while (lc)
    {
        for (itCompCo = comp_.begin(); itCompCo != comp_.end(); ++itCompCo )
        {
            if ( (*itCompCo)->getCompcoStruct() == lc )
            {
                break;
            }
            if ( itCompCo == ( comp_.end() - 1 ) )
            {
                throw std::string("The C compco and C++ are not the same");
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
                robot_->getRobotStruct(), node->getNodeStruct()->comp,
                node->getNodeStruct(), longStep);

    p3d_list_node* savedListDistNodePt = listDistNodePt;

    shared_ptr<LocalPath> LP;

    while (listDistNodePt)
    {
        if (!p3d_IsSmallDistInGraph(graph_, node->getNodeStruct(),
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
                if ( PlanEnv->getBool(PlanParam::orientedGraph) )
                    addEdges(node, nodes_Table[listDistNodePt->N], false, LP->length(), true, 0.0 );
                else
                    addEdge(node, nodes_Table[listDistNodePt->N], false, LP->length(), true, 0.0 );

                // p3d_create_edges(graph_, node->getNodeStruct(),
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
    for(unsigned int i=0; i<nodes_.size();i++)
    {
        nodes_[i]->cost();
    }

    for(unsigned int i=0;i<edges_.size();i++)
    {
        edges_[i]->cost();
    }
}

/**
 * Check if all edges are valid
 */
bool Graph::checkAllEdgesValid()
{
    int collTest = 0;
    for(unsigned int i=0;i<edges_.size();i++)
    {
        shared_ptr<LocalPath> ptrLP = edges_[i]->getLocalPath();

        if(!(ptrLP->isValid()))
        {
            return false;
        }

        collTest += ptrLP->getNbColTest();
    }

    cout << "Graph Total Number of coll. test = " << collTest << endl;

    return true;
}

//---------------------------------------------------------------------
//---------------------------------------------------------------------
// Warning Broken, WIP
API::Trajectory* Graph::extractDijkstraShortestPathsTraj( confPtr_t qi, confPtr_t qf )
{
    boost::property_map<BGL_Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, boost_graph_);
    BGL_EdgeDataMapT EdgeData = boost::get(EdgeData_t(), boost_graph_);
    BOOST_FOREACH(BGL_Edge e, boost::edges(boost_graph_))
    {
        weightmap[e] = EdgeData[e]->cost();
    }

//    boost::property_map<BGL_Graph, boost::vertex_distance_t>::type d = boost::get(boost::vertex_distance, boost_graph_);
//    boost::property_map<BGL_Graph, boost::vertex_predecessor_t>::type p = boost::get(boost::vertex_predecessor, boost_graph_);
    std::vector<BGL_Vertex> p(boost::num_vertices(boost_graph_));
    std::vector<double> d(boost::num_vertices(boost_graph_));

//    Node* node_init = searchConf( *qi );
    Node* node_init = nearestNeighbour(qi);
    if( node_init == NULL )
    {
        cout << "node init not in graph" << endl;
        return NULL;
    }

    BGL_Vertex s = node_init->getDescriptor();

    // dijkstra_shortest_paths(boost_graph_, s, predecessor_map(p).distance_map(d));
    dijkstra_shortest_paths( boost_graph_, s, boost::weight_map(weightmap).predecessor_map(&p[0]).distance_map(&d[0]) );

    std::cout << "distances and parents:" << std::endl;
    boost::graph_traits<BGL_Graph>::vertex_iterator vi, vend;
    for (boost::tie(vi, vend) = vertices(boost_graph_); vi != vend; ++vi) {
        std::cout << "distance(" << *vi << ") = " << d[*vi] << ", ";
        std::cout << "parent(" << *vi << ") = " << p[*vi] << std::endl;
    }
    std::cout << std::endl;

    std::ofstream dot_file("dijkstra-eg.dot");
    dot_file << "digraph D {\n"
             << "  rankdir=LR\n"
             << "  size=\"4,3\"\n"
             << "  ratio=\"fill\"\n"
             << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

    boost::graph_traits<BGL_Graph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(boost_graph_); ei != ei_end; ++ei)
    {
        boost::graph_traits<BGL_Graph>::edge_descriptor e = *ei;
        boost::graph_traits<BGL_Graph>::vertex_descriptor u = source(e, boost_graph_), v = target(e, boost_graph_);
        dot_file << u << " -> " << v << "[label=\"" << get( weightmap, e ) << "\"";
        if (p[v] == u)
            dot_file << ", color=\"black\"";
        else
            dot_file << ", color=\"grey\"";
        dot_file << "]";
    }
    dot_file << "}";

    return NULL;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

// euclidean distance heuristic
template <class TGraph, class CostType, class LocMap>
class distance_heuristic : public boost::astar_heuristic<TGraph, CostType>
{
public:
    typedef typename boost::graph_traits<TGraph>::vertex_descriptor Vertex;

    distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}

    CostType operator()(Vertex u)
    {
        if( ENV.getBool(Env::isCostSpace))
        {
            return 0.0;
        }
        else{
            confPtr_t q1 = m_location[m_goal]->getConfiguration();
            confPtr_t q2 = m_location[u]->getConfiguration();
            return q1->dist(*q2);
        }
    }
private:
    LocMap m_location;
    Vertex m_goal;
};


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
    astar_goal_visitor(Vertex goal) : m_goal(goal) {}
    template <class Graph>
    void examine_vertex(Vertex u, Graph& g) {
        if(u == m_goal)
            throw found_goal();
    }
private:
    Vertex m_goal;
};


// Extarct a shortes path between two nodes 
// of the same connected component
std::vector<Node*> Graph::extractAStarShortestNodePaths( Node* node_init, Node* node_goal, bool use_cost )
{
    std::vector<Node*> nodes;

    if (node_init == node_goal) {
        return nodes;
    }

    if (node_init->getConnectedComponent() != node_goal->getConnectedComponent()) {
        return nodes;
    }

    BGL_Vertex start = node_init->getDescriptor();
    BGL_Vertex goal = node_goal->getDescriptor();

    BGL_VertexDataMapT NodeData = boost::get(NodeData_t(), boost_graph_);
    std::vector<BGL_Vertex> p(boost::num_vertices(boost_graph_));
    std::vector<double> d(boost::num_vertices(boost_graph_));

    BGL_EdgeDataMapT EdgeData = boost::get(EdgeData_t(), boost_graph_);
    boost::property_map<BGL_Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, boost_graph_);

    // int i=0;
    BOOST_FOREACH(BGL_Edge e, boost::edges(boost_graph_)) {
        if( use_cost )
            weightmap[e] = EdgeData[e]->cost();
        else
            weightmap[e] = EPS6;

        if ( weightmap[e] == 0.0 )
        {
            cout << "weightmap" << e << " == 0.0" << endl;
        }
        // cout << "weightmap[" << i++ << "] : " << weightmap[e] << endl;
        // cout << "length[" << i++ << "] : " << EdgeData[e]->length() << endl;
    }

    typedef double cost;

    try // call astar named parameter interface
    {
        astar_search( boost_graph_,
                      start,
                      distance_heuristic<BGL_Graph, cost, BGL_VertexDataMapT>( NodeData, goal ),
                      boost::weight_map(weightmap).predecessor_map(&p[0]).distance_map(&d[0]).visitor( astar_goal_visitor<BGL_Vertex>(goal) )
                      //boost::predecessor_map(&p[0]),
                      );

    }
    catch( found_goal fg)
    {
        std::list<BGL_Vertex> shortest_path;
        for( BGL_Vertex v = goal;; v = p[v] ) {
            shortest_path.push_front(v);
            if (p[v] == v)
                break;
        }

        for( std::list<BGL_Vertex>::iterator spi = shortest_path.begin(); spi != shortest_path.end(); ++spi)
            nodes.push_back(NodeData[*spi]);
    }

    return nodes;
}

// Searches for the shortest path 
// bewteen the configuration qi and qf which has to be in the same connected component
std::vector<Node*> Graph::extractAStarShortestNodePaths( confPtr_t qi, confPtr_t qf )
{
    std::vector<Node*> nodes;

    Node* node_init = searchConf( *qi );
    if( node_init == NULL ) {
        cout << " node_init not found" << endl;
        return nodes;
    }

    Node* node_goal = node_init->getConnectedComponent()->searchConf( *qf );
    //Node* node_goal = nearestWeightNeighbour( node_init, qf, false, ENV.getInt(Env::DistConfigChoice));
    if( node_goal == NULL ) {
        cout << " node_goal not found" << endl;
        return nodes;
    }

    if( node_init == node_goal ) {
        cout << "node_init == node_goal" << endl;
        return nodes;
    }

    return extractAStarShortestNodePaths( node_init, node_goal );
}

//! This function works for tree type of graphs
API::Trajectory* Graph::extractBestAStarPathSoFar( confPtr_t qi, confPtr_t qf )
{
    // Start needs to be in the graph
    // TODO decide which one to use
    //    Node* source = searchConf(*qi);
    Node* source = nearestNeighbour(qi);
    if( source == NULL ) {
        cout << "qi not in graph in " << __PRETTY_FUNCTION__ << endl;
        return NULL;
    }

    if( source->getConnectedComponent()->getNumberOfNodes() < 2 ) {
        cout << "Not enough nodes in source component in " << __PRETTY_FUNCTION__ << endl;
        return NULL;
    }

    Node* target = nearestWeightNeighbour( source, qf, false, ENV.getInt(Env::DistConfigChoice));
    if( target == NULL ) {
        cout << "No goal nearest neighbour in graph in " << __PRETTY_FUNCTION__ << endl;
        return NULL;
    }

    std::vector<Node*> nodes = extractAStarShortestNodePaths( source, target );
    return trajectoryFromNodeVector( nodes );
}

// Searches for the shortest path 
// bewteen the configuration qi and qf which has to be in the same connected component
API::Trajectory* Graph::extractAStarShortestPathsTraj( confPtr_t qi, confPtr_t qf )
{
    std::vector<Node*> nodes = extractAStarShortestNodePaths( qi, qf );
    return trajectoryFromNodeVector( nodes );
}

API::Trajectory* Graph::trajectoryFromNodeVector( const std::vector<Node*>& nodes )
{
    if( nodes.size() < 2 ) {
        cout << "No path in " << __PRETTY_FUNCTION__ << endl;
        return NULL;
    }

    cout << "Trajectory has " << nodes.size() << " nodes" << endl;

    // Build the trajectory
    API::Trajectory* traj = new API::Trajectory( robot_ );

    for(int i=0; i<int(nodes.size()-1); i++)
    {
        Edge* edge = isEdgeInGraph( nodes[i], nodes[i+1] );

        if( edge == NULL ) {
            cout << "Edge is not in graph in " << __PRETTY_FUNCTION__ << endl;
            return NULL;
        }

        traj->push_back( nodes[i]->getConfiguration() );

        // Last configuration
        if( i+1 == int(nodes.size()-1) )
            traj->push_back( nodes[i+1]->getConfiguration() );

        // Warning does not work with local path and undirected graphs
        // traj->push_back( edge->getLocalPath() );
    }

    traj->replaceP3dTraj();
    return traj;
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

//! This function works for tree type of graphs
std::pair<bool, std::vector<Node*> > Graph::extractBestNodePathSoFar( confPtr_t qi, confPtr_t qf )
{
    std::vector<Node*> traj_nodes_reverse;

    // Start needs to be in the graph
    Node* source = searchConf(*qi);
    if( source == NULL ) {
        cout << "Start not in graph in " << __PRETTY_FUNCTION__ << endl;
        return make_pair(false,traj_nodes_reverse) ;
    }

    Node* node = nearestWeightNeighbour( source, qf, false, ENV.getInt(Env::DistConfigChoice));
    if( node == NULL ) {
        cout << "No goal nearest neihbour in graph in " << __PRETTY_FUNCTION__ << endl;
        return make_pair(false,traj_nodes_reverse);
    }

    // Extract the nodes in reverse order
    traj_nodes_reverse.push_back( node );
    while ( !node->getConfiguration()->equal(*qi) )
    {
        node = node->parent();
        traj_nodes_reverse.push_back( node );

        if (node == NULL) {
            cout << "Error finding nodes in " << __PRETTY_FUNCTION__ << endl;
            return make_pair(false,traj_nodes_reverse);
        }
    }

    if( int(traj_nodes_reverse.size()) <= 1 ) {
        return make_pair(true,traj_nodes_reverse);
    }

    // inverse traj
    std::vector<Node*> traj_nodes;
    for (int i=int(traj_nodes_reverse.size())-1; i>=0; i--) {
        traj_nodes.push_back( traj_nodes_reverse[i] );
    }

    return make_pair(true,traj_nodes_reverse);
}

// This function works for tree type of graphs
API::Trajectory* Graph::extractBestTrajSoFar( confPtr_t qi, confPtr_t qf )
{  
    // Extract the nodes trajectory
    std::pair<bool, std::vector<Node*> > traj_nodes = extractBestNodePathSoFar( qi, qf );

    if( traj_nodes.first == false ) {
        return NULL;
    }

    if( int(traj_nodes.second.size()) <= 1 ) {
        return NULL;
    }

    return trajectoryFromNodeVector( traj_nodes.second );
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

// This function extracts a trajectory
// it first converts the grapgh to a p3d_graph
API::Trajectory* Graph::extractBestTraj( confPtr_t qi, confPtr_t qf )
{  	
    //  cout << "----------------------------------------------" << endl;
    //	cout << "Extracting the trajectory" << endl;

    Node  *Ns=NULL,*Ng=NULL;
    p3d_traj* trajPt = NULL;

    bool drawWarning = false;

    if( graph_ == NULL )
    {
        cout << "Warning: cannot extract the best path\
                as there is no current graph" << endl;
                return NULL;
    }

    if( qf->isInCollision() )
    {
        (graph_->nb_test_coll)++;
        cout << "Computation of approximated optimal cost stopped: \
                Goal configuration in collision" << endl;
                return NULL;
    }

    // start and goal nodes creation and initialisation
    Ns = searchConf(*qi);
    if (Ns == NULL)  {
        Ns = new Node(this,qi);
        addNode(Ns);
        cout << "Create start node" <<  endl;
    }

    if (drawWarning) {
        cout << "Start comp : " << Ns->getConnectedComponent()->getId() << endl;
    }

    Ng = searchConf(*qf);
    if (Ng == NULL) {
        Ng = new Node(this,qf);
        addNode(Ng);
        cout << "Create goal node" <<  endl;
    }

    if (drawWarning) {
        cout << "Goal comp : " << Ng->getConnectedComponent()->getId() << endl;
    }

    GraphConverter gc;
    graph_ = gc.convert(*this);

    // start and goal nodes creation and initialisation
    p3d_node* Ns_ = p3d_TestConfInGraph( graph_, qi->getConfigStruct() );
    p3d_node* Ng_ = p3d_TestConfInGraph( graph_, qf->getConfigStruct() );

    if( Ns_== NULL || Ng_ == NULL ) {
        cout << "Error in " << __PRETTY_FUNCTION__ << endl;
        return NULL;
    }

    // Set the graph variables for a motion planning querry
    initMotionPlanning(Ns_,Ng_);

    robot_->setAndUpdate(*Ns->getConfiguration());

    // Search for the main constructed connected compoant
    p3d_compco* actualCompco = graph_->comp;
    p3d_compco* mainCompPt = actualCompco;
    //cout << "Nb nb of comp : " << graph_->ncomp << endl;
    for(int i=0;i<graph_->ncomp;i++)
    {
        if (drawWarning) {
            cout << "Id of actual comp : " << actualCompco->num << endl;
            cout << "Nb of nodes in actual comp : " << actualCompco->nnode << endl;
        }

        if (actualCompco->nnode > mainCompPt->nnode) {
            mainCompPt = actualCompco;
        }
        actualCompco = actualCompco->suiv;
    }

    if (drawWarning)
    {
        cout << "Number of connected component : " << graph_->ncomp << endl;
        cout << "Main connected component : " << mainCompPt->num << endl;
    }

    // Connection of the the extremal nodes to the main componant
    bool ConnectRes = true;

    ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(graph_, Ns_, mainCompPt));
    ConnectRes = (ConnectRes && p3d_ConnectNodeToComp(graph_, Ng_, mainCompPt));

    if(!ConnectRes)
    {
        cout << "Connection of the extremal nodes failed" << endl;
    }
    else
    {
        // on construit la trajectoire entre les points etapes
        (graph_->nb_test_coll)++;
        cout << "Connection of the extremal nodes succeeded" << endl;
        //cout << graph_->ncomp << endl;
        trajPt = p3d_graph_to_traj(robot_->getRobotStruct());
    }

    // time info
    // graph_->time = graph_->time + tu;

    if( ConnectRes && trajPt)
    {
        API::Trajectory* traj = new API::Trajectory(robot_,trajPt);
        //cout << "Trajectory cost  = " << traj->cost() << endl;
        return traj;
    }
    else
    {
        cout << "Trajectory extraction failed" << endl;
        return NULL;
    }
}

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

/**
 * Replaces p3d_InitRun
 */
void Graph::initMotionPlanning( p3d_node* start, p3d_node* goal )
{
#ifdef ENERGY
    int n_coldeg,icoldeg;
    double *coldeg_qs;
#endif
    graph_->search_start = start;
    if(ENV.getBool(Env::expandToGoal)== true)
    {
        graph_->search_goal = goal;
    }

    graph_->search_done = FALSE;
    p3d_InitDynDomainParam(graph_,start,goal);

    if( ENV.getBool(Env::isCostSpace) == true )
    {
        global_costSpace->initMotionPlanning( graph_, start, goal);
    }
    if( p3d_GetIsWeightedChoice()== TRUE )
    {
        p3d_init_root_weight(graph_);
    }
    if( ENV.getInt(Env::ExpansionNodeMethod) == RANDOM_IN_SHELL_METH )
    {
        p3d_init_pb_variables(graph_);
    }
    if( start != NULL )
    {
        start->rankFromRoot = 1;
        start->comp->nbTests = 0;
    }
    if( goal != NULL )
    {
        goal->rankFromRoot = 1;
        goal->comp->nbTests = 0;
    }
}

void Graph::drawNode(BGL_Vertex v) 
{
    p3d_jnt* 	drawnjnt=NULL;
    int indexjnt = p3d_get_user_drawnjnt();
    if (indexjnt != -1 && indexjnt >= 0 && indexjnt <= robot_->getRobotStruct()->njoints )
    {
        drawnjnt = robot_->getRobotStruct()->joints[indexjnt];
    }

    if (drawnjnt == NULL) {
        return;
    }

    double ray, x1, x2, y1, y2, z1, z2;

    if( drawnjnt->o != NULL ) {
        p3d_get_box_obj(drawnjnt->o, &x1, &x2, &y1, &y2, &z1, &z2); //get the object bounding box
        ray = sqrt(SQR(x2 - x1) + SQR(y2 - y1) + SQR(z2 - z1)) / 5.; // TODO get better method for size
    }
    else {
        ray = 0.05;
    }

    BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , boost_graph_ );

    robot_->setAndUpdate(*NodeData[v]->getConfiguration());
    p3d_vector3 pos;
    p3d_jnt_get_cur_vect_point(drawnjnt, pos);

    if(GroundCostObj) {
        double cost(0);
        GHintersectionVerticalLineWithGround(GroundCostObj, pos[0], pos[1], &cost);
        pos[2] = cost+0.5;
        ray = 0.5;
    }

    double color_array[4];
    int color = NodeData[v]->color_;
    if( color == 0 )
        color = NodeData[v]->getConnectedComponent()->getId();
    g3d_set_color( color, color_array );
    g3d_draw_solid_sphere(pos[0], pos[1], pos[2], ray, 10);
    //g3d_drawColorSphere(pos[0], pos[1], pos[2], ray, color, NULL);
}

void Graph::drawEdge(BGL_Vertex v1, BGL_Vertex v2) 
{
    int color=0;
    p3d_jnt* 	drawnjnt=NULL;
    int indexjnt = p3d_get_user_drawnjnt();
    if (indexjnt != -1 && indexjnt >= 0 && indexjnt <= robot_->getRobotStruct()->njoints )
    {
        drawnjnt = robot_->getRobotStruct()->joints[indexjnt];
    }

    if (drawnjnt == NULL) {
        return;
    }

    BGL_VertexDataMapT NodeData = boost::get( NodeData_t() , boost_graph_ );

    robot_->setAndUpdate(*NodeData[v1]->getConfiguration());
    p3d_vector3 source_pos;
    p3d_jnt_get_cur_vect_point(drawnjnt, source_pos);

    robot_->setAndUpdate(*NodeData[v2]->getConfiguration());
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
    //cout << "Draw BGL Graph , nb of nodes : " << nodes_.size() << endl;
    confPtr_t q = robot_->getCurrentPos();

    BOOST_FOREACH(BGL_Vertex v, boost::vertices(boost_graph_))
    {
        drawNode(v);
    }

    if( ENV.getBool(Env::drawEdges) )
    {
        BOOST_FOREACH(BGL_Edge e, boost::edges(boost_graph_))
        {
            BGL_Vertex v1(boost::source(e, boost_graph_));
            BGL_Vertex v2(boost::target(e, boost_graph_));
            drawEdge(v1, v2);
        }
    }

    robot_->setAndUpdate(*q);
}

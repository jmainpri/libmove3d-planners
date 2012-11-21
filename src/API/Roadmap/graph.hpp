#ifndef Graph_HPP
#define Graph_HPP

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/edge.hpp"
#include "API/Trajectory/trajectory.hpp"

#ifndef COMPCO_HPP
class ConnectedComponent;
#endif

#include <map>

/**
 @ingroup ROADMAP
 \brief A Graph representing class for a robot
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */

#ifndef _ROADMAP_H
struct node;
struct edge;
struct graph;
struct list_node;
struct list_edge;
#endif

class Graph {

public:

    //contructors and destructor
    Graph() {}
    Graph(Robot* R, graph* ptGraph);
    Graph(Robot* R);
    Graph(graph* G);
    Graph(const Graph& G);
    ~Graph();

    /**
     * Returns the value of the Change flag
     * @return true if the Graph has change since last export
     */
    bool isGraphChanged() { return m_graphChanged; }

    //Accessors
    /**
     * Gets the stored p3d_graph structure
     * @return the stored p3d_graph structure
     */
    graph* getGraphStruct() const;

    /**
     * Stores the p3d_graph structure
     * @param G the p3d_graph to store
     */
    void setGraph(graph* G);

    /**
     * Gets the robot for which the Graph has been created
     * @return the Robot object
     */
    Robot* getRobot() const;

    /**
     * Setss the Robot of the Graph
     * @param the Robot object
     */
    void setRobot(Robot* R);

    /**
     * Gets the map of p3d_node and Node of the Graph
     * @return a map containing the pointers to p3d_node and Node of the Graph
     */
    std::map<node*, Node*> getNodesTable();

    /**
     * Gets the number of Node in the Graph
     * @return the number of Node in the Graph
     */
    unsigned int getNumberOfNodes();

    /**
     * Gets the vector of Node of the Graph
     * @return the vector of Node of the Graph
     */
    std::vector<Node*> getNodes() const;

    /**
     * Gets the ith Node in the Graph
     * @return the ith Node in the Graph
     */
    Node* getNode(unsigned int i) const { return m_Nodes[i]; }

    /**
     * Gets the number of Edge in the Graph
     * @return the number of Edge in the Graph
     */
    unsigned int getNumberOfEdges() const;

    /**
     * Gets the number of Edge in the Graph
     * @return the number of Edge in the Graph
     */
    std::vector<Edge*> getEdges() const;

    /**
     * Gets the ith Edge in the Graph
     * @return the ith Edge in the Graph
     */
    Edge* getEdge(unsigned int i) const { return m_Edges[i]; }

    /**
     * Gets the number of ConnectedComponent in the Graph
     * @return the number of ConnectedComponent in the Graph
     */
    unsigned int getNumberOfCompco() const;//TODO : change name to getNumberOfConnectedComponents

    /**
     * Gets the ConnectedComponent of the ith Node in the Graph
     * @return the ConnectedComponent of the ith Node in the Graph
     */
    ConnectedComponent* getConnectedComponent(int ith) const;

    /**
     * Gets the vector of ConnectedComponent of the Graph
     * @return the vector of ConnectedComponent of the Graph
     */
    std::vector<ConnectedComponent*> getConnectedComponents() const;

    /**
     * Gets the Node corresponding to a p3d_node
     * @param N the p3d_node
     * @return the corresponding Node; NULL if it doesn't exist
     */
    Node* getNode(node* N);

    /**
     * Gets the last added Node to the Graph
     * @return the last added Node to the Graph
     */
    Node* getLastNode();

    /**
     * Gets the Graph name
     * @return the Graph name
     */
    std::string getName();

    /**
     * Sets a new standard name to the Graph
     */
    void setName();

    /**
     * Sets the name of the Graph
     * @param Name the new name
     */
    void setName(std::string Name);

    /**
     * Checks if two Graph are the same by comparing their list of Node
     * @param G the Graph to compare with
     * @return true if the two Graph are the same
     */
    bool equal(Graph* G);

    /**
     * Checks if two Graph have the same name
     * @param G the Graph to compare with
     * @return true if the two Graph have the same name
     */
    bool equalName(Graph* G);

    /**
     * Checks if a Node is in the Graph
     * @param N the Node to find
     * @return true if the Node is found
     */
    bool isInGraph(Node* N);

    /**
     * Looks for an Edge linking two Node in the Graph
     * @param N1 the first Node
     * @param N2 the second Node
     * @return the Edge if found else NULL					//To verify
     */
    Edge* isEdgeInGraph(Node* N1,Node* N2);

    /**
     * Gets the Node corresponding to a Configuration
     * @param q the Configuration to find
     * @return the Node if it exists else NULL
     */
    Node* searchConf(Configuration& q);//TODO : change name to searchConfiguration

    /**
     * Creates a new ConnectedComponent for a Node
     * @param node the Node to create a ConnectedComponent for
     */
    void createCompco(Node* node);//TODO : change name to createConnectedComponent

    /**
     * Inserts a Node in the Graph
     * @param node the Node to insert
     * @return the inserted Node
     */
    Node* insertNode(Node* node);

    /**
     * Inserts a Node of ISOLATE type in the Graph
     * @param node the Node to insert
     * @return the inserted Node
     */
    Node* insertExtremalNode(Node* node);			//Extremal!=ISOLATE??

    /**
     * \brief Link a Configuration to a Node for the RRT Planner		//TODO : change description
     * @param q the Configuration to link
     * @param expansionNode the Node
     * @param currentCost the current cost to reach the Node
     * @param step the max distance for one step of RRT expansion
     * @return the inserted Node
     */
    Node* insertNode(Node* expansionNode, LocalPath& path);

    /**
     * Link a Configuration to a Node for the RRT Planner				//TODO : change description
     * @param q the Configuration to link
     * @param from the Node
     * @return the linked Node
     */
    Node* insertConfigurationAsNode(std::tr1::shared_ptr<Configuration> q, Node* from, double step );

    /**
     * Sorts Edges by length
     */
    void sortEdges();

    /**
     * Adds an Edge to the Graph
     * @param source the initial Node of the Edge
     * @param target the final Node of the Edge
     * @param compute_length whether or not to compute the length of the resulting Edge
     * @param length the length of the Edge (if already computed)
     * @param compute_cost whether or not to compute the cost of the resulting Edge
     * @param cost the cost of the Edge (if already computed)
     * @return the added Egde
     */
    Edge* addEdge(Node* source, Node* target, bool compute_length, double length , bool compute_cost, double cost );

    /**
     * Adds two Edges (N1->N2 & N2->N1) to the Graph
     * @param N1 the first end Node of the Edge
     * @param N2 the seconde end Node of the Edge
     * @param compute_length whether or not to compute the length of the resulting Edge
     * @param length the length of the Edge (if already computed)
     * @param compute_cost whether or not to compute the cost of the resulting Edge
     * @param cost the cost of the Edge (if already computed)
     * @return the added Egdes
     */
    std::vector<Edge*> addEdges(Node* N1, Node* N2, bool compute_length, double length , bool compute_cost, double cost );

    /**
     * Removes an Edge of the Graph
     * @param E the Edge to be removed
     */
    void removeEdge(Edge* E);

    /**
     * Removes the Edge between source Node and target Node
     * @param source the initial Node of the Edge
     * @param target the final Node of the Edge
     */
    void removeEdge( Node* source, Node* target );

    /**
     * Removes the Edges between source Node and target Node
     * @param N1 the first end Node of the Edges
     * @param N2 the seconde end Node of the Edges
     */
    void removeEdges( Node* N1, Node* N2 );

    /**
     * Sorts Nodes by distance from a given Node
     * @param N the reference Node
     */
    void sortNodesByDist(Node* N);

    /**
     * Sorts Nodes by distance from a given configuration
     * @param q the reference configuration
     */
    void sortNodesByDist(confPtr_t q);

    /**
     * Sorts a list of Nodes by distance from a given configuration			//to check
     * @param config the reference configuration
     * @return nodes (by pointer) the list of sorted Nodes
     */
    static void sortNodesByDist(std::vector<Node*>& nodes, confPtr_t config);

    /**
     * Adds a Node to the Node list and the other structures
     * @param N the Node to be added
     */
    void addNode(Node* N);

    /**
     * Adds a Node if it is closer than a maximal distance from the Graph
     * @param N the Node to be added
     * @param maxDist the maximal distance
     */
    void addNode(Node* N, double maxDist);

    /**
     * Adds Nodes if they are closer than a maximal distance from the Graph
     * @param N the Nodes to be added
     * @param maxDist the maximal distance
     */
    void addNodes(std::vector<Node*> N, double maxDist);	//TODO : change N to Ns~~

    /**
     * Removes a Node from Graph
     * @param N the Node to be removed
     * @param rebuild_compco whether or not to rebuild the corresponding ConnectedComponent
     */
    void removeNode(Node* N, bool rebuild_compco = true);

    /**
     * Links a Node to the Graph
     * @param N the Node to be linked
     * @return True if the Node has been linked
     */
    bool linkNode(Node* N);

    /**
     * Links a Node to a ConnectedComponent with multisol
     * @param N the Node to be linked
     * @param compPt the ConnectedComponent to be linked with
     */
    bool linkNodeCompMultisol(Node *N, ConnectedComponent* compPt);

    /**
     * Links a Node to the Graph without taking into account the maximal distance
     * @param N the Node to be linked
     * @return True if the Node has been linked
     */
    bool linkNodeWithoutDist(Node* N);

    /**
     * Links a Node to the Graph by taking into account the maximal distance
     * @param N the Node to be linked
     * @return True if the Node has been linked
     */
    bool linkNodeAtDist(Node* N);

    /**
     * Links a Node to the Graph to all the visible Nodes
     * @param N the Node to be linked
     * @return True if the Node has been linked
     */
    bool linkToAllNodes(Node* N);


    /**
     * Computes if a Node is linked to another Node			// is not he same as isEdgeInGraph
     * @param node1 the first Node
     * @param node2 the second Node
     * @return true of the nodes can be linked								// can be linked or are linked ???
     * @return dist (by pointers) the distance between the nodes
     */
    bool areNodesLinked(Node* node1, Node* node2, double & dist);

    /**
     * Links 2 Nodes and merge them
     * @param node1 the first Node
     * @param node2 the second Node
     * @param compute_cost whether or not to compute the cost of the resulting Node/Link/Edge ???
     * @return True if successful
     */
    bool linkNodeAndMerge(Node* node1, Node* node2, bool compute_cost);

    /**
     * Creates random Nodes from random configurations
     * @param NMAX the number of Nodes to be created
     */
    void createRandConfs(int NMAX);

    /**
     * Gets a random Node in the connected component of a given Node
     * @param comp the Node in the connected component
     * @return the random Node
     */
    Node* randomNodeFromComp(Node* comp);							//connectedcomponent != node???

    /**
     * Tries to connect a Node to a given connected component
     * @param node the Node to be connected
     * @param compco the Node in the connected component
     * @return True if successful
     */
    bool connectNodeToCompco(Node* node, Node* compco);

    /**
     * Computes the K nearest neighbour Nodes of a configuration
     * @param config the configuration to find the K nearest neighbors
     * @param K the maximal number of neighbors
     * @param radius the maximal distance to the neighbors
     * @param weighted
     * @param distConfigChoice the type of distance computing method
     * @return the list of neighbors
     */
    std::vector<Node*> KNearestWeightNeighbour(confPtr_t config, int K, double radius, bool weighted, int distConfigChoice);

    /**
     * Computes the nearest neighbour Node of a configuration
     * @param compco the Node in the connected component
     * @param C the configuration to find the nearest neighbour
     * @param weighted
     * @param distConfigChoice the type of distance computing method
     * @return the nearest neighbour
     */
    Node* nearestWeightNeighbour(Node* compco, confPtr_t C, bool weighted, int distConfigChoice);

    /**
     * Gets the Ith ConnectedComponent
     * @param ith the rank of the ConnectedComponent
     * @return the ith ConnectedComponent					//Node
     */
    Node* getCompco(unsigned int ith);

    /**
     * Gets the vector of Nodes in the same ConnectedComponent
     * @param node a Node in the ConnectedComponent
     * @return the vector of Nodes in the same ConnectedComponent
     */
    std::vector<Node*> getNodesInTheCompCo(Node* node);
    //----------------------------------------------
    /**
     * ajoute des Edges formant des cycles dans le Graph
     * @param node le nouveau Node ajouté
     * @param step la distance d'expansion
     */
    void addCycles(Node* node, double step);

    /*fonction mergeant les compco N1 et N2*/
    /**
     * merge deux composantes connexes
     * @param CompCo1 la première composante connexe
     * @param CompCo2 la seconde composante connexe
     * @param DistNodes la distance entre les deux composantes connexes
     * @return les deux composantes sont mergées
     */
    int mergeComp(Node* CompCo1, Node* CompCo2, double DistNodes, bool compute_edge_cost);

    /**
     * Rebuild the connected component
     * using the strongly connected component algo from the BGL
     */
    int rebuildCompcoFromBoostGraph();

    /**
     * teste si des composantes connexes doivent être merger et le fait le cas échéant
     */
    void mergeCheck();

    /**
     * Delete the compco
     */
    void removeCompco(ConnectedComponent* CompCo);

    /**
     * Checks C and C++ Connected Components
     */
    bool checkConnectedComponents();

    /**
     * Recompute the Graph cost (Edges and Nodes)
     */
    void recomputeCost();

    /**
     * Recompute All Edges Valid
     */
    bool checkAllEdgesValid();

    /**
  * Extract best vector of nodes from qi that is closest to q_f
  */
    std::pair<bool, std::vector<Node*> > extractBestNodePathSoFar( confPtr_t qi, confPtr_t qf );

    /**
  * Extract best traj from qi that is closest to q_f
  */
    API::Trajectory* extractBestTrajSoFar( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    API::Trajectory* extractDijkstraShortestPathsTraj( confPtr_t qi, confPtr_t qf);

    /**
     * Extract best traj
     * @param the configuration
     */
    std::vector<Node*> extractAStarShortestNodePaths( Node* node1, Node* node2, bool use_cost = true );

    /**
     * Extract best traj
     * @param the configuration
     */
    std::vector<Node*> extractAStarShortestNodePaths( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    API::Trajectory* extractAStarShortestPathsTraj( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    API::Trajectory* extractBestAStarPathSoFar( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    API::Trajectory* extractBestTraj( confPtr_t qi, confPtr_t qf);

    /**
     * Init Motion planning problem
     */
    void initMotionPlanning(node* start, node* goal);


    // BGL functions ----------------------------------------
    // ------------------------------------------------------
    BGL_Vertex	findVertexDescriptor(Node* N);
    BGL_Edge		findEdgeDescriptor(Edge* E);
    void				saveBGLGraphToDotFile(const std::string& filename);
    BGL_Graph&	get_BGL_Graph() { return m_BoostGraph; }

    void    draw();

private:

    void    init();

    // BGL functions ----------------------------------------
    // ------------------------------------------------------
    void				initBGL();
    void				setAllDescriptorsInvalid();
    void    drawEdge(BGL_Vertex v1, BGL_Vertex v2);
    void    drawNode(BGL_Vertex v);

    void    deleteGraphStruct();
    void    updateCompcoFromStruct();
    void    freeResources();
    static bool compareEdges(Edge* E1, Edge* E2);
    static bool compareNodes(Node* N1, Node* N2);


    // Old p3d Graph
    graph* m_Graph;
    Robot* m_Robot;

    // Boost Graph Lib
    BGL_Graph m_BoostGraph;

    // Flag that is set to false
    // each time the Graph is exported to p3d_Graph
    bool m_graphChanged;
    bool m_initBGL;

    std::vector<Node*> m_Nodes;
    std::vector<Edge*> m_Edges;
    std::vector<ConnectedComponent*> m_Comp;

    // Maps between old and new nodes
    std::map<node*, Node*> m_NodesTable;

    // Graph name
    std::string m_Name;
};

extern Graph* API_activeGraph;

#endif

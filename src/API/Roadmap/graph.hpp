/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/edge.hpp"
#include "API/Trajectory/trajectory.hpp"

#include <map>

#ifndef _ROADMAP_H
struct node;
struct edge;
struct graph;
struct list_node;
struct list_edge;
#endif

/**
 @ingroup ROADMAP
 \brief Classe représentant un Graph pour un Robot
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */

namespace Move3D
{

#ifndef COMPCO_HPP
class ConnectedComponent;
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

    //! return the value of the Change flag
    //! @return true if the graph has change since last export
    bool isGraphChanged() { return graph_is_modified_; }

    //! Creates a p3d_graph from Boost Graph
    //graph* exportCppToGraphStruct(bool deleteGraphStruct = false);

    //Accessors
    /**
     * obtient la structure p3d_graph stockée
     * @return la structure p3d_graph stockée
     */
    graph* getGraphStruct() const;

    /**
     * stocke la structure p3d_graph
     * @param G la structure à stocker
     */
    void setGraph(graph* G);

    /**
     * obtient le Robot pour lequel le Graph est créé
     * @return le Robot pour lequel le Graph est créé
     */
    Robot* getRobot() const;

    /**
     * sets the Robot of the Graph
     */
    void setRobot(Robot* R);

    /**
     * obtient la table des noeuds du Graph
     * @return la table des noeuds du Graph
     */
    std::map<node*, Node*> getNodesTable();

    /**
     * obtient le nombre de Node dans le Graph
     * @return le nombre de Node dans le Graph
     */
    unsigned int getNumberOfNodes();

    /**
     * obtient le vecteur des Node du Graph
     * @return le vecteut des Node du Graph
     */
    const std::vector<Node*>& getNodes() const;

    /**
     * Get Node ith node in Graph
     */
    Node* getNode(unsigned int i) const { return nodes_[i]; }

    /**
     * obtient le nombre de Node dans le Graph
     * @return le nombre de Node dans le Graph
     */
    unsigned int getNumberOfEdges() const;

    /**
     * obtient le vecteur des Edge du Graph
     * @return le vecteur des Edge du Graph
     */
    const std::vector<Edge*>& getEdges() const;

    /**
     * Returns the ith edge
     */
    Edge* getEdge(unsigned int i)  const { return edges_[i]; }

    /*
     * Get the number of compco in the grap
     */
    unsigned int getNumberOfCompco() const;

    /**
     * Get Compco
     */
    ConnectedComponent* getConnectedComponent(int ith) const;

    /**
     * Get Compcos
     */
    std::vector<ConnectedComponent*> getConnectedComponents() const;

    /**
     * obtient le Node correspondant au p3d_node
     * @param N le p3d_node
     * @return le Node correspondant; NULL si le Node n'existe pas
     */
    Node* getNode(node* N);

    /**
     * obtient le dernier Node ajouté au Graph
     * @return le dernier Node ajouté au Graph
     */
    Node* getLastNode();

    /**
     * obtient le nom du Graph
     * @return le nom du Graph
     */
    std::string getName();

    /**
     * modifie le nom du Graph; un nom standard est alors mis
     */
    void setName();

    /**
     * modifie le nom du Graph
     * @param Name le nouveau nom
     */
    void setName(std::string Name);

    /**
     * teste si deux Graph sont égaux en comparant leur listNode
     * @param G le Graph à comparer
     * @return les deux Graph sont égaux
     */
    bool equal(Graph* G);

    /**
     * teste si deux Graph ont le même nom
     * @param G le Graph à comparer
     * @return les deux Graph ont le même nom
     */
    bool equalName(Graph* G);

    /**
     * vérifie si un node est dans le Graph
     * @param N le Node à tester
     * @return le Node est dans le Graph
     */
    bool isInGraph(Node* N);

    /**
     * Looks if two Node make an edge in the Graph
     * @param N1 the first node
     * @param N2 the second node
     * @return true if the node is the graph
     */
    Edge* isEdgeInGraph(Node* N1,Node* N2);

    /**
     * Gets the node that corresponds to a configuration
     * @param q la Configuration recherchée
     * @return le Node s'il existe, NULL sinon
     */
    Node* searchConf(Configuration& q);

    /**
     * New Compco
     * @param node to create a compco for
     */
    void createCompco(Node* node);

    /**
     * \brief Link a Configuration to a Node for the RRT Planner
     * @param q the Configuration to link
     * @param expansionNode the Node
     * @param currentCost the current cost to reach the Node
     * @param step the max distance for one step of RRT expansion
     * @return the inserted Node
     */
    Node* insertNode(Node* expansionNode, LocalPath& path);

    /**
     * Link a Configuration to a Node for the RRT Planner
     * @param q the Configuration to link
     * @param from the Node
     * @return the linked Node
     */
    Node* insertConfigurationAsNode(confPtr_t q, Node* from, double step );

    /**
     * trie les Edges en fonction de leur longueur
     */
    void sortEdges();

    /**
     * ajoute un Edge au Graph
     * @param N1 le Node initial de l'Edge
     * @param N2 le Node final de l'Edge
     * @param Long la longueur de l'Edge
     */
    Edge* addEdge(Node* source, Node* target, bool compute_length, double length , bool compute_cost,  double cost );

    /**
     * ajoute deux Edge au Graph
     * @param N1 l'une des extrémités des Edge
     * @param N2 l'autre extrémité des Edge
     * @param Long la longueur des Edge
     */
    std::vector<Edge*> addEdges(Node* N1, Node* N2, bool compute_length, double length , bool compute_cost,  double cost );

    /**
     * Remove edge
     */
    void removeEdge( Edge* E );

    /**
     * Remove edges between source node and target node
     */
    void removeEdge( Node* source, Node* target );

    /**
     * Remove the edges between the nodes N1 and N2
     */
    void removeEdges( Node* N1, Node* N2 );

    /**
     * Sort nodes against one given node
     * @param N le Node de référence
     */
    void sortNodesByDist(Node* N);

    /**
     * Sort nodes against one given configuration
     * @param N le Node de référence
     */
    void sortNodesByDist(confPtr_t q);

    /**
     * Adds a node to the node list and the other structures
     * @param N the node to be added
     */
    void addNode(Node* N);

    /**
     * ajoute un Node s'il est à moins d'une distance max du Graph
     * @param N le Node à ajouter
     * @param maxDist la distance max
     */
    void addNode(Node* N, double maxDist);

    /**
     * ajoute des Node s'ils sont à moins d'une distance max du Graph
     * @param N les Node à ajouter
     * @param maxDist la distance max
     */
    void addNodes(std::vector<Node*> N, double maxDist);

    /**
     * Remove Node from graph
     */
    void removeNode(Node* N, bool rebuild_compco = true);

    /**
     * lie un Node au Graph
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkNode(Node* N);

    /**
   * Links a node to a compco with multisol
   */
    bool linkNodeCompMultisol(Node *N, ConnectedComponent* compPt);

    /**
     * Lie un node au Graph sans prendre en compte la distance max
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkNodeWithoutDist(Node* N);

    /**
     * Lie un Node au Graph en prenant en compte la distance max
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkNodeAtDist(Node* N);

    /**
     * lie un Node à tous les autres Nodes visibles
     * @param N le Node à lier
     * @return le Node est lié
     */
    bool linkToAllNodes(Node* N);


    /**
     * Function that computes if
     * a node is linked to another node
     * @return true of the nodes can be linked
     * @return the distance between the nodes
     */
    bool areNodesLinked(Node* node1, Node* node2, double & dist);

    /**
     * Link 2 nodes and merge them
     */
    bool linkNodeAndMerge(Node* node1, Node* node2, bool compute_cost);

    /**
     * crée des Node à des Configurations aléatoires
     * @param NMAX le nombre de Node à crér
     * @param (*fct_stop)(void) la fonction d'arret
     * @param (*fct_draw)(void) la fonction d'affichage
     */
    void createRandConfs(int NMAX);

    /**
     * Gets Random node in the connected component
     */
    Node* randomNodeFromComp(Node* comp);

    /**
     * Function that trys to connect
     * a node to a given connected component
     */
    bool connectNodeToCompco(Node* node, Node* compco);


    /**
     * Compute the K nearest nodes
     * @param K the maximal number of neighbors
     */
    static void sortNodesByDist(std::vector<Node*>& nodes, confPtr_t config);

    /**
      * Basic nearest neighbour
      */
    Node* nearestNeighbour( confPtr_t q ) const;

    /**
     * obtient le plus proche voisin d'une composante connexe
     * @param compco la composante connexe dans laquelle on cherche le Node
     * @param C la Configuration dont on veut determier le plus proche voisin
     * @param weighted
     * @param distConfigChoice le type de calcul de distance
     * @return le Node le plus proche de la Configuration appartenant à la composante connexe
     */
    Node* nearestWeightNeighbour(Node* compco, confPtr_t C, int distConfigChoice);

    /**
     * Compute the K nearest on the graph nodes
     * @param K the maximal number of neighbors
     */
    std::vector<Node*> KNearestWeightNeighbour(confPtr_t config, int K, double radius, int distConfigChoice, Node* node_to_discard=NULL );

    /**
   * Compute the KNearestWeightNeighbour
   */
    static std::vector<Node*> KNearestWeightNeighbour( const std::vector<Node*>& nodes, confPtr_t config, int K, double radius, int distConfigChoice, Node* node_to_discard=NULL );

    /**
     * Get Ith Compco
     */
    Node* getCompco(unsigned int ith);

    /**
     * Vector of nodes in the same compco
     */
    std::vector<Node*> getNodesInTheCompCo(Node* node);

    /**
      * returns the largest connected component
      */
    ConnectedComponent* getLargestCompco() const;

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
    bool mergeComp(Node* CompCo1, Node* CompCo2, double DistNodes, bool compute_edge_cost);

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
     * Check C and C++ Connected Components
     */
    bool checkConnectedComponents();

    /**
     * Reset the Graph cost (Edges and Nodes)
     */
    void resetAllEdgesCost();

    /**
     * Recompute all edges cost (Edges and Nodes)
     */
    void computeAllEdgesAndNodesCost();

    /**
     * Reset the graph features (Edges and Nodes)
     */
    void resetAllEdgesFeatures();

    /**
     * Recompute all edges features (Edges and Nodes)
     */
    void computeAllEdgesAndNodesFeatures();

    /**
     * Reset the Graph Edges validity
     */
    void resetAllEdgesValid();

    /**
     * Recompute All Edges Valid
     */
    bool checkAllEdgesValid();

    /**
      * Get number of cost calls per edge
      */
    int getNumberOfColTestCalls();

    /**
      * Get number of cost calls per edge
      */
    int getNumberOfCostCalls();

    /**
   * Extract best vector of nodes from qi that is closest to q_f
   */
    std::pair<bool, std::vector<Node*> > extractBestNodePathSoFar( confPtr_t qi, confPtr_t qf );

    /**
   * Extract best traj from qi that is closest to q_f
   */
    Move3D::Trajectory* extractBestTrajSoFar( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    Move3D::Trajectory* extractDijkstraShortestPathsTraj( confPtr_t qi, confPtr_t qf);

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
    Move3D::Trajectory* extractAStarShortestPathsTraj( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    Move3D::Trajectory* extractBestAStarPathFromLargestComponent( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    Move3D::Trajectory* extractBestAStarPathSoFar( confPtr_t qi, confPtr_t qf );

    /**
     * Extract best traj
     * @param the configuration
     */
    Move3D::Trajectory* extractBestTraj( confPtr_t qi, confPtr_t qf);

    /**
     * Get trajectory from node vector
     * @param vector of nodes
     */
    Move3D::Trajectory* trajectoryFromNodeVector( const std::vector<Node*>& nodes );

    /**
     * Init Motion planning problem
     */
    void initMotionPlanning(node* start, node* goal);


    // BGL functions ----------------------------------------
    // ------------------------------------------------------
    BGL_Vertex	findVertexDescriptor(Node* N);
    BGL_Edge	findEdgeDescriptor(Edge* E);
    void		saveBGLGraphToDotFile(const std::string& filename);
    BGL_Graph&	get_BGL_Graph() { return boost_graph_; }

    void        draw();

private:

    void        init();

    // BGL functions ----------------------------------------
    // ------------------------------------------------------
    void		initBGL();
    void		setAllDescriptorsInvalid();
    void        drawEdge(BGL_Edge e);
    void        drawNode(BGL_Vertex v);

    void        deleteGraphStruct();
    void        updateCompcoFromStruct();
    void        freeResources();
    static bool compareEdges(Edge* E1, Edge* E2);
    static bool compareNodes(Node* N1, Node* N2);


    // Old p3d graph
    graph*                              graph_;
    Robot*                              robot_;

    // Boost Graph Lib
    BGL_Graph                           boost_graph_;

    // Flag that is set to false
    // each time the graph is exported to p3d_graph
    bool                                graph_is_modified_;
    bool                                init_bgl_;

    std::vector<Node*>                  nodes_;
    std::vector<Edge*>                  edges_;
    std::vector<ConnectedComponent*>    comp_;

    // Maps between old and new nodes
    std::map<node*, Node*>              nodes_Table;

    // Graph name
    std::string                         name_;
};

extern Graph* API_activeGraph;

}

#endif

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
#ifndef NODE_HPP
#define NODE_HPP

#include "API/ConfigSpace/configuration.hpp"

#ifndef _ROADMAP_H
struct node;
struct edge;
struct compco;
#endif

#include "BGL_Graph.hpp"

namespace Move3D {

#ifndef EDGE_HPP
class Edge;
#endif

#ifndef COMPCO_HPP
class ConnectedComponent;
#endif

class Graph;

class NodeData
{
public :
    // Constructor & Destructor
    NodeData();
    virtual ~NodeData();
};

/**
 @ingroup ROADMAP
 \brief Classe représentant un Node d'un Graph
 @author Florian Pilardeau,B90,6349 <fpilarde@jolimont>
 */
class Node
{
public:

    //Constructor and destructor
    Node();
    /**
     * Constructeur de la classe
     * @param G le Graph pour lequel le Node est créé
     * @param C la Configuration stockée dans le Node
   * @param id the number in the graph (Graph->getNumberOfNodes)
     */
    Node(Graph* G, confPtr_t C, bool newCompco=true);

    /**
     * Constructeur de la classe
     * @param G le Graph pour lequelle Node est créé
     * @param N la structure de p3d_node qui sera stockée
     */
    Node(Graph* G, node* N);

    /**
     * destructeur de la classe
     */
    ~Node();

    /**
     * Deletes the p3d node without deleting the
     * configuration allready stored in a shared_ptr
     */
    void deleteNode();

    /**
     * Equal operator
     */
    bool operator==(Node& N);

    //Accessors
    /**
     * obtient la structure p3d_node
     * @return la structure p3d_node
     */
    node* getNodeStruct();

    /**
     * obtient le Graph pour lequel le Node a été créé
     * @return le GRaph pour lequel le Node a été créé
     */
    Graph* getGraph();

    /**
     * obtient le Robot pour lequel le Node a été créé
     * @return le Robot pour lequel le Node a été créé
     */
    Robot* getRobot();

    /**
     * obtient la Configuration stockée
     * @return la Configuration stockée
     */
    confPtr_t getConfiguration() { return m_Configuration; }

    /**
     * modifie la valeur de activ
     * @param b la nouvelle valeur de activ
     */
    void activ(bool b);

    /**
     * obtient la valeur de activ
     * @return la valeur de activ
     */
    bool isActiv();

    /**
     * Returns the node id
     */
    unsigned int getId() const;

    /**
     * Sets the Id
     */
    void setId(unsigned int id);

    /**
     * returns a reference to the node parent when in a tree like structure
     * the pointer is editable
     */
    Node*& parent() { return m_parent; }

    /**
     * returns a reference to the if leaf data
     * the pointer is editable
     */
    bool& isLeaf() { return m_is_leaf; }

    /**
     * obtient le cout du Node
     * @return le cout du Node
     */
    double cost();

    /**
     * Returns the Sum of cost along the path
     */
    double& sumCost(bool recompute=false);

    /**
     * obtient la temperature du Node
     * @return la temperature du Node
     */
    double getTemp();

    /**
     * modifie la temperature du Node
     * @param t la nouvelle temperature
     */
    void setTemp(double t);

    /**
     * obtient la distance au Node courant créé
     * @return la distance au Node courant créé
     */
    double getDist();

    /**
   * copy of p3d_APInode_dist_multisol
   */
    double distMultisol(Node *node) const;

    /**
     * calcule la distance entre deux Node et stocke la valeur dans DistNew
     * @param N le Node à partir duquel on veut calculer la distance
     * @return la distance entre les deux Node
     */
    double dist(Node* N) const;

    /**
     * Compute the distance between the node and a configuration q and store it to DistNew
     * @param q the configuration
     * @return the distance
     */
    double dist(confPtr_t q) const;

    /**
     * teste si deux Node sont égaux
     * @param N le Node à tester
     * @return les Node sont égaux
     */
    bool equal(Node* N) const;

    /**
     * teste si deux Node sont dans la même composante connexe
     * @param N le Node à tester
     * @return les deux Node sont dans la même composante connexe
     */
    bool inSameComponent(Node* N) const;

    /**
     * Get Number of neighbors
     */
    int getNumberOfNeighbors();

    /**
     * Get All neighbors
     */
    std::vector<Node*> getNeighbors();

    /**
     * Get Number of Edges
     */
    int getNumberOfEdges();

    /**
     * Get All Edges
     */
    std::vector<Edge*> getEdges();

    /**
     * @brief Checks if two nodes can be linked
     * @param node the node to test
     * @param dist the distance between the nodes
     * @return true if nodes can be linked
     */
    bool isLinkable(Node* node, double* dist) const;

    /**
   * @brief Check if two node are connectable
   * @param node the node to test
   * @param dist the distance between the nodes
   * @return TRUE if connected FALSE otherwise
   */
    bool isLinkableMultisol(Node* node, double* dist) const;

    /**
     * vérifie si le poids max du Node a été atteint
     */
    void checkStopByWeight();

    /**
     * Gets the number of nodes
     * in the connected componnent
     */
    unsigned int getNumberOfNodesInCompco() const;

    /**
     * obtient la structure de composante connexe à laquelle appartient le Node
     * @return la structure de composante connexe à laquelle appartient le Node
     */
    ConnectedComponent* getConnectedComponent() const
    {
        return m_Compco;
    }

    /**
     * Sets the compco of the node
     * @param the connected component in which is the node
     */
    void setConnectedComponent(ConnectedComponent* compco);

    /**
     * detruit la composante connexe
     */
    void deleteCompco();

    /**
     * teste si la composante connexe a atteint le nombre max de Nodes
     * @return la composante connexe a atteint le nombre max de Nodes
     */
    bool maximumNumberNodes();

    /**
     * connect un Node à la composante connexe
     * @param N le Node à connecter
     * @return le Node est connecté
     */
    bool connectNodeToCompco(Node* N, double step);

    /**
     * merge deux composantes connexes
     * @param compco la composante connexe à merger
     */
    void merge( Node* compco, bool compute_edge_cost=false );

    /**
     * teste si deux composante connexe sont égales
     * @param compco la composante connexe à tester
     * @return les deux composantes sont égales
     */
    bool equalCompco(Node* compco) const;

    /**
     * tire un Node aléatoirement dans la composante connexe
     * @return le Node tiré
     */
    Node* randomNodeFromComp() const;

    /**
     * Method for EST
     */
    void			setSelectCost(double Cost) { _SelectCost = Cost; }
    double          getSelectCost() { return _SelectCost; }
    void			setExpandFailed() { _nbExpan++;  }
    int				getNbExpandFailed() { return _nbExpan; }

    std::vector<Node*>& getSortedNodes() {return _SortedNodes;}
    void			setSortedNodes( std::vector<Node*>& nodes ) { _SortedNodes = nodes;}


    /**
     * Prints the node to the standard output
     */
    void print() const;
    void printNeighbors();

    //--------------------------------------
    // BGL
    BGL_Vertex	getDescriptor();
    void				setDescriptor(const BGL_Vertex& V);
    void				unSetDescriptor();

    // Node color
    int color_;

private:

    // Old Node structure
    node* m_Node;
    NodeData* _specificNodeData;

    bool m_is_BGL_Descriptor_Valid;
    BGL_Vertex m_BGL_Descriptor;

    Graph* m_Graph;
    Robot* m_Robot;
    ConnectedComponent* m_Compco;

    // In tree graphs
    Node* m_parent;
    bool m_is_leaf;

    confPtr_t m_Configuration;
    bool _activ;

    double _SelectCost;
    int _nbExpan;
    std::vector<Node*> _SortedNodes;

    // Used for debug
    bool m_specificNode;
};

}

#endif

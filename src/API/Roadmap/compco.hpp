/*
 *  compco.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 CNRS/LAAS. All rights reserved.
 *
 */
#ifndef COMPCO_HPP
#define COMPCO_HPP

#include "API/ConfigSpace/configuration.hpp"

#include <vector>

#ifndef _ROADMAP_H
struct compco;
#endif

namespace Move3D {

class Node;
class Graph;

class ConnectedComponent
{
public:
    ConnectedComponent(Graph* G, compco* Comp);

    /**
     * Creates a compco by setting the structure
     * adding the node and setting its connected compco
     * to this
     */
    ConnectedComponent(Graph* G, Node* N);

    /**
   * Destructor
   */
    ~ConnectedComponent();

    /**
     * Returns the Connected component
     * Structure
     */
    compco* getCompcoStruct();

    /**
     * Get the id of the connected component
     */
    unsigned int getId() const;

    /**
     * Returns the number of nodes in
     * the connected component
     * @return the number of nodes in the Compco
     */
    unsigned int getNumberOfNodes() const;

    //! Get the number of refinement nodes.
    unsigned getNumberOfRefinementNodes() const;

    //! Add the given number to the current number of refinement nodes.
    void updateNumberOfRefinementNodes(unsigned nb);

    /**
     * Returns the nodes in the connected Compco
     * @return the nodes
     */
    std::vector<Node*>& getNodes();

    /**
     * Add the compco to the reachable Compco
     */
    void addToReachableList(ConnectedComponent* Comp);

    /**
     * Add the compco to the reachable Compco and update predecessors
     */
    void addToReachableListAndUpdatePredecessors(ConnectedComponent* Comp);

    /**
     * Returns the ith node of the compco
     * @param ith the id of the node in the connected component
     */
    Node* getNode(unsigned int ith) const { return m_Nodes[ith]; }

    /**
     * Adds a node to the connected component
     * @param N the node to add
     */
    void addNode(Node* N);

    /**
     * Merge CompcoPt with this compco and delete it
     * @param Pointer to the Connected component that will be freed
     */
    void mergeWith(ConnectedComponent* CompcoPt);

    /**
     * Can reach the compco
     */
    bool isLinkedToCompco(ConnectedComponent* CompcoPt);

    /**
     * Nearest weithed Neigboor
     */
    Node* nearestWeightNeighbour( confPtr_t q, int distConfigChoice );

    /**
   * KNearest Weight Neighbours
   */
    std::vector<Node*> KNearestWeightNeighbour( confPtr_t config, int K, double radius, int distConfigChoice );

    /**
   * Search configuration in connected compco
   */
    Node* searchConf(Configuration& q);

    //! Return a node, randomly chosen in the connected component.
    Node* randomNode();

    //! Get the temperature of the component.
    double getTemperature();

    //! Set the temperature of the component.
    void setTemperature(double temp);

private:
    compco*			m_Compco;
    Graph*			m_Graph;

    int m_Id;

    std::vector<Node*> m_Nodes;
    std::vector<ConnectedComponent*> m_CanReach;
};

}

#endif

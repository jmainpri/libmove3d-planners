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

class Node;
class Graph;

#ifndef _ROADMAP_H
struct compco;
#endif

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
	~ConnectedComponent();
	
	/**
	 * Returns the Connected component
	 * Structure
	 */
	compco* getCompcoStruct();
	
	/**
	 * Get the id of the connected component
	 */
	unsigned int getId();
	
	/**
	 * Returns the number of nodes in
	 * the connected component
	 * @return the number of nodes in the Compco
	 */
	unsigned int getNumberOfNodes();
	
	/**
	 * Returns the nodes in the connected Compco
	 * @return the nodes
	 */
	const std::vector<Node*>& getNodes() const;
	
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
//	Node* getNode(unsigned int ith) { return m_Nodes[i]; }
	
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
	Node* nearestWeightNeighbour(std::tr1::shared_ptr<Configuration> q, bool weighted, int distConfigChoice);
	
	
private:
	compco*			m_Compco;
	Graph*			m_Graph;
	
	int m_Id;
	
	std::vector<Node*> m_Nodes;
	std::vector<ConnectedComponent*> m_CanReach;
};
#endif
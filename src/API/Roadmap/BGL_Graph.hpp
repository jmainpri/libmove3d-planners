/*
 *  BGL_Graph.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 15/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef BGL_GRAPH_HPP
#define BGL_GRAPH_HPP

#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

//----------------------------------------------------------------------
// Node and Edge data

struct NodeData_t {
	typedef boost::vertex_property_tag kind;
};

struct EdgeData_t { 
	typedef boost::edge_property_tag kind;
};

class Node;
class Edge;

typedef boost::property<NodeData_t, Node*>														VertexProperty;
typedef boost::property<EdgeData_t, Edge*>														EdgeProperty;

// Defaut type of a graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
															VertexProperty,EdgeProperty >						BGL_Graph;

/// Defaut type of a vertex in a graph
typedef boost::graph_traits<BGL_Graph>::vertex_descriptor							BGL_Vertex;

/// Defaut type of an edge in a graph
typedef boost::graph_traits<BGL_Graph>::edge_descriptor								BGL_Edge;

/// Vertex map type
typedef boost::property_map<BGL_Graph, NodeData_t>::type							BGL_VertexDataMapT;

/// Edge map type
typedef boost::property_map<BGL_Graph, EdgeData_t>::type							BGL_EdgeDataMapT;

#endif

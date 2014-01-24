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

typedef boost::adjacency_list_traits<boost::listS, boost::vecS, boost::bidirectionalS>::vertex_descriptor vertex_descriptor;

//typedef VertexProperty;
//typedef EdgeProperty;

// bidirectionalS
// undirectedS

typedef boost::adjacency_list<
boost::listS,          //  The container used for egdes : here, std::list.
boost::vecS,           //  The container used for vertices: here, std::vector.
boost::undirectedS, //  directed or undirected edges ?.

boost::property<NodeData_t, Node*, 
boost::property<boost::vertex_distance_t, int,
boost::property<boost::vertex_predecessor_t, vertex_descriptor> > >, // The type that describes a Vertex

boost::property<EdgeData_t, Edge*, 
boost::property<boost::edge_weight_t, double> > // The type that describes en Edge
>	BGL_Graph;

/// Defaut type of a vertex in a graph
typedef boost::graph_traits<BGL_Graph>::vertex_descriptor							BGL_Vertex;

/// Defaut type of an edge in a graph
typedef boost::graph_traits<BGL_Graph>::edge_descriptor								BGL_Edge;

/// Vertex map type
typedef boost::property_map<BGL_Graph, NodeData_t>::type							BGL_VertexDataMapT;

/// Edge map type
typedef boost::property_map<BGL_Graph, EdgeData_t>::type							BGL_EdgeDataMapT;

#endif


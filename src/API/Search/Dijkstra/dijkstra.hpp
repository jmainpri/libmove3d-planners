/*
 * dijkstra.hpp
 *
 *  Created on: Oct 5, 2009
 *      Author: jmainpri
 */

#ifndef DIJKSTRA_HPP_
#define DIJKSTRA_HPP_

#include "API/Trajectory/trajectory.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Roadmap/graph.hpp"
#include "../../Grids/ThreeDGrid.hpp"

#include <list>
#include <map>

typedef int vertex_t;
typedef double weight_t;

/**
  * @ingroup SEARCH
  */
struct edge_dijkstra {
    vertex_t target;
    weight_t weight;
    edge_dijkstra(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

typedef std::map<vertex_t, std::list<edge_dijkstra> > adjacency_map_t;
typedef std::map<int,Move3D::Node*> node_map_t;

/**
  * @ingroup SEARCH
  */
template <typename T1, typename T2>
struct pair_first_less
{
    bool operator()(std::pair<T1,T2> p1, std::pair<T1,T2> p2) const
    {
       if(p1.first == p2.first) {
          //Otherwise the initial vertex_queue will have the size 2 { 0,source ; inf;n }
          return p1.second < p2.second;
       }
       return p1.first < p2.first;
    }
};

/**
  * @ingroup SEARCH
  * @brief Implement Dijkstra graph search algorithm
  */
class Dijkstra {

public :
	Dijkstra();
    Dijkstra(Move3D::Graph* ptrG);
	~Dijkstra();

	/**
	 * Creates the Maps out of the p3d Graph struct
	 */
	void creatStructures();

	void creatStructuresFromGrid(Move3D::ThreeDGrid* grid);

	void computePaths(vertex_t source,
	                          adjacency_map_t& adjacency_map,
	                          std::map<vertex_t, weight_t>& min_distance,
	                          std::map<vertex_t, vertex_t>& previous);

	std::list<vertex_t> getShortestPathTo(
		    vertex_t target, std::map<vertex_t, vertex_t>& previous);

	/**
	 * Example using the maps
	 */
	int example();

	/**
	 * Extract Trajectory beetween two Configurations
	 */
    Move3D::Trajectory* extractTrajectory( Move3D::confPtr_t init, Move3D::confPtr_t goal );

	/**
	 * Extract Trajectory beetween two Nodes
	 */
    Move3D::Trajectory* extractTrajectory( vertex_t source, vertex_t target );


private :
    Move3D::Graph* 		m_graph;
	adjacency_map_t 	m_graph_adjacency_map;
	node_map_t 			m_graph_node_map;
};

#endif /* DIJKSTRA_HPP_ */

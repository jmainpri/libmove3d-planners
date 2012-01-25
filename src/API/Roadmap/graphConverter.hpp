//
//  graphConverter.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 19/01/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef P3D_GRAPH_CONVERTER_HPP
#define P3D_GRAPH_CONVERTER_HPP

#include "API/Roadmap/graph.hpp"

#include "P3d-pkg.h"

#include <map>

class GraphConverter {
  
public:
  GraphConverter() {}
  ~GraphConverter() {}
  
  graph* convert(const Graph& g, bool deleteGraphStruct = true ) const;
  graph* exportGraphStruct(const Graph& g) const;
  
  Graph* convert(graph* g) const;
  
private:
  
  p3d_list_node* copyNodeList(std::map<p3d_node*,p3d_node*>& NodeMap, p3d_list_node* ln, p3d_list_node* end = NULL) const;
  p3d_list_edge* copyEdgeList(std::map<p3d_edge*,p3d_edge*>& EdgeMap, p3d_list_edge* le, p3d_list_edge* end = NULL) const;
  
  p3d_list_edge* createEdgeList(std::map<Edge*,p3d_edge*>& EdgeMap, const std::vector<Edge*>& edges, p3d_list_edge* end = NULL) const;
  p3d_list_node* createNodeList(std::map<Node*,p3d_node*>& NodeMap, const std::vector<Node*>& nodes, p3d_list_node* end = NULL) const;
};

#endif

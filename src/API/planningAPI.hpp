#ifndef PLANNING_API_HPP_
#define PLANNING_API_HPP_

/**
 * C++ basic headers (they all have name spaces)
 */
#include <iostream>
#include <iomanip>
#include <iosfwd>
#include <sstream>
#include <fstream>
#include <string>
//#include <vector>
#include <set>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <limits>
#include <algorithm>
#include <tr1/memory>

/**
 * Environment has to be included before anything (weird)
 */
#include "../p3d/env.hpp"

/**
 * The CPP API so that
 * Robot is first and Graph is last (kind of tricky because its backwards)
 */
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>
#include <Eigen/Geometry> 
#include <Eigen/LU>

// Configuration has no dependencies and is used by most other classes
#include "API/ConfigSpace/configuration.hpp"
#include "API/ConfigSpace/localpath.hpp"

// Dependency to robot
#include "API/Device/robot.hpp"

// Graph, Node, Edge, Robot and Localpath have no interdependencies
#include "API/Roadmap/node.hpp"
#include "API/Roadmap/edge.hpp"
//#include "Roadmap/compco.hpp"
#include "API/Roadmap/graph.hpp"

// LocalPathValidTest inherits LocalPath, include it after localpath.hpp
#include "API/ConfigSpace/localPathValidTest.hpp"

#include "API/Search/AStar/AStar.hpp"
#include "API/Search/Dijkstra/dijkstra.hpp"

//#include "planner/TrajectoryOptim/costOptimization.hpp"
//#include "planner/TrajectoryOptim/smoothing.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "API/scene.hpp"
#include "API/project.hpp"

#endif

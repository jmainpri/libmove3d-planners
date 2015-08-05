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
#ifndef PLANNING_API_HPP_
#define PLANNING_API_HPP_

///**
// * C++ basic headers (they all have name spaces)
// */
//#include <iostream>
//#include <iomanip>
//#include <iosfwd>
//#include <sstream>
//#include <fstream>
//#include <string>
////#include <vector>
//#include <set>
//#include <map>
//#include <list>
//#include <utility>
//#include <cstdlib>
//#include <limits>
//#include <algorithm>
//#include <tr1/memory>

///**
// * Environment has to be included before anything (weird)
// */
//#include <libmove3d/p3d/env.hpp>

///**
// * The CPP API so that
// * Robot is first and Graph is last (kind of tricky because its backwards)
// */
//////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
//#include <Eigen/Core>
//#define EIGEN_USE_NEW_STDVECTOR
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>
//#include <Eigen/LU>

//// Configuration has no dependencies and is used by most other classes
//#include "API/ConfigSpace/configuration.hpp"
//#include "API/ConfigSpace/localpath.hpp"

//// Dependency to robot
//#include "API/Device/robot.hpp"

//// Graph, Node, Edge, Robot and Localpath have no interdependencies
//#include "API/Roadmap/node.hpp"
//#include "API/Roadmap/edge.hpp"
////#include "Roadmap/compco.hpp"
//#include "API/Roadmap/graph.hpp"

//// LocalPathValidTest inherits LocalPath, include it after localpath.hpp
//#include "API/ConfigSpace/localPathValidTest.hpp"

//#include "API/Search/AStar/AStar.hpp"
//#include "API/Search/Dijkstra/dijkstra.hpp"

////#include "planner/TrajectoryOptim/costOptimization.hpp"
////#include "planner/TrajectoryOptim/smoothing.hpp"
//#include "API/Trajectory/trajectory.hpp"

//#include "API/scene.hpp"
//#include "API/project.hpp"

#endif

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
#ifndef GRAPHSAMPLER_HPP
#define GRAPHSAMPLER_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"

#include "API/Roadmap/graph.hpp"

namespace Move3D {

//! Sampler of noisy trajectories
class graphSampler
{
public:
    graphSampler();
    graphSampler( int num_points_per_dim, int num_joints );
    ~graphSampler();

    //! Initializes the different data structures
    void initialize();

    // get random graph
    Graph* sample();

    // Make graph in a grid
    Graph* makeGrid(int DichotomicFactor = 6 );

private:

    //! Samples a noisy trajectory
    Eigen::VectorXd sample_noisy(double std_dev=0.0);

    //! Allocate a multivariate gaussian sampler
    //! the sampler produces one dimenssional noisy trajectories
    bool preAllocateMultivariateGaussianSampler();

    //! set nodes in the graph
    void setNodesInGraph(Graph* g);

    Eigen::MatrixXd precision_;
    Eigen::MatrixXd inv_precision_;
    MultivariateGaussian* noise_generator_;
    int num_points_per_dim_;
    int num_joints_;
    int vect_length_;
    Eigen::VectorXd tmp_noise_;
};

}

#endif // GRAPHSAMPLER_HPP

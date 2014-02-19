#ifndef GRAPHSAMPLER_HPP
#define GRAPHSAMPLER_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"

#include "API/Roadmap/graph.hpp"

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

#endif // GRAPHSAMPLER_HPP

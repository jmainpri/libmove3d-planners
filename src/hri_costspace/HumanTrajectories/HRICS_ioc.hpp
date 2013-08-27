#ifndef HRICS_IOC_HPP
#define HRICS_IOC_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"

namespace HRICS
{

//! Trajectory structure
struct IocTrajectory
{
    IocTrajectory() { }
    IocTrajectory( int nb_joints, int nb_var );

    std::vector<Eigen::VectorXd> parameters_;                       /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_;                            /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_projected_;                  /**< [num_dimensions][num_time_steps] num_parameters */
    std::vector<Eigen::VectorXd> parameters_noise_projected_;       /**< [num_dimensions][num_time_steps] num_parameters */
    std::vector<Eigen::VectorXd> control_costs_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs_;                      /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> cumulative_costs_;                 /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> probabilities_;                    /**< [num_dimensions] num_time_steps */

    Eigen::VectorXd state_costs_;                                   /**< num_time_steps */
    Eigen::VectorXd feature_count_;                                 /**< num_features */

    bool out_of_bounds_; /**< Wether the rollout is violating dof limits */

    double getCost();   /**< Gets the rollout cost = state cost + control costs per dimension */
    void printCost();
    void printProbabilities();

    API::Trajectory getMove3DTrajectory( const ChompPlanningGroup* planning_group ) const;
};

//! Sampler of noisy trajectories
class IocSampler
{
public:
    IocSampler();
    IocSampler( int num_var_free, int num_joints );

    //! Initializes the different data structures
    void initialize();

    //! Samples a noisy trajectory
    Eigen::MatrixXd sample(double std_dev);

private:
    //! Initializes a coviarant trajectory policy
    //! Computes the control cost vector for the multivariate gaussian sampler
    void initPolicy();

    //! Allocate a multivariate gaussian sampler
    //! the sampler produces one dimenssional noisy trajectories
    bool preAllocateMultivariateGaussianSampler();

    stomp_motion_planner::CovariantTrajectoryPolicy policy_;
    std::vector<Eigen::MatrixXd> control_costs_;             /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<Eigen::MatrixXd> inv_control_costs_;         /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<MultivariateGaussian> noise_generators_;     /**< objects that generate noise for each dimension */
    int num_vars_free_;
    int num_joints_;
    Eigen::VectorXd tmp_noise_;
};

//! Main IOC class
class Ioc
{
public:
    Ioc( int num_vars, const ChompPlanningGroup* planning_group );

    //! Add a trajectory to the set of demonstrated trajectories
    bool addDemonstration(const Eigen::MatrixXd& demo);

    //! Generate the sampled trajectories
    //! around the demonstrations
    void generateSamples(int nb_samples);

    //! Returns Move3D trajectories
    std::vector<API::Trajectory> getSamples();

    //! Drawing function
    void addTrajectoryToDraw( const IocTrajectory& t, int color );
    void addAllToDraw();

    //! solve the ioc problem
    void solve( const Eigen::VectorXd& phi_demo, const std::vector<Eigen::VectorXd>& phi_k );

private:
    std::vector<IocTrajectory> demonstrations_;
    std::vector< std::vector<IocTrajectory> > samples_;
    double noise_stddev_;

    const ChompPlanningGroup* planning_group_;
    int num_vars_;
    int num_joints_;
    int num_demonstrations_;
    IocSampler sampler_;
};

}

void HRICS_run_sphere_ioc();

#endif // HRICS_IOC_HPP

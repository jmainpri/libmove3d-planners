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

struct IocTrajectory
{
    IocTrajectory() { }
    IocTrajectory( int nb_var, int nb_joints );

    std::vector<Eigen::VectorXd> parameters_;                       /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_;                            /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_projected_;                  /**< [num_dimensions][num_time_steps] num_parameters */
    std::vector<Eigen::VectorXd> parameters_noise_projected_;       /**< [num_dimensions][num_time_steps] num_parameters */
    Eigen::VectorXd state_costs_;                                   /**< num_time_steps */
    std::vector<Eigen::VectorXd> control_costs_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs_;                      /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> cumulative_costs_;                 /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> probabilities_;                    /**< [num_dimensions] num_time_steps */

    bool out_of_bounds_; /**< Wether the rollout is violating dof limits */

    double getCost();   /**< Gets the rollout cost = state cost + control costs per dimension */
    void printCost();
    void printProbabilities();
};

class IocSampler
{
public:
    IocSampler();
    IocSampler( int num_var_free, int num_joints );
    void initialize();
    void initPolicy();
    bool preAllocateMultivariateGaussianSampler();

    Eigen::VectorXd sample( int j );

private:
    stomp_motion_planner::CovariantTrajectoryPolicy policy_;
    std::vector<Eigen::MatrixXd> control_costs_;             /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<Eigen::MatrixXd> inv_control_costs_;         /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<MultivariateGaussian> noise_generators_;     /**< objects that generate noise for each dimension */
    int num_vars_free_;
    int num_joints_;
    Eigen::VectorXd tmp_noise_;
};

class Ioc
{
public:
    Ioc( int num_vars, const ChompPlanningGroup* planning_group );

    void addDemonstration(const Eigen::MatrixXd& demo);
    void generateSamples(int nb_samples);
    void getSingleRollout(const std::vector<Eigen::VectorXd>& rollout, std::vector<confPtr_t>& traj);
    void addTrajectoryToDraw( const std::vector<Eigen::VectorXd>& rollout, int color );
    void addAllToDraw();

    std::vector<IocTrajectory> demonstrations_;
    std::vector< std::vector<IocTrajectory> > samples_;
    std::vector<double> noise_stddev_;

    const ChompPlanningGroup* planning_group_;
    int num_vars_;
    int num_joints_;
    int num_demonstrations_;
    IocSampler sampler_;
};

}

#endif // HRICS_IOC_HPP

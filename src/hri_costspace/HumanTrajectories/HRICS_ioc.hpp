#ifndef HRICS_IOC_HPP
#define HRICS_IOC_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "HRICS_features.hpp"

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"

namespace HRICS
{

//! Evaluation Class
class IocEvaluation
{
public:
    IocEvaluation(Robot* rob, int nb_samples);

    //! Run learning using the C++ library
    virtual void runLearning();

    //! Generate demonstration using optimal planning
    void generateDemonstrations();

    //! Load recorded traectories in the move3d format
    void loadDemonstrations();

    //! Load weight vector from CSV format
    void loadWeightVector();

    //! Save 2D traj to file for Matlab
    void saveDemoToMatlab();

    //! Compute costs using the original costs and the learned costs
    Eigen::VectorXd compareDemosAndPlanned();

protected:

    //! Compute the cost of the demos
    Eigen::VectorXd getCostsOfDemonstrations() const;

    //! Save trajectory to matrix
    void saveTrajToMatlab(const API::Trajectory& t) const;

    //! Save all the feature in a matrix
    //! that can be read by Matlab
    void saveToMatrix(const std::vector<FeatureVect>& demos, const std::vector< std::vector<FeatureVect> >& samples );

    //! Plans a motion using the costmap
    API::Trajectory planMotionRRT();
    API::Trajectory planMotionStomp();

    virtual void setLearnedWeights();
    virtual void setOriginalWeights();

    Robot* robot_;
    int nb_demos_;
    int nb_samples_;
    int nb_weights_;
    int nb_way_points_;
    std::string folder_;
    std::vector<API::Trajectory> demos_;
    std::vector<API::Trajectory> samples_;
    std::vector<API::Trajectory> learned_;
    WeightVect learned_vect_;
    WeightVect original_vect_;
    std::string feature_matrix_name_;
    std::vector<int> active_joints_;
    Feature* feature_fct_;
    TrajectorySmoothness* smoothness_fct_;
    ChompPlanningGroup* plangroup_;

};

//! Trajectory structure
struct IocTrajectory
{
    IocTrajectory() { }
    IocTrajectory( int nb_joints, int nb_var );

    std::vector<Eigen::VectorXd> nominal_parameters_;               /**< [num_dimensions] num_parameters */
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

    //! Reduces the trajectory magnitude
    bool jointLimits( IocTrajectory& traj ) const;

    //! Generate the sampled trajectories
    //! around the demonstrations
    void generateSamples(int nb_samples);

    //! Returns Move3D trajectories
    std::vector< std::vector<API::Trajectory> > getSamples();

    //! Drawing function
    void addTrajectoryToDraw( const IocTrajectory& t, int color );
    void addAllToDraw();

    //! Solve the ioc problem
    Eigen::VectorXd solve( const std::vector<Eigen::VectorXd>& phi_demo, const std::vector< std::vector<Eigen::VectorXd> >& phi_k );

    //! Set feature function
    void setFeatureFct( Feature* fct ) { feature_fct_ = fct; }

private:
    std::vector<IocTrajectory> demonstrations_;
    std::vector< std::vector<IocTrajectory> > samples_;
    double noise_stddev_;

    const ChompPlanningGroup* planning_group_;
    int num_vars_;
    int num_joints_;
    int num_demonstrations_;
    IocSampler sampler_;

    Feature* feature_fct_;
};

}

void HRICS_run_sphere_ioc();

#endif // HRICS_IOC_HPP

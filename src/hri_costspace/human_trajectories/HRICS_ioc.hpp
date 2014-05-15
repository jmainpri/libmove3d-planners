#ifndef HRICS_IOC_HPP
#define HRICS_IOC_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "feature_space/features.hpp"

#include "HRICS_run_multiple_planners.hpp"

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

    std::vector<Eigen::VectorXd> nominal_parameters_;               /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> parameters_;                       /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_;                            /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_projected_;                  /**< [num_dimensions][num_time_steps] num_parameters */
    std::vector<Eigen::VectorXd> parameters_noise_projected_;       /**< [num_dimensions][num_time_steps] num_parameters */
    std::vector<Eigen::VectorXd> control_costs_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs_;                      /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> cumulative_costs_;                 /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> probabilities_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> straight_line_;                    /**< [num_dimensions] num_parameters */

    Eigen::VectorXd state_costs_;                                   /**< num_time_steps */
    Eigen::VectorXd feature_count_;                                 /**< num_features */

    bool out_of_bounds_; /**< Wether the rollout is violating dof limits */

    double getCost();   /**< Gets the rollout cost = state cost + control costs per dimension */
    void printCost();
    void printProbabilities();

    // Returns the move3d trajectory
    Move3D::Trajectory getMove3DTrajectory( const Move3D::ChompPlanningGroup* planning_group ) const;

    // Interpolation between two vector
    Eigen::VectorXd interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const;

    // Sets the interpolated trajectory
    void setSraightLineTrajectory();
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
    Ioc( int num_vars, const Move3D::ChompPlanningGroup* planning_group );

    //! Add a trajectory to the set of demonstrated trajectories
    bool addDemonstration(const Eigen::MatrixXd& demo);

    //! Add a trajectory to the set of sample trajectories
    //! @param d is the id of the demonstration
    //! @param sample is the trajectory
    bool addSample( int d, const Eigen::MatrixXd& sample );

    //! Set mean
    bool setNominalSampleValue( int d, int i, const Eigen::MatrixXd& sample ) ;

    //! Set costs
    bool setTotalCostsSampleValue( int d, int i, const Eigen::VectorXd& sample ) ;

    //! Reduces the trajectory magnitude
    bool jointLimits( IocTrajectory& traj ) const;

    //! Generate the sampled trajectories
    //! around the demonstrations
    void generateSamples(int nb_samples);

    //! Returns Move3D trajectories
    std::vector< std::vector<Move3D::Trajectory> > getSamples();

    //! Returns Move3D trajectories
    std::vector< Move3D::Trajectory > getDemonstrations();

    //! Drawing function
    void addTrajectoryToDraw( const IocTrajectory& t, int color );
    void addAllToDraw();

    //! Solve the ioc problem
    Eigen::VectorXd solve( const std::vector<Eigen::VectorXd>& phi_demo, const std::vector< std::vector<Eigen::VectorXd> >& phi_k );

    // Returns the number of demonstrations
    int getNbOfDemonstrations() { return demonstrations_.size(); }

private:
    std::vector< IocTrajectory > demonstrations_;
    std::vector< std::vector<IocTrajectory> > samples_;
    double noise_stddev_;

    const Move3D::ChompPlanningGroup* planning_group_;
    int num_vars_;
    int num_joints_;
    IocSampler sampler_;
};

//! Evaluation Class
class IocEvaluation
{
public:
    IocEvaluation( Move3D::Robot* rob, int nb_demos, int nb_samples, int nb_way_points, MultiplePlanners& planners );

    //! Sample trajectories around the demonstrations
    virtual void runSampling();

    //! Run learning using the C++ library
    virtual void runLearning();

    //! Run Sampling using saved trajectories
    void runFromFileSampling( int offset );

    //! Run Stomp for multiple feature functions
    void runPlannerMultipleFeature( int nb_runs=1 );

    //! Run Stomp on weighted features
    void runPlannerWeightedFeature( int nb_runs=1 );

    //! Generate demonstration using optimal planning
    void generateDemonstrations();

    //! Load recorded traectories in the move3d format
    void loadDemonstrations();

    //! Load trajectories in planner class
    void loadPlannerTrajectories( int nb_trajs=-1, int offset=-1, int random=0 );

    //! Load weight vector from CSV format
    void loadWeightVector();

    //! Save 2D traj to file for Matlab
    void saveDemoToMatlab();

    //! Compute costs using the original costs and the learned costs
    Eigen::VectorXd compareDemosAndPlanned();

    //! Check start and goal of sampled trajectories
    bool checkStartAndGoal( const std::vector< std::vector<Move3D::Trajectory> >& samples ) const;

    //! Generate a distribution that maximizes entropy
    void monteCarloSampling( double factor, int nb_tries );

protected:

    std::vector<Move3D::FeatureVect> addDemonstrations(Ioc& ioc);
    std::vector< std::vector<Move3D::FeatureVect> > addSamples(Ioc& ioc);

    //! Compute the cost of the demos
    Eigen::VectorXd getCostsOfDemonstrations() const;

    //! Save trajectory to matrix
    void saveTrajToMatlab(const Move3D::Trajectory& t, int id) const;

    //! Save all the feature in a matrix
    //! that can be read by Matlab
    void saveToMatrixFile(const std::vector<Move3D::FeatureVect>& demos, const std::vector< std::vector<Move3D::FeatureVect> >& samples, std::string name );

    //! Save sample trajectories
    void saveTrajectories(const std::vector<Move3D::Trajectory>& trajectories);

    // Return the feature gradien norm sum along each trajectory
    std::vector< std::vector<Move3D::FeatureVect> > getFeatureCount( const std::vector< std::vector<Move3D::Trajectory> >& all_trajs );

    // Return the feature gradien norm sum along each trajectory
    std::vector< std::vector<Move3D::FeatureVect> > getFeatureJacobianSum( const std::vector< std::vector<Move3D::Trajectory> >& all_trajs );

    //! Plans a motion using the costmap
    Move3D::Trajectory planMotion( planner_t type );

    //! Set all features active
    void activateAllFeatures();

    //! Compute weights when planning multiple times.
    Move3D::WeightVect computeOptimalWeights();

    //! Compute if samples dominate demonstration
    bool checkDegeneration( const std::vector<Move3D::FeatureVect>& demos, const std::vector< std::vector<Move3D::FeatureVect> >& samples ) const;

    //! Returns true if the sample2 is domintated by sample1
    bool isSampleDominated( const Move3D::FeatureVect& sample1, const Move3D::FeatureVect& sample2 ) const;

    //! Removes the dominated samples by resampling
    void removeDominatedSamplesAndResample( Ioc &ioc, std::vector< std::vector<Move3D::FeatureVect> >& phi_k );

    //! Returns trajectory that best fits
    Move3D::Trajectory selectBestSample( double detla_mean, const std::vector<Move3D::Trajectory>& trajs );

    virtual void setLearnedWeights();
    virtual void setOriginalWeights();

    Move3D::Robot* robot_;
    int nb_demos_;
    int nb_samples_;
    int nb_weights_;
    int nb_way_points_;
    std::string folder_;
    std::vector<Move3D::Trajectory> demos_;
    std::vector<Move3D::Trajectory> samples_;
    std::vector<Move3D::Trajectory> learned_;
    Move3D::WeightVect learned_vect_;
    Move3D::WeightVect original_vect_;

    std::vector<int> active_joints_;
    Move3D::StackedFeatures* feature_fct_;
    std::string feature_type_;
    Move3D::TrajectorySmoothness* smoothness_fct_;
    Move3D::ChompPlanningGroup* plangroup_;

    bool load_sample_from_file_;
    MultiplePlanners& planners_;
    int round_id_;

    std::vector<Move3D::FeatureVect> stored_features_;

    std::vector<Move3D::FeatureVect> phi_demos_;
    std::vector<Move3D::FeatureVect> phi_jac_demos_;
};

}

void HRICS_run_sphere_ioc();

#endif // HRICS_IOC_HPP

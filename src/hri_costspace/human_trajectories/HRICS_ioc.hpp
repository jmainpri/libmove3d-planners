#ifndef HRICS_IOC_HPP
#define HRICS_IOC_HPP

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "HRICS_features.hpp"
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
    API::Trajectory getMove3DTrajectory( const ChompPlanningGroup* planning_group ) const;

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
    Ioc( int num_vars, const ChompPlanningGroup* planning_group );

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
    std::vector< std::vector<API::Trajectory> > getSamples();

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

    const ChompPlanningGroup* planning_group_;
    int num_vars_;
    int num_joints_;
    IocSampler sampler_;
};

//! Evaluation Class
class IocEvaluation
{
public:
    IocEvaluation( Robot* rob, int nb_demos, int nb_samples, int nb_way_points, MultiplePlanners& planners );

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
    bool checkStartAndGoal( const std::vector< std::vector<API::Trajectory> >& samples ) const;

protected:

    std::vector<FeatureVect> addDemonstrations(Ioc& ioc);
    std::vector< std::vector<FeatureVect> > addSamples(Ioc& ioc);

    //! Compute the cost of the demos
    Eigen::VectorXd getCostsOfDemonstrations() const;

    //! Save trajectory to matrix
    void saveTrajToMatlab(const API::Trajectory& t, int id) const;

    //! Save all the feature in a matrix
    //! that can be read by Matlab
    void saveToMatrix(const std::vector<FeatureVect>& demos, const std::vector< std::vector<FeatureVect> >& samples );

    //! Get all the feature from a matrix file
    bool loadFromMatrix( std::vector<FeatureVect>& demos, std::vector< std::vector<FeatureVect> >& samples );

    //! Plans a motion using the costmap
    API::Trajectory planMotionRRT();
    API::Trajectory planMotionStomp();
    API::Trajectory planAStar();

    void activateAllFeatures();

    //! Compute weights when planning multiple times.
    WeightVect computeOptimalWeights();

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

    bool load_sample_from_file_;
    MultiplePlanners& planners_;
    int round_id_;

    std::vector<FeatureVect> stored_features_;
};

}

void HRICS_run_sphere_ioc();

#endif // HRICS_IOC_HPP

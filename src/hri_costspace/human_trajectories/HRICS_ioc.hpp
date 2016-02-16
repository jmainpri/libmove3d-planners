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
#ifndef HRICS_IOC_HPP
#define HRICS_IOC_HPP

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

#include "feature_space/features.hpp"
#include "feature_space/smoothness.hpp"

#include "HRICS_run_multiple_planners.hpp"

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"
#include "planner/TrajectoryOptim/jointlimits.hpp"

namespace HRICS {

typedef std::vector<std::vector<Move3D::confPtr_t> > context_t;

//! Trajectory structure
struct IocTrajectory {
  IocTrajectory() : dt_(0.0) {}
  IocTrajectory(int nb_joints, int nb_var, double dt);

  std::vector<Eigen::VectorXd>
      nominal_parameters_; /**< [num_dimensions] num_parameters */
  std::vector<Eigen::VectorXd>
      parameters_;                     /**< [num_dimensions] num_parameters */
  std::vector<Eigen::VectorXd> noise_; /**< [num_dimensions] num_parameters */
  std::vector<Eigen::VectorXd>
      control_costs_; /**< [num_dimensions] num_time_steps */
  std::vector<Eigen::VectorXd>
      total_costs_; /**< [num_dimensions] num_time_steps */
  std::vector<Eigen::VectorXd>
      cumulative_costs_; /**< [num_dimensions] num_time_steps */
  std::vector<Eigen::VectorXd>
      probabilities_; /**< [num_dimensions] num_time_steps */
  std::vector<Eigen::VectorXd>
      straight_line_; /**< [num_dimensions] num_parameters */

  double dt_; /**< time discretization */

  Eigen::VectorXd state_costs_;   /**< num_time_steps */
  Eigen::VectorXd feature_count_; /**< num_features */

  bool out_of_bounds_; /**< Wether the rollout is violating dof limits */

  double getCost(); /**< Gets the rollout cost = state cost + control costs per
                       dimension */
  void printCost();
  void printProbabilities();

  //! Get last waypoint
  int last_waypoint() const { return parameters_[0].size() - 1; }

  //! Returns the move3d trajectory
  Move3D::Trajectory getMove3DTrajectory(
      const Move3D::ChompPlanningGroup* planning_group) const;

  //! Returns the move3d configuration at the waypoint instant
  Move3D::confPtr_t getMove3DConfig(
      int waypoint,
      Move3D::Robot* robot,
      const std::vector<Move3D::ChompDof>& joints) const;

  //! Interpolation between two vector
  Eigen::VectorXd interpolate(const Eigen::VectorXd& a,
                              const Eigen::VectorXd& b,
                              double u) const;

  //! Sets the interpolated trajectory
  void setSraightLineTrajectory();

  //! \brief Get the value at a given point for a given dof
  double& operator()(int dof, int waypoint) {
    return parameters_[dof][waypoint];
  }

  //! \brief Get the value at a given point for a given dof
  double operator()(int dof, int waypoint) const {
    return parameters_[dof][waypoint];
  }
};

//! Ioc structure
struct IocIk {
  IocIk() {}
  IocIk(int nb_joints);

  Eigen::VectorXd nominal_parameters_; /**< [num_dimensions] num_parameters */
  Eigen::VectorXd parameters_;         /**< [num_dimensions] num_parameters */
  Eigen::VectorXd noise_;              /**< [num_dimensions] num_parameters */

  bool out_of_bounds_; /**< Wether the rollout is violating dof limits */

  Move3D::confPtr_t getMove3DConfig(
      const Move3D::ChompPlanningGroup* planning_group) const;
};

//! Sampler of noisy trajectories
class IocSampler {
 public:
  IocSampler();
  IocSampler(int num_var_free,
             int num_dofs,
             const Move3D::ChompPlanningGroup* planning_group);

  //! Initializes the different data structures
  bool Initialize();

  //! Project trajectory sample to joint limits
  void ProjectToJointLimitsQuadProg(HRICS::IocTrajectory& traj);

  //! Samples a noisy trajectory
  Eigen::MatrixXd sample(double std_dev);

  //! Samples a noisy configuration
  IocIk sample_ik(double std_dev);

  //! Structure for dof bound
  struct dof_bounds {
    bool circular_;  // no bounds
    double min_;
    double max_;
    double range_;
  };

  int num_vars_free() const { return num_vars_free_; }
  int num_dofs() const { return num_dofs_; }

  const std::vector<dof_bounds>& joint_bounds() const { return dofs_bounds_; }

  const std::vector<Eigen::MatrixXd>& control_costs() const {
    return control_costs_;
  }

 protected:
  //! Initializes a coviarant trajectory policy
  //! Computes the control cost vector for the multivariate gaussian sampler
  virtual void initPolicy();

  //! Allocate a multivariate gaussian sampler
  //! the sampler produces one dimenssional noisy trajectories
  bool preAllocateMultivariateGaussianSampler();

  //! Initialize the joint limit computers
  void InitializeJointLimitProjector();

  //! computes the control cost matrices for sampling
  stomp_motion_planner::CovariantTrajectoryPolicy policy_;

  //! [num_dimensions] num_parameters x num_parameters */
  std::vector<Eigen::MatrixXd> control_costs_;

  //! [num_dimensions] num_parameters x num_parameters
  std::vector<Eigen::MatrixXd> convariances_;

  //! objects that generate noise for each dimension */
  std::vector<MultivariateGaussian> noise_generators_;

  int num_vars_free_;
  int num_dofs_;
  Eigen::VectorXd tmp_noise_;
  std::vector<dof_bounds> dofs_bounds_;
  const Move3D::ChompPlanningGroup* planning_group_;
  std::vector<Move3D::TrajOptJointLimit> joint_limits_computers_;
};

class IocSamplerGoalSet : public IocSampler {
 public:
  IocSamplerGoalSet() {}
  IocSamplerGoalSet(int num_var_free,
                    int num_joints,
                    const Move3D::ChompPlanningGroup* planning_group);

 protected:
  //! Initializes a coviarant trajectory policy
  //! Computes the control cost vector for the multivariate gaussian sampler
  virtual void initPolicy();
};

//! Sampler of noisy trajectories
class IocSamplerIk : public IocSampler {
 public:
  IocSamplerIk() {}
  IocSamplerIk(int nb_joints);
};

//! Goalset data structure
struct GoalsetData_t {
  GoalsetData_t()
      : sample_goal_set_(false),
        ratio_projected_(0),
        strength_(0),
        x_task_goal_(Eigen::Vector3d::Zero()),
        eef_(NULL) {}

  bool sample_goal_set_;
  double ratio_projected_;
  double strength_;
  Eigen::VectorXd x_task_goal_;
  Move3D::Joint* eef_;

  friend std::ostream& operator<<(std::ostream& os, const GoalsetData_t& data) {
    os << "sample_goal_set_ : " << data.sample_goal_set_ << std::endl;
    os << "ratio_projected_ : " << data.ratio_projected_ << std::endl;
    os << "strength_ : " << data.strength_ << std::endl;
    os << "x_task_goal_ : " << data.x_task_goal_.transpose() << std::endl;
    if (data.eef_ != NULL)
      os << "eef_ : " << data.eef_->getName() << std::endl;
    else
      os << "eef_ : NULL" << std::endl;
    return os;
  }
};

//! Main IOC class
class Ioc {
 public:
  Ioc(int num_vars,
      const Move3D::ChompPlanningGroup* planning_group,
      const GoalsetData_t& goalset_data);

  //! Add a trajectory to the set of demonstrated trajectories
  bool addDemonstration(const Eigen::MatrixXd& demo, double discretization);

  //! Add a trajectory to the set of sample trajectories
  //! @param d is the id of the demonstration
  //! @param sample is the trajectory
  bool addSample(int d, const Eigen::MatrixXd& sample);

  //! Set mean
  bool setNominalSampleValue(int d, int i, const Eigen::MatrixXd& sample);

  //! Set costs
  bool setTotalCostsSampleValue(int d, int i, const Eigen::VectorXd& sample);

  //! Reduces the trajectory magnitude
  bool checkJointLimits(const IocTrajectory& traj) const;
  bool jointLimitsQuadProg(IocTrajectory& traj) const;
  bool jointLimits(IocTrajectory& traj) const;
  bool jointLimits(IocIk& q) const;

  //! Generate the sampled trajectories
  //! around one demonstration
  int generateDemoSamples(int demo_id,
                          int nb_samples,
                          bool check_in_collision,
                          context_t context);

  //! Generate the sampled trajectories
  //! around all the demonstrations
  int generateSamples(int nb_samples,
                      bool check_in_collision,
                      context_t context = context_t());

  //! Generates samples around configuration
  int generateIKSamples(int nb_samples,
                        bool check_in_collision,
                        context_t context);

  //! Returns Move3D trajectories
  std::vector<std::vector<Move3D::Trajectory> > getSamples() const;

  //! Returns Move3D trajectories for demo d
  std::vector<Move3D::Trajectory> getDemoSamples(int d) const;

  //! Returns configurations
  std::vector<std::vector<Move3D::confPtr_t> > getSamplesIk() const;

  //! Returns Move3D trajectories
  std::vector<Move3D::Trajectory> getDemonstrations() const;

  //! Drawing function
  Move3D::confPtr_t addTrajectoryToDraw(const IocTrajectory& t, int color);
  Move3D::confPtr_t addAllToDraw(const std::vector<int>& demos_ids,
                                 const std::vector<std::string>& demo_names);

  //! Solve the ioc problem
  Eigen::VectorXd solve(
      const std::vector<Eigen::VectorXd>& phi_demo,
      const std::vector<std::vector<Eigen::VectorXd> >& phi_k);

  // Returns the number of demonstrations
  int getNbOfDemonstrations() { return demonstrations_.size(); }

  //! Rethe the last configuration of demo
  IocIk getLastConfigOfDemo(int d) const;

  //! Delete samples for demo d
  void deleteSampleForDemo(int d);

  //! Reset all samples
  void resetAllSamples();

 private:
  bool isTrajectoryValid(const IocTrajectory& traj, bool relax_collision_check);
  bool isIkValid(const IocIk& ik);
  bool projectConfiguration(IocIk& q, int d);
  bool projectToGolset(IocTrajectory& traj) const;

  //! Set the task goal
  void set_task_goal(const Eigen::VectorXd& x_goal) {
    goalset_data_.x_task_goal_ = x_goal;
  }

  std::vector<IocTrajectory> demonstrations_;

  std::vector<std::vector<IocTrajectory*> > samples_;
  double noise_stddev_;

  std::vector<std::vector<IocIk> > samples_ik_;
  double noise_stddev_ik_;

  //! Planning Group
  const Move3D::ChompPlanningGroup* planning_group_;

  //! goal set stucture
  GoalsetData_t goalset_data_;

  int num_vars_;
  int num_joints_;

  MOVE3D_PTR_NAMESPACE::shared_ptr<IocSampler> sampler_;
};

//! Evaluation Class
class IocEvaluation {
 public:
  IocEvaluation(Move3D::Robot* rob,
                int nb_demos,
                int nb_samples,
                int nb_way_points,
                MultiplePlanners& planners,
                Move3D::StackedFeatures* features,
                std::vector<int> active_joints,
                std::string folder,
                std::string traj_folder,
                std::string tmp_data_folder);

  virtual ~IocEvaluation() {
    stored_features_.clear();
    phi_demos_.clear();
    phi_jac_demos_.clear();
    delete_all_samples();
  }

  //! Sample trajectories around the demonstrations
  virtual std::vector<std::vector<Move3D::Trajectory> > runSampling();

  //! Sample ik around the demonstrations
  virtual std::vector<std::vector<Move3D::confPtr_t> > runIKSampling();

  //! Delete Alls samples
  void delete_all_samples();

  //! Run learning using the C++ library
  virtual void runLearning();

  //! Run Sampling using saved trajectories
  void runFromFileSampling(int offset);

  //! Run Stomp for multiple feature functions
  void runPlannerMultipleFeature(int nb_runs = 1);

  //! Run Stomp on weighted features
  void runPlannerWeightedFeature(int nb_runs = 1);

  //! Generate demonstration using optimal planning
  void generateDemonstrations(int nb_demos);

  //! Load recorded traectories in the move3d format
  bool loadDemonstrations(
      const std::vector<int>& demo_ids = std::vector<int>());

  //! Load trajectories in planner class
  void loadPlannerTrajectories(int nb_trajs = -1,
                               int offset = -1,
                               int random = 0);

  //! Load weight vector from CSV format
  void loadWeightVector(std::string filename = "");

  //! Save 2D traj to file for Matlab
  void saveDemoToMatlab();

  //! Save nb of demos and nb of features
  void saveNbDemoAndNbFeatures();

  //! Compute costs using the original costs and the learned costs
  Eigen::VectorXd compareDemosAndPlanned();

  //! Check start and goal of sampled trajectories
  bool checkStartAndGoal(
      const std::vector<std::vector<Move3D::Trajectory> >& samples) const;

  //! Generate a distribution that maximizes entropy
  void monteCarloSampling(double factor, int nb_tries);

  //! Set planner type for the generation phase
  void setPlannerType(planner_t planner_type) { planner_type_ = planner_type; }

  //! Save demo to file
  void saveDemoToFile(const std::vector<Move3D::Trajectory>& demos,
                      std::vector<Move3D::confPtr_t> context =
                          std::vector<Move3D::confPtr_t>());

  //! Save samples to files
  void saveSamplesToFile(
      const std::vector<std::vector<Move3D::Trajectory> >& samples) const;

  //! Load samples from files
  std::vector<std::vector<Move3D::Trajectory> > loadSamplesFromFile(
      int nb_demos, int nb_samples) const;

  virtual void setLearnedWeights();
  virtual void setOriginalWeights();

  //! Set use of saved context
  void setUseContext(bool use_context) { use_context_ = use_context; }
  void setUseSimulator(bool use_simulator) { use_simulator_ = use_simulator; }
  void setDemoId(int demo_id) { demo_id_ = demo_id; }
  void setOriginalDemoFolder(std::string folder) {
    original_demo_folder_ = folder;
  }
  void setDemoIds(const std::vector<int>& ids) { demo_ids_ = ids; }
  void setDemoNames(const std::vector<std::string>& demo_names) {
    demo_names_ = demo_names;
  }
  void setWeightDim(int dim) { nb_weights_ = dim; }

  void setProcessDirectory(std::string process_dir) {
    process_dir_ = process_dir;
  }
  void setSampleGoalSet(bool with_goal_set) {
    goalset_data_.sample_goal_set_ = with_goal_set;
    if (with_goal_set) {
      initializeGoalSetData();
    }
  }

 protected:
  std::vector<Move3D::FeatureVect> addDemonstrationsIk(HRICS::Ioc& ioc);
  std::vector<Move3D::FeatureVect> addDemonstrations(Ioc& ioc);
  std::vector<std::vector<Move3D::FeatureVect> > addSamples(Ioc& ioc);

  //! Compute the cost of the demos
  Eigen::VectorXd getCostsOfDemonstrations() const;

  //! Saves the configurations of humands and objects in the scene
  void saveContextToFile(const std::vector<Move3D::confPtr_t>& context) const;

  //! Save trajectory to matrix
  void saveTrajToMatlab(const Move3D::Trajectory& t, int id) const;

  //! Save all the feature in a matrix
  //! that can be read by Matlab
  void saveToMatrixFile(
      const std::vector<Move3D::FeatureVect>& demos,
      const std::vector<std::vector<Move3D::FeatureVect> >& samples,
      std::string name);

  //! Save sample trajectories
  void saveTrajectories(const std::vector<Move3D::Trajectory>& trajectories);

  // Return the feature gradien norm sum along each trajectory
  std::vector<std::vector<Move3D::FeatureVect> > getFeatureCount(
      const std::vector<std::vector<Move3D::Trajectory> >& all_trajs);

  // Return the feature gradien norm sum along each trajectory
  std::vector<std::vector<Move3D::FeatureVect> > getFeatureJacobianSum(
      const std::vector<std::vector<Move3D::Trajectory> >& all_trajs);

  //! Plans a motion using the costmap
  Move3D::Trajectory planMotion(planner_t type);

  //! Set all features active
  void activateAllFeatures();

  //! Compute weights when planning multiple times.
  Move3D::WeightVect computeOptimalWeights();

  //! Compute if samples dominate demonstration
  bool checkDegeneration(
      const std::vector<Move3D::FeatureVect>& demos,
      const std::vector<std::vector<Move3D::FeatureVect> >& samples) const;

  //! Returns true if the sample2 is domintated by sample1
  bool isSampleDominated(const Move3D::FeatureVect& sample1,
                         const Move3D::FeatureVect& sample2) const;

  //! Removes the dominated samples by resampling
  void removeDominatedSamplesAndResample(
      Ioc& ioc, std::vector<std::vector<Move3D::FeatureVect> >& phi_k);

  //! Returns trajectory that best fits
  Move3D::Trajectory selectBestSample(
      double detla_mean, const std::vector<Move3D::Trajectory>& trajs);

  //! Returns true if the trajectory is valid
  bool isTrajectoryValid(Move3D::Trajectory& path);

  //! Set the context for that trajectory
  void setContext(int d);

  //! Set the buffer for that trajectory
  void setBuffer(int d);

  //! Initialize goal set data
  void initializeGoalSetData();

  Move3D::Robot* robot_;
  int nb_demos_;
  int nb_samples_;
  int nb_weights_;
  int nb_way_points_;

  std::vector<int> demo_ids_;
  std::vector<std::string> demo_names_;
  std::vector<Move3D::Trajectory> demos_;
  std::vector<Move3D::Trajectory> samples_;
  std::vector<Move3D::Trajectory> learned_;

  // Weight vectors
  Move3D::WeightVect learned_vect_;
  Move3D::WeightVect original_vect_;

  std::string process_dir_;

  std::vector<int> active_joints_;

  std::string feature_type_;

  // Data for goaset projection
  GoalsetData_t goalset_data_;

  Move3D::StackedFeatures* feature_fct_;
  Move3D::TrajectorySmoothness* smoothness_fct_;
  Move3D::ChompPlanningGroup* planning_group_;

  bool load_sample_from_file_;
  bool remove_samples_in_collision_;
  MultiplePlanners& planners_;
  int round_id_;

  bool use_simulator_;
  int demo_id_;

  std::vector<Move3D::FeatureVect> stored_features_;
  std::vector<Move3D::FeatureVect> phi_demos_;
  std::vector<Move3D::FeatureVect> phi_jac_demos_;

  // Folders
  std::string folder_;
  std::string traj_folder_;
  std::string tmp_data_folder_;
  std::string original_demo_folder_;

  // Planner type
  planner_t planner_type_;
  int nb_planning_test_;

  // Context
  bool use_context_;
  context_t context_;
};
}

#endif  // HRICS_IOC_HPP

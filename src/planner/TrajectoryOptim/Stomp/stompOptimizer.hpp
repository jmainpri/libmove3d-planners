/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef STOMP_OPTIMIZER_H_
#define STOMP_OPTIMIZER_H_

#include <vector>

#include "stompParameters.hpp"

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "collision_space/collision_space.hpp"

#include <boost/shared_ptr.hpp>

#include "task.hpp"
#include "covariant_trajectory_policy.hpp"
#include "policy_improvement_loop.hpp"
#include "stompStatistics.hpp"
#include "cost_computation.hpp"

namespace Move3D { class ConfGenerator; }

//#include <motion_planning_msgs/Constraints.h>
//#include "constraint_evaluator.hpp"
//#include <stomp_motion_planner/STOMPStatistics.h>
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

//#include <kdl/frames.hpp>
//#include <kdl/chainidsolver_recursive_newton_euler.hpp>

namespace stomp_motion_planner
{

class StompOptimizer: public Task
{
public:
    StompOptimizer( Move3D::ChompTrajectory *trajectory,
                   const StompParameters *parameters,
                   const Move3D::ChompPlanningGroup *planning_group,
                   const Move3D::CollisionSpace *collision_space, int id=0);

    //  StompOptimizer(StompTrajectory *trajectory, const StompRobotModel *robot_model,
    //      const StompRobotModel::StompPlanningGroup *planning_group, const StompParameters *parameters,
    //      const ros::Publisher& vis_marker_array_publisher,
    //      const ros::Publisher& vis_marker_publisher,
    //      const ros::Publisher& stats_publisher,
    //      StompCollisionSpace *collision_space,
    //      const motion_planning_msgs::Constraints& constraints);

    virtual ~StompOptimizer();

    /**
   * Main optimizer function
   * @param nbIteration max number of iteration
   * @param idRun
   * @return
   */
    void runDeformation( int nbIteration , int idRun=0 );

    /**
   * Set compute handover position
   */
    void setUseOtp(bool use_otp) { use_handover_config_generator_= use_otp; }

    /**
   * Set the use of iteration limit
   * @param Time in second
   */
    void setUseIterationLimit(bool use_limit) { use_iteration_limit_ = use_limit; }

    /**
   * Set the use of a time limit
   * @param Time in second
   */
    void setUseTimeLimit(bool use_limit) { use_time_limit_ = use_limit; }

    /**
      * set the robots for parallel
      */
    void setRobotPool( const std::vector<Move3D::Robot*>& robots );

    /**
      * get the cost conputer
      */
    const std::vector<costComputation*>& getCostComputers();


    /**
   * Set the passive Dofs
   */
    void setPassiveDofs( std::vector<Move3D::confPtr_t> passive_dofs ) { passive_dofs_ = passive_dofs; }

    /**
   * Generate the noisy trajectories
   * @param num_time_steps
   */
    void generateNoisyTrajectory(const Move3D::Trajectory& traj, std::vector< std::vector<Move3D::confPtr_t> >& noisy_trajectory);

    /**
   * Functions to set the shared pointer
   */
    void setSharedPtr(MOVE3D_BOOST_PTR_NAMESPACE<StompOptimizer>& ptr);
    void resetSharedPtr();

    /**
   * Test the noisy trajectory sampler
   */
    void testMultiVariateGaussianSampler();

    /**
   * Get the current configuration collision cost
   */
    double getCollisionSpaceCost( Move3D::Configuration& q );

    /**
   * Get the current trajectory cost profile
   */
    void getTrajectoryCost( std::vector<double>& cost, double step );

    /**
   * Get the current joint violation
   */
    int getJointLimitViolations() { return joint_limits_violation_; }
    bool getJointLimitViolationSuccess() { return succeded_joint_limits_; }

    // stuff derived from Task:
    /**
   * Initializes the task for a given number of time steps
   * @param num_time_steps
   * @return
   */
    bool initialize(/*ros::NodeHandle& node_handle,*/ int num_time_steps);

    /**
   * Initializes from a new trajectory
   */
    bool initializeFromNewTrajectory(const Move3D::Trajectory& traj);

    /**
   * Initializes the handover generator
   */
    void initHandover();

    /**
   * Executes the task for the given policy parameters, and returns the costs per timestep
   * @param parameters [num_dimensions] num_parameters - policy parameters to execute
   * @param costs Vector of num_time_steps, state space cost per timestep (do not include control costs)
   * @return
   */
    bool execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number, bool joint_limits, bool resample );

    /**
   * Get the Policy object of this Task
   * @param policy
   * @return
   */
    bool getPolicy(MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy>& policy);

    /**
   * Sets the Policy object of this Task
   * @param policy
   * @return
   */
    bool setPolicy(const MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy> policy);

    /**
   * Gets the weight of the control cost
   * @param control_cost_weight
   * @return
   */
    bool getControlCostWeight(double& control_cost_weight);


    /**
   * Gets the weight of the state cost
   * @param control_cost_weight
   * @return
   */
    bool getStateCostWeight(double& state_cost_weight);

    /**
   * Returns the planning group of the optimizer
   */
    const Move3D::ChompPlanningGroup* getPlanningGroup() const { return planning_group_; }

    /**
   * Get the robot
   */
    Move3D::Robot* getRobot() { return robot_model_; }

    /**
   * Draw function to be called outside
   */
    void draw();

    /**
   * Draw real collision points
   */
    void drawCollisionPoints();

    /**
   * Retreive source and target
   */
    void setSource( Move3D::confPtr_t q ) { source_ = q; }
    Move3D::confPtr_t getSource();
    Move3D::confPtr_t getTarget();

    /**
   * Retreive best trajectory
   */
    Move3D::Trajectory getBestTraj() const { return best_traj_; }

    /**
      * Get Group traj
      */
    void setGroupTrajectoryToMove3DTraj(Move3D::Trajectory& traj);

    /**
      * Returns cost profiles
      */
    void getCostProfiles( std::vector<double>& smoothness_cost, std::vector<double>& collision_cost, std::vector<double>& general_cost );

    /**
      * Set the use of costspace even when the collision
      * space is created
      */
    void setUseCostSpace( bool use_costspace ) { use_costspace_ = use_costspace; }

    /**
      * Set the path color when drawn in parallel
      */
    void setTrajColor( std::vector<double> traj_color ) { traj_color_ = traj_color; }

    /**
      * Get the vector of all move3d trajs
      */
    const std::vector<Move3D::Trajectory>& getAllTrajs() const { return all_move3d_traj_; }

private:

    int id_;
    int collision_space_id_;

    Move3D::Robot* robot_model_;
    Move3D::Robot* human_model_;

    double time_;

    std::vector<Move3D::Trajectory> all_move3d_traj_;
    Move3D::Trajectory move3d_traj_;
    Move3D::Trajectory best_traj_;

    int num_joints_;
    int num_vars_free_;
    int num_vars_all_;
    int num_collision_points_;
    int free_vars_start_;
    int free_vars_end_;
    int iteration_;
    int collision_free_iteration_;

    bool succeded_joint_limits_;
    int joint_limits_violation_;

    bool use_costspace_;

    bool recompute_handover_once_;
    bool handover_has_been_recomputed_;
    bool use_human_sliders_;
    bool use_handover_config_generator_;
    bool use_handover_config_list_;
    bool use_handover_auto_;
    bool recompute_handover_cell_list_;
    Move3D::ConfGenerator* handoverGenerator_;

    double last_move3d_cost_;

    bool use_time_limit_;
    double time_limit_;
    bool use_iteration_limit_;
    bool use_collision_free_limit_limit_;

    std::vector<Move3D::confPtr_t> passive_dofs_;

    std::vector<costComputation*> compute_fk_;
    std::vector<double> traj_color_;

    Move3D::ChompTrajectory *full_trajectory_;
    const Move3D::ChompPlanningGroup *planning_group_;
    const StompParameters *stomp_parameters_;
    const Move3D::CollisionSpace *move3d_collision_space_;
    bool use_external_collision_space_;
    Move3D::ChompTrajectory group_trajectory_;
    std::vector<Move3D::ChompCost> joint_costs_;

    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::CovariantTrajectoryPolicy> policy_;
    std::vector<Eigen::VectorXd> policy_parameters_;
    MOVE3D_BOOST_PTR_NAMESPACE<StompOptimizer> this_shared_ptr_;

    std::vector<int> planner_p3d_joints_;
    std::vector<int> group_joint_to_move3d_joint_index_;

    //  std::vector<std::vector<KDL::Vector> > joint_axis_;
    //  std::vector<std::vector<KDL::Vector> > joint_pos_;
    //  std::vector<std::vector<KDL::Frame> > segment_frames_;
    //  std::vector<std::vector<KDL::Vector> > collision_point_pos_;
    //  std::vector<std::vector<KDL::Vector> > collision_point_vel_;
    //  std::vector<std::vector<KDL::Vector> > collision_point_acc_;

    std::vector<std::vector<Eigen::Transform3d> > segment_frames_;
    std::vector<std::vector<Eigen::Vector3d> >  joint_axis_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  joint_pos_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  collision_point_pos_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  collision_point_vel_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  collision_point_acc_eigen_;

    Eigen::VectorXd general_cost_potential_;
    Eigen::VectorXd dt_;

    Eigen::MatrixXd collision_point_potential_;
    Eigen::MatrixXd collision_point_vel_mag_;
    std::vector<std::vector<Eigen::Vector3d> > collision_point_potential_gradient_;
    Eigen::MatrixXd group_trajectory_backup_;
    Eigen::MatrixXd best_group_trajectory_;
    Eigen::MatrixXd best_group_trajectory_in_collision_;
    double best_group_trajectory_cost_;
    double best_group_trajectory_in_collsion_cost_;
    double last_trajectory_cost_;
    bool last_trajectory_collision_free_;
    bool last_trajectory_constraints_satisfied_;
    int last_improvement_iteration_;
    int best_iteration_;

    // HMC stuff:
    Eigen::MatrixXd momentum_;
    Eigen::MatrixXd random_momentum_;
    Eigen::VectorXd random_joint_momentum_; //temporary variable
    std::vector<MultivariateGaussian> multivariate_gaussian_;
    double stochasticity_factor_;

    std::vector<int> state_is_in_collision_;      /**< Array containing a boolean about collision info for each point in the trajectory */
    std::vector<std::vector<int> > point_is_in_collision_;
    bool is_collision_free_;
    double worst_collision_cost_state_;

    Eigen::MatrixXd smoothness_increments_;
    Eigen::MatrixXd collision_increments_;
    Eigen::MatrixXd final_increments_;

    // temporary variables for all functions:
    Eigen::VectorXd smoothness_derivative_;
    //  KDL::JntArray kdl_joint_array_;
    //  KDL::JntArray kdl_vel_joint_array_;
    //  KDL::JntArray kdl_acc_joint_array_;
    //
    //  KDL::JntArray kdl_group_joint_array_;
    //  KDL::JntArray kdl_group_vel_joint_array_;
    //  KDL::JntArray kdl_group_acc_joint_array_;
    //  KDL::JntArray kdl_group_torque_joint_array_;

    //Eigen::VectorXd
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_pseudo_inverse_;
    Eigen::MatrixXd jacobian_jacobian_tranpose_;
    Eigen::VectorXd random_state_;
    Eigen::VectorXd joint_state_velocities_;
    Eigen::VectorXd joint_state_accelerations_;
    Eigen::VectorXd full_joint_state_velocities_;
    Eigen::VectorXd full_joint_state_accelerations_;

    bool human_has_moved_;
    std::vector<double> last_human_pos_;
    bool reset_reused_rollouts_;

    Move3D::confPtr_t source_;
    Move3D::confPtr_t target_;
    Move3D::confPtr_t target_new_;

    //  ros::Publisher vis_marker_array_pub_;
    //  ros::Publisher vis_marker_pub_;
    //  ros::Publisher stats_pub_;
    int animate_endeffector_segment_number_;

    //  motion_planning_msgs::Constraints constraints_;
    //  std::vector<MOVE3D_BOOST_PTR_NAMESPACE<ConstraintEvaluator> > constraint_evaluators_;
    MOVE3D_BOOST_PTR_NAMESPACE<StompStatistics>  stomp_statistics_;

    void initialize();
    void initPolicy();
    void calculateSmoothnessIncrements();
    void calculateCollisionIncrements();
    void calculateTotalIncrements();
    void addIncrementsToTrajectory();
    void updateFullTrajectory();
    void debugCost();
    void eigenMapTest();
    bool handleJointLimits();
    void animatePath();
    void animateEndeffector(bool print_cost = false);
    void animateTrajectoryPolicy();
    void visualizeState(int index);
    void saveEndeffectorTraj();

    double getTrajectoryCost();
    double getSmoothnessCost();
    double getCollisionCost();
    double getGeneralCost();
    void perturbTrajectory();
    void getRandomMomentum();
    void updateMomentum();
    void updatePositionFromMomentum();
    void calculatePseudoInverse();

    void getFrames( int segment, const Eigen::VectorXd& joint_array, Move3D::Configuration& q );

    bool getCollisionPointObstacleCost( int segment, int coll_point, double& collion_point_potential, Eigen::Vector3d& pos );
    bool getConfigObstacleCost( Move3D::Robot* robot, int i, Eigen::MatrixXd& collision_point_potential, std::vector< std::vector<Eigen::Vector3d> >& collision_point_pos );

    bool performForwardKinematics();
    void doChompOptimization();

    void copyPolicyToGroupTrajectory();
    void copyGroupTrajectoryToPolicy();
    void clearAnimations();

    //----------------------------------------------------------------------------
    // Jim functions
    //----------------------------------------------------------------------------
    void setGroupTrajectoryFromVectorConfig(const std::vector<Move3D::confPtr_t>& traj);
    void setGroupTrajectoryToVectorConfig(std::vector<Move3D::confPtr_t>& traj);

    bool replaceEndWithNewConfiguration();
    bool getManipulationHandOver();
    bool getMobileManipHandOver();
    bool humanHasMoved();
    bool getNewTargetFromHandOver();
    double resampleParameters(std::vector<Eigen::VectorXd>& parameters);
    double computeMove3DCost();
    Move3D::confPtr_t getConfigurationOnGroupTraj(int ith);

    void saveTrajectoryCostStats();
    void saveCostFromConvergenceTraj();
    void saveOptimToFile(std::string fileName);

    int getNumberOfCollisionPoints(Move3D::Robot* R);



    //  void getTorques(int index, std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches);
};

}

extern std::map< Move3D::Robot*, std::vector<Eigen::Vector3d> > global_MultiStomplinesToDraw;
extern std::map< Move3D::Robot*, std::vector<double> >          global_MultiStomplinesColors;
extern MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer> global_optimizer;

void move3d_set_api_functions_collision_space( bool use_move3d_fct );
bool move3d_use_api_functions_collision_space();
void move3d_set_fct_get_nb_collision_points( boost::function<int(Move3D::Robot*)> fct, int id=0 );
void move3d_set_fct_get_config_collision_cost( boost::function<bool( Move3D::Robot* robot, int i, Eigen::MatrixXd&, std::vector< std::vector<Eigen::Vector3d> >& )> fct, int id=0 ) ;

#endif /* STOMP_OPTIMIZER_H_ */

/*
 *  chompOptimizer.hpp
 *  Move3D-motionPlanner-libs
 *
 *  Created by Jim Mainprice on 06/06/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef CHOMPOPTIMIZER
#define CHOMPOPTIMIZER

#include "chompMultivariateGaussian.hpp"
#include "chompParameters.hpp"
#include "chompCost.hpp"
#include "chompTrajectory.hpp"
#include "chompPlanningGroup.hpp"

#include "planner/Greedy/CollisionSpace.hpp"

#include <Eigen/Core>

#include <vector>
//namespace API
//{
  class ChompOptimizer 
  {
  public:
    //  ChompOptimizer(ChompTrajectory *trajectory, const ChompRobotModel *robot_model,
    //                 const ChompRobotModel::ChompPlanningGroup *planning_group, const ChompParameters *parameters,
    //                 const ros::Publisher& vis_marker_array_publisher,
    //                 const ros::Publisher& vis_marker_publisher,
    //                 ChompCollisionSpace *collision_space);
    ChompOptimizer(ChompTrajectory *trajectory, 
                   const ChompParameters *parameters, 
                   const ChompPlanningGroup *planning_group,
                   const CollisionSpace *collision_space);
      
    virtual ~ChompOptimizer();
    
    void runDeformation( int nbIteration , int idRun=0 );
    
  private:
    Robot* robot_model_;
    
    int num_joints_;
    int num_vars_free_;
    int num_vars_all_;
    int num_collision_points_;
    int free_vars_start_;
    int free_vars_end_;
    int iteration_;
    int collision_free_iteration_;
    
    ChompTrajectory *full_trajectory_;
    ChompTrajectory group_trajectory_;
    //  const ChompRobotModel *robot_model_;
    const ChompPlanningGroup *planning_group_;
    const ChompParameters *parameters_;
    const CollisionSpace *collision_space_;
    std::vector<ChompCost> joint_costs_;
    
    std::vector<int> planner_p3d_joints_;
    std::vector<int> group_joint_to_move3d_joint_index_;
    
    //  std::vector<std::vector<KDL::Vector> > joint_axis_;
    //  std::vector<std::vector<KDL::Vector> > joint_pos_;
    //  std::vector<std::vector<KDL::Frame> > segment_frames_;
    //  std::vector<std::vector<KDL::Vector> > collision_point_pos_;
    //  std::vector<std::vector<KDL::Vector> > collision_point_vel_;
    //  std::vector<std::vector<KDL::Vector> > collision_point_acc_;
    
    // Frame are stored in a 12 floating point vector
    std::vector<std::vector<std::vector<double> > > segment_frames_;
    std::vector<std::vector<Eigen::Vector3d> >  joint_axis_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  joint_pos_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  collision_point_pos_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  collision_point_vel_eigen_;
    std::vector<std::vector<Eigen::Vector3d> >  collision_point_acc_eigen_;
    
    Eigen::MatrixXd collision_point_potential_;
    Eigen::MatrixXd collision_point_vel_mag_;
    std::vector<std::vector<Eigen::Vector3d> > collision_point_potential_gradient_;
    
    Eigen::MatrixXd group_trajectory_backup_;
    Eigen::MatrixXd best_group_trajectory_;
    double best_group_trajectory_cost_;
    int last_improvement_iteration_;
    
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
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_pseudo_inverse_;
    Eigen::MatrixXd jacobian_jacobian_tranpose_;
    Eigen::VectorXd random_state_;
    Eigen::VectorXd joint_state_velocities_;
    
    //  ros::Publisher vis_marker_array_pub_;
    //  ros::Publisher vis_marker_pub_;
    int animate_endeffector_segment_number_;
    
    void initialize();
    void createEigenMapsFromP3dRobot();
    void calculateSmoothnessIncrements();
    void calculateCollisionIncrements();
    void calculateTotalIncrements();
    void getFrames(int i, const Eigen::VectorXd& joint_array);
    void performForwardKinematics();
    void addIncrementsToTrajectory();
    void updateFullTrajectory();
    void debugCost();
    void eigenMapTest();
    void handleJointLimits();
    //void animatePath();
    void animateEndeffector();
    //void visualizeState(int index);
    double getTrajectoryCost();
    double getSmoothnessCost();
    double getCollisionCost();
    void perturbTrajectory();
    void getRandomMomentum();
    void updateMomentum();
    void updatePositionFromMomentum();
    void calculatePseudoInverse();
  };
//}
#endif
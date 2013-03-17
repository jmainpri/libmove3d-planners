#ifndef COST_COMPUTATION_HPP
#define COST_COMPUTATION_HPP

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompTrajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompCost.hpp"

#include "collision_space/CollisionSpace.hpp"

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <vector>

class costComputation
{
public:
    costComputation(Robot* robot,
                    const CollisionSpace *collision_space,
                    const ChompPlanningGroup* planning_group,
                    std::vector< ChompCost > joint_costs,
                    ChompTrajectory group_trajectory,
                    double obstacle_weight,
                    bool use_costspace );

    ~costComputation();

    bool getCost(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, int iteration=1 );

    bool getJointLimitViolationSuccess() const { return succeded_joint_limits_; }

private:
    Robot* robot_model_;
    const CollisionSpace *collision_space_;
    const ChompPlanningGroup* planning_group_;

    bool handleJointLimits( ChompTrajectory& group_traj );
    bool performForwardKinematics( const ChompTrajectory& group_traj );
    void getFrames(int segment, const Eigen::VectorXd& joint_array, Configuration& q);
    bool getConfigObstacleCost(int segment, int dof, Configuration& q);

    ChompTrajectory group_trajectory_;

    bool is_collision_free_;
    bool succeded_joint_limits_;

    bool use_costspace_;
    std::vector<int> state_is_in_collision_;
    std::vector< std::vector<std::vector<double> > > segment_frames_;
    std::vector< std::vector<Eigen::Vector3d> >  joint_axis_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  joint_pos_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_pos_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_vel_eigen_;
    std::vector< std::vector<Eigen::Vector3d> >  collision_point_acc_eigen_;
    Eigen::MatrixXd collision_point_potential_;
    Eigen::MatrixXd collision_point_vel_mag_;
    Eigen::VectorXd general_cost_potential_;
    std::vector< ChompCost > joint_costs_;
    std::vector< std::vector<Eigen::Vector3d> > collision_point_potential_gradient_;
    int free_vars_start_;
    int free_vars_end_;
    int num_vars_all_;
    int num_vars_free_;
    int iteration_;
    int num_collision_points_;
    int num_joints_;

    double hack_tweek_;
    double obstacle_weight_;
};

#endif // COST_COMPUTATION_HPP

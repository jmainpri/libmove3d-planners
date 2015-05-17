#ifndef TRAJ_OPT_LOOP_HPP
#define TRAJ_OPT_LOOP_HPP

#include <vector>

#include "lampTrajectory.hpp"

#include "planner/TrajectoryOptim/Stomp/stompParameters.hpp"
#include "planner/TrajectoryOptim/Stomp/stompStatistics.hpp"
#include "planner/TrajectoryOptim/Stomp/covariant_trajectory_policy.hpp"

#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "collision_space/collision_space.hpp"

#include <boost/shared_ptr.hpp>


class LampOptimizer
{
public:
    LampOptimizer();
    void runDeformation( int nbIteration, int idRun );

    virtual bool initialize( bool single_rollout, double discretization ) { return true; }
    virtual void run_single_iteration() { }
    virtual void animateEndeffector() { }
    virtual void end() {}

protected:

    Move3D::Robot*          robot_model_;
    Move3D::LampTrajectory  group_trajectory_;

    MOVE3D_BOOST_PTR_NAMESPACE<StompStatistics> stomp_statistics_;
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompParameters> stomp_parameters_;

    int                     iteration_;

private:

    void copyPolicyToGroupTrajectory() { }
    void handleJointLimits() { }
    void updateFullTrajectory() { }
    void performForwardKinematics(bool) { }
    virtual double getSmoothnessCost() { return 0.0; }
    virtual double getGeneralCost() { return 0.0; }
    virtual double getTrajectoryCost() { return 0.0; }
    virtual double getCollisionCost() { return 0.0; }

    Move3D::Trajectory      move3d_traj_;
    Move3D::Trajectory      last_traj_;
    Move3D::Trajectory      best_traj_;

    double                  time_;
    double                  best_group_trajectory_cost_;
    double                  best_group_trajectory_in_collsion_cost_;
    bool                    use_time_limit_;
    int                     best_iteration_;
    int                     last_improvement_iteration_;
    bool                    reset_reused_rollouts_;
    bool                    last_trajectory_collision_free_;
    int                     collision_free_iteration_;
    bool                    last_trajectory_constraints_satisfied_;
    bool                    is_collision_free_;
    bool                    use_iteration_limit_;
    bool                    use_collision_free_limit_limit_;


    Move3D::LampTrajectory  best_group_trajectory_;
    Move3D::LampTrajectory  best_group_trajectory_in_collision_;

};

#endif

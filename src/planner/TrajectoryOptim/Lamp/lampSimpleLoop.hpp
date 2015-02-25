#include "lampOptimizer.hpp"
#include "lampTrajectory.hpp"
#include "lampComputeCost.hpp"

namespace Move3D
{

class LampSimpleLoop : public LampOptimizer
{
public:
    LampSimpleLoop( Move3D::Robot* robot );
    bool initialize( bool single_rollout, double discretization );
    void run_single_iteration();
    void animateEndeffector();

    double getTrajectoryCost();
    double getSmoothnessCost();
    double getCollisionCost();

private:

    double getTrajectoryCost( Move3D::LampTrajectory& traj, bool check_joint_limits = false );
    double getCollisionCost( Move3D::LampTrajectory& traj , bool check_joint_limits = false );
    double getSmoothnessCost(const Move3D::LampTrajectory& traj);
    void normalize_update( const Eigen::MatrixXd& deltas, const std::vector<bool>& valid, Eigen::VectorXd& update ) const;

    Move3D::LampSampler sampler_;
    Move3D::Trajectory initial_traj_seed_;
    Move3D::ChompPlanningGroup* planning_group_;
    Move3D::LampCostComputation cost_computer_;

    const stomp_motion_planner::StompParameters *stomp_parameters_const_;
    const Move3D::CollisionSpace *move3d_collision_space_;
};

};

void lamp_simple_loop();

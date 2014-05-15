#ifndef COLLISIONSPACE_FACTORY_HPP
#define COLLISIONSPACE_FACTORY_HPP

#include "API/Device/robot.hpp"

#include "collision_point.hpp"
#include "collision_space.hpp"

namespace traj_optim
{

enum ScenarioType {
    Default,
    CostMap,
    Simple,
    Shelf,
    Navigation,
    HumanAwareNav,
    HumanAwareManip,
    HumanAwareMobileManip,
    HumanSimulation,
    LegiblePlane
};

}

bool traj_optim_init_collision_spaces( traj_optim::ScenarioType sce, Move3D::Robot* rob );
void traj_optim_add_human_to_collision_space(bool add);

const Move3D::CollisionSpace* traj_optim_get_collision_space();
std::vector<Move3D::CollisionPoint> traj_optim_get_collision_points();

std::vector<int> traj_optim_get_active_joints();
std::vector<int> traj_optim_get_planner_joints();

#endif // COLLISIONSPACE_FACTORY_HPP

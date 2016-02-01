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
#ifndef COLLISIONSPACE_FACTORY_HPP
#define COLLISIONSPACE_FACTORY_HPP

#include "API/Device/robot.hpp"

#include "collision_point.hpp"
#include "collision_space.hpp"

namespace traj_optim {

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

bool traj_optim_init_collision_spaces(traj_optim::ScenarioType sce,
                                      Move3D::Robot* rob);
void traj_optim_reset_collision_space();
void traj_optim_add_human_to_collision_space(bool add);

bool traj_optim_hrics_human_trajectory_biomech_dof_bounds(Move3D::Robot* robot);

const Move3D::CollisionSpace* traj_optim_get_collision_space();
std::vector<Move3D::CollisionPoint> traj_optim_get_collision_points();

std::vector<int> traj_optim_get_active_joints();
std::vector<int> traj_optim_get_planner_joints();

#endif  // COLLISIONSPACE_FACTORY_HPP

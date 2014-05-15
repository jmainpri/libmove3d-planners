#ifndef MULTILOCALPATH_UTILS_HPP
#define MULTILOCALPATH_UTILS_HPP

#include "API/Device/robot.hpp"

bool traj_optim_switch_cartesian_mode(bool cartesian);

void traj_optim_hrics_mobile_manip_localpath_and_cntrts();
void traj_optim_navigation_set_localpath_and_cntrts();
void traj_optim_shelf_set_localpath_and_cntrts();
void traj_optim_hrics_set_localpath_and_cntrts();

bool traj_optim_set_MultiLP();
bool traj_optim_invalidate_cntrts();

bool traj_optim_init_mlp_cntrts_and_fix_joints(Move3D::Robot* rob);

#endif // MULTILOCALPATH_UTILS_HPP

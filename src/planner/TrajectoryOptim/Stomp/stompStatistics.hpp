//
//  stompStatistics.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 10/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef STOMP_STATISTICS_HPP_
#define STOMP_STATISTICS_HPP_

#include "API/Trajectory/trajectory.hpp"

#include <vector>

class StompStatistics 
{  
public:
  int run_id;
  int collision_success_iteration;
  double collision_success_duration;
  int success_iteration;
  bool success;
  double success_time;
  std::vector<double> costs;
  std::vector< std::pair<double, std::vector<confPtr_t> > > convergence_trajs;
  std::vector< std::pair<double, TrajectoryStatistics> > convergence_rate;
  double best_cost;
  double success_duration;
};

#endif
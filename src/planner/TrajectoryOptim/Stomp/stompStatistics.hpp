//
//  stompStatistics.hpp
//  libmove3d-motion
//
//  Created by Jim Mainprice on 10/07/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef STOMP_STATISTICS_HPP_
#define STOMP_STATISTICS_HPP_

#include <vector>

class StompStatistics 
{  
public:
  int collision_success_iteration;
  double collision_success_duration;
  int success_iteration;
  bool success;
  std::vector<double> costs;
  double best_cost;
  double success_duration;
};

#endif
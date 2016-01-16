/*
Copyright (c) 2014, Calder Phillips-Grafflin (calder.pg@gmail.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Modified by Jim Mainprice

#ifndef HRICS_DYNAMIC_TIME_WARPING_HPP
#define HRICS_DYNAMIC_TIME_WARPING_HPP

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <boost/function.hpp>

#include "API/Trajectory/trajectory.hpp"
#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"

namespace DTW {

class SimpleDTW {
 public:
  SimpleDTW(size_t x_dim,
            size_t y_dim,
            const Move3D::ChompPlanningGroup* planning_group = NULL);
  SimpleDTW(boost::function<double(const std::vector<double>&,
                                   const std::vector<double>&)> f);
  SimpleDTW();

  ~SimpleDTW() {}

  void setDistFunction(boost::function<double(const std::vector<double>&,
                                              const std::vector<double>&)> f) {
    distance_fn_ = f;
  }
  double EvaluateWarpingCost(
      const std::vector<std::vector<double> >& sequence_1,
      const std::vector<std::vector<double> >& sequence_2);

  double euclidean_distance(const std::vector<double>&,
                            const std::vector<double>&);
  double tansform_distance(const std::vector<double>&,
                           const std::vector<double>&);
  double centers_distance(const std::vector<double>& P1,
                          const std::vector<double>& P2);

 private:
  void Initialize(size_t x_size, size_t y_size);

  inline size_t GetDataIndex(size_t x, size_t y) { return (x * y_dim_) + y; }

  inline double GetFromDTWMatrix(size_t x, size_t y) {
    return data_[GetDataIndex(x, y)];
  }

  inline void SetInDTWMatrix(size_t x, size_t y, double val) {
    data_[GetDataIndex(x, y)] = val;
  }

  boost::function<double(const std::vector<double>&,
                         const std::vector<double>&)> distance_fn_;
  std::vector<double> data_;
  size_t x_dim_;
  size_t y_dim_;
  bool initialized_;

  const Move3D::ChompPlanningGroup* planning_group_;
};
}

int dtw_compare_performance(int traj_length, int iterations);

std::vector<double> dtw_compare_performance(
    const Move3D::ChompPlanningGroup* planning_group,
    const Move3D::Trajectory& t1,
    const std::vector<Move3D::Trajectory>& t_all,
    std::vector<Move3D::Joint*> joint = std::vector<Move3D::Joint*>());

#endif  // HRICS_DYNAMIC_TIME_WARPING_HPP

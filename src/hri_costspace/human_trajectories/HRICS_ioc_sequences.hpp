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
#ifndef HRICS_IOC_SEQUENCES_HPP
#define HRICS_IOC_SEQUENCES_HPP

#include "feature_space/features.hpp"
#include "HRICS_human_ioc.hpp"
#include "utils/NumsAndStrings.hpp"

#include <iostream>

namespace HRICS
{

class IocSequences
{
 public:
  IocSequences();
  ~IocSequences() {}

  bool run();
  void set_features();

 private:
  void setSamplingFeatures();
  void setSamplingFeaturesIk();
  void setGenerationFeatures();
  void setCompareFeatures();
  void GenerateResults();

  // IOC PHASES
  enum phase_t {
    generate = 0,
    sample = 1,
    compare = 2,
    run_planner = 3,
    default_phase = 4,
    monte_carlo = 5,
    simulation = 6,
    save_feature_and_demo_size = 7,
    generate_results = 8
  };

  // TYPE OF FEATURES
  enum feature_t { no_features, spheres, human_trajs };

  // SAMPLE IK
  enum sample_ik_t { no_ik = 0, only_ik = 1, ik_and_traj = 2 };

  phase_t phase_;
  feature_t features_type_;
  sample_ik_t sample_ik_;
  double regularizer_;

  std::vector<int> active_joints_;
  Move3D::StackedFeatures* feature_fct_;
  bool use_human_simulation_demo_;
  HRICS::IocEvaluation* eval_;

  // Trajectories results
  std::vector<Move3D::Trajectory> demos_;
};

// On object per type of algorithm
// baseline0, baseline1, baseline2
struct ioc_statistics_t {
  ioc_statistics_t(int nb_samples, int nb_demos)
  {
    sum = Eigen::VectorXd::Zero(nb_samples);
    sum_of_sqares = Eigen::VectorXd::Zero(nb_samples);
    max = std::numeric_limits<double>::min();
    min = std::numeric_limits<double>::max();
    stddev = 0.;
    avg = 0.;
    nb_samples = nb_samples;
    values.clear();
    in_collision.clear();
  }

  void setMin(double val)
  {
    if (val < min) min = val;
  }

  void setMax(double val)
  {
    if (val > max) max = val;
  }

  void ToFile(std::string filename)
  {
    if (values.empty()) {
      return;
    }
    int rows = values.size();  // nb of demos
    int cols = 0;              // nb of stomp runs
    for (size_t i = 0; i < values.size(); i++) {
      if (int(values[i].size()) > cols) {
        cols = values[i].size();
      }
    }
    Eigen::MatrixXd mat_tmp = Eigen::MatrixXd::Zero(rows, cols);
    for (size_t i = 0; i < values.size(); i++) {
      for (int j = 0; j < values[i].size(); j++) {
        mat_tmp(i, j) = values[i][j];
      }
    }
    move3d_save_matrix_to_csv_file(mat_tmp, filename + "_.csv");

    rows = in_collision.size();  // nb of demos
    cols = 0;                    // nb of stomp runs
    for (size_t i = 0; i < in_collision.size(); i++) {
      if (int(in_collision[i].size()) > cols) {
        cols = in_collision[i].size();
      }
    }
    mat_tmp = Eigen::MatrixXd::Zero(rows, cols);
    for (size_t i = 0; i < in_collision.size(); i++) {
      for (size_t j = 0; j < in_collision[i].size(); j++) {
        mat_tmp(i, j) = in_collision[i][j];
      }
    }
    move3d_save_matrix_to_csv_file(mat_tmp, filename + "_in_collision.csv");
  }

  // v are the dtw values found for this demo
  void dtw_update_stat(const Eigen::VectorXd& v,
                       const std::vector<bool>& runs_in_collision)
  {
    for (int i = 0; i < v.size(); i++) {  // for each sample
      sum[i] += v[i];
    }
    for (int i = 0; i < v.size(); i++) {  // for each sample
      sum_of_sqares[i] += (v[i] * v[i]);
    }

    setMin(v.minCoeff());
    setMax(v.maxCoeff());

    values.push_back(v);                        // one vector for each demo
    in_collision.push_back(runs_in_collision);  // one vector for each demo
  }

  Eigen::VectorXd sum;
  Eigen::VectorXd sum_of_sqares;
  double min;
  double max;
  double stddev;
  double avg;
  int nb_samples;
  std::vector<Eigen::VectorXd> values;
  std::vector<std::vector<bool> > in_collision;
};
}

#endif  // HRICS_IOC_SEQUENCES_HPP

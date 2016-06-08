/*
 * Copyright (c) 2010-2015 LAAS/CNRS, WPI
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
 *                                               Jim Mainprice Sun 5 July 2015
 */

#include <iostream>
#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"

using std::cout;
using std::endl;

bool TestEquality(const std::vector<Eigen::VectorXd>& samples)
{
  bool success = true;
  for (size_t i = 0; i < samples.size(); i++) {
    for (size_t j = i + 1; j < samples.size(); j++) {
      for (int d = 0; d < samples[0].size(); d++) {
        if (samples[i][d] != samples[j][d]) {
          success = false;
          break;
        }
      }
    }
  }
  return success;
}

int main(int argc, char* argv[])
{
  int nb_samples = 20;
  int dimension = 10;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(dimension, dimension);
  Eigen::VectorXd mean = Eigen::VectorXd::Ones(dimension);

  std::vector<Eigen::VectorXd> samples;
  for (int i = 0; i < nb_samples; i++) {
    MultivariateGaussian sampler(mean, covariance, 1);
    // cout << "sample : " << endl;
    Eigen::VectorXd s(dimension);
    sampler.sample(s);
    samples.push_back(s);
  }

  if (!TestEquality(samples)) {
    cout << "Error" << endl;
    return EXIT_FAILURE;
  }

  cout << "success (MultivariateGaussian) " << endl;

  samples.clear();
  for (int i = 0; i < nb_samples; i++) {
    std::srand((unsigned int)time(0));
    samples.push_back(Eigen::VectorXd::Random(dimension));
  }

  if (!TestEquality(samples)) {
    cout << "Error" << endl;
    return EXIT_FAILURE;
  }

  cout << "success (Eigen)" << endl;
  return EXIT_SUCCESS;
}

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

int main(int argc, char *argv[]) {
  int dimension = 10;
  Eigen::MatrixXd covariance = Eigen::MatrixXd::Identity(dimension, dimension);
  Eigen::VectorXd mean = Eigen::VectorXd::Ones(dimension);
  MultivariateGaussian sampler( mean, covariance );
  cout << "sample : " << endl;
  Eigen::VectorXd sample( dimension );
  sampler.sample( sample );
  cout << sample << endl;
  return EXIT_SUCCESS;
}

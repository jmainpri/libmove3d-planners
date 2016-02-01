/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef MULTIVARIATE_GAUSSIAN_H_
#define MULTIVARIATE_GAUSSIAN_H_

////#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <cstdlib>
#include <iostream>

// For random number generator (seed can be passed as argument)
#include "utils/misc_functions.hpp"
// namespace chomp
//{

/**
   * \brief Generates samples from a multivariate gaussian distribution
   */
class MultivariateGaussian {
 public:
  template <typename Derived1, typename Derived2>
  MultivariateGaussian(const Eigen::MatrixBase<Derived1>& mean,
                       const Eigen::MatrixBase<Derived2>& covariance,
                       int seed=0);

  template <typename Derived>
  void sample(Eigen::MatrixBase<Derived>& output);

 private:
  Eigen::VectorXd mean_;       /**< Mean of the gaussian distribution */
  Eigen::MatrixXd covariance_; /**< Covariance of the gaussian distribution */
  Eigen::MatrixXd covariance_cholesky_; /**< Cholesky decomposition (LL^T) of
                                           the covariance */

  int size_;
  boost::mt19937 rng_;
  boost::normal_distribution<> normal_dist_;
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
      gaussian_;
};

//////////////////////// template function definitions follow
/////////////////////////////////

template <typename Derived1, typename Derived2>
MultivariateGaussian::MultivariateGaussian(
    const Eigen::MatrixBase<Derived1>& mean,
    const Eigen::MatrixBase<Derived2>& covariance,
    int seed)
    : mean_(mean),
      covariance_(covariance),
      covariance_cholesky_(covariance_.llt().matrixL()),
      size_(mean.rows()),
      //rng_(seed),
      rng_( seed > 0 ? seed : move3d_random_integer(seed, RAND_MAX)),
      gaussian_(rng_, normal_dist_) {}

template <typename Derived>
void MultivariateGaussian::sample(Eigen::MatrixBase<Derived>& output) {
  for (int i = 0; i < size_; ++i) output(i) = gaussian_();
  output = mean_ + covariance_cholesky_ * output;
}

//}

#endif /* MULTIVARIATE_GAUSSIAN_H_ */

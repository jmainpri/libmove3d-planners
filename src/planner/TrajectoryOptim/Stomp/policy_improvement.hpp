/*********************************************************************
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 
 \file    policy_improvement.h
 
 \author  Ludovic Righetti, Peter Pastor, Mrinal Kalakrishnan
 \date    May 26, 2010
 
 **********************************************************************/

#ifndef POLICYIMPROVEMENT_H_
#define POLICYIMPROVEMENT_H_

// ros includes
//#include <ros/ros.h>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/Core>

// local includes
#include "policy.hpp"
#include "task.hpp"

#include "planner/TrajectoryOptim/Chomp/chompMultivariateGaussian.hpp"
#include "planner/TrajectoryOptim/Chomp/chompPlanningGroup.hpp"

#include <boost/shared_ptr.hpp>

extern std::vector<double> global_noiseTrajectory1;
extern std::vector<double> global_noiseTrajectory2;

namespace stomp_motion_planner
{
  
  struct Rollout
  {
    std::vector<Eigen::VectorXd> parameters_;                       /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_;                            /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> noise_projected_;                  /**< [num_dimensions][num_time_steps] num_parameters */
    std::vector<Eigen::VectorXd> parameters_noise_projected_;       /**< [num_dimensions][num_time_steps] num_parameters */
    Eigen::VectorXd state_costs_;                                   /**< num_time_steps */
    std::vector<Eigen::VectorXd> control_costs_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> total_costs_;                      /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> cumulative_costs_;                 /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> probabilities_;                    /**< [num_dimensions] num_time_steps */
    std::vector<Eigen::VectorXd> nominal_parameters_;                    /**< [num_dimensions] num_time_steps */
    
    bool out_of_bounds_; /**< Wether the rollout is violating dof limits */
    
    double getCost();   /**< Gets the rollout cost = state cost + control costs per dimension */
    
    void printCost(double weight=1.0);
    void printProbabilities();
  };
  
  class PolicyImprovement
  {
  public:
    /*!
     * Constructor for the policy improvement class
     */
    PolicyImprovement();
    
    /*!
     * Destructor
     */
    ~PolicyImprovement();
    
    /**
     * Initializes the object which is required for all operations to succeed.
     * @param num_rollouts
     * @param num_time_steps
     * @param num_reused_rollouts
     * @param policy
     * @return true on success, false on failure
     */
    bool initialize(const int num_rollouts, const int num_time_steps, const int num_reused_rollouts,
                    const int num_extra_rollouts, 
                    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy> policy,
                    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Task>   task,
                    double discretization,
                    bool use_cumulative_costs=true);
    
    /**
     * Resets the number of rollouts
     * @param num_rollouts
     * @return
     */
    bool setNumRollouts(const int num_rollouts, const int num_reused_rollouts, const int num_extra_rollouts);
    
    /**
     * Gets the next set of rollouts. Only "new" rollouts that need to be executed are returned,
     * not rollouts which might be reused from the previous set.
     * @param rollouts_ [num_rollouts][num_dimensions] num_parameters
     * @param noise_variance [num_dimensions] noise standard deviation per dimension
     * @return
     */
    bool getRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, const std::vector<double>& noise_stddev,
                     bool get_reused, std::vector<std::vector<Eigen::VectorXd> >& reused_rollouts);
    
    /*!
     * Set the costs of each rollout per time-step
     * Only the first "n" rows of the costs matrix is used, where n is the number of rollouts
     * generated by getRollouts(), because some rollouts may be used from previous iterations.
     * Outputs the total cost for each rollout (generated and reused) in rollout_costs_total
     * @param costs
     */
    bool setRolloutCosts(const Eigen::MatrixXd& costs, const std::vector<Eigen::MatrixXd>& control_costs, const double control_cost_weight, std::vector<double>& rollout_costs_total);
    
    /**
     * Performs the PI^2 update and provides parameter updates at every time step
     *
     * @param parameter_updates [num_dimensions] num_time_steps x num_parameters
     * @return
     */
    bool improvePolicy(std::vector<Eigen::MatrixXd>& parameter_updates);
    
    /**
     * Adds extra rollouts to the set of rollouts to be reused
     */
    bool addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rollout_costs);
    
    /**
     * Tests the noise generators
     * Function added by jim
     */
    void testNoiseGenerators();
    
    /**
     * Set rollout as out of bounds
     * Function added by jim
     */
    void setRolloutOutOfBounds(int id, bool out_of_bounds);
    
    /**
     * Reset extra rollouts
     */
    bool resetReusedRollouts();

    /**
     * set rollouts
     */
    bool setRollouts( const std::vector<std::vector<Eigen::VectorXd> >& rollouts );



    std::vector<Eigen::MatrixXd> projection_matrix_;                        /**< [num_dimensions] num_parameters x num_parameters */
    
  private:
    
    bool initialized_;
    
    int num_dimensions_;
    std::vector<int> num_parameters_;
    int num_rollouts_;
    int num_time_steps_;
    int num_rollouts_reused_;
    int num_rollouts_extra_;
    
    bool rollouts_reused_;                                                  /**< Are we reusing rollouts for this iteration? */
    bool rollouts_reused_next_;                                             /**< Can we reuse rollouts for the next iteration? */
    bool extra_rollouts_added_;                                             /**< Have the "extra rollouts" been added for use in the next iteration? */
    int num_rollouts_gen_;                                                  /**< How many new rollouts have been generated in this iteration? */
    
    bool use_multiplication_by_m_;
    bool use_cumulative_costs_;                                             /**< Use cumulative costs or state costs? */

    
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy> policy_;
    MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Task>   task_;
    
    std::vector<Eigen::MatrixXd> control_costs_;                            /**< [num_dimensions] num_parameters x num_parameters */
    std::vector<Eigen::MatrixXd> inv_control_costs_;                        /**< [num_dimensions] num_parameters x num_parameters */
//    std::vector<Eigen::MatrixXd> covariances_;

    bool multiple_smoothness_;
    Eigen::VectorXd control_cost_weights_;
    double control_cost_weight_;
    double discretization_;
    
    std::vector<Eigen::MatrixXd> basis_functions_;                          /**< [num_dimensions] num_time_steps x num_parameters */

    std::vector<Eigen::VectorXd> parameters_;                               /**< [num_dimensions] num_parameters */
    
    std::vector<Rollout> rollouts_;
    std::vector<Rollout> reused_rollouts_;
    std::vector<Rollout> extra_rollouts_;
    
    std::vector<MultivariateGaussian> noise_generators_;                    /**< objects that generate noise for each dimension */
    std::vector<Eigen::MatrixXd> parameter_updates_;                        /**< [num_dimensions] num_time_steps x num_parameters */
    
    // temporary variables pre-allocated for efficiency:
    std::vector<Eigen::VectorXd> tmp_noise_;                /**< [num_dimensions] num_parameters */
    std::vector<Eigen::VectorXd> tmp_parameters_;           /**< [num_dimensions] num_parameters */
    Eigen::VectorXd tmp_max_cost_;                          /**< num_time_steps */
    Eigen::VectorXd tmp_min_cost_;                          /**< num_time_steps */
    Eigen::VectorXd tmp_max_minus_min_cost_;                /**< num_time_steps */
    Eigen::VectorXd tmp_sum_rollout_probabilities_;         /**< num_time_steps */
    std::vector<std::pair<double, int> > rollout_cost_sorter_;  /**< vector used for sorting rollouts by their cost */
    std::vector<double> rollout_costs_total_;
    
    bool preAllocateMultivariateGaussianSampler();
    bool preAllocateTempVariables();
    bool preComputeProjectionMatrices();
    
    void resampleUpdates();
    bool computeProjectedNoise();
    bool computeRolloutCumulativeCosts();
    bool computeRolloutProbabilities();
    bool computeParameterUpdates();
    
    bool computeNoise(Rollout& rollout);
    bool computeProjectedNoise(Rollout& rollout);
    bool copyParametersFromPolicy();

    bool covarianceMatrixAdaptaion();
    
    void addStraightLines( std::vector<int> points, Rollout& rollouts);
    bool generateRollouts(const std::vector<double>& noise_variance);
    bool simpleJointLimits( Rollout& traj ) const;

    void addParmametersToDraw(const std::vector<Eigen::VectorXd>& rollout, int color);
  };
  
}

#endif /* POLICYIMPROVEMENT_H_ */

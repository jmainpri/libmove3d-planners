/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
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

 \file    policy_improvement.cpp

 \author  Ludovic Righetti, Peter Pastor, Mrinal Kalakrishnan
 \date    May 26, 2010

 **********************************************************************/

// system includes
#include <time.h>
#include <cfloat>
#include <stdio.h>
//#include <ros/assert.h>

#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/LU>
#include <Eigen/Array>

// local includes
//#include <ros/ros.h>
#include "lampStomp.hpp"
//#include <stomp_motion_planner/assert.h>
#include <algorithm>
#include <iostream>

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "planner/planEnvironment.hpp"
#include "feature_space/smoothness.hpp"
#include "Graphic-pkg.h"
#include "P3d-pkg.h"

#include "stompOptimizer.hpp"

std::vector<double> global_noiseTrajectory1;
std::vector<double> global_noiseTrajectory2;

using std::cout;
using std::endl;
using namespace Move3D;

MOVE3D_USING_BOOST_NAMESPACE

USING_PART_OF_NAMESPACE_EIGEN

namespace Move3D
{
    //! Computes the cost the rollout
    //! the function sums the control costs of all dimensions
    double Rollout::getCost()
    {
        double cost = state_costs_.sum();
        int num_dim = control_costs_.size();
        for (int d=0; d<num_dim; ++d)
            cost += control_costs_[d].sum();
        return cost;
    }

    void Rollout::printCost( double weight )
    {
        double control_cost = 0.0;

        for (int d=0; d<int(control_costs_.size()); ++d)
            control_cost += ( control_costs_[d].sum() );

        cout.precision(6);
        cout << "control cost : " << control_cost << " , state cost : " << state_costs_.sum() << endl;
    }

    void Rollout::printProbabilities()
    {
        cout << "state_costs_ : " << endl;
        cout <<  state_costs_.transpose() << endl;

        cout << "control_costs_ : " << endl;
        for (int d=0; d<int(control_costs_.size()); ++d)
        {
            cout <<  control_costs_[d].transpose() << endl;
        }

        cout << "cumulative_cost_ : " << endl;
        for (int d=0; d<int(probabilities_.size()); ++d)
        {
            cout <<  cumulative_costs_[d].transpose() << endl;
        }

        cout << "probabilities_ : " << endl;
        for (int d=0; d<int(probabilities_.size()); ++d)
        {
            cout <<  probabilities_[d].transpose() << endl;
        }
        cout << endl;
    }

    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    //----------------------------------------------------------------------

    PolicyImprovement::PolicyImprovement() : initialized_(false)
    {

    }

    PolicyImprovement::~PolicyImprovement()
    {

    }

    //! Initialize policy improvment
    //! 1 - Allocates a multivariate gaussian sampler
    //! 2 - Computes the projection matrix
    bool PolicyImprovement::initialize(const int num_rollouts,
                                       const int num_time_steps,
                                       const int num_reused_rollouts,
                                       const int num_extra_rollouts,
//                                       MOVE3D_BOOST_PTR_NAMESPACE<Policy> policy,
//                                       MOVE3D_BOOST_PTR_NAMESPACE<Task>   task,
                                       double discretization,
                                       bool use_cumulative_costs)
    {
        num_time_steps_ = num_time_steps;
        use_cumulative_costs_ = use_cumulative_costs;
        use_multiplication_by_m_ = PlanEnv->getBool(PlanParam::trajStompMultiplyM);
//        policy_ = policy;
//        task_ = task;
        discretization_ = discretization;

//        policy_->setNumTimeSteps(num_time_steps_);
//        policy_->getControlCosts(control_costs_);
//        policy_->getCovariances(inv_control_costs_);
//        policy_->getNumDimensions(num_dimensions_);
//        policy_->getNumParameters(num_parameters_);
//        policy_->getBasisFunctions(basis_functions_);
//        policy_->getParameters(parameters_);

//        cout << "num_reused_rollouts : " << num_reused_rollouts << endl;

        preAllocateMultivariateGaussianSampler();
        setNumRollouts( num_rollouts, num_reused_rollouts, num_extra_rollouts );
        preAllocateTempVariables();
        preComputeProjectionMatrices();

//        project_last_config_ = true;

//        if( project_last_config_ && ( planning_group_ != NULL ) )
//        {
//            Move3D::Robot* robot = planning_group_->robot_;
//            Move3D::confPtr_t q_goal = robot->getGoalPos();
//            robot->setAndUpdate(*q_goal);
//            task_goal_ = robot->getJoint( "J3" )->getVectorPos();
//        }

        multiple_smoothness_ = true;

        // TODO move somewhere else
        if( multiple_smoothness_ )
        {
            Move3D::StackedFeatures* fct = dynamic_cast<Move3D::StackedFeatures*>( global_activeFeatureFunction );

            if( fct != NULL &&
                    fct->getFeatureFunction("SmoothnessAll") != NULL &&
                    fct->getFeatureFunction("SmoothnessAll")->is_active_ )
            {
                control_cost_weights_ = fct->getFeatureFunction("SmoothnessAll")->getWeights();
//                cout << "control_cost_weights_ : " << control_cost_weights_.transpose() << endl;
            }
            else{
                multiple_smoothness_ = false;
            }
        }

        return (initialized_ = true);
    }

    //! Allocates the sampler
    //! also initializes the control cost inverse structure
    bool PolicyImprovement::preAllocateMultivariateGaussianSampler()
    {
        // invert the control costs, initialize noise generators:
//        inv_control_costs_.clear();
        noise_generators_.clear();

        for (int d=0; d<num_dimensions_; ++d)
        {
            //cout << "control_costs_[" << d << "] = " << endl << control_costs_[d] << endl;
            //cout << "inv_control_costs_[" << d << "] = " << endl << control_costs_[d].inverse() << endl;
//            inv_control_costs_.push_back(control_costs_[d].inverse());

//            move3d_save_matrix_to_file( inv_control_costs_[d], "../matlab/invcost_matrix.txt" );

            if( !PlanEnv->getBool(PlanParam::trajStompMatrixAdaptation) )
            {
                MultivariateGaussian mvg( VectorXd::Zero(num_parameters_[d]), inv_control_costs_[d] );
                noise_generators_.push_back(mvg);
            }
            else
            {
                cout << "Error preAllocateMultivariateGaussianSampler" << endl;
                MultivariateGaussian mvg( VectorXd::Zero(num_parameters_[d]), MatrixXd::Identity(num_time_steps_,num_time_steps_) );
                noise_generators_.push_back(mvg);
                return false;
            }
        }

        return true;
    }

    //! Allocates the rollout structures
    //! @param num_rollouts the total number of rollouts
    //! @param num_reused_rollouts the reused number of rollouts
    //! @param num_extra_rollouts the number of extra rollouts
    bool PolicyImprovement::setNumRollouts(const int num_rollouts, const int num_reused_rollouts, const int num_extra_rollouts)
    {
        num_rollouts_ = num_rollouts;
        num_rollouts_reused_ = num_reused_rollouts;
        num_rollouts_extra_ = num_extra_rollouts;
        num_rollouts_gen_ = 0;
        if (num_rollouts_reused_ >= num_rollouts)
        {
            printf("Number of reused rollouts must be strictly less than number of rollouts.");
            return false;
        }

        // preallocate memory for a single rollout:
        Rollout rollout;

        rollout.parameters_.clear();
        rollout.noise_.clear();
        rollout.noise_projected_.clear();
        rollout.parameters_noise_projected_.clear();
        rollout.control_costs_.clear();
        rollout.total_costs_.clear();
        rollout.cumulative_costs_.clear();
        rollout.probabilities_.clear();

        //cout << "num_time_steps_  = " << num_time_steps_ << endl;

        for (int d=0; d<num_dimensions_; ++d)
        {
//            cout << "num_parameters_[" << d << "] = " << num_parameters_[d] << endl;
//            cout << "num_time_steps_ = " << num_time_steps_ << endl;
            rollout.parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
            rollout.noise_.push_back(VectorXd::Zero(num_parameters_[d]));
            rollout.noise_projected_.push_back(VectorXd::Zero(num_parameters_[d]));

            rollout.control_costs_.push_back(VectorXd::Zero(num_time_steps_));
            rollout.total_costs_.push_back(VectorXd::Zero(num_time_steps_));
            rollout.cumulative_costs_.push_back(VectorXd::Zero(num_time_steps_));
            rollout.probabilities_.push_back(VectorXd::Zero(num_time_steps_));
            rollout.nominal_parameters_.push_back(VectorXd::Zero(num_time_steps_));
        }
        rollout.state_costs_ = VectorXd::Zero(num_time_steps_);
        rollout.out_of_bounds_ = false;

        // duplicate this rollout:
        for (int r=0; r<num_rollouts; ++r)
            rollouts_.push_back(rollout);

        for (int r=0; r<num_reused_rollouts; ++r)
            reused_rollouts_.push_back(rollout);

        for (int r=0; r<num_extra_rollouts; ++r)
            extra_rollouts_.push_back(rollout);


        rollouts_reused_ = false;
        rollouts_reused_next_ = false;
        //    rollouts_reused_ = true;
        //    rollouts_reused_next_ = true;
        extra_rollouts_added_ = false;

        rollout_costs_total_.resize(num_rollouts);
        rollout_cost_sorter_.reserve(num_rollouts_);

        return true;
    }

    //! Allocates Temporary Variables
    bool PolicyImprovement::preAllocateTempVariables()
    {
        tmp_noise_.clear();
        tmp_parameters_.clear();
        parameter_updates_.clear();
        for (int d=0; d<num_dimensions_; ++d)
        {
            tmp_noise_.push_back(VectorXd::Zero(num_parameters_[d]));
            tmp_parameters_.push_back(VectorXd::Zero(num_parameters_[d]));
            parameter_updates_.push_back(MatrixXd::Zero(num_time_steps_, num_parameters_[d]));
        }
        tmp_max_cost_ = VectorXd::Zero(num_time_steps_);
        tmp_min_cost_ = VectorXd::Zero(num_time_steps_);
        tmp_sum_rollout_probabilities_ = VectorXd::Zero(num_time_steps_);

        return true;
    }

    //! Precomputes the projection matrices M
    //! using the inverse of the control cost matrix
    //! Each column of the
    bool PolicyImprovement::preComputeProjectionMatrices()
    {
        //  ROS_INFO("Precomputing projection matrices..");
        projection_matrix_.resize(num_dimensions_);
        for (int d=0; d<num_dimensions_; ++d)
        {
            projection_matrix_[d] = inv_control_costs_[d];

            //cout << "Inv control Matrix = " << endl << inv_control_costs_[d] << endl;

            for (int p=0; p<num_parameters_[d]; ++p)
            {
                double column_max = inv_control_costs_[d](0,p);

                for (int p2 = 1; p2 < num_parameters_[d]; ++p2)
                {
                    if (inv_control_costs_[d](p2,p) > column_max)
                        column_max = inv_control_costs_[d](p2,p);
                }
                projection_matrix_[d].col(p) *= (1.0/(num_parameters_[d]*column_max));
            }

            move3d_save_matrix_to_file(projection_matrix_[0],"../matlab/m_mat.txt");

            //cout << "Projection Matrix = " << endl << projection_matrix_[d] << endl;
        }

        //  ROS_INFO("Done precomputing projection matrices.");
        return true;
    }

    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    bool PolicyImprovement::copyParametersFromPolicy()
    {
//        if (!policy_->getParameters(parameters_))
//        {
//            //        ROS_ERROR("Failed to get policy parametertes.");
//            return false;
//        }

        //    // draw traj
        //    const std::vector<ChompDof>& joints = optimizer->getPlanningGroup()->chomp_dofs_;
        //    Robot* robot = optimizer->getPlanningGroup()->robot_;
        //    Move3D::Trajectory traj(robot);
        //
        //    for ( int j=0; j<num_time_steps_; ++j)
        //    {
        //      confPtr_t q = robot->getCurrentPos();
        //
        //      for ( int i=0; i<optimizer->getPlanningGroup()->num_joints_; ++i)
        //      {
        //        (*q)[joints[i].move3d_dof_index_] = parameters_[i][j];
        //      }
        //      traj.push_back(q);
        //    }
        //    traj.replaceP3dTraj();
        //    //    traj.print();
        //    g3d_draw_allwin_active();
        //    cout << "num_time_steps_ : " << num_time_steps_ << endl;

        return true;
    }

    bool PolicyImprovement::computeProjectedNoise(Rollout& rollout)
    {
//        for (int d=0; d<num_dimensions_; ++d)
//        {
//            rollout.noise_projected_[d] = projection_matrix_[d] * rollout.noise_[d];
            //rollout.parameters_noise_projected_[d] = rollout.parameters_[d] + rollout.noise_projected_[d];
            //cout << "rollout.noise_projected_[" << d << "] = " << rollout.noise_projected_[d].transpose() << endl;
//        }

        return true;
    }

    bool PolicyImprovement::computeProjectedNoise()
    {
//        for (int r=0; r<num_rollouts_; ++r)
//        {
//            computeProjectedNoise(rollouts_[r]);
//        }
        return true;
    }

    bool PolicyImprovement::resetReusedRollouts()
    {
        rollouts_reused_next_ = false;
        return true;
    }

    /*
    bool PolicyImprovement::simpleJointLimits( Rollout& traj ) const
    {
        for( int j=0;j<num_dimensions_;j++)
        {
            double coeff = 1.0;

            double j_max = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup()->chomp_dofs_[j].joint_limit_max_;
            double j_min = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup()->chomp_dofs_[j].joint_limit_min_;

            for( int i=0;i<num_time_steps_;i++)
            {
                int k=0;
                while( traj.parameters_[j][i] < j_min || traj.parameters_[j][i] > j_max )
                {
                    coeff *= 0.90; // 90 percent
                    traj.noise_[j] *= coeff;
                    traj.parameters_[j] = traj.nominal_parameters_[j] +  traj.noise_[j];
                    if( k++ > 3 ){ // this was at 100
                        break;
                    }
                }

                // cout << "Joint limit coefficient : " << coeff << endl;
            }

            // Set the start and end values constant
            traj.parameters_[j].start(1) = traj.parameters_[j].start(1);
            traj.parameters_[j].end(1) = traj.parameters_[j].end(1);
        }

        return true;
    }
    */

    bool PolicyImprovement::generateRollouts(const std::vector<double>& noise_stddev)
    {
        assert(initialized_);
        assert(static_cast<int>(noise_stddev.size()) == num_dimensions_);

        // save the latest policy parameters:
        copyParametersFromPolicy();

        // we assume here that rollout_parameters_ and rollout_noise_ have already been allocated
        num_rollouts_gen_ = num_rollouts_ - num_rollouts_reused_;

        // The rollouts of the run are not used
        if (!rollouts_reused_next_)
        {
            num_rollouts_gen_ = num_rollouts_;
            if (num_rollouts_reused_ > 0)
            {
                rollouts_reused_next_ = true;
            }

            rollouts_reused_ = false;
        }
        else
        {
            // figure out which rollouts to reuse
            rollout_cost_sorter_.clear();
            for (int r=0; r<num_rollouts_; ++r)
            {
                double cost = rollout_costs_total_[r];
                //  cout << "rollouts_(" << r << ") cost : " << rollouts_[r].getCost() << " , stored cost : "  << rollout_costs_total_[r] << endl;

                // discard out of bounds rollouts
                if ( rollouts_[r].out_of_bounds_ ) {
                    cost = std::numeric_limits<double>::max();
                }
                // cout << "rollout_cost_sorter_(" << r << ") cost : " << cost << endl;
                rollout_cost_sorter_.push_back(std::make_pair(cost,r));
            }
            if ( /*extra_rollouts_added_*/ false )
            {
                for (int r=0; r<num_rollouts_extra_; ++r)
                {
                    double cost;
                    if(r==0)
                        cost = 0.0;
                    else
                        cost = extra_rollouts_[r].getCost();

                    rollout_cost_sorter_.push_back(std::make_pair(cost,-r-1));
                    // cout << "extra cost["<< r <<"]: " << cost << endl;
                    // index is -ve if rollout is taken from extra_rollouts
                }
                extra_rollouts_added_ = false;
            }
            std::sort(rollout_cost_sorter_.begin(), rollout_cost_sorter_.end());

//            for (int r=0; r<num_rollouts_reused_; r++)
//                cout << "reused_rollouts_(" << r << ") cost : " << reused_rollouts_[r].getCost() << endl;

            // use the best ones: (copy them into reused_rollouts)
            for (int r=0; r<num_rollouts_reused_; ++r)
            {
                double reuse_index = rollout_cost_sorter_[r].second;
                // double reuse_cost = rollout_cost_sorter_[r].first;

//                cout <<  "Reuse "<< r << ", cost = " << rollout_cost_sorter_[r].first << endl;

                if (reuse_index >=0)
                    reused_rollouts_[r] = rollouts_[reuse_index];
                else
                {
                    //ROS_INFO("Reused noise-less rollout of cost %f", reuse_cost);
                    reused_rollouts_[r] = extra_rollouts_[-reuse_index-1];
                }
            }

            // copy them back from reused_rollouts_ into rollouts_
            for (int r=0; r<num_rollouts_reused_; ++r)
            {
                rollouts_[num_rollouts_gen_+r] = reused_rollouts_[r];

                // update the noise based on the new parameters:
                for (int d=0; d<num_dimensions_; ++d)
                {
                    rollouts_[num_rollouts_gen_+r].noise_[d] = rollouts_[num_rollouts_gen_+r].parameters_[d] - parameters_[d];
                }
            }
            rollouts_reused_ = true;
        }

        // Generate new rollouts
        for (int d=0; d<num_dimensions_; ++d)
        {
            for (int r=0; r<num_rollouts_gen_; ++r)
            {
                noise_generators_[d].sample( tmp_noise_[d] );

                //cout << "noise(" << r << "," << d << ") : " << tmp_noise_[d].transpose() << endl;

                rollouts_[r].noise_[d] = noise_stddev[d]*tmp_noise_[d];
                rollouts_[r].parameters_[d] = parameters_[d] + rollouts_[r].noise_[d];
                rollouts_[r].nominal_parameters_[d] = parameters_[d];

                bool add_straight_lines = false;
                if( add_straight_lines )
                {
                    std::vector<int> points;
                    points.resize(7);
                    points[0] = 0;
                    points[1] = num_time_steps_-1;

                    for (int i=2; i<int(points.size()); i++) {
                        points[i] = round(p3d_random( 1.0, (num_time_steps_-2)));
                    }

                    std::sort(points.begin(), points.end());

                    //          if( init != end )
                    addStraightLines( points, rollouts_[r] );
                }

                // Saves the first rollout for first dimenstion

                /**
         if (r == 0 && d == 0)
        {
          global_noiseTrajectory1.clear();

          for (int i=0; i<rollouts_[r].parameters_[d].size(); i++)
          {
            global_noiseTrajectory1.push_back(rollouts_[r].noise_[d][i]);
            //global_noiseTrajectory1.push_back(rollouts_[r].parameters_[d][i]);
          }
        }
        // Saves the first rollout for second dimenstion
        if (r == 0 && d == 1)
        {
          global_noiseTrajectory2.clear();

          //cout << "rollouts_[r].parameters_[d].size() : " << rollouts_[r].parameters_[d].size() << endl;
          for (int i=0; i<rollouts_[r].parameters_[d].size(); i++)
          {
            global_noiseTrajectory2.push_back(rollouts_[r].noise_[d][i]);
            //global_noiseTrajectory2.push_back(rollouts_[r].parameters_[d][i]);
          }
        }
         */
                //cout << "rollouts_[" << r << "].noise_[" << d << "] = "<< rollouts_[r].noise_[d].transpose() << endl;
                //cout << "rollouts_[" << r << "].parameters_[" << d << "] = "<< rollouts_[r].parameters_[d].transpose() << endl;
            }
        }
/**
 * if( false )
            for (int r=0; r<num_rollouts_gen_; ++r)
            {
                // WARNING ADDED FOR IOC
                simpleJointLimits( rollouts_[r] );
            }
            */

        return true;
    }

    bool PolicyImprovement::setRollouts(const std::vector<std::vector<Eigen::VectorXd> >& rollouts )
    {
        for (int r=0; r<rollouts.size(); ++r)
        {
            rollouts_[r].parameters_ = rollouts[r];
            computeNoise( rollouts_[r] );
        }

        return true;
    }

    bool PolicyImprovement::getRollouts(std::vector<std::vector<Eigen::VectorXd> >& generated_rollouts, const std::vector<double>& noise_variance,
                                        bool get_reused, std::vector<std::vector<Eigen::VectorXd> >& reused_rollouts)
    {
        if ( !generateRollouts( noise_variance ) )
        {
            cout << "Failed to generate rollouts." << endl;
            return false;
        }

        generated_rollouts.clear();
        for (int r=0; r<num_rollouts_gen_; ++r)
        {
            generated_rollouts.push_back( rollouts_[r].parameters_ );
        }

        if( get_reused )
        {
            reused_rollouts.clear();

            if (rollouts_reused_)
            {
                for (int r=0; r<num_rollouts_reused_; ++r)
                {
                    reused_rollouts.push_back(rollouts_[num_rollouts_gen_+r].parameters_);
                }
            }
        }
        // computeProjectedNoise();
        return true;
    }

    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    //----------------------------------------------------------------------
    void PolicyImprovement::setRolloutOutOfBounds(int r, bool out_of_bounds)
    {
        assert(initialized_);

        if( r<0 || r>=num_rollouts_gen_ )
            return;

        // cout << "rollout[" << r << "] out of bounds > " << out_of_bounds << endl;

        rollouts_[r].out_of_bounds_ = out_of_bounds;
    }

    bool PolicyImprovement::setRolloutCosts(const Eigen::MatrixXd& costs, const std::vector<Eigen::MatrixXd>& control_costs, const double control_cost_weight, std::vector<double>& rollout_costs_total)
    {
        assert(initialized_);

        control_cost_weight_ = control_cost_weight;

        // Warning some control costs may be added to the state costs
        for (int r=0; r<num_rollouts_gen_; ++r)
        {
            rollouts_[r].state_costs_ = costs.row(r).transpose();

            // TODO add control costs
            for (int d=0; d<num_dimensions_; ++d)
                rollouts_[r].control_costs_[d] = control_costs[r].row(d).transpose();
        }

        // set control costs
        // computeRolloutControlCosts();

        // set the total costs
        rollout_costs_total.resize(num_rollouts_);
        for (int r=0; r<num_rollouts_; ++r)
        {
            rollout_costs_total[r] = rollout_costs_total_[r] = rollouts_[r].getCost();
            // rollouts_[r].printCost();
        }
        return true;
    }

    bool PolicyImprovement::computeRolloutCumulativeCosts()
    {
        // compute cumulative costs at each timestep
        for (int r=0; r<num_rollouts_; ++r)
        {
            for (int d=0; d<num_dimensions_; ++d)
            {
                rollouts_[r].total_costs_[d] = rollouts_[r].state_costs_ + rollouts_[r].control_costs_[d];
                rollouts_[r].cumulative_costs_[d] = rollouts_[r].total_costs_[d];
                // cout << "------------------------------------------------------------" << endl;
                // cout << "rollouts_["<<r<<"].state_costs       = " << rollouts_[r].state_costs_.transpose() << endl;
                // cout << "rollouts_["<<r<<"].control_costs_["<<d<<"] = " << rollouts_[r].control_costs_[d].transpose() << endl;
                // cout << "rollouts_["<<r<<"].total_costs_["<<d<<"]   = " << rollouts_[r].total_costs_[d].transpose() << endl;

                if (use_cumulative_costs_)
                {
                    for (int t=num_time_steps_-2; t>=0; --t)
                    {
                        rollouts_[r].cumulative_costs_[d](t) += rollouts_[r].cumulative_costs_[d](t+1);
                    }
                }
            }
        }
        return true;
    }

    bool PolicyImprovement::computeRolloutProbabilities()
    {
        for (int d=0; d<num_dimensions_; ++d)
        {
            // cout  << "dimension " << d << ", Cumulative costs " << rollouts_[0].cumulative_costs_[d] << endl;
            // tmp_min_cost_ = rollout_cumulative_costs_[d].colwise().minCoeff().transpose();
            // tmp_max_cost_ = rollout_cumulative_costs_[d].colwise().maxCoeff().transpose();
            // tmp_max_minus_min_cost_ = tmp_max_cost_ - tmp_min_cost_;

            for (int t=0; t<num_time_steps_; t++)
            {
                // find min and max cost over all rollouts:
                double min_cost = rollouts_[0].cumulative_costs_[d](t);
                double max_cost = min_cost;
                for (int r=1; r<num_rollouts_; ++r)
                {
                    double c = rollouts_[r].cumulative_costs_[d](t);
                    if (c < min_cost)
                        min_cost = c;
                    if (c > max_cost)
                        max_cost = c;
                }

                double denom = max_cost - min_cost;

                // prevent divide by zero:
                if (denom < 1e-8)
                    denom = 1e-8;

                double p_sum = 0.0;
                for (int r=0; r<num_rollouts_; ++r)
                {
                    // the -10.0 here is taken from the paper:
                    rollouts_[r].probabilities_[d](t) = exp(-10.0*(rollouts_[r].cumulative_costs_[d](t) - min_cost)/denom);
                    p_sum += rollouts_[r].probabilities_[d](t);
                }
                for (int r=0; r<num_rollouts_; ++r)
                {
                    rollouts_[r].probabilities_[d](t) /= p_sum;
                }
            }
        }

        //    for (int r=0; r<num_rollouts_; ++r)
        //    {
        //      cout << "Rollout nb : " << r << endl;
        //      rollouts_[r].printProbabilities();
        //    }
        return true;
    }


    bool PolicyImprovement::computeParameterUpdates()
    {
        //MatrixXd noise_update = MatrixXd(num_time_steps_, num_dimensions_);

        std::vector<Eigen::VectorXd> parameters;

        for (int d=0; d<num_dimensions_; ++d)
        {
            parameter_updates_[d] = MatrixXd::Zero(num_time_steps_, num_parameters_[d]);

            for (int r=0; r<num_rollouts_; ++r)
            {
                //cout << "rollouts_[" << r << "].probabilities_[" << d << "] = " << endl << rollouts_[r].probabilities_[d] << endl;
                //cout << "rollouts_[" << r << "].noise_[" << d << "] = " << endl << rollouts_[r].noise_[d] << endl;
                //cout << "parameter_updates_[" << d << "].row(0).transpose() = ";
                //cout << endl << parameter_updates_[d].row(0).transpose() << endl;

                if( !rollouts_[r].out_of_bounds_ )
                {
                    parameter_updates_[d].row(0).transpose() += rollouts_[r].noise_[d].cwise() * rollouts_[r].probabilities_[d];
                }
                else
                {
                    cout << "joint limits out of bounds" << endl;
                }
            }

            parameters.push_back( parameter_updates_[d].row(0).transpose() + parameters_[d] );

            // cout << "parameter_updates_[" << d << "].row(0).transpose() = " << endl << parameter_updates_[d].row(0).transpose() << endl;
            // This is the multiplication by M
            if( use_multiplication_by_m_ )
            {
                parameter_updates_[d].row(0).transpose() = projection_matrix_[d]*parameter_updates_[d].row(0).transpose();
            }
        }

        addParmametersToDraw( parameters, 0 );

        //resampleUpdates();
        return true;
    }

    bool PolicyImprovement::improvePolicy(std::vector<Eigen::MatrixXd>& parameter_updates)
    {
        assert(initialized_);

        computeRolloutCumulativeCosts();
        computeRolloutProbabilities();
        computeParameterUpdates();

        parameter_updates = parameter_updates_;

        // for (int d=0; d<num_dimensions_; d++)
        // {
        //   cout << "parameter_updates[" << d << "] = " << endl << parameter_updates_[d] << endl;
        // }
        return true;
    }

    /// TODO implement it
    void PolicyImprovement::addParmametersToDraw(const std::vector<Eigen::VectorXd>& parameters, int color)
    {
        /**
        const Move3D::ChompPlanningGroup* planning_group = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup();
        const std::vector<Move3D::ChompDof>& dofs = planning_group->chomp_dofs_;

        Move3D::Trajectory traj( planning_group->robot_ );

        for ( int j=0; j<num_time_steps_; ++j)
        {
            confPtr_t q = traj.getRobot()->getCurrentPos();

            for ( int i=0; i<planning_group->num_dofs_; ++i)
            {
                (*q)[dofs[i].move3d_dof_index_] = parameters[i][j];
            }
            traj.push_back(q);
        }

        //T.print();
        traj.setColor( color );
//        global_trajToDraw.clear();
        global_trajToDraw.push_back( traj );
        **/
    }

    bool PolicyImprovement::covarianceMatrixAdaptaion()
    {
        std::vector<MatrixXd> covariance( num_dimensions_ );

        for (int d=0; d<num_dimensions_; ++d)
        {
            for (int r=0; r<num_rollouts_; ++r)
            {
                covariance[d] += rollouts_[r].probabilities_[d] * ( rollouts_[r].noise_[d] * rollouts_[r].noise_[d].transpose() ) ;
            }
        }

        MatrixXd new_covariance = MatrixXd( num_time_steps_, num_time_steps_ );

        noise_generators_.clear();
        for (int d=0; d<num_dimensions_; ++d)
        {
            MultivariateGaussian mvg( VectorXd::Zero(num_parameters_[d]), new_covariance );
            noise_generators_.push_back(mvg);
        }
        return true;
    }

    bool PolicyImprovement::addExtraRollouts(std::vector<std::vector<Eigen::VectorXd> >& rollouts, std::vector<Eigen::VectorXd>& rollout_costs)
    {
        assert(int(rollouts.size()) == num_rollouts_extra_);

        // update our parameter values, so that the computed noise is correct:
        copyParametersFromPolicy();

        for (int r=0; r<num_rollouts_extra_; ++r)
        {
            extra_rollouts_[r].parameters_ = rollouts[r];
            extra_rollouts_[r].state_costs_ = rollout_costs[r];
            computeNoise(extra_rollouts_[r]);
            // computeProjectedNoise(extra_rollouts_[r]);
            // computeRolloutControlCosts(extra_rollouts_[r]);

            // TODO
            // extra_rollouts_[r].rollout.control_costs_ = rollout_control_costs[r];
        }

        extra_rollouts_added_ = true;
        return true;
    }

    bool PolicyImprovement::computeNoise(Rollout& rollout)
    {
        for (int d=0; d<num_dimensions_; ++d)
        {
            rollout.noise_[d] =  rollout.parameters_[d] - parameters_[d];
        }
        return true;
    }

    void PolicyImprovement::testNoiseGenerators()
    {
        for (int d=0; d<num_dimensions_; ++d)
        {
            // For all dimensions
            const unsigned int N = 10000;

            Eigen::VectorXd sum(VectorXd::Zero(num_parameters_[d]));
            Eigen::MatrixXd samples(N,num_parameters_[d]);

            for (unsigned int i=0; i<N; ++i)
            {
                Eigen::VectorXd tmp(VectorXd::Zero(num_parameters_[d]));

                noise_generators_[d].sample( tmp );

                //cout << "sampler = " << tmp.transpose() << endl;
                samples.row(i) = tmp.transpose();
                sum += tmp;
            }

            Eigen::VectorXd mean((1/((double)N))*sum);
            Eigen::MatrixXd covariance(MatrixXd::Zero(num_parameters_[d],num_parameters_[d]));

            for ( int k=0; k<num_parameters_[d]; ++k)
            {
                for ( int j=0; j<num_parameters_[d]; ++j)
                {
                    for ( int i=0; i<int(N); ++i)
                    {
                        covariance(j,k) += ( samples(i,j) - mean(j) )*( samples(i,k) - mean(k) ) ;
                    }
                }
            }
            covariance /= N;

            cout << "CoV dimension [" << d << "] = " << endl << covariance << endl;
            cout << "Sum dimension [" << d << "] = " << endl << sum.transpose() << endl;
            cout << "Mean dimension [" << d << "] = " << endl << mean.transpose() << endl;
        }
    }
};

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

// system includes
#include <cassert>
#include <stdio.h>
// ros includes
//#include <ros/package.h>
#include "policy_improvement_loop.hpp"
//#include <stomp_motion_planner/assert.h>
//#include "param_server.hpp"
//#include <boost/filesystem.hpp>

#include "stompOptimizer.hpp"

#include "planner/planEnvironment.hpp"

#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Device/generalik.hpp"

#include "TrajectoryOptim/Classic/smoothing.hpp"

#include <libmove3d/p3d/env.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

MOVE3D_USING_BOOST_NAMESPACE

using std::cout;
using std::endl;
using namespace Move3D;

namespace Move3D
{
    const std::string PI_STATISTICS_TOPIC_NAME = std::string("policy_improvement_statistics");

    PolicyImprovementLoop::PolicyImprovementLoop()
        : initialized_(false), policy_iteration_counter_(0)
    {
    }

    PolicyImprovementLoop::~PolicyImprovementLoop()
    {

    }

    bool PolicyImprovementLoop::initialize( bool singleRollout, double discretization )
    {
        //node_handle_ = node_handle;
//        task->getPolicy(policy_);
//        policy_->getNumDimensions(num_dimensions_);
//        policy_->getNumTimeSteps(num_time_steps_);

        printf("Learning policy with %i dimensions.\n", num_dimensions_);

        if( !singleRollout )
        {
            readParameters();
        }
        else {
            readParametersSingleRollout();
        }


//        task_ = task;
//        task_->initialize(/*node_handle_,*/ num_time_steps_);
//        task_->getControlCostWeight( control_cost_weight_ );
//        task_->getStateCostWeight( state_cost_weight_ );


        assert( num_dimensions_ == static_cast<int>(noise_decay_.size()));
        assert( num_dimensions_ == static_cast<int>(noise_stddev_.size()));

        joint_limits_ = false;
        use_annealing_ = false;
        use_cumulative_costs_ = false;
        set_parameters_in_policy_ = true;
        limits_violations_ = 0;
        K_ = 1.0;

        //    int num_extra_rollouts = 2;
        int num_extra_rollouts = 1;

//        cout << "wait for key" << endl;
//        std::cin.ignore();


        ///--------------------------------------------------------------------------

        // ADD A GOAL TO THE POLICY IMPROVEMENT LOOP
/**
        project_last_config_ = PlanEnv->getBool(PlanParam::trajStompMoveEndConfig);
        const Move3D::ChompPlanningGroup* planning_group = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup();

        if( project_last_config_ && ( planning_group != NULL ) )
        {
            ratio_projected_ = 1.0; // 1.0 = all along the trajectory

            Move3D::Robot* robot = planning_group->robot_;
            eef_ = robot->getJoint( PlanEnv->getString(PlanParam::end_effector_joint) ); // plannar J4

            if( eef_ != NULL )
            {
                Move3D::confPtr_t q_goal = robot->getGoalPos();
                robot->setAndUpdate(*q_goal);
                x_task_goal_ = eef_->getVectorPos().head(3); // head(2) only (x,y)
            }
            else {
                project_last_config_ = false;
                x_task_goal_ = Eigen::VectorXd::Zero(0);
            }
        }
**/
        ///--------------------------------------------------------------------------

        // INITIALIZE POLICY IMPROVEMENT LOOP
        policy_improvement_.initialize( num_rollouts_, num_time_steps_, num_reused_rollouts_,
                                        num_extra_rollouts, /*policy_, task_,*/ discretization, use_cumulative_costs_ );


        tmp_rollout_cost_ = Eigen::VectorXd::Zero( num_time_steps_ );
        rollout_costs_ = Eigen::MatrixXd::Zero( num_rollouts_, num_time_steps_ );

        rollout_control_costs_.resize( num_rollouts_ );
        for(int r=0; r<num_rollouts_; r++)
            rollout_control_costs_[r] = Eigen::MatrixXd::Zero( num_dimensions_, num_time_steps_ );

        reused_rollouts_.clear();

        policy_iteration_counter_ = 0;

        int nb_parallel_rollouts = static_pointer_cast<StompOptimizer>(task_)->getCostComputers().size();
        parallel_cost_.resize( nb_parallel_rollouts, Eigen::VectorXd::Zero(num_time_steps_) );
        parrallel_is_rollout_running_.resize( nb_parallel_rollouts );
        threads_.resize( nb_parallel_rollouts );

        printf("num_rollouts_ : %d, num_reused_rollouts_ : %d\n", num_rollouts_, num_reused_rollouts_);
        return (initialized_ = true);
    }

    bool PolicyImprovementLoop::readParametersSingleRollout()
    {
        num_rollouts_ = 10;
        num_reused_rollouts_ = 5;

        noise_decay_.clear();
        noise_decay_.resize(num_dimensions_,.99);

        // noise is now recomputed dynamicaly
        noise_stddev_.clear();
        noise_stddev_.resize(num_dimensions_,PlanEnv->getDouble(PlanParam::trajOptimStdDev));

        write_to_file_ = false; // defaults are sometimes good!
        use_cumulative_costs_ =  false;
        return true;
    }

    bool PolicyImprovementLoop::readParameters()
    {
        // assert(stomp_motion_planner::read(node_handle_, std::string("num_rollouts"), num_rollouts_));
        // assert(stomp_motion_planner::read(node_handle_, std::string("num_reused_rollouts"), num_reused_rollouts_));
        // assert(stomp_motion_planner::read(node_handle_, std::string("num_time_steps"), num_time_steps_));
        //
        // assert(stomp_motion_planner::readDoubleArray(node_handle_, "noise_stddev", noise_stddev_));
        // assert(stomp_motion_planner::readDoubleArray(node_handle_, "noise_decay", noise_decay_));
        // node_handle_.param("write_to_file", write_to_file_, true); // defaults are sometimes good!
        // node_handle_.param("use_cumulative_costs", use_cumulative_costs_, true);

        num_rollouts_ = 10;
        num_reused_rollouts_ = 5;
        //num_time_steps_ = 51;


        noise_decay_.clear();
        noise_decay_.resize( num_dimensions_, .99 );

        // noise is now recomputed dynamicaly
        noise_stddev_.clear();
        noise_stddev_.resize( num_dimensions_, PlanEnv->getDouble(PlanParam::trajOptimStdDev) );

        // noise_stddev
        // noise_decay

        write_to_file_ = false; // defaults are sometimes good!
        use_cumulative_costs_ =  true;

        return true;
    }

    bool PolicyImprovementLoop::readPolicy(const int iteration_number)
    {
        // check whether reading the policy from file is neccessary
        if(iteration_number == (policy_iteration_counter_))
        {
            return true;
        }
        /*    ROS_INFO("Read policy from file %s.", policy_->getFileName(iteration_number).c_str());
     assert(policy_->readFromDisc(policy_->getFileName(iteration_number)));
     assert(task_->setPolicy(policy_));
     */
        return true;
    }

    bool PolicyImprovementLoop::writePolicy(const int iteration_number, bool is_rollout, int rollout_id)
    {
        return true;
    }

    //------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------
    bool PolicyImprovementLoop::setParallelRolloutsEnd(int r)
    {
        mtx_set_end_.lock();
        parrallel_is_rollout_running_[r] = false;

        for( int i=0; i<int(parrallel_is_rollout_running_.size()); i++)
        {
            if( parrallel_is_rollout_running_[i] )
            {
                mtx_set_end_.unlock();
                return true;
            }
        }

        mtx_end_.unlock();
        mtx_set_end_.unlock();
        return false;
    }

    void PolicyImprovementLoop::parallelRollout(int r, int iteration_number )
    {
        costComputation* traj_evaluation  = static_pointer_cast<StompOptimizer>(task_)->getCostComputers()[r];
        traj_evaluation->getCost( rollouts_[r], parallel_cost_[r], iteration_number, joint_limits_, false, true );

        // Get costs
        rollout_costs_.row(r) = parallel_cost_[r].transpose();
        for(int d=0; d<num_dimensions_; d++)
            rollout_control_costs_[r].row(d) = traj_evaluation->getControlCosts()[d];

        policy_improvement_.setRolloutOutOfBounds( r, !traj_evaluation->getJointLimitViolationSuccess() );

        setParallelRolloutsEnd( r );
    }

    void PolicyImprovementLoop::executeRollout(int r, int iteration_number )
    {
        if( r < int(parrallel_is_rollout_running_.size())  )
        {
            parrallel_is_rollout_running_[r] = true;
            // cout << "spawns thread : " << r << endl;
            threads_[r] = new boost::thread( &PolicyImprovementLoop::parallelRollout, this, r, iteration_number );
        }
        else
        {
            shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);
            optimizer->execute( rollouts_[r], tmp_rollout_cost_, iteration_number, joint_limits_, false, true );

            // Get costs
            rollout_costs_.row(r) = tmp_rollout_cost_.transpose();
            for(int d=0; d<num_dimensions_; d++)
                rollout_control_costs_[r].row(d) = optimizer->getMainCostComputer()->getControlCosts()[d];

            policy_improvement_.setRolloutOutOfBounds( r, !optimizer->getMainCostComputer()->getJointLimitViolationSuccess() );

            if( use_annealing_ )
            {
                limits_violations_ += !optimizer->getMainCostComputer()->getJointLimitViolationSuccess();
            }
        }
    }

    //------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------
    bool PolicyImprovementLoop::runSingleIteration(const int iteration_number)
    {
        assert(initialized_);
        policy_iteration_counter_++;

        task_->getControlCostWeight( control_cost_weight_ );
        task_->getStateCostWeight( state_cost_weight_ );

        if( write_to_file_ )
        {
            // load new policy if neccessary
            readPolicy( iteration_number );
        }

        // compute appropriate noise values
        std::vector<double> noise;
        noise.resize(num_dimensions_);
        for (int i=0; i<num_dimensions_; ++i)
        {
            // noise[i] = noise_stddev_[i] /* K_ * pow(noise_decay_[i], iteration_number-1)*/;
            noise[i] = PlanEnv->getDouble(PlanParam::trajOptimStdDev);
            //cout << "noise_stddev_[" << i << "] = " << noise_stddev_[i] << endl;
            //cout << "noise_stddev = " << noise[i] << endl;
        }

        // get rollouts and execute them
        bool get_reused_ones = true;
        policy_improvement_.getRollouts( rollouts_, noise, get_reused_ones, reused_rollouts_ );


        //printRollouts();

        if( !parrallel_is_rollout_running_.empty() )
            mtx_end_.lock();

        for (int r=0; r<int(rollouts_.size()); ++r)
        {
            executeRollout ( r, iteration_number );
        }

        if( !parrallel_is_rollout_running_.empty() )
        {
            mtx_end_.lock();
            mtx_end_.unlock();

            for(int r=0;r<int(threads_.size());r++)
            {
                threads_[r]->join();
                delete threads_[r];
            }

            //cout << rollout_costs_ << endl;
            //cout << "end parallel computing" << endl;
        }



        // TODO: fix this std::vector<>
        std::vector<double> all_costs;
        //cout << "control_cost_weight_ : " << control_cost_weight_ << endl;

        // Improve the policy
        policy_improvement_.setRollouts( rollouts_ ); // TODO see if we need to set noise etc...

        //cout << "Run single interation of stomp " << endl;
        if ( ENV.getBool(Env::drawTrajVector) )
            addRolloutsToDraw( get_reused_ones );

        policy_improvement_.setRolloutCosts( rollout_costs_, rollout_control_costs_, control_cost_weight_, all_costs );
        policy_improvement_.improvePolicy( parameter_updates_ );

        policy_->updateParameters( parameter_updates_ );

        // Get the parameters from the policy
        // Warning joint limits
        policy_->getParameters( parameters_ );

        int num_extra_rollouts = 1;
        std::vector<std::vector<Eigen::VectorXd> > extra_rollout;
        std::vector<Eigen::VectorXd> extra_rollout_cost;
        extra_rollout.resize( num_extra_rollouts );
        extra_rollout_cost.resize( num_extra_rollouts );

        //    addStraightLineRollout( extra_rollout, extra_rollout_cost );

        // Get the trajectory cost
        bool resample = false; //!PlanEnv->getBool( PlanParam::trajStompMultiplyM );
        // Warning!!!!
        // not return the modified trajectory when is out of bounds
        task_->execute( parameters_, tmp_rollout_cost_, iteration_number, true, resample, false );

        // PROJECT TO CONSTRAINT
        if( project_last_config_ )
        {
            projectToConstraints( parameters_ );
            task_->execute( parameters_, tmp_rollout_cost_, iteration_number, true, resample, false );
        }
        //printf("Noiseless cost = %lf\n", stats_msg.noiseless_cost);

        // Only set parameters for the changed chase
//        if( set_parameters_in_policy_ )
//        {
//            cout << "set parameters to the policy" << endl;
//            policy_->setParameters( parameters_ );
//        }

        // add the noiseless rollout into policy_improvement:
        extra_rollout[num_extra_rollouts-1] = parameters_;
        extra_rollout_cost[num_extra_rollouts-1] = tmp_rollout_cost_;
        policy_improvement_.addExtraRollouts( extra_rollout, extra_rollout_cost );

        //cout << "rollout_cost_ = " << tmp_rollout_cost_.sum() << endl;

        //    if (write_to_file_)
        //    {
        // store updated policy to disc
        // assert(writePolicy(iteration_number));
        // assert(writePolicyImprovementStatistics(stats_msg));
        //    }

        return true;
    }

    void PolicyImprovementLoop::resetReusedRollouts()
    {
        policy_improvement_.resetReusedRollouts();
    }

    void PolicyImprovementLoop::projectToConstraints( std::vector<Eigen::VectorXd>& parameters )
    {
        const Move3D::ChompPlanningGroup* planning_group = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup();
        const std::vector<ChompDof>& joints = planning_group->chomp_dofs_;
        Robot* robot = planning_group->robot_;

        Move3D::confPtr_t q_end = robot->getCurrentPos();
        for ( int i=0; i<planning_group->num_dofs_; ++i)
            (*q_end)[joints[i].move3d_dof_index_] = parameters[i][num_time_steps_-1];

        Move3D::GeneralIK ik( robot );
        robot->setAndUpdate( *q_end );

        ik.initialize( planning_group->getActiveJoints(), eef_ );

        // cout << "x_task_goal_.size() : " << x_task_goal_.size() << endl;

//        ik.solve( x_task_goal_ );
//        Move3D::confPtr_t q_cur = robot->getCurrentPos();
//        const std::vector<int> active_dofs = ik.getActiveDofs();
//        Eigen::VectorXd q_1 = q_end->getEigenVector( active_dofs );
//        Eigen::VectorXd q_2 = q_cur->getEigenVector( active_dofs );
//        Eigen::VectorXd dq = q_2 - q_1;

        // Get dq through J+
        ik.magnitude_ = 1.0;
        Eigen::VectorXd dq = ik.single_step_joint_limits( x_task_goal_ );

        int start = double(1. - ratio_projected_) * num_time_steps_;
        double alpha = 0.0;
        double delta = 1.0 / double(num_time_steps_-start);
        for( int j=start; j<num_time_steps_; j++ )
        {
            for ( int i=0; i<parameters.size(); ++i)
                parameters[i][j] += alpha * dq[i];

            alpha += delta;
        }
    }

    void PolicyImprovementLoop::addStraightLineRollout(std::vector<std::vector<Eigen::VectorXd> >& extra_rollout,
                                                       std::vector<Eigen::VectorXd>& extra_rollout_cost)
    {
        shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);
        shared_ptr<CovariantTrajectoryPolicy> policy = static_pointer_cast<CovariantTrajectoryPolicy>(policy_);

        const std::vector<Move3D::ChompDof>& joints = optimizer->getPlanningGroup()->chomp_dofs_;

        Eigen::MatrixXd parameters( num_dimensions_, num_time_steps_ );

        for ( int i=0; i<num_dimensions_; ++i) {
            parameters.row(i) = parameters_[i].transpose();
        }

        std::vector<Move3D::confPtr_t> confs(2);
        confs[0] = optimizer->getPlanningGroup()->robot_->getCurrentPos();
        confs[1] = optimizer->getPlanningGroup()->robot_->getCurrentPos();

        for ( int i=0; i<optimizer->getPlanningGroup()->num_dofs_; ++i)
            (*confs[0])[joints[i].move3d_dof_index_] = parameters(i,0);


        for ( int i=0; i<optimizer->getPlanningGroup()->num_dofs_; ++i)
            (*confs[1])[joints[i].move3d_dof_index_] = parameters(i,num_time_steps_-1);


        Move3D::Trajectory traj(confs);

        double step = traj.getParamMax() / num_time_steps_;
        double param = step;
        for ( int j=1; j<num_time_steps_-1; ++j)
        {
            confPtr_t q = traj.configAtParam(param);

            for ( int i=0; i<optimizer->getPlanningGroup()->num_dofs_; ++i )
                parameters(i,j) = (*q)[joints[i].move3d_dof_index_];

            param += step;
        }

        extra_rollout[0].resize( num_dimensions_);

        for ( int i=0; i<num_dimensions_; ++i) {
            extra_rollout[0][i] = parameters.row(i);
        }

        int iteration_number=1;
        task_->execute(extra_rollout[0], tmp_rollout_cost_, iteration_number, false, false, false );
        //    cout << "Cost" << tmp_rollout_cost_[0] << endl;

        extra_rollout_cost[0] = tmp_rollout_cost_;
    }

    //------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------
    void PolicyImprovementLoop::resampleParameters()
    {
        shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);
        shared_ptr<CovariantTrajectoryPolicy> policy = static_pointer_cast<CovariantTrajectoryPolicy>(policy_);

        const std::vector<Move3D::ChompDof>& joints = optimizer->getPlanningGroup()->chomp_dofs_;
        Move3D::Smoothing traj( optimizer->getPlanningGroup()->robot_ );
        Eigen::MatrixXd parameters( num_dimensions_, num_time_steps_ );

        for ( int i=0; i<num_dimensions_; ++i) {
            parameters.row(i) = parameters_[i].transpose();
        }
        /**
    double dt = policy->movement_dt_;
    vector< pair<int,int> > pair_of_config;

    for ( int i=0; i<num_time_steps_-1; i++)
    {
      for ( int j=i+1; j<num_time_steps_; j++)
      {
        Eigen::VectorXd q1 = parameters.col(i);
        Eigen::VectorXd q2 = parameters.col(j);

        if( ( q1-q2 ).norm() < dt/2 )
        {
          pair_of_config.push_back(make_pair(i,j));
        }
      }
    }
    cout << "Nb of nodes to removed : " << pair_of_config.size() << endl;

    vector<int> points;
    for ( int i=0; i<int(pair_of_config.size()); i++)
    {
      points.push_back( pair_of_config[i].first );

      for ( int j=i+1; j<int(pair_of_config.size()); j++)
      {
        if( pair_of_config[i].second > pair_of_config[j].first )
        {
          points.push_back( pair_of_config[j].first );
        }
      }
    }
    cout << "Really removed : " << points.size() << endl;
    */
        traj.clear();
        for ( int j=0; j<num_time_steps_; ++j)
        {
            /**
      bool continue_loop = false;
      for ( int i=0; i<int(points.size()); i++)
      {
        if (points[i] == j) {
          continue_loop = true;
          break;
        }
      }

      if( continue_loop ) {
        continue;
      }
      */

            confPtr_t q = optimizer->getPlanningGroup()->robot_->getCurrentPos();

            for ( int i=0; i<optimizer->getPlanningGroup()->num_dofs_; ++i)
            {
                (*q)[joints[i].move3d_dof_index_] = parameters(i,j);
            }
            traj.push_back(q);
        }

        traj.runShortCut(15);

        double step = traj.getParamMax() / num_time_steps_;
        double param = step;
        for ( int j=1; j<num_time_steps_-1; ++j)
        {
            confPtr_t q = traj.configAtParam(param);

            for ( int i=0; i<optimizer->getPlanningGroup()->num_dofs_; ++i )
            {
                parameters(i,j) = (*q)[joints[i].move3d_dof_index_];
            }
            param += step;
        }

        for ( int i=0; i<num_dimensions_; ++i) {
            parameters_[i].transpose() = parameters.row(i);
        }
    }


    //------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------
    void PolicyImprovementLoop::parametersToVector(std::vector<Eigen::VectorXd>& rollout)
    {
        rollout.clear();
        for ( int i=0; i<num_dimensions_;i++)
        {
            rollout.push_back( parameter_updates_[i].row(0) );
        }
    }

    //------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------
    void PolicyImprovementLoop::testSampler()
    {
        policy_improvement_.testNoiseGenerators();
    }

    //------------------------------------------------------------------------------------
    //------------------------------------------------------------------------------------

    void PolicyImprovementLoop::getRollouts(std::vector<std::vector<confPtr_t> >& traj)
    {
        shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);

        traj.clear();
        traj.resize( rollouts_.size() + reused_rollouts_.size() );

        for ( int k=0; k<int(rollouts_.size()); ++k)
        {
            getSingleRollout( rollouts_[k], traj[k] );
        }

        for ( int k=0; k<int(reused_rollouts_.size()); ++k)
        {
            getSingleRollout( reused_rollouts_[k], traj[k] );
        }
    }

    void PolicyImprovementLoop::getSingleRollout(const std::vector<Eigen::VectorXd>& rollout, std::vector<confPtr_t>& traj)
    {
        shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);
        const std::vector<ChompDof>& joints = optimizer->getPlanningGroup()->chomp_dofs_;
        Robot* robot = optimizer->getPlanningGroup()->robot_;

        traj.clear();

        //    for ( int d=0; d<int(rollout.size()); ++d)
        //    {
        //      cout << "rollout[" << d << "] : " << rollout[d].transpose() << endl;
        //    }

        for ( int j=0; j<num_time_steps_; ++j)
        {
            confPtr_t q = robot->getCurrentPos();

            for ( int i=0; i<optimizer->getPlanningGroup()->num_dofs_; ++i)
            {
                (*q)[joints[i].move3d_dof_index_] = rollout[i][j];
            }
            traj.push_back(q);
        }
    }

    void PolicyImprovementLoop::addSingleRolloutsToDraw(const std::vector<Eigen::VectorXd>& rollout, int color)
    {
        std::vector<confPtr_t> traj;
        getSingleRollout( rollout, traj );

        Move3D::Trajectory T(static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup()->robot_);
        for ( int i=0; i<int(traj.size()); ++i )
        {
            T.push_back( traj[i] );
        }
        //T.print();
        T.setColor( color );
        global_trajToDraw.push_back( T );
    }

    void PolicyImprovementLoop::addRolloutsToDraw(bool add_reused)
    {
        global_trajToDraw.clear();
        //shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);

        cout << "Add rollouts to draw" << endl;

        for ( int k=0; k<int(rollouts_.size()); ++k)
        {
//            cout << "Add rollout(" << k << ") to draw" << endl;
            addSingleRolloutsToDraw( rollouts_[k], k );
        }

        for ( int k=0; k<int(reused_rollouts_.size()); ++k)
        {
//            cout << "Add reused rollout(" << k << ") to draw" << endl;
            addSingleRolloutsToDraw( reused_rollouts_[k], k+int(rollouts_.size()) );
        }
    }

    void PolicyImprovementLoop::printSingleRollout( const std::vector<Eigen::VectorXd>& rollout, int id ) const
    {
        for( int i=0; i<int(rollout.size()); i++ )
        {
            cout << "rollout[" << id << "]" << rollout[i].transpose() << endl;
        }
    }

    void PolicyImprovementLoop::printRollouts() const
    {
        int id=0;

        for ( int k=0; k<int(rollouts_.size()); ++k)
        {
            printSingleRollout( rollouts_[k], id++ );
        }

        for ( int k=0; k<int(reused_rollouts_.size()); ++k)
        {
            printSingleRollout( reused_rollouts_[k], id++ );
        }
    }
}


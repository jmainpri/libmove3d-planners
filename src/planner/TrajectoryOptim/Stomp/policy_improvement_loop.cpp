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

namespace stomp_motion_planner
{
    const std::string PI_STATISTICS_TOPIC_NAME = std::string("policy_improvement_statistics");

    PolicyImprovementLoop::PolicyImprovementLoop()
        : initialized_(false), policy_iteration_counter_(0)
    {
    }

    PolicyImprovementLoop::~PolicyImprovementLoop()
    {

    }

    bool PolicyImprovementLoop::initialize(MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Task> task, bool singleRollout, double discretization )
    {
        //node_handle_ = node_handle;
        task->getPolicy(policy_);
        policy_->getNumDimensions(num_dimensions_);
        policy_->getNumTimeSteps(num_time_steps_);

        printf("Learning policy with %i dimensions.\n", num_dimensions_);

        if( !singleRollout )
        {
            readParameters();
        }
        else {
            readParametersSingleRollout();
        }
        task_ = task;
        task_->initialize(/*node_handle_,*/ num_time_steps_);
        task_->getControlCostWeight( control_cost_weight_ );
        task_->getStateCostWeight( state_cost_weight_ );


        assert( num_dimensions_ == static_cast<int>(noise_decay_.size()));
        assert( num_dimensions_ == static_cast<int>(noise_stddev_.size()));

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

        project_last_config_ = PlanEnv->getBool(PlanParam::trajStompMoveEndConfig);
        const Move3D::ChompPlanningGroup* planning_group = static_pointer_cast<StompOptimizer>(task_)->getPlanningGroup();

        if( project_last_config_ && ( planning_group != NULL ) )
        {
            ratio_projected_ = 1.0; // 1.0 = all along the trajectory

            Move3D::Robot* robot = planning_group->robot_;
            eef_ = robot->getJoint( "J4" ); // plannar J4

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

        ///--------------------------------------------------------------------------

        // INITIALIZE POLICY IMPROVEMENT LOOP
        policy_improvement_.initialize( num_rollouts_, num_time_steps_, num_reused_rollouts_,
                                        num_extra_rollouts, policy_, task_, discretization, use_cumulative_costs_ );


        tmp_rollout_cost_ = Eigen::VectorXd::Zero( num_time_steps_ );
        rollout_costs_ = Eigen::MatrixXd::Zero( num_rollouts_, num_time_steps_ );

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

    bool PolicyImprovementLoop::generateSingleNoisyTrajectory()
    {
        int iteration_number = 0;

        assert( initialized_ );
        policy_iteration_counter_++;

        // compute appropriate noise values
        std::vector<double> noise;
        noise.resize( num_dimensions_ );
        for( int i=0; i<num_dimensions_; ++i )
        {
            noise[i] = noise_stddev_[i];
        }

        // get rollouts and execute them
        const bool get_reused_ones = false;
        policy_improvement_.getRollouts(rollouts_, noise, get_reused_ones, reused_rollouts_);

        if( ENV.getBool(Env::drawTrajVector) )
            addRolloutsToDraw(get_reused_ones);

        if( use_annealing_ )
            limits_violations_ = 0;

        shared_ptr<StompOptimizer> optimizer = static_pointer_cast<StompOptimizer>(task_);

        for (int r=0; r<int(rollouts_.size()); ++r)
        {
            task_->execute(rollouts_[r], tmp_rollout_cost_, iteration_number, true, false, false );

            rollout_costs_.row(r) = tmp_rollout_cost_.transpose();
            //printf("Rollout %d, cost = %lf\n", r+1, tmp_rollout_cost_.sum());

            policy_improvement_.setRolloutOutOfBounds( r, !optimizer->getJointLimitViolationSuccess() );
        }

        std::vector<double> all_costs;
        policy_improvement_.setRolloutCosts( rollout_costs_, control_cost_weight_, all_costs );

        // improve the policy
        // get a noise-less rollout to check the cost
        policy_improvement_.improvePolicy(parameter_updates_);
        policy_->updateParameters(parameter_updates_);
        policy_->getParameters(parameters_);

        // get the trajectory cost
        assert(task_->execute(parameters_, tmp_rollout_cost_, iteration_number, true, false, false ));

        // add the noiseless rollout into policy_improvement:
        std::vector<std::vector<Eigen::VectorXd> > extra_rollout;
        std::vector<Eigen::VectorXd> extra_rollout_cost;
        extra_rollout.resize(1);
        extra_rollout_cost.resize(1);
        extra_rollout[0] = parameters_;
        extra_rollout_cost[0] = tmp_rollout_cost_;
        policy_improvement_.addExtraRollouts(extra_rollout, extra_rollout_cost);
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

        traj_evaluation->getCost( rollouts_[r], parallel_cost_[r] );
        rollout_costs_.row(r) = parallel_cost_[r].transpose();
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
            bool joint_limits = false;
            optimizer->execute( rollouts_[r], tmp_rollout_cost_, iteration_number, joint_limits, false, true );
            rollout_costs_.row(r) = tmp_rollout_cost_.transpose();
            policy_improvement_.setRolloutOutOfBounds( r, !optimizer->getJointLimitViolationSuccess() );

            if( use_annealing_ )
            {
                limits_violations_ += optimizer->getJointLimitViolations();
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

        //cout << "Run single interation of stomp " << endl;
        if ( ENV.getBool(Env::drawTrajVector) )
            addRolloutsToDraw( get_reused_ones );


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
        policy_improvement_.setRolloutCosts( rollout_costs_, control_cost_weight_, all_costs );
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
        bool resample = !PlanEnv->getBool( PlanParam::trajStompMultiplyM );
        // Warning!!!!
        // not return the modified trajectory when is out of bounds
        bool joint_limits = true;
        task_->execute( parameters_, tmp_rollout_cost_, iteration_number, joint_limits, resample, false );

        // PROJECT TO CONSTRAINT
        if( project_last_config_ )
        {
            projectToConstraints( parameters_ );
            bool joint_limits = true;
            task_->execute( parameters_, tmp_rollout_cost_, iteration_number, joint_limits, resample, false );
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
        Move3D::Smoothing traj(optimizer->getPlanningGroup()->robot_);
        Eigen::MatrixXd parameters(num_dimensions_,num_time_steps_);

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

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

#include "stompOptimizer.hpp"
//#include <ros/ros.h>
//#include <visualization_msgs/MarkerArray.h>
#include "Chomp/chompUtils.hpp"
#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API
#include <Eigen/LU>

#include "run_parallel_stomp.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

#include "API/project.hpp"
#include "utils/ConfGenerator.h"
#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/HRICS_Legibility.hpp"
#include "hri_costspace/Gestures/HRICS_HumanPredictionCostSpace.hpp"

#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Util-pkg.h"

#include "move3d-headless.h"

#include <boost/shared_ptr.hpp>
#include <sys/time.h>

using namespace std;
USING_PART_OF_NAMESPACE_EIGEN

const double hack_tweek = 1;

//--------------------------------------------------------
// External
//--------------------------------------------------------
MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompOptimizer> global_optimizer;

std::map< Robot*, std::vector<double> >             global_MultiStomplinesColors;
std::map< Robot*, std::vector<Eigen::Vector3d> >    global_MultiStomplinesToDraw;

namespace stomp_motion_planner
{
StompOptimizer::StompOptimizer(ChompTrajectory *trajectory,
                               const StompParameters *parameters,
                               const ChompPlanningGroup *planning_group,
                               const CollisionSpace *collision_space) :
    use_time_limit_(false),
    use_iteration_limit_(false),
    full_trajectory_(trajectory),
    planning_group_(planning_group),
    stomp_parameters_(parameters),
    collision_space_(collision_space),
    group_trajectory_(*full_trajectory_, DIFF_RULE_LENGTH)
{
    initialize();
}

void StompOptimizer::initialize()
{
    Scene* sce = global_Project->getActiveScene();
    
    //    robot_model_ = group_trajectory_.getRobot();
    robot_model_ = planning_group_->robot_;
    human_model_ = sce->getRobotByNameContaining("HERAKLES");
    
    // Handover end configuration generator
    use_human_sliders_ = PlanEnv->getBool( PlanParam::trajMoveHuman );
    use_handover_config_generator_ = PlanEnv->getBool( PlanParam::trajUseOtp );
    use_handover_config_list_ = false;
    use_handover_auto_ = true;
    recompute_handover_cell_list_ = true;
    handoverGenerator_ = NULL;
    
    human_has_moved_ = false;
    last_human_pos_.resize(4);
    last_human_pos_[0] = 0;
    last_human_pos_[1] = 0;
    last_human_pos_[2] = 0;
    last_human_pos_[3] = 0;
    
    reset_reused_rollouts_ = false;
    
    if( use_handover_config_generator_ )
    {
        initHandover();
    }

    // TODO move it somewhere else
    if( PlanEnv->getBool(PlanParam::useLegibleCost) )
    {
        HRICS_activeLegi = new HRICS::Legibility();
        Robot* goal_0 = global_Project->getActiveScene()->getRobotByName("GOAL_0");
        Robot* goal_1 = global_Project->getActiveScene()->getRobotByName("GOAL_1");
        cout <<"goal 0 : " << goal_0->getCurrentPos()->getEigenVector(6,7).transpose() << endl;
        cout <<"goal 1 : " << goal_1->getCurrentPos()->getEigenVector(6,7).transpose() << endl;
        HRICS_activeLegi->addGoal( goal_0->getCurrentPos()->getEigenVector(6,7) );
        HRICS_activeLegi->addGoal( goal_1->getCurrentPos()->getEigenVector(6,7) );
    }

    // Use costspace
    // use_costspace_ = ( collision_space_== NULL );
    
    //clearAnimations();
    
    // init some variables:
    num_vars_free_ = group_trajectory_.getNumFreePoints();
    num_vars_all_ = group_trajectory_.getNumPoints();
    num_joints_ = group_trajectory_.getNumJoints();
    cout << "num_vars_free_ : " << num_vars_free_ << endl;
    cout << "num_vars_all_ : " << num_vars_all_ << endl;
    
    free_vars_start_ = group_trajectory_.getStartIndex();
    free_vars_end_ = group_trajectory_.getEndIndex();
    
    //ROS_INFO_STREAM("Setting free vars start to " << free_vars_start_ << " end " << free_vars_end_);
    
    // set up joint index:
    group_joint_to_move3d_joint_index_.resize(num_joints_);
    for (int i=0; i<num_joints_; ++i)
    {
        group_joint_to_move3d_joint_index_[i] = planning_group_->chomp_joints_[i].move3d_joint_index_;
    }
    
    num_collision_points_ = planning_group_->collision_points_.size();
    
    // set up the joint costs:
    joint_costs_.reserve(num_joints_);
    
    double max_cost_scale = 0.0;
    //  ros::NodeHandle nh("~");
    for (int i=0; i<num_joints_; i++)
    {
        double joint_cost = 1.0;
        std::string joint_name = planning_group_->chomp_joints_[i].joint_name_;
        //    nh.param("joint_costs/"+joint_name, joint_cost, 1.0);
        std::vector<double> derivative_costs(3);
        derivative_costs[0] = joint_cost*stomp_parameters_->getSmoothnessCostVelocity();
        derivative_costs[1] = joint_cost*stomp_parameters_->getSmoothnessCostAcceleration();
        derivative_costs[2] = joint_cost*stomp_parameters_->getSmoothnessCostJerk();
        //        cout << "deriv cost for joint(" << i << ") : " << derivative_costs[0] << " " << derivative_costs[1] << " " << derivative_costs[2] << endl;

        joint_costs_.push_back(ChompCost(group_trajectory_, i, derivative_costs, stomp_parameters_->getRidgeFactor()));
        double cost_scale = joint_costs_[i].getMaxQuadCostInvValue();
        if (max_cost_scale < cost_scale)
            max_cost_scale = cost_scale;
    }
    
    // scale the smoothness costs
    for (int i=0; i<num_joints_; i++)
    {
        joint_costs_[i].scale(max_cost_scale);
    }
    
    general_cost_potential_ = Eigen::VectorXd::Zero(num_vars_all_);
    
    // allocate memory for matrices:
    smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
    collision_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
    final_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
    smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
    jacobian_ = Eigen::MatrixXd::Zero(3, num_joints_);
    jacobian_pseudo_inverse_ = Eigen::MatrixXd::Zero(num_joints_, 3);
    jacobian_jacobian_tranpose_ = Eigen::MatrixXd::Zero(3, 3);
    random_state_ = Eigen::VectorXd::Zero(num_joints_);
    joint_state_velocities_ = Eigen::VectorXd::Zero(num_joints_);
    joint_state_accelerations_ = Eigen::VectorXd::Zero(num_joints_);
    
    // SEE IMPORTANT
    //  full_joint_state_velocities_ = Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());
    //  full_joint_state_accelerations_ = Eigen::VectorXd::Zero(robot_model_->getKDLTree()->getNrOfJoints());
    
    group_trajectory_backup_ = group_trajectory_.getTrajectory();
    best_group_trajectory_ = group_trajectory_.getTrajectory();
    
    joint_axis_eigen_.resize(num_vars_all_);
    joint_pos_eigen_.resize(num_vars_all_);
    collision_point_pos_eigen_.resize(num_vars_all_);
    collision_point_vel_eigen_.resize(num_vars_all_);
    collision_point_acc_eigen_.resize(num_vars_all_);
    
    segment_frames_.resize( num_vars_all_ );
    
    for(int i=0; i<num_vars_all_;i++)
    {
        segment_frames_[i].resize(planning_group_->num_joints_);
        joint_pos_eigen_[i].resize(planning_group_->num_joints_);
    }
    
    if (num_collision_points_ > 0)
    {
        collision_point_potential_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_vel_mag_ = Eigen::MatrixXd::Zero(num_vars_all_, num_collision_points_);
        collision_point_potential_gradient_.resize(num_vars_all_, std::vector<Eigen::Vector3d>(num_collision_points_));

        for(int i=0; i<num_vars_all_;i++)
        {
            joint_axis_eigen_[i].resize(num_collision_points_);
            joint_pos_eigen_[i].resize(num_collision_points_);
            collision_point_pos_eigen_[i].resize(num_collision_points_);
            collision_point_vel_eigen_[i].resize(num_collision_points_);
            collision_point_acc_eigen_[i].resize(num_collision_points_);
        }
    }
    
    collision_free_iteration_ = 0;
    is_collision_free_ = false;
    state_is_in_collision_.resize(num_vars_all_);
    point_is_in_collision_.resize(num_vars_all_, std::vector<int>(num_collision_points_));
    
    last_improvement_iteration_ = -1;
    
    // HMC initialization:
    momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
    random_momentum_ = Eigen::MatrixXd::Zero(num_vars_free_, num_joints_);
    random_joint_momentum_ = Eigen::VectorXd::Zero(num_vars_free_);
    multivariate_gaussian_.clear();
    stochasticity_factor_ = 1.0;
    for (int i=0; i<num_joints_; i++)
    {
        multivariate_gaussian_.push_back(MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_), joint_costs_[i].getQuadraticCostInverse()));
    }
    
    // animation init:
    //  animate_endeffector_segment_number_ = robot_model_->getForwardKinematicsSolver()->segmentNameToIndex(stomp_parameters_->getAnimateEndeffectorSegment());
    
    // initialize the policy
    initPolicy();
    
    // initialize the constraints:
    //  for (int i=0; i<int(constraints_.orientation_constraints.size()); ++i)
    //  {
    //    MOVE3D_BOOST_PTR_NAMESPACE<OrientationConstraintEvaluator> eval(new OrientationConstraintEvaluator(
    //        constraints_.orientation_constraints[i], *robot_model_));
    //    constraint_evaluators_.push_back(eval);
    //  }
}

bool StompOptimizer::initializeFromNewTrajectory(const API::Trajectory& traj)
{
    cout << "initialize stomp from new trajectory" << endl;

    if( traj.getRobot()->getName() != robot_model_->getName() ){
        cout << "Can not initialize stomp with this trajectory" << endl;
        return false;
    }
    
    double s(0.0), param_max(traj.getRangeMax()), step(param_max/num_vars_free_);
    
    vector<confPtr_t> configs;
    
    for(int i=0;i<num_vars_free_;i++)
    {
        configs.push_back( traj.configAtParam( s ) );
        s += step;
    }
    
    API::Trajectory T( configs );

    delete full_trajectory_;
    full_trajectory_ = new ChompTrajectory( T, DIFF_RULE_LENGTH, *planning_group_);
    group_trajectory_= ChompTrajectory(*full_trajectory_,DIFF_RULE_LENGTH);
    
    // initialize the policy trajectory
    Eigen::VectorXd start = group_trajectory_.getTrajectoryPoint(free_vars_start_-1).transpose();
    Eigen::VectorXd end = group_trajectory_.getTrajectoryPoint(free_vars_end_+1).transpose();
    
    // set paramters
    int free_vars_start_index = DIFF_RULE_LENGTH - 1;
    vector< Eigen::VectorXd > parameters(num_joints_);
    for (int i=0; i<num_joints_; i++)
    {
        parameters[i] =  group_trajectory_.getJointTrajectory(i).segment(free_vars_start_index, num_vars_free_);
    }
    policy_->setToMinControlCost( start, end );
    policy_->setParameters( parameters );
    
    if( use_handover_config_generator_ )
    {
        initHandover();
    }
    
    return true;
}

void StompOptimizer::initHandover()
{
    recompute_handover_once_ = true;
    handover_has_been_recomputed_ = false;
    
    use_human_sliders_ = PlanEnv->getBool(PlanParam::trajMoveHuman);
    
    const char* home = getenv("HOME_MOVE3D");
    if( home == NULL ) {
        cout << "ERROR home is not defined for config generator in " << __func__ << endl;
    }
    
    string dir(home);
    string file("/statFiles/OtpComputing/confHerakles.xml");
    
    handoverGenerator_ = new ConfGenerator( robot_model_, human_model_ );
    handoverGenerator_->initialize( dir+file, HRICS_activeNatu );
    
    if( ENV.getInt(Env::setOfActiveJoints) == 2 )
    {
        use_handover_config_list_ = true;
    }
    
    HRI_AGENT* robot = hri_get_agent_by_name( GLOBAL_AGENTS, robot_model_->getName().c_str() );
    hri_activate_coll_robot_and_all_humans_arms( robot, GLOBAL_AGENTS, false );
}

StompOptimizer::~StompOptimizer()
{
    delete handoverGenerator_;

    for(int i=0;i<int(compute_fk_.size());i++)
    {
        delete compute_fk_[i];
    }

    cout << "destroy stomp" << endl;
}

void StompOptimizer::doChompOptimization()
{
    calculateSmoothnessIncrements();
    calculateCollisionIncrements();
    calculateTotalIncrements();
    
    if (!stomp_parameters_->getUseHamiltonianMonteCarlo())
    {
        // non-stochastic version:
        addIncrementsToTrajectory();
    }
    else
    {
        // hamiltonian monte carlo updates:
        getRandomMomentum();
        updateMomentum();
        updatePositionFromMomentum();
        stochasticity_factor_ *= stomp_parameters_->getHmcAnnealingFactor();
    }
    
    handleJointLimits();
    updateFullTrajectory();
    last_trajectory_collision_free_ = performForwardKinematics();
    last_trajectory_constraints_satisfied_ = true;
    
    if (!last_trajectory_collision_free_ && stomp_parameters_->getAddRandomness())
    {
        performForwardKinematics();
        double original_cost = getTrajectoryCost();
        group_trajectory_backup_ = group_trajectory_.getTrajectory();
        perturbTrajectory();
        handleJointLimits();
        updateFullTrajectory();
        performForwardKinematics();
        double new_cost = getTrajectoryCost();

        if (new_cost > original_cost)
        {
            //printf("Random jump worsened cost from %.10f to %.10f!\n", original_cost, new_cost);
            group_trajectory_.getTrajectory() = group_trajectory_backup_;
            updateFullTrajectory();
        }
    }

    //Tajectory cost calculation
    Eigen::VectorXd costs = Eigen::VectorXd::Zero(free_vars_end_-free_vars_start_+1);

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;
        double cumulative = 0.0;

        if (collision_space_)
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
                state_collision_cost += cumulative;
            }
        }
        else
        {
            Eigen::VectorXd q_i,q_f;

            int id1 = i;
            int id2 = i+1;

            if (i == free_vars_end_) {
                id1--;
                id2--;
            }

            int index_i = group_trajectory_.getFullTrajectoryIndex(id1);
            int index_f = group_trajectory_.getFullTrajectoryIndex(id2);

            full_trajectory_->getTrajectoryPointP3d(index_i, q_i);
            full_trajectory_->getTrajectoryPointP3d(index_f, q_f);

            state_collision_cost += ( pow( general_cost_potential_(i) , hack_tweek ) /* ( q_f - q_i ).norm() */) ;
        }
        costs(i-free_vars_start_) = stomp_parameters_->getObstacleCostWeight() * state_collision_cost;
    }

    last_trajectory_cost_ = costs.sum();
}

//void StompOptimizer::optimize()
void StompOptimizer::runDeformation( int nbIteration , int idRun )
{
    ChronoTimeOfDayOn();
    
    timeval tim;
    gettimeofday(&tim, NULL);
    double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);
    time_ = 0.0;
    
    stomp_statistics_ = MOVE3D_BOOST_PTR_NAMESPACE<StompStatistics>(new StompStatistics());
    stomp_statistics_->run_id = idRun;
    stomp_statistics_->collision_success_iteration = -1;
    stomp_statistics_->success_iteration = -1;
    stomp_statistics_->success = false;
    stomp_statistics_->costs.clear();
    stomp_statistics_->convergence_rate.clear();
    stomp_statistics_->convergence_trajs.clear();
    
    move3d_traj_ = API::Trajectory( robot_model_ );
    traj_convergence_with_time.clear();
    
    // Set this variable to false to cancel printing
    const bool print_cost=true;
    cout << "use_time_limit_ : " << use_time_limit_ << endl;

    int ith_save = 1;
    
    PolicyImprovementLoop pi_loop;
    pi_loop.initialize(/*nh,*/ this_shared_ptr_, false );
    
    //group_trajectory_.print();
    
    iteration_ = 0;
    copyPolicyToGroupTrajectory();
    handleJointLimits();
    updateFullTrajectory();
    performForwardKinematics();
    //group_trajectory_.print();
    
    //    last_move3d_cost_ = computeMove3DCost();
    //    printf("%3d, time: %f, cost: %f\n", 0, 0.0, last_move3d_cost_ );
    //    traj_convergence_with_time.push_back( make_pair( 0.0, last_move3d_cost_ ) );
    
    if ( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) && stomp_parameters_->getAnimateEndeffector() )
        animateEndeffector();
    
    if( PlanEnv->getBool(PlanParam::trajSaveCost) ) {
        saveTrajectoryCostStats();
    }
    
    double intitialization_time=0.0;
    ChronoTimeOfDayTimes(&intitialization_time);
    ChronoTimeOfDayOff();
    cout << "Stomp init time : " << intitialization_time << endl;

    // double last_move3d_cost = numeric_limits<double>::max();
    
    source_ = getSource();
    target_ = getTarget();

    best_iteration_=0;
    best_group_trajectory_cost_ = numeric_limits<double>::max();
    best_group_trajectory_in_collsion_cost_ = numeric_limits<double>::max();

    confPtr_t q_tmp = robot_model_->getCurrentPos();
    // iterate
    for ( iteration_=0; !PlanEnv->getBool(PlanParam::stopPlanner); iteration_++)
    {
        // Compute wether to draw at a certain iteration
        bool do_draw = false;
        if( PlanEnv->getInt(PlanParam::stompDrawIteration) >  0) {
            do_draw = (iteration_!=0) && (iteration_%PlanEnv->getInt(PlanParam::stompDrawIteration) == 0);
        }

        reset_reused_rollouts_ = false;
        if( (!handover_has_been_recomputed_) && humanHasMoved() )
        {
            cout << "Human has moved" << endl;
            reset_reused_rollouts_ = true;

            if( reset_reused_rollouts_ ) {
                pi_loop.resetReusedRollouts();
                reset_reused_rollouts_ = false;
            }

            // Recompute the handover position
            if( use_handover_config_generator_ )
            {
                if( replaceEndWithNewConfiguration() ) {
                    q_tmp = target_new_;
                    copyGroupTrajectoryToPolicy();
                    if( recompute_handover_once_ ) {
                        handover_has_been_recomputed_ = true;
                    }
                    cout << "copyGroupTrajectoryToPolicy" << endl;
                }
            }
        }

        // Policy improvement loop
        // pi_loop.runSingleIteration(iteration_+1);
        if (!stomp_parameters_->getUseChomp())
        {
            pi_loop.runSingleIteration(iteration_+1);
        }
        else
        {
            doChompOptimization();
        }

        if ( do_draw && (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) &&
             stomp_parameters_->getAnimateEndeffector() )
        {
            robot_model_->setAndUpdate(*q_tmp);
            animateEndeffector();
            //animateTrajectoryPolicy();
        }

        double cost = getTrajectoryCost();

        //double cost = last_trajectory_cost_;
        stomp_statistics_->costs.push_back(cost);

        if( reset_reused_rollouts_ )
        {
            best_group_trajectory_ = group_trajectory_.getTrajectory();
            best_group_trajectory_cost_ = cost;
            cout << "Change best to actual current trajectory" << endl;
        }

        if (last_trajectory_collision_free_ && last_trajectory_constraints_satisfied_)
            collision_free_iteration_++;
        else
            collision_free_iteration_ = 0;

        if (last_trajectory_collision_free_ &&
                stomp_statistics_->collision_success_iteration == -1)
        {
            stomp_statistics_->collision_success_iteration = iteration_;
        }
        if (last_trajectory_collision_free_ &&
                last_trajectory_constraints_satisfied_ &&
                stomp_statistics_->success_iteration == -1)
        {
            stomp_statistics_->success_iteration = iteration_;
            stomp_statistics_->success = true;
            stomp_statistics_->success_time = time_;
        }

        if (iteration_==0)
        {
            best_group_trajectory_ = group_trajectory_.getTrajectory();
            best_group_trajectory_cost_ = cost;
        }
        else
        {
            if( cost < best_group_trajectory_cost_ )
            {
                if ( is_collision_free_)
                {
                    if( print_cost )
                        cout << "New best" << endl;

                    best_group_trajectory_ = group_trajectory_.getTrajectory();
                    best_group_trajectory_cost_ = cost;
                    last_improvement_iteration_ = iteration_;
                    best_iteration_++;
                }
                else
                {
                    best_group_trajectory_in_collision_ = group_trajectory_.getTrajectory();
                    best_group_trajectory_in_collsion_cost_ = cost;
                    if( print_cost )
                        cout << "New best in collision" << endl;
                }
            }
        }

        //if (iteration_%1==0)
        gettimeofday(&tim, NULL);
        time_ = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;

        if( PlanEnv->getBool(PlanParam::trajSaveCost) && /**time_>double(ith_save)*/ do_draw ) {
            saveTrajectoryCostStats();
            ith_save++;
        }

        //last_move3d_cost_ = computeMove3DCost();

        double move3d_cost =0.0;
        if ( do_draw && (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) )
        {
            //        move3d_cost = computeMove3DCost();
            //
            //        if( move3d_cost < last_move3d_cost )
            //        {
            //          last_move3d_cost = move3d_cost;

            if ( stomp_parameters_->getAnimateEndeffector() )
            {
                robot_model_->setAndUpdate(*q_tmp);
                animateEndeffector();
                //animateTrajectoryPolicy();
            }
            //        }
        }

        if( PlanEnv->getBool(PlanParam::drawParallelTraj) && ( global_stompRun != NULL ))
        {
            ENV.setBool(Env::drawTraj,false);
            global_stompRun->lockDraw();
            robot_model_->setAndUpdate(*q_tmp);
            saveEndeffectorTraj();
            global_stompRun->unlockDraw();
        }

        // save the cost and time as pair
        //      traj_convergence_with_time.push_back( make_pair( time_, move3d_cost ) );

        if( print_cost )
        {
            if( (!ENV.getBool(Env::drawDisabled)) && ( ENV.getBool(Env::drawTraj) || PlanEnv->getBool(PlanParam::drawParallelTraj) ))
            {
                printf("%3d, time: %3f, cost: %f (s=%f, c=%f, g=%f), move3d cost: %f\n", iteration_, time_, cost,
                       getSmoothnessCost(), getCollisionCost(), getGeneralCost(), move3d_cost );
            }
            else {
                printf("%3d, time: %3f, cost: %f\n", iteration_, time_, cost );
                //          printf("%3d, time: %3f, cost: %f (s=%f, c=%f, g=%f), move3d cost: %f\n", iteration_, time_, cost,
                //                 getSmoothnessCost(), getCollisionCost(), getGeneralCost(), move3d_cost );
            }
        }
        //printf( "%3d, time: %f, cost: %f (s=%f, c=%f)\n", iteration_, time, getTrajectoryCost(), getSmoothnessCost(), getCollisionCost());

        //cout << "We think the path is collision free: " << last_trajectory_collision_free_ << endl;

        if( use_time_limit_ )
        {
            if( time_ >= stomp_parameters_->max_time_ )
            {
                cout << "Stopped at time limit (" << time_ << ")" << endl;
                break;
            }
        }

        if( use_iteration_limit_ )
        {
            if( iteration_ >= stomp_parameters_->max_iterations_ )
            {
                cout << "Stopped at iteration (" << iteration_ << ")" << endl;
                break;
            }
        }
    }


    if (last_improvement_iteration_>-1)
        cout << "We think the path is collision free: " << is_collision_free_ << endl;


    // set the output in the best trajectory

    if( best_group_trajectory_in_collsion_cost_ < best_group_trajectory_cost_ )

        group_trajectory_.getTrajectory() = best_group_trajectory_in_collision_;
    else
        group_trajectory_.getTrajectory() = best_group_trajectory_;

    // Set this anywhere
    //    if( PlanEnv->getBool(PlanParam::drawParallelTraj) && ( global_stompRun != NULL ))
    //    {
    //        ENV.setBool(Env::drawTraj,false);
    //        global_stompRun->lockDraw();
    //        robot_model_->setAndUpdate(*q_tmp);
    //        saveEndeffectorTraj();
    //        global_stompRun->unlockDraw();
    //    }
    
    //group_trajectory_.print();
    //updateFullTrajectory();
    //performForwardKinematics();
    
    if ( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj)  )
    {
        animateEndeffector(true);
    }
    
    best_traj_ = API::Trajectory( robot_model_ );
    setGroupTrajectoryToApiTraj( best_traj_ );
    best_traj_.replaceP3dTraj();
    
    printf("Collision free success iteration = %d (time : %f)\n",
           stomp_statistics_->collision_success_iteration, stomp_statistics_->success_time);
    printf("Terminated after %d iterations, using path from iteration %d\n", iteration_, last_improvement_iteration_);
    printf("Best cost = %f\n", best_group_trajectory_cost_);
    printf("Stomp has run for : %f sec\n", time_ );
    
    //printf("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());
    stomp_statistics_->best_cost = best_group_trajectory_cost_;
    
    traj_optim_convergence.clear();
    traj_optim_convergence.push_back( stomp_statistics_->costs );
    
    if( PlanEnv->getBool(PlanParam::trajSaveCost) )
    {
        stringstream s;
        s << "StompOptim_" << setfill('0') << setw(4) << idRun ;
        saveOptimToFile( s.str() );
    }
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------

void StompOptimizer::setRobotPool( const std::vector<Robot*>& robots )
{
    compute_fk_.clear();

    for( int i=0; i<int(robots.size()); i++)
    {
        compute_fk_.push_back(new costComputation(robots[i], collision_space_, planning_group_, joint_costs_,
                                                  group_trajectory_,
                                                  stomp_parameters_->getObstacleCostWeight(),
                                                  use_costspace_));
    }
}

/**
  * get the cost conputer
  */
const vector<costComputation*>& StompOptimizer::getCostComputers()
{
    return compute_fk_;
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------
//! Generate noisy trajectories
void StompOptimizer::generateNoisyTrajectory(const API::Trajectory& traj, vector<vector<confPtr_t> >& noisy_trajectories)
{
    // Set the full and group trajectory
    if( full_trajectory_ != NULL ) delete full_trajectory_;
    full_trajectory_ = new ChompTrajectory( traj, DIFF_RULE_LENGTH, *planning_group_ );
    group_trajectory_ = *full_trajectory_;
    
    stomp_statistics_ = MOVE3D_BOOST_PTR_NAMESPACE<StompStatistics>(new StompStatistics());
    stomp_statistics_->collision_success_iteration = -1;
    stomp_statistics_->success_iteration = -1;
    stomp_statistics_->success = false;
    stomp_statistics_->costs.clear();
    
    ChronoTimeOfDayOn();

    // Initialize
    initPolicy();
    
    PolicyImprovementLoop pi_loop;
    pi_loop.initialize( this_shared_ptr_, true );
    
    //group_trajectory_.print();
    iteration_ = 0;
    copyPolicyToGroupTrajectory();
    handleJointLimits();
    updateFullTrajectory();
    performForwardKinematics();
    //group_trajectory_.print();
    
    //    if (stomp_parameters_->getAnimateEndeffector())
    //    {
    //      animateEndeffector();
    //    }
    
    pi_loop.generateSingleNoisyTrajectory(/*iteration_+1*/);
    
    pi_loop.getRollouts( noisy_trajectories );
    
    //    if (stomp_parameters_->getAnimateEndeffector())
    //    {
    //      animateEndeffector();
    //    }
    
    //setGroupTrajectoryToVectorConfig( noisy_trajectory[0] );
    
    //    updateFullTrajectory();
    //    performForwardKinematics();
    ChronoTimeOfDayPrint("");
    ChronoTimeOfDayOff();
}

//-------------------------------------------------------------------
//-------------------------------------------------------------------
//-------------------------------------------------------------------
void StompOptimizer::calculateSmoothnessIncrements()
{
    for (int i=0; i<num_joints_; i++)
    {
        joint_costs_[i].getDerivative( group_trajectory_.getJointTrajectory(i), smoothness_derivative_ );
        smoothness_increments_.col(i) = -smoothness_derivative_.segment( group_trajectory_.getStartIndex(), num_vars_free_ );
    }
    
    //cout <<  "smoothness_increments_  = " << endl << smoothness_increments_ << endl;
}

void StompOptimizer::calculateCollisionIncrements()
{
    double potential;
    double vel_mag_sq;
    double vel_mag;
    Vector3d potential_gradient;
    Vector3d normalized_velocity;
    Matrix3d orthogonal_projector;
    Vector3d curvature_vector;
    Vector3d cartesian_gradient;
    
    //cout <<  "collision_point_potential_  = " << endl << collision_point_potential_ << endl;
    
    collision_increments_.setZero(num_vars_free_, num_joints_);
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        for (int j=0; j<num_collision_points_; j++)
        {
            potential = collision_point_potential_(i,j);
            if (potential <= 1e-10)
                continue;

            potential_gradient = collision_point_potential_gradient_[i][j];
            //cout << collision_point_potential_gradient_[i][j] << endl;

            vel_mag = collision_point_vel_mag_(i,j);
            vel_mag_sq = vel_mag*vel_mag;
            //cout << collision_point_vel_mag_(i,j) << endl;

            // all math from the STOMP paper:
            normalized_velocity = collision_point_vel_eigen_[i][j] / vel_mag;
            orthogonal_projector = Matrix3d::Identity() - (normalized_velocity * normalized_velocity.transpose());
            curvature_vector = (orthogonal_projector * collision_point_acc_eigen_[i][j]) / vel_mag_sq;
            cartesian_gradient = vel_mag*(orthogonal_projector*potential_gradient - potential*curvature_vector);

            // pass it through the jacobian transpose to get the increments
            planning_group_->collision_points_[j].getJacobian(joint_pos_eigen_[i],
                                                              joint_axis_eigen_[i],
                                                              collision_point_pos_eigen_[i][j],
                                                              jacobian_,
                                                              group_joint_to_move3d_joint_index_);

            if (stomp_parameters_->getUsePseudoInverse())
            {
                calculatePseudoInverse();
                collision_increments_.row(i-free_vars_start_).transpose() -=
                        jacobian_pseudo_inverse_ * cartesian_gradient;
            }
            else
            {
                collision_increments_.row(i-free_vars_start_).transpose() -=
                        jacobian_.transpose() * cartesian_gradient;
            }
            if (point_is_in_collision_[i][j])
                break;
        }
    }
    
    //cout <<  "collision_increments_  = " << endl << collision_increments_ << endl;
}

void StompOptimizer::calculatePseudoInverse()
{
    jacobian_jacobian_tranpose_ = jacobian_*jacobian_.transpose() + Eigen::MatrixXd::Identity(3,3)*stomp_parameters_->getPseudoInverseRidgeFactor();
    jacobian_pseudo_inverse_ = jacobian_.transpose() * jacobian_jacobian_tranpose_.inverse();
}

void StompOptimizer::calculateTotalIncrements()
{
    for (int i=0; i<num_joints_; i++)
    {
        final_increments_.col(i) = stomp_parameters_->getLearningRate() *
                (
                    joint_costs_[i].getQuadraticCostInverse() *
                    (
                        stomp_parameters_->getSmoothnessCostWeight() * smoothness_increments_.col(i) +
                        stomp_parameters_->getObstacleCostWeight() * collision_increments_.col(i)
                        )
                    );
    }
}

void StompOptimizer::addIncrementsToTrajectory()
{
    //  double scale = 1.0;
    for (int i=0; i<num_joints_; i++)
    {
        double scale = 1.0;
        double max = final_increments_.col(i).maxCoeff();
        double min = final_increments_.col(i).minCoeff();
        double max_scale = planning_group_->chomp_joints_[i].joint_update_limit_ / fabs(max);
        double min_scale = planning_group_->chomp_joints_[i].joint_update_limit_ / fabs(min);
        if (max_scale < scale)
            scale = max_scale;
        if (min_scale < scale)
            scale = min_scale;
        group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments_.col(i);
    }
    //ROS_DEBUG("Scale: %f",scale);
    //group_trajectory_.getFreeTrajectoryBlock() += scale * final_increments_;
}

void StompOptimizer::updateFullTrajectory()
{
    full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void StompOptimizer::debugCost()
{
    double cost = 0.0;
    for (int i=0; i<num_joints_; i++)
        cost += joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
    cout << "Cost = " << cost << endl;
}

double StompOptimizer::getTrajectoryCost()
{
    double cost = getSmoothnessCost();
    if(collision_space_) cost += getCollisionCost();
    if(use_costspace_ ) cost += getGeneralCost();
    return cost;
}

double StompOptimizer::getSmoothnessCost()
{
    double smoothness_cost = 0.0;
    //    double joint_cost = 0.0;
    //    // joint costs:
    //    for ( int i=0; i<num_joints_; i++ )
    //    {
    //        joint_cost = joint_costs_[i].getCost(group_trajectory_.getJointTrajectory(i));
    //        smoothness_cost += joint_cost;
    //    }
    //    return stomp_parameters_->getSmoothnessCostWeight() * smoothness_cost;

    std::vector<Eigen::MatrixXd> control_cost_matrices;
    std::vector<Eigen::VectorXd> noise(num_joints_);
    std::vector<Eigen::VectorXd> control_costs(num_joints_);

    for (int d=0; d<num_joints_; ++d)
    {
        //policy_parameters_[d] = group_trajectory_.getFreeTrajectoryBlock();
        policy_parameters_[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
        noise[d] = VectorXd::Zero( policy_parameters_[d].size() );
    }

    policy_->computeControlCosts(control_cost_matrices, policy_parameters_, noise, 0.5*stomp_parameters_->getSmoothnessCostWeight(), control_costs );

    for (int d=0; d<int(control_costs.size()); ++d)
    {
        //        cout << "control_costs[" << d << "] = " << control_costs[d].sum() << endl;
        smoothness_cost += control_costs[d].sum();
    }

    return smoothness_cost;
}

double StompOptimizer::getCollisionCost()
{
    double collision_cost = 0.0;
    
    double worst_collision_cost = 0.0;
    worst_collision_cost_state_ = -1;
    
    // collision costs:
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;

        if( collision_space_ )
        {
            // TODO watch out for cumulative case
            double cumulative = 0.0;
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
                state_collision_cost += cumulative;
                // state_collision_cost += collision_point_potential_(i,j);
            }
        }

        collision_cost += state_collision_cost;

        if (state_collision_cost > worst_collision_cost)
        {
            worst_collision_cost = state_collision_cost;
            worst_collision_cost_state_ = i;
        }
    }
    
    return stomp_parameters_->getObstacleCostWeight() * collision_cost;
}

double StompOptimizer::getGeneralCost()
{
    if( !use_costspace_ && !PlanEnv->getBool(PlanParam::useLegibleCost) )
    {
        return 0.0;
    }

    double general_cost = 0.0;

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        general_cost += pow( general_cost_potential_(i) , hack_tweek );
    }

    //cout << general_cost_potential_.segment(free_vars_start_,free_vars_end_-free_vars_start_).transpose() << endl;

    return stomp_parameters_->getObstacleCostWeight() * general_cost;
}

void StompOptimizer::getCostProfiles( vector<double>& smoothness_cost, vector<double>& collision_cost, vector<double>& general_cost )
{
    //    cout << "perform forward kinematics" << endl;
    group_trajectory_.getTrajectory() = best_group_trajectory_;
    performForwardKinematics();

    std::vector<Eigen::MatrixXd> control_cost_matrices;
    std::vector<Eigen::VectorXd> noise(num_joints_);
    std::vector<Eigen::VectorXd> control_costs(num_joints_);

    for (int d=0; d<num_joints_; ++d)
    {
        //policy_parameters_[d] = group_trajectory_.getFreeTrajectoryBlock();
        policy_parameters_[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
        noise[d] = VectorXd::Zero( policy_parameters_[d].size() );
    }

    policy_->computeControlCosts(control_cost_matrices, policy_parameters_, noise,
                                 0.5*stomp_parameters_->getSmoothnessCostWeight(), control_costs );

    int state = 0;

    smoothness_cost.resize(free_vars_end_-free_vars_start_,0.0);
    collision_cost.resize(free_vars_end_-free_vars_start_,0.0);
    general_cost.resize(free_vars_end_-free_vars_start_,0.0);

    // collision and general costs
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_smoothness_cost = 0.0;
        double state_collision_cost = 0.0;

        if( collision_space_ )
        {
            // TODO watch out for cumulative case
            double cumulative = 0.0;
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
                state_collision_cost += cumulative;
                //                state_collision_cost += collision_point_potential_(i,j);
            }

            collision_cost[state] = stomp_parameters_->getObstacleCostWeight() * state_collision_cost;
        }

        general_cost[state] = stomp_parameters_->getObstacleCostWeight() * ( pow( general_cost_potential_(i) , hack_tweek ) );

        for (int d=0; d<num_joints_; ++d)
        {
            state_smoothness_cost += control_costs[d][state];
        }

        smoothness_cost[state] = state_smoothness_cost;

        state++;
    }
}

bool StompOptimizer::handleJointLimits()
{
    bool succes_joint_limits = true;
    joint_limits_violation_ = 0;

    for (int joint=0; joint<num_joints_; joint++)
    {
        if (!planning_group_->chomp_joints_[joint].has_joint_limits_)
            continue;

        // Added by jim for pr2 free flyer
        if( planning_group_->robot_->getName() == "PR2_ROBOT" )
        {
            int index = planning_group_->chomp_joints_[joint].move3d_dof_index_;

            if( index == 8 || index == 9 || index == 10 ) {
                group_trajectory_.getFreeJointTrajectoryBlock(joint) = Eigen::VectorXd::Zero(num_vars_free_);
                continue;
            }
        }

        double joint_max = planning_group_->chomp_joints_[joint].joint_limit_max_;
        double joint_min = planning_group_->chomp_joints_[joint].joint_limit_min_;

        // cout << "Joint max : " << joint_max << endl;
        // cout << "Joint min : " << joint_min << endl;
        int count = 0;
        bool violation = false;
        int count_violation = false;

        do
        {
            double max_abs_violation =  1e-6;
            double max_violation = 0.0;
            int max_violation_index = 0;
            violation = false;
            count_violation = false;
            //            bool violate_max = false;
            //            bool violate_min = false;
            double violate_max_with = 0;
            double violate_min_with = 0;

            for (int i=free_vars_start_; i<=free_vars_end_; i++)
            {
                double amount = 0.0;
                double absolute_amount = 0.0;
                if ( group_trajectory_(i, joint) > joint_max )
                {
                    amount = joint_max - group_trajectory_(i, joint);
                    absolute_amount = fabs(amount);
                    //                    violate_max = true;
                    if( absolute_amount > violate_max_with )
                    {
                        violate_max_with = absolute_amount;
                    }
                }
                else if ( group_trajectory_(i, joint) < joint_min )
                {
                    amount = joint_min - group_trajectory_(i, joint);
                    absolute_amount = fabs(amount);
                    //                    violate_min = true;
                    if( absolute_amount > violate_min_with )
                    {
                        violate_min_with = absolute_amount;
                    }
                }
                if (absolute_amount > max_abs_violation)
                {
                    max_abs_violation = absolute_amount;
                    max_violation = amount;
                    max_violation_index = i;
                    violation = true;

                    // cout << "Violation (" << absolute_amount ;
                    // cout << " , " << max_abs_violation << ")" << endl;

                    if ( i != free_vars_start_ && i != free_vars_end_ ) {
                        count_violation = true;
                    }
                }
            }

            if (violation)
            {
                // Only count joint limits violation for
                // interpolated configurations
                if( count_violation )
                {
                    joint_limits_violation_++;
                }
                // cout << "Violation of Limits (joint) : " <<  joint << endl;
                int free_var_index = max_violation_index - free_vars_start_;
                double multiplier = max_violation / joint_costs_[joint].getQuadraticCostInverse()(free_var_index,free_var_index);

                group_trajectory_.getFreeJointTrajectoryBlock(joint) += multiplier * joint_costs_[joint].getQuadraticCostInverse().col(free_var_index);
                //                double offset = ( joint_max + joint_min )  / 2 ;

                //                cout << "multiplier : " << multiplier << endl;
                //                cout << "joint limit max : " << joint_max << endl;
                //                cout << "joint limit min : " << joint_min << endl;
                //                cout << "offset : " << offset << endl;
                //                cout << group_trajectory_.getFreeJointTrajectoryBlock(joint).transpose() << endl;

                //                group_trajectory_.getFreeJointTrajectoryBlock(joint) = ( group_trajectory_.getFreeJointTrajectoryBlock(joint).array() - offset )*multiplier + ( offset );

                //                cout << group_trajectory_.getFreeJointTrajectoryBlock(joint).transpose() << endl;
            }
            if ( ++count > 10 )
            {
                succes_joint_limits = false;
                //cout << "group_trajectory_(i, joint) = " << endl << group_trajectory_.getFreeJointTrajectoryBlock(joint) << endl;
                break;
            }
        }
        while(violation);

        //        if( violation || !succes_joint_limits )
        //        {
        //            cout << "Violation of joint limits (joint) = " << joint << endl;
        //        }
    }

    //    cout << "succes_joint_limits : " << succes_joint_limits << endl;

    //    exit(1);

    return succes_joint_limits;
}


double StompOptimizer::getCollisionSpaceCost( Configuration& q )
{
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    
    // Get the configuration dof values in the joint array
    Eigen::VectorXd joint_array(planning_group_->num_joints_);
    
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
        joint_array[j] = q[ joints[j].move3d_dof_index_ ];
    }
    
    getFrames( free_vars_start_, joint_array, q );
    
    double state_collision_cost=0.0;
    
    // Calculate the 3d position of every collision point
    for (int j=0; j<num_collision_points_; j++)
    {
        /* bool colliding = */ getConfigObstacleCost( free_vars_start_, j );
        state_collision_cost += collision_point_potential_( free_vars_start_, j ); // * collision_point_vel_mag_( free_vars_start_, j );

        cout << "collision_point_potential_( " << free_vars_start_ << " , " <<  j << " ) : " << collision_point_potential_( free_vars_start_, j ) << endl;
    }
    
    return state_collision_cost;
}

void StompOptimizer::getFrames( int segment, const Eigen::VectorXd& joint_array, Configuration& q )
{
    q = *robot_model_->getCurrentPos();

    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    // Set the configuration to the joint array value
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
        int dof = joints[j].move3d_dof_index_;

        if ( !std::isnan(joint_array[j]) )
        {
            q[dof]= joint_array[j];
        }
        else {
            cout << "q[" << dof << "] is nan" << endl;
            q[dof]= 0;
        }
    }

    robot_model_->setAndUpdate( q );
    // g3d_draw_allwin_active();

    // Get the collision point position
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
        Eigen::Transform3d t = joints[j].move3d_joint_->getMatrixPos();

        //cout << "joints[" << j<< "].move3d_joint_ : " << joints[j].move3d_joint_->getName() << endl;

        std::vector<double> vect;
        eigenTransformToStdVector(t,vect);

        segment_frames_[segment][j]  = vect;
        joint_pos_eigen_[segment][j] = t.translation();

        if (collision_space_)
        {
            joint_axis_eigen_[segment][j](0) = t(0,2);
            joint_axis_eigen_[segment][j](1) = t(1,2);
            joint_axis_eigen_[segment][j](2) = t(2,2);
        }
    }
}

bool StompOptimizer::getConfigObstacleCost( int segment, int coll_point )
{
    bool colliding = false;

    if( collision_space_ /*&& (use_costspace_==false)*/ )
    {
        int i = segment;
        int j = coll_point;
        double distance;

        planning_group_->collision_points_[j].getTransformedPosition( segment_frames_[i], collision_point_pos_eigen_[i][j] );

        // To fix in collision space
        // The joint 1 is allways colliding
        colliding = collision_space_->getCollisionPointPotentialGradient( planning_group_->collision_points_[j],
                                                                          collision_point_pos_eigen_[i][j],
                                                                          distance,
                                                                          collision_point_potential_(i,j),
                                                                          collision_point_potential_gradient_[i][j] );

        //        cout << "distance( " << i << ", " << j << " ) : " << distance << endl;
        //cout << "collision_point_potential_(" << i << ", " << j << " ) : " << collision_point_potential_(i,j) << endl;
        //        if( collision_point_potential_(i,j) != 0.0 )
        //        {
        //            cout << "collision_point_potential_(" << i << ", " << j << " ) : " << collision_point_potential_(i,j) << endl;
        //        }
    }

    return colliding;
}

bool StompOptimizer::performForwardKinematics()
{
    double invTime = 1.0 / group_trajectory_.getDiscretization();
    double invTimeSq = invTime*invTime;

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
    if ( iteration_==0)
    {
        start = 0;
        end = num_vars_all_-1;
    }

    is_collision_free_ = true;

    Eigen::VectorXd joint_array;

    Configuration q(robot_model_);

    if( PlanEnv->getBool(PlanParam::useLegibleCost) )
    {
        HRICS_activeLegi->setTrajectory( group_trajectory_.getTrajectory() );
    }

    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        int full_traj_index = group_trajectory_.getFullTrajectoryIndex(i);
        full_trajectory_->getTrajectoryPointP3d( full_traj_index, joint_array );
        getFrames( i, joint_array, q );
        state_is_in_collision_[i] = false;

        if( use_costspace_ )
        {
            general_cost_potential_[i] = global_costSpace->cost(q);
        }

        if( PlanEnv->getBool(PlanParam::useLegibleCost) )
        {
            general_cost_potential_[i] = HRICS_activeLegi->legibilityCost(i);
        }

        if( collision_space_ )
        {
            // calculate the position of every collision point:
            for (int j=0; j<num_collision_points_; j++)
            {
                bool colliding = getConfigObstacleCost( i, j );

                if ( colliding )
                {
                    // This is the function that discards joints too close to the base
                    if( planning_group_->collision_points_[j].getSegmentNumber() > 1 )
                    {
                        state_is_in_collision_[i] = true;
                    }
                }
            }
        }

        if( ( PlanEnv->getBool(PlanParam::useLegibleCost) || use_costspace_ ) && (collision_space_==NULL) )
        {
            state_is_in_collision_[i] = false;
        }

        if ( state_is_in_collision_[i] )
        {
            is_collision_free_ = false;
        }
    }

    //    if(is_collision_free_)
    //    {
    //        // for each point in the trajectory
    //        for (int i=free_vars_start_; i<free_vars_end_; i++)
    //        {
    //            LocalPath* LP = new LocalPath(getConfigurationOnGroupTraj(i),getConfigurationOnGroupTraj(i+1));
    //            if(!LP->isValid())
    //            {
    //                is_collision_free_=false;
    //                break;
    //            }
    //        }
    //    }

    // now, get the vel and acc for each collision point (using finite differencing)
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        for (int j=0; j<num_collision_points_; j++)
        {
            collision_point_vel_eigen_[i][j] = Eigen::Vector3d::Zero();
            collision_point_acc_eigen_[i][j] = Eigen::Vector3d::Zero();

            for (int k=-DIFF_RULE_LENGTH/2; k<=DIFF_RULE_LENGTH/2; k++)
            {
                collision_point_vel_eigen_[i][j] += (invTime * DIFF_RULES[0][k+DIFF_RULE_LENGTH/2]) *
                        collision_point_pos_eigen_[i+k][j];
                collision_point_acc_eigen_[i][j] += (invTimeSq * DIFF_RULES[1][k+DIFF_RULE_LENGTH/2]) *
                        collision_point_pos_eigen_[i+k][j];
            }
            // get the norm of the velocity:
            collision_point_vel_mag_(i,j) = collision_point_vel_eigen_[i][j].norm();

            //      cout << "collision_point_pos_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_pos_eigen_[i][j] << endl;
            //      cout << "collision_point_potential_(" << i << " , "  << j << ") = " << endl << collision_point_potential_[i][j] << endl;
            //      cout << "collision_point_potential_gradient_(" << i << " , "  << j << ") = " << endl << collision_point_potential_gradient_[i][j] << endl;
            //      cout << "collision_point_vel_mag_(" << i << " , "  << j << ") = " << endl << collision_point_vel_mag_[i][j] << endl;
            //      cout << "collision_point_vel_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_vel_eigen_[i][j] << endl;
            //      cout << "collision_point_acc_eigen_(" << i << " , "  << j << ") = " << endl << collision_point_acc_eigen_[i][j] << endl;
        }
    }

    //  if (is_collision_free_)
    //  {
    //    cout << "-----------------------" << endl;
    //    cout << "Collision Free Traj" << endl;
    //    cout << "-----------------------" << endl;
    //  }
    //  else {
    //    cout << "------------------------------------" << endl;
    //    for (int i=start; i<=end; ++i)
    //    {
    //      if (state_is_in_collision_[i])
    //      {
    //        cout << "state[" << i << "] in collision" << endl;
    //      }
    //    }
    //    cout << "------------------------------------" << endl;
    //  }

    return is_collision_free_;
}

bool StompOptimizer::execute(std::vector<Eigen::VectorXd>& parameters, Eigen::VectorXd& costs, const int iteration_number, bool joint_limits, bool resample )
{
    //cout << "size : " << parameters.size() << " , " << parameters[0].size() <<  endl;

    // copy the parameters into group_trajectory_:
    for (int d=0; d<num_joints_; ++d) {
        group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
    }
    //cout << "group_trajectory_ = " << endl << group_trajectory_.getTrajectory() << endl;

    // respect joint limits:

    succeded_joint_limits_ = handleJointLimits();

    // Fix joint limits
    //    if( !succeded_joint_limits_ )
    //    {
    //        for (int d=0; d<num_joints_; ++d) {
    //            group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
    //        }
    //    }
    //cout << "Violation number : " << joint_limits_violation_ << endl;
    //succeded_joint_limits_ = true;

    // copy the group_trajectory_ parameters:
    if( joint_limits )
    {
        //cout << "return with joint limits" << endl;
        for (int d=0; d<num_joints_; ++d) {
            parameters[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
        }
    }

    if( resample )
    {
        // Resample
        last_move3d_cost_ = resampleParameters( parameters );

        for (int d=0; d<num_joints_; ++d) {
            group_trajectory_.getFreeJointTrajectoryBlock(d) = parameters[d];
        }
    }

    // copy to full traj:
    updateFullTrajectory();

    // do forward kinematics:
    last_trajectory_collision_free_ = performForwardKinematics();
    last_trajectory_constraints_satisfied_ = true;

    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;
        double state_general_cost = 0.0;
        double cumulative = 0.0;

        if( collision_space_ )
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                cumulative += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
                state_collision_cost += cumulative;
            }
        }
        if( use_costspace_ || PlanEnv->getBool(PlanParam::useLegibleCost) )
        {
            state_general_cost = ( pow( general_cost_potential_(i) , hack_tweek ) /* ( q_f - q_i ).norm() */) ;
        }

        costs(i-free_vars_start_) = stomp_parameters_->getObstacleCostWeight() * ( state_collision_cost + state_general_cost );

        //cout << "state_collision_cost : " << state_collision_cost << endl;
    }

    //cout << "StompOptimizer::execute::cost => " << costs.sum() << endl;
    last_trajectory_cost_ = costs.sum();
    //last_trajectory_constraints_satisfied_ = (constraint_cost < 1e-6);
    return true;
}

void StompOptimizer::getTrajectoryCost( std::vector<double>& cost, double step )
{
    cost.clear();
    cout << "best_group_trajectory_ = " << best_group_trajectory_ << endl;
    
    group_trajectory_.getTrajectory() = best_group_trajectory_;
    group_trajectory_.print();
    
    updateFullTrajectory();
    performForwardKinematics();
    
    vector<double> collision_cost;
    
    // collision costs:
    double collision_cost_sum = 0.0;
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        double state_collision_cost = 0.0;

        if(collision_space_)
        {
            for (int j=0; j<num_collision_points_; j++)
            {
                state_collision_cost += collision_point_potential_(i,j) * collision_point_vel_mag_(i,j);
            }
        }
        else
        {
            state_collision_cost = pow( general_cost_potential_(i) , hack_tweek );
        }

        collision_cost.push_back(state_collision_cost);
        collision_cost_sum += state_collision_cost;
    }
    cout << "Collision cost : " << stomp_parameters_->getObstacleCostWeight() * collision_cost_sum << endl;
    double vector_steps = (double(collision_cost.size())/100);
    
    for (double s = 0; s<=double(collision_cost.size()); s += vector_steps )
    {
        if( int(s) >= int(collision_cost.size()) )
            break;

        cost.push_back( collision_cost[int(s)] );
    }
}

void StompOptimizer::eigenMapTest()
{
    //    double foo_eigen;
    //    double foo_kdl;
    
}

void StompOptimizer::perturbTrajectory()
{
    //int mid_point = (free_vars_start_ + free_vars_end_) / 2;
    if (worst_collision_cost_state_ < 0)
        return;
    int mid_point = worst_collision_cost_state_;
    planning_group_->getRandomState(random_state_);
    
    // convert the state into an increment
    random_state_ -= group_trajectory_.getTrajectoryPoint(mid_point).transpose();
    
    // project the increment orthogonal to joint velocities
    group_trajectory_.getJointVelocities(mid_point, joint_state_velocities_);
    joint_state_velocities_.normalize();
    random_state_ = (Eigen::MatrixXd::Identity(num_joints_, num_joints_) - joint_state_velocities_*joint_state_velocities_.transpose()) * random_state_;
    
    int mp_free_vars_index = mid_point - free_vars_start_;
    for (int i=0; i<num_joints_; i++)
    {
        group_trajectory_.getFreeJointTrajectoryBlock(i) +=
                joint_costs_[i].getQuadraticCostInverse().col(mp_free_vars_index) * random_state_(i);
    }
}

void StompOptimizer::getRandomMomentum()
{
    if (is_collision_free_)
        random_momentum_.setZero(num_vars_free_, num_joints_);
    else
        for (int i=0; i<num_joints_; ++i)
        {
            multivariate_gaussian_[i].sample(random_joint_momentum_);
            random_momentum_.col(i) = stochasticity_factor_ * random_joint_momentum_;
        }
}

void StompOptimizer::updateMomentum()
{
    double eps = stomp_parameters_->getHmcDiscretization();
    
    //  if (iteration_ > 0)
    //    momentum_ = (momentum_ + eps*final_increments_);
    //  else
    //    momentum_ = random_momentum_;
    
    double alpha = 1.0 - stomp_parameters_->getHmcStochasticity();
    //if (iteration_ > 0)
    momentum_ = alpha * (momentum_ + eps*final_increments_) + sqrt(1.0-alpha*alpha)*random_momentum_;
    //else
    //  momentum_ = random_momentum_;
}

void StompOptimizer::updatePositionFromMomentum()
{
    double eps = stomp_parameters_->getHmcDiscretization();
    group_trajectory_.getFreeTrajectoryBlock() += eps * momentum_;
}

void StompOptimizer::animatePath()
{
    for (int i=free_vars_start_; i<=free_vars_end_; i++)
    {
        visualizeState(i);
        //ros::WallDuration(group_trajectory_.getDiscretization()).sleep();
        //    ros::WallDuration(0.05).sleep();
    }
}

void StompOptimizer::clearAnimations()
{

}

void StompOptimizer::setGroupTrajectoryFromVectorConfig(const vector<confPtr_t>& traj)
{
    int start = free_vars_start_;
    int end = free_vars_end_;
    if (iteration_==0) {
        start = 0;
        end = num_vars_all_-1;
    }
    
    // Get the map from move3d index to group trajectory
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    for (int i=start; i<=end; ++i)
    {
        for(int j=0; j<planning_group_->num_joints_;j++)
        {
            double point = (*traj[i])[joints[j].move3d_dof_index_];
            group_trajectory_.getTrajectoryPoint(i).transpose()[j] = point;

        }
    }
}

void StompOptimizer::setGroupTrajectoryToVectorConfig(vector<confPtr_t>& traj)
{
    int start = free_vars_start_;
    int end = free_vars_end_;
    if (iteration_==0) {
        start = 0;
        end = num_vars_all_-1;
    }
    
    // Get the map fro move3d index to group trajectory
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    confPtr_t q = robot_model_->getCurrentPos();
    
    for (int i=start; i<=end; ++i)
    {
        Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

        for(int j=0; j<planning_group_->num_joints_;j++) {
            (*q)[joints[j].move3d_dof_index_] = point[j];
        }
        traj.push_back( confPtr_t(new Configuration(*q)) );
    }
}

void StompOptimizer::setGroupTrajectoryToApiTraj(API::Trajectory& traj)
{
    int start = free_vars_start_;
    int end = free_vars_end_;
    if (iteration_==0) {
        start = 0;
        end = num_vars_all_-1;
    }
    
    traj.clear();
    
    // Get the map fro move3d index to group trajectory
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    
    confPtr_t q = robot_model_->getCurrentPos();
    source_->setConstraints();
    
    traj.push_back( source_ );
    
    for (int i=start; i<=end; ++i)
    {
        Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

        for(int j=0; j<planning_group_->num_joints_;j++)
            (*q)[joints[j].move3d_dof_index_] = point[j];

        q->setConstraints();
        traj.push_back( confPtr_t(new Configuration(*q)) );
    }
}

void StompOptimizer::animateTrajectoryPolicy()
{
    //    API::Trajectory T(robot_model_);
    //    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    //    std::vector<Eigen::VectorXd> parameters;
    //
    //    policy_->getParameters(parameters);
    //
    //    // for each point in the trajectory
    //    for (int i=0; i<int(parameters[0].size()); ++i)
    //    {
    //      confPtr_t q = robot_model_->getCurrentPos();
    //
    //      for(int j=0; j<planning_group_->num_joints_;j++)
    //      {
    //        (*q)[joints[j].move3d_dof_index_] = parameters[j][i];
    //      }
    //      T.push_back(q);
    //    }
    //    T.replaceP3dTraj();
    //g3d_draw_allwin_active();
}

void StompOptimizer::saveEndeffectorTraj()
{
    API::Trajectory T(robot_model_);

    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
    if (iteration_==0)
    {
        start = 0;
        end = num_vars_all_-1;
    }

    confPtr_t q     = robot_model_->getCurrentPos();

    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

        for(int j=0; j<planning_group_->num_joints_;j++)
            (*q)[joints[j].move3d_dof_index_] = point[j];

        T.push_back( confPtr_t(new Configuration(*q)) );
    }

    // Set the robot to the first configuration
    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(1).transpose();

    for(int j=0; j<planning_group_->num_joints_;j++)
        (*q)[joints[j].move3d_dof_index_] = point[j];

    p3d_jnt* drawnjnt=NULL;
    int indexjnt = p3d_get_user_drawnjnt();
    cout << "indexjnt : " << indexjnt << endl;
    if (indexjnt != -1 && indexjnt >= 0 && indexjnt <= robot_model_->getRobotStruct()->njoints )
    {
        drawnjnt = robot_model_->getRobotStruct()->joints[indexjnt];
    }

    traj_color_.resize(4,0);

    global_MultiStomplinesColors[robot_model_] = traj_color_;
    global_MultiStomplinesToDraw[robot_model_].clear();

    for (int i=0; i<T.getNbOfViaPoints(); ++i)
    {
        robot_model_->setAndUpdate(*T[i]);
        p3d_vector3 pos; p3d_jnt_get_cur_vect_point( drawnjnt, pos );
        global_MultiStomplinesToDraw[robot_model_].push_back(Eigen::Vector3d( pos[0], pos[1], pos[2] ));
    }

    robot_model_->setAndUpdate( *source_ );

    g3d_draw_allwin_active();
}

void StompOptimizer::animateEndeffector(bool print_cost)
{
    API::Trajectory T(robot_model_);
    
    // calculate the forward kinematics for the fixed states only in the first iteration:
    int start = free_vars_start_;
    int end = free_vars_end_;
    if (iteration_==0)
    {
        start = 0;
        end = num_vars_all_-1;
    }
    
    confPtr_t q_tmp = robot_model_->getCurrentPos();
    confPtr_t q     = robot_model_->getCurrentPos();
    // cout << "animateEndeffector()" << endl;
    // cout << "group_trajectory : " << endl;
    // cout << group_trajectory_.getTrajectory() << endl;
    
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    
    // for each point in the trajectory
    for (int i=start; i<=end; ++i)
    {
        Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(i).transpose();

        for(int j=0; j<planning_group_->num_joints_;j++)
            (*q)[joints[j].move3d_dof_index_] = point[j];

        //q.print();

        T.push_back( confPtr_t(new Configuration(*q)) );
    }
    
    if( print_cost )
    {
        cout << "Move3D Trajectory Cost : " << T.cost() ;
        if( T.isValid() )
            cout << " and is valid" << endl;
        else
            cout << " and is NOT valid" << endl;
    }
    
    if(!ENV.getBool(Env::drawTrajVector))
    {
        T.replaceP3dTraj();
    }
    else
    {
        T.setColor( Red );
        global_trajToDraw.push_back( T );
    }
    
    // Set the robot to the first configuration
    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(1).transpose();
    
    for(int j=0; j<planning_group_->num_joints_;j++)
        (*q)[joints[j].move3d_dof_index_] = point[j];

    //    robot_model_->setAndUpdate( *q_tmp );
    robot_model_->setAndUpdate( *source_ );
    
    if(! ENV.getBool(Env::drawDisabled) )
        g3d_draw_allwin_active();
}

bool StompOptimizer::getMobileManipHandOver()
{
    double best_cost=0.0;
    pair<confPtr_t,confPtr_t> best_handover_conf;
    
    // Parse list to find
    // the best feasible hand-over configuration
    bool found_hover = handoverGenerator_->computeHandoverConfigFromList( best_handover_conf, best_cost );
    
    if( found_hover ) {
        human_model_->setAndUpdate( *best_handover_conf.first );
        robot_model_->setAndUpdate( *best_handover_conf.second );
        target_new_ = best_handover_conf.second;
        //g3d_draw_allwin_active();
        //cout << "Hand-over found with cost : " << best_cost << endl;
    }
    
    return found_hover;
}

bool StompOptimizer::getManipulationHandOver()
{
    std::vector<Eigen::Vector3d> points;
    
    if( use_handover_auto_ ) {
        HRICS_humanCostMaps->getHandoverPointList( points, recompute_handover_cell_list_, true );
        recompute_handover_cell_list_ = false;
    }
    else {
        points.resize(1);
        points[1] = human_model_->getJoint("rPalm")->getVectorPos();
        points[1][2] += 0.10;
    }
    
    // Compute IK
    bool found_ik = false;
    configPt q = NULL;
    ChronoTimeOfDayOn(); double time=0.0;
    for (int i=0; i<int(points.size()) && time<1.0 && found_ik==false; i++) {
        found_ik = handoverGenerator_->computeRobotIkForGrabing( q, points[i] );
        ChronoTimeOfDayTimes(&time);
    }
    ChronoTimeOfDayOff();
    
    // Disable cntrts
    p3d_cntrt* ct;
    p3d_rob* rob = robot_model_->getRobotStruct();
    for(int i=0; i<rob->cntrt_manager->ncntrts; i++) {
        ct = rob->cntrt_manager->cntrts[i];
        p3d_desactivateCntrt( rob, ct );
    }
    
    ArmManipulationData& armData  = (*rob->armManipulationData)[0];
    
    deactivateCcCntrts(rob, 0);
    setAndActivateTwoJointsFixCntrt(rob,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
    
    if( found_ik ) {
        target_new_ = confPtr_t(new Configuration( robot_model_, q ));
        robot_model_->setAndUpdate( *target_new_ );
        //g3d_draw_allwin_active();
    }
    
    return found_ik;
}

bool StompOptimizer::humanHasMoved()
{
    if( human_model_ == NULL ) {
        return false;
    }
    
    human_has_moved_ = false;
    
    confPtr_t q_hum = human_model_->getCurrentPos();
    
    (*q_hum)[6] =  PlanEnv->getDouble(PlanParam::env_futurX);
    (*q_hum)[7] =  PlanEnv->getDouble(PlanParam::env_futurY);
    (*q_hum)[8] =  PlanEnv->getDouble(PlanParam::env_futurZ);
    (*q_hum)[11] = PlanEnv->getDouble(PlanParam::env_futurRZ);
    
    if( last_human_pos_[0] != (*q_hum)[6] ||
            last_human_pos_[1] != (*q_hum)[7] ||
            last_human_pos_[2] != (*q_hum)[8] ||
            last_human_pos_[3] != (*q_hum)[11] )
    {
        human_has_moved_ = true;
        human_model_->setAndUpdate(*q_hum);
    }
    
    last_human_pos_[0] = (*q_hum)[6];
    last_human_pos_[1] = (*q_hum)[7];
    last_human_pos_[2] = (*q_hum)[8];
    last_human_pos_[3] = (*q_hum)[11];
    
    return human_has_moved_;
}

bool StompOptimizer::getNewTargetFromHandOver()
{
    if( handoverGenerator_ == NULL ) {
        return false;
    }
    
    reset_reused_rollouts_=human_has_moved_;
    
    if( !reset_reused_rollouts_ ) {
        return true;
    }
    
    bool success = false;
    if( use_handover_config_list_ )
    {
        success=getMobileManipHandOver();
    }
    else
    {
        success=getManipulationHandOver();
    }
    return success;
}

bool StompOptimizer::replaceEndWithNewConfiguration()
{
    // Retrieve a new configuration
    if( !getNewTargetFromHandOver() ) {
        cout << "Fail to get new target to handover configuration" << endl;
        return false;
    }
    
    if( !reset_reused_rollouts_ ) {
        // Do not replace group trajectory
        return false;
    }
    
    // Generate new trajectory
    API::CostOptimization T( robot_model_ );
    setGroupTrajectoryToApiTraj( T );
    double step = T.getRangeMax()/20;
    
    // Try connection
    if( T.connectConfigurationToEnd( target_new_, step ) )
    {
        target_ = target_new_;

        // Calculate the forward kinematics for the fixed states only in the first iteration
        int start = free_vars_start_;
        int end = free_vars_end_;
        if (iteration_==0)
        {
            start = 0;
            end = num_vars_all_-1;
        }

        double param = 0.0;
        double step = T.getRangeMax()/(num_vars_free_-1);

        vector<confPtr_t> vect(num_vars_all_);
        for (int i=start; i<=end; ++i)
        {
            vect[i] = T.configAtParam( param );
            param += step;
        }
        setGroupTrajectoryFromVectorConfig( vect );
        return true;
    }
    else {
        cout << "Fail to connect to handover configuration" << endl;
        return false;
    }
}

void StompOptimizer::draw()
{
    // Draws two points in the trajectory
    //    int middle = (free_vars_start_+free_vars_end_)/2;
    
    //    int start = middle - 3;
    //    int end = middle + 3;
    
    //    planning_group_->draw(segment_frames_[start]);
    //    planning_group_->draw(segment_frames_[end]);

    //    planning_group_->draw();

    if( collision_space_ )
    {
        //drawCollisionPoints();
    }
}

void StompOptimizer::drawCollisionPoints()
{
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    confPtr_t q = robot_model_->getCurrentPos();

    // Get the configuration dof values in the joint array
    Eigen::VectorXd joint_array(planning_group_->num_joints_);

    for(int j=0; j<planning_group_->num_joints_;j++)
    {
        joint_array[j] = (*q)[ joints[j].move3d_dof_index_ ];
    }

    getFrames( free_vars_start_, joint_array, *q );

    if( collision_space_ )
    {
        // calculate the position of every collision point:
        for (int j=0; j<num_collision_points_; j++)
        {
            const CollisionPoint& cp = planning_group_->collision_points_[j];
            Eigen::Transform3d T;
            //cout << "segment (" << j << ") : " << cp.getSegmentNumber() << endl;
            stdVectorToEigenTransform( segment_frames_[free_vars_start_][cp.getSegmentNumber()], T );
            cp.draw( T );
        }
    }
}

void StompOptimizer::visualizeState(int index)
{
    
}

void StompOptimizer::testMultiVariateGaussianSampler()
{
    PolicyImprovementLoop pi_loop;
    pi_loop.initialize( this_shared_ptr_, false );
    pi_loop.testSampler();
}

bool StompOptimizer::initialize(/*ros::NodeHandle& node_handle,*/ int num_time_steps)
{
    //  // we already know these things, so do nothing
    return true;
}

/*
   void StompOptimizer::getTorques(int index, std::vector<double>& torques, const std::vector<KDL::Wrench>& wrenches)
   {
   int i=index;
   group_trajectory_.getTrajectoryPointKDL(i, kdl_group_joint_array_);
   group_trajectory_.getJointVelocities(i, joint_state_velocities_);
   group_trajectory_.getJointAccelerations(i, joint_state_accelerations_);
   for (int j=0; j<num_joints_; ++j)
   {
   //kdl_group_joint_array_(j) = kdl_joint_array_(full_joint_num);
   kdl_group_vel_joint_array_(j) = joint_state_velocities_(j);
   kdl_group_acc_joint_array_(j) = joint_state_accelerations_(j);
   }
   //  ROS_INFO("Index = %d", index);
   //  ROS_INFO_STREAM("Input = " << kdl_group_joint_array_.data<< " | "
   //                  << kdl_group_vel_joint_array_.data << " | "
   //                  << kdl_group_acc_joint_array_.data);
   planning_group_->id_solver_->CartToJnt(kdl_group_joint_array_,
   kdl_group_vel_joint_array_,
   kdl_group_acc_joint_array_,
   wrenches,
   kdl_group_torque_joint_array_);
   //  ROS_INFO_STREAM("Output = " << kdl_group_torque_joint_array_.data);
   
   for (int j=0; j<num_joints_; ++j)
   {
   torques[j] = kdl_group_torque_joint_array_(j);
   }
   
   }
   */

//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------

confPtr_t StompOptimizer::getSource()
{
    return getConfigurationOnGroupTraj(free_vars_start_);
}

confPtr_t StompOptimizer::getTarget()
{
    return getConfigurationOnGroupTraj(free_vars_end_);
}

confPtr_t StompOptimizer::getConfigurationOnGroupTraj(int ith)
{
    confPtr_t q = planning_group_->robot_->getCurrentPos();
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;

    Eigen::VectorXd point = group_trajectory_.getTrajectoryPoint(ith).transpose();
    
    for(int j=0; j<planning_group_->num_joints_;j++)
    {
        (*q)[joints[j].move3d_dof_index_] = point[j];
    }
    return q;
}

double StompOptimizer::computeMove3DCost()
{
    // calculate the forward kinematics for the fixed states only in the first iteration:
    API::Trajectory T(robot_model_);
    //const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    int ith_point=0;
    
    // for each point in the trajectory
    for (int i=free_vars_start_; i<=free_vars_end_; ++i)
    {
        T.push_back(getConfigurationOnGroupTraj(i));
        ith_point++;
    }
    
    // cout << "T.getRangeMax() : " << T.getRangeMax() << endl;
    // cout << "T.getNbOfPaths() : " << T.getNbOfPaths() << endl;
    // cout << "T.cost() : " << T.cost() << endl;
    // T.print();
    // T.replaceP3dTraj();
    // return T.cost();
    // return T.costNPoints(ith_point);
    return T.costSum();
}

double StompOptimizer::resampleParameters(std::vector<Eigen::VectorXd>& parameters)
{
    const std::vector<ChompJoint>& joints = planning_group_->chomp_joints_;
    API::Smoothing traj(planning_group_->robot_);
    Eigen::MatrixXd parameters_tmp(num_joints_,num_vars_free_);
    
    for ( int i=0; i<num_joints_; ++i) {
        parameters_tmp.row(i) = parameters[i].transpose();
    }

    traj.clear();
    for (int j=0; j<num_vars_free_; ++j)
    {
        // Set the configuration from the stored source and target
        confPtr_t q;

        if( j==0 ) {
            q = source_;
        }
        else if( j == num_vars_free_-1 ) {
            q = target_;
        }
        else
        {
            //q = passive_dofs_[j];
            q = planning_group_->robot_->getCurrentPos();

            for ( int i=0; i<planning_group_->num_joints_; ++i)
            {
                (*q)[joints[i].move3d_dof_index_] = parameters_tmp(i,j);
            }
        }

        traj.push_back(q);
    }
    
    PlanEnv->setBool(PlanParam::trajStompComputeColl, false );
    traj.runShortCut(15);
    PlanEnv->setBool(PlanParam::trajStompComputeColl, true );
    
    // calculate the forward kinematics for the fixed states only in the first iteration:
    double step = traj.getRangeMax() / (num_vars_free_-1);
    double param = step;
    for (int j=0; j<num_vars_free_; ++j)
    {
        confPtr_t q = traj.configAtParam(param);

        for ( int i=0; i<this->getPlanningGroup()->num_joints_; ++i )
        {
            parameters_tmp(i,j) = (*q)[joints[i].move3d_dof_index_];
        }
        param += step;
    }
    
    for ( int i=0; i<num_joints_; ++i) {
        parameters[i].transpose() = parameters_tmp.row(i);
    }
    
    return traj.cost();
}



void StompOptimizer::initPolicy()
{
    policy_.reset(new CovariantTrajectoryPolicy());
    policy_->setPrintDebug( false );
    
    std::vector<double> derivative_costs = stomp_parameters_->getSmoothnessCosts();
    
    policy_->initialize(/*nh,*/
                        num_vars_free_,
                        num_joints_,
                        group_trajectory_.getDuration(),
                        stomp_parameters_->getRidgeFactor(),
                        derivative_costs,
                        planning_group_);
    
    // initialize the policy trajectory
    Eigen::VectorXd start = group_trajectory_.getTrajectoryPoint(free_vars_start_-1).transpose();
    Eigen::VectorXd end = group_trajectory_.getTrajectoryPoint(free_vars_end_+1).transpose();
    
    // set paramters
    int free_vars_start_index = DIFF_RULE_LENGTH - 1;
    vector< Eigen::VectorXd > parameters(num_joints_);
    for (int i=0; i<num_joints_; i++)
    {
        parameters[i] =  group_trajectory_.getJointTrajectory(i).segment(free_vars_start_index, num_vars_free_);
    }
    policy_->setToMinControlCost(start, end);
    policy_->setParameters( parameters );
}

bool StompOptimizer::getPolicy(MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy>& policy)
{
    policy = policy_;
    return true;
}

bool StompOptimizer::setPolicy(const MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::Policy> policy)
{
    return true;
}

bool StompOptimizer::getControlCostWeight(double& control_cost_weight)
{
    control_cost_weight = stomp_parameters_->getSmoothnessCostWeight();
    return true;
}

bool StompOptimizer::getStateCostWeight(double& state_cost_weight)
{
    state_cost_weight = stomp_parameters_->getObstacleCostWeight();
    return true;
}

void StompOptimizer::copyPolicyToGroupTrajectory()
{
    policy_->getParameters(policy_parameters_);
    
    for (int d=0; d<num_joints_; ++d)
    {
        //cout << "group_trajectory_ : " << endl << group_trajectory_.getFreeJointTrajectoryBlock(d) << endl;
        //cout << "policy_parameters_ : " << endl << policy_parameters_[d] << endl;
        group_trajectory_.getFreeJointTrajectoryBlock(d) = policy_parameters_[d];
    }
}

void StompOptimizer::copyGroupTrajectoryToPolicy()
{
    for (int d=0; d<num_joints_; ++d)
    {
        //policy_parameters_[d] = group_trajectory_.getFreeTrajectoryBlock();
        policy_parameters_[d] = group_trajectory_.getFreeJointTrajectoryBlock(d);
    }
    
    // draw traj
    //    const std::vector<ChompJoint>& joints = optimizer->getPlanningGroup()->chomp_joints_;
    //    Robot* robot = optimizer->getPlanningGroup()->robot_;
    //    API::Trajectory traj(robot);
    //
    //    for ( int j=0; j<policy_parameters_[0].size(); ++j)
    //    {
    //      confPtr_t q = robot->getCurrentPos();
    //
    //      for ( int i=0; i<optimizer->getPlanningGroup()->num_joints_; ++i)
    //      {
    //        (*q)[joints[i].move3d_dof_index_] = policy_parameters_[i][j];
    //      }
    //      traj.push_back(q);
    //    }
    //    traj.replaceP3dTraj();
    //    //    traj.print();
    //    g3d_draw_allwin_active();
    //    cout << "num_time_steps_ : " << policy_parameters_[0].size() << endl;
    
    policy_->setParameters(policy_parameters_);
}

void StompOptimizer::setSharedPtr( MOVE3D_BOOST_PTR_NAMESPACE<StompOptimizer>& ptr )
{
    this_shared_ptr_ = ptr;
}

void StompOptimizer::resetSharedPtr()
{
    this_shared_ptr_.reset();
}

void StompOptimizer::saveTrajectoryCostStats()
{
    //    TrajectoryStatistics stat;
    //    setGroupTrajectoryToApiTraj( move3d_traj_ );
    //    move3d_traj_.costStatistics( stat );
    //    stomp_statistics_->convergence_rate.push_back( make_pair( time_, stat ));
    
    vector<confPtr_t> traj;
    setGroupTrajectoryToVectorConfig( traj );
    stomp_statistics_->convergence_trajs.push_back( make_pair( time_, traj) );
    //    move3d_traj_.replaceP3dTraj();
    //    g3d_draw_allwin_active();
}

void StompOptimizer::saveCostFromConvergenceTraj()
{
    stomp_statistics_->convergence_rate.clear();
    
    for (int i=0; i<int(stomp_statistics_->convergence_trajs.size()); i++)
    {
        double time = stomp_statistics_->convergence_trajs[i].first;
        API::Trajectory traj( stomp_statistics_->convergence_trajs[i].second );

        TrajectoryStatistics stat;
        traj.costStatistics( stat );
        stomp_statistics_->convergence_rate.push_back( make_pair( time, stat ));
    }
}

/*!
   * This function saves the
   * trajectory cost to a file along the smoothing process
   */
void StompOptimizer::saveOptimToFile(string fileName)
{
    saveCostFromConvergenceTraj();
    
    std::ostringstream oss;
    std::ofstream s;
    
    //----------------------------------------------------
    oss << "statFiles/convergence_traj_stomp_" << std::setfill('0') << std::setw(4) << stomp_statistics_->run_id << ".csv";
    
    const char *res = oss.str().c_str();
    
    s.open(res);
    cout << "Opening save file : " << res << endl;
    
    s << "TIME" << ";";
    s << "LENGTH" << ";";
    s << "MAX" << ";";
    s << "AVERAGE" << ";";
    s << "INTEGRAL" << ";";
    s << "MECHA WORK" << ";";
    s << "IS_VALID" << ";";
    s << "SUM" << ";";
    s << endl;
    
    for (int i=0; i<int(stomp_statistics_->convergence_rate.size()); i++)
    {
        s << stomp_statistics_->convergence_rate[i].first << ";";
        s << stomp_statistics_->convergence_rate[i].second.length << ";";
        s << stomp_statistics_->convergence_rate[i].second.max << ";";
        s << stomp_statistics_->convergence_rate[i].second.average << ";";
        s << stomp_statistics_->convergence_rate[i].second.integral << ";";
        s << stomp_statistics_->convergence_rate[i].second.mecha_work << ";";
        s << stomp_statistics_->convergence_rate[i].second.is_valid << ";";
        s << stomp_statistics_->convergence_rate[i].second.sum << ";";
        s << endl;
    }
    
    s.close();
    cout << "Closing save file" << endl;
    
    //----------------------------------------------------
    std::ostringstream oss2;
    oss2.clear();
    oss2 << "statFiles/costsum_traj_stomp_" << std::setfill('0') << std::setw(4) << stomp_statistics_->run_id << ".csv";
    
    const char *res2 = oss2.str().c_str();
    
    s.open(res2);
    cout << "Opening save file : " << res2 << endl;
    
    s << "Cost" ;
    s << endl;
    
    for (int i=0; i<int(stomp_statistics_->costs.size()); i++)
    {
        s << stomp_statistics_->costs[i] ;
        s << endl;
    }
    
    cout << "Closing save file" << endl;
    s.close();
}


} // namespace stomp

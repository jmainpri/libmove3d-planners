#include "trajOptimizer.hpp"
#include "planEnvironment.hpp"
#include "API/Graphic/drawModule.hpp"

#include <time.h>
#include <sys/time.h>

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Util-pkg.h>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;

using std::cout;
using std::endl;

TrajectoryOptimizer::TrajectoryOptimizer()
{
    stomp_parameters_ = MOVE3D_BOOST_PTR_NAMESPACE<stomp_motion_planner::StompParameters>(new stomp_motion_planner::StompParameters());
    stomp_parameters_->init();

    use_time_limit_ = false;
    use_iteration_limit_ = true;
    use_collision_free_limit_limit_  = false;
}

void TrajectoryOptimizer::runDeformation( int nbIteration, int idRun )
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

    move3d_traj_ = Move3D::Trajectory( robot_model_ );
    traj_convergence_with_time.clear();

    if( !PlanEnv->getBool(PlanParam::trajStompNoPrint) )
        cout << "use_time_limit_ : " << use_time_limit_ << endl;

    initialize( false, group_trajectory_.use_time_ ? group_trajectory_.discretization_ : 0.0 );

    iteration_ = 0;
    copyPolicyToGroupTrajectory();
    handleJointLimits();
    updateFullTrajectory();
    performForwardKinematics(false);

    // Print smoothness cost
    getStateCost();
    getSmoothnessCost();
    getGeneralCost();

    if ( (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) && stomp_parameters_->getAnimateEndeffector() )
        animateEndeffector();

    best_iteration_=0;
    last_improvement_iteration_ = -1;
    best_group_trajectory_cost_                 = std::numeric_limits<double>::max();
    best_group_trajectory_in_collsion_cost_     = std::numeric_limits<double>::max();

    Move3D::confPtr_t q_tmp = robot_model_->getCurrentPos();

    cout << "Start loop" << endl;

    // iterate
    for ( iteration_=0; !PlanEnv->getBool(PlanParam::stopPlanner); iteration_++)
    {
        // Compute wether to draw at a certain iteration
        bool do_draw = false;
        if( PlanEnv->getInt(PlanParam::stompDrawIteration) >  0) {
            do_draw = (iteration_!=0) && (iteration_%PlanEnv->getInt(PlanParam::stompDrawIteration) == 0);
        }

        reset_reused_rollouts_ = false;

        //! RUN THE ITERATION
        run_single_iteration();

        if ( do_draw && (!ENV.getBool(Env::drawDisabled)) && ENV.getBool(Env::drawTraj) && stomp_parameters_->getAnimateEndeffector() )
        {
            move3d_draw_clear_handles( robot_model_ );
            robot_model_->setAndUpdate(*q_tmp);
            animateEndeffector();
            //animateTrajectoryPolicy();
        }

        double cost = getTrajectoryCost();

        //double cost = last_trajectory_cost_;
        stomp_statistics_->costs.push_back(cost);

        if( reset_reused_rollouts_ )
        {
            best_group_trajectory_ = group_trajectory_;
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
            best_group_trajectory_ = group_trajectory_;
            best_group_trajectory_cost_ = cost;
        }
        else
        {
            if( cost < best_group_trajectory_cost_
              /*|| ( PlanEnv->getBool(PlanParam::trajStompDrawImprovement) && (
                cost < best_group_trajectory_in_collsion_cost_ ))*/ )
            {
                if ( is_collision_free_ )
                {
                    if( !PlanEnv->getBool(PlanParam::trajStompNoPrint) )
                        cout << "New best" << endl;

                    best_group_trajectory_ = group_trajectory_;
                    best_group_trajectory_cost_ = cost;
                    last_improvement_iteration_ = iteration_;
                    best_iteration_++;
                }
                else
                {
                    if( !PlanEnv->getBool(PlanParam::trajStompNoPrint) )
                        cout << "New best in collision" << endl;

                    best_group_trajectory_in_collision_ = group_trajectory_;
                    best_group_trajectory_in_collsion_cost_ = cost;
                }
            }
        }

        //if (iteration_%1==0)
        gettimeofday(&tim, NULL);
        time_ = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;

//        cout << "move3d_cost : " << computeMove3DCost() << endl;
        //last_move3d_cost_ = computeMove3DCost();

        double move3d_cost =0.0;

//        if( PlanEnv->getBool(PlanParam::drawParallelTraj) && ( global_stompRun != NULL ))
//        {
//            ENV.setBool(Env::drawTraj,false);
//            global_stompRun->lockDraw();
//            robot_model_->setAndUpdate(*q_tmp);
//            saveEndeffectorTraj();
//            global_stompRun->unlockDraw();
//        }

        // save the cost and time as pair
        // traj_convergence_with_time.push_back( make_pair( time_, move3d_cost ) );

        if( !PlanEnv->getBool(PlanParam::trajStompNoPrint) )
        {
            if( (!ENV.getBool(Env::drawDisabled)) && ( ENV.getBool(Env::drawTraj) || PlanEnv->getBool(PlanParam::drawParallelTraj) ))
            {
                printf( "%3d, time: %3f, cost: %f (s=%f, c=%f, g=%f), move3d cost: %f\n", iteration_, time_, cost,
                        getSmoothnessCost(), getStateCost(), getGeneralCost(), move3d_cost );
            }
            else {
                printf( "%3d, time: %3f, cost: %f\n", iteration_, time_, cost );
            }
        }

        if( use_time_limit_ )

            if( time_ >= stomp_parameters_->max_time_ )
            {
                cout << "Stopped at time limit (" << time_ << "), limit : " << stomp_parameters_->max_time_ << endl;
                break;
            }


        if( use_iteration_limit_ )

            if( iteration_ >= stomp_parameters_->max_iterations_ )
            {
                cout << "Stopped at iteration (" << iteration_ << "), limit " << stomp_parameters_->max_iterations_ << endl;
                break;
            }


        if( use_collision_free_limit_limit_ )

            if( is_collision_free_ )
            {
                cout << "Stopped at iteration (" << iteration_ << "), because found collision free" << endl;
                break;
            }
    }

    cout << " -------------------------------------- " << endl;

    if (best_iteration_ > 0)
        cout << "Found a collision free path!!! " << endl;
    else
        cout << "NO collision free path found!!! " << endl;

    if (last_improvement_iteration_>-1)
        cout << "We think the last path is collision free: " << is_collision_free_ << endl;


    if( best_iteration_ > 0 )

        group_trajectory_ = best_group_trajectory_;

    else if( last_improvement_iteration_ > -1 )

        group_trajectory_ = best_group_trajectory_in_collision_;

    // Run code on the virtual class
    end();

    // convert to move3d trajectory
    best_traj_ = group_trajectory_.getMove3DTrajectory();
    robot_model_->setCurrentMove3DTraj( best_traj_ );
    robot_model_->getCurrentMove3DTraj().replaceP3dTraj();

    // Best path cost
    printf("Best trajectory : iter=%3d, cost (s=%f, c=%f, g=%f)\n", last_improvement_iteration_, getSmoothnessCost(), getStateCost(), getGeneralCost() );

    printf("Collision free success iteration = %d for robot %s (time : %f)\n",
           stomp_statistics_->collision_success_iteration, robot_model_->getName().c_str(), stomp_statistics_->success_time);
    printf("Terminated after %d iterations, using path from iteration %d\n", iteration_, last_improvement_iteration_);
    printf("Best cost = %f\n", best_group_trajectory_cost_);
    printf("Lamp has run for : %f sec\n", time_ );

    //printf("Optimization core finished in %f sec", (ros::WallTime::now() - start_time).toSec());
    stomp_statistics_->best_cost = best_group_trajectory_cost_;

    ChronoTimeOfDayOff();
}

SamplingBasedInnerLoop::SamplingBasedInnerLoop( Move3D::Robot* robot ) : sampler_(robot)
{
    robot_model_ = robot;
    Move3D::confPtr_t q_source = robot_model_->getInitPos();
    Move3D::confPtr_t q_target = robot_model_->getGoalPos();

    initial_traj_seed_ = Move3D::Trajectory( robot_model_ );
    initial_traj_seed_.push_back( q_source );
    initial_traj_seed_.push_back( q_target );
    robot->setCurrentMove3DTraj( initial_traj_seed_ );

    double duration     = PlanEnv->getDouble(PlanParam::trajDuration);
    int nb_points       = PlanEnv->getInt(PlanParam::nb_pointsOnTraj);
    sampler_.initialize( stomp_parameters_->getSmoothnessCosts() , nb_points );
    planning_group_ = sampler_.planning_group_;
    group_trajectory_ = Move3D::VectorTrajectory( planning_group_->chomp_dofs_.size(), nb_points, duration );
    group_trajectory_.planning_group_ = planning_group_;
    group_trajectory_.setFromMove3DTrajectory( initial_traj_seed_ );

    num_rollouts_ = PlanEnv->getInt(PlanParam::lamp_nb_samples);
    num_rollouts_gen_ = num_rollouts_;
    num_rollouts_reused_ = 0;
    rollouts_reused_next_ = false;
    reused_rollouts_.clear();


    // Cost computer...
    move3d_collision_space_ = global_collisionSpace;

    bool use_costspace = move3d_collision_space_ == NULL;
    bool use_external_collision_space = false;
    int collision_space_id = 0;
    stomp_parameters_const_ = stomp_parameters_.get();

    cost_computer_ = Move3D::LampCostComputation(robot_model_,
                                                 move3d_collision_space_,
                                                 planning_group_,
                                                 group_trajectory_,
                                                 stomp_parameters_->getObstacleCostWeight(),
                                                 use_costspace,
                                                 q_source,
                                                 q_target,
                                                 use_external_collision_space,
                                                 collision_space_id,
                                                 stomp_parameters_const_,
                                                 &sampler_.policy_,
                                                 sampler_.control_ );


    if( !use_external_collision_space )
    {
        cost_computer_.set_fct_get_nb_collision_points( boost::bind( &Move3D::LampCostComputation::getNumberOfCollisionPoints, &cost_computer_, _1 ), collision_space_id );
        cost_computer_.set_fct_get_config_collision_cost( boost::bind( &Move3D::LampCostComputation::getConfigObstacleCost, &cost_computer_, _1, _2, _3, _4 ), collision_space_id );
    }
}

void SamplingBasedInnerLoop::animateEndeffector()
{
    robot_model_->setCurrentMove3DTraj( group_trajectory_.getMove3DTrajectory() );
    robot_model_->getCurrentMove3DTraj().replaceP3dTraj();

    // cout << "traj : " << group_trajectory_.trajectory_.transpose() << endl;

    g3d_draw_allwin_active();
}

double SamplingBasedInnerLoop::getTrajectoryCost( Move3D::VectorTrajectory& traj, bool check_joint_limits )
{
    double coll_cost =  getStateCost( traj, check_joint_limits );
    double cont_cost =  getSmoothnessCost( traj );

    traj.total_cost_ = coll_cost + cont_cost;

    return traj.total_cost_;
}

double SamplingBasedInnerLoop::getStateCost( Move3D::VectorTrajectory& traj, bool check_joint_limits )
{
//    double collision_cost = 0.;
//    for( int i=0; i<traj.getNumPoints(); i++)
//    {
//        collision_cost += traj.getMove3DConfiguration( i )->cost();
//    }

    Eigen::VectorXd costs(Eigen::VectorXd::Zero(traj.getNumPoints()));

    // Computes costs and joint limits
    cost_computer_.getCost( traj, costs, iteration_, check_joint_limits, false, false );

    // Get start cost
    traj.state_costs_ = costs;

    // Get out of bounds
    traj.out_of_bounds_ = !cost_computer_.getJointLimitViolationSuccess();

    // Return sum of state costs
    return costs.sum();
}

double SamplingBasedInnerLoop::getSmoothnessCost(const Move3D::VectorTrajectory& traj)
{
    // return PlanEnv->getDouble(PlanParam::lamp_control_cost) * traj.trajectory_.transpose() * sampler_.control_ * traj.trajectory_;

    const std::vector<Eigen::VectorXd>& control_costs = cost_computer_.getControlCosts();

    double cost = 0.;
    for (size_t d=0; d<control_costs.size(); ++d)
        cost += control_costs[d].sum();

    return cost;
}

double SamplingBasedInnerLoop::getTrajectoryCost()
{
    return getTrajectoryCost( group_trajectory_ );
}

double SamplingBasedInnerLoop::getSmoothnessCost()
{
    return getSmoothnessCost( group_trajectory_ );
}

double SamplingBasedInnerLoop::getStateCost()
{
    return getStateCost( group_trajectory_ );
}

Eigen::VectorXd SamplingBasedInnerLoop::generateSamples()
{
    std::vector<Move3D::VectorTrajectory>  samples = sampler_.sampleTrajectories( num_rollouts_gen_, group_trajectory_ );

    Eigen::VectorXd costs( num_rollouts_ );
    for( size_t i=0; i<samples.size(); i++ )
    {
        costs[i] = getTrajectoryCost( samples[i], true ) ; //- group_trajectory_cost;
    }

    // The rollouts of the run are not used
    if( !rollouts_reused_next_ )
    {
        num_rollouts_gen_ = num_rollouts_;

        if( num_rollouts_reused_ > 0 )
            rollouts_reused_next_ = true;
    }
    else
    {
        // we assume here that rollout_parameters_ and rollout_noise_ have already been allocated
        num_rollouts_gen_ = num_rollouts_ - num_rollouts_reused_;

        for( size_t r=0; r<reused_rollouts_.size(); ++r )
            samples.push_back( reused_rollouts_[r] );

        // figure out which rollouts to reuse
        std::vector< std::pair<double,int> > rollout_cost_sorter;
        for( size_t r=0; r<samples.size(); ++r)
        {
            double cost = costs[r];

            // discard out of bounds rollouts
            if ( samples[r].out_of_bounds_ )
                cost = std::numeric_limits<double>::max();

            rollout_cost_sorter.push_back( std::make_pair(cost,r) );
        }
        std::sort( rollout_cost_sorter.begin(), rollout_cost_sorter.end() );


        // use the best ones: (copy them into reused_rollouts)
        reused_rollouts_.resize( num_rollouts_reused_ );

        for( int r=0; r<num_rollouts_reused_; ++r )
        {
            int reuse_index = rollout_cost_sorter[r].second;

            if( reuse_index >=0 )
                reused_rollouts_[r] = samples[ reuse_index ];
        }

        // copy them back from reused_rollouts_ into rollouts_
        if( samples.size() >= ( num_rollouts_gen_ + reused_rollouts_.size() ) ) //! TODO fix this
        {
            for( int r=0; r<num_rollouts_reused_; ++r)
                samples[num_rollouts_gen_+r] = reused_rollouts_[r];

            for( int i=0; i<num_rollouts_; i++ )
                costs[i] = samples[i].total_cost_ ; //- group_trajectory_cost;
        }
    }

    samples_ = samples;

    return costs;
}

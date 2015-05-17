#include "lampSimpleLoop.hpp"
#include "lampComputeCost.hpp"
#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

void lamp_simple_loop()
{
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    Move3D::LampSimpleLoop optimizer( robot );
    optimizer.runDeformation( 50, 0 );
}

LampSimpleLoop::LampSimpleLoop( Move3D::Robot* robot ) : sampler_(robot)
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
    group_trajectory_ = Move3D::LampTrajectory( planning_group_->chomp_dofs_.size(), nb_points, duration );
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

bool LampSimpleLoop::initialize(bool single_rollout, double discretization)
{

}

void LampSimpleLoop::end()
{
    cost_computer_.print_time();
}

void LampSimpleLoop::run_single_iteration()
{
    std::vector<bool> succeeded( samples_.size(), false );
    Eigen::VectorXd costs( generateSamples() );
    Eigen::MatrixXd deltas( samples_.size(), group_trajectory_.trajectory_.size() );

    int nb_valid_samples= 0;
    double group_trajectory_cost = getTrajectoryCost();

    // Compute cost
    for( int i=0; i<samples_.size(); i++)
    {
        deltas.row(i) = samples_[i].trajectory_ - group_trajectory_.trajectory_;

        if( !samples_[i].out_of_bounds_ )
        {
            succeeded[i] = true;
            nb_valid_samples++;
        }

        if( ENV.getBool(Env::drawTrajVector) && ENV.getBool(Env::drawTraj) )
        {
            global_trajToDraw.push_back( samples_[i].getMove3DTrajectory() );
            global_trajToDraw.back().setColor( i );
        }
    }


    Eigen::VectorXd baises( group_trajectory_.trajectory_.size() );

    for( int j=0; j<group_trajectory_.trajectory_.size(); j++ )
    {
        double b_i_num = 0.;
        double b_i_den = 0.;

        for( int i=0; i<samples_.size(); i++)
        {
            b_i_num += costs[i] * std::pow( deltas(i, j), 2. );
            b_i_den += std::pow( deltas(i, j), 2. );
        }

        baises[j] = b_i_num / b_i_den;
    }

    cout << "nb_valid_samples : " << nb_valid_samples << endl;

    if( nb_valid_samples > 0 )
    {
        LampTrajectory new_traj( group_trajectory_ );
        new_traj.trajectory_ = Eigen::VectorXd::Zero( group_trajectory_.trajectory_.size() );

        // Compute update
        // double delta = 0.;
        for( int i=0; i<samples_.size(); i++)
        {
            if( succeeded[i] )
            {
                Eigen::MatrixXd m( Eigen::MatrixXd::Identity( baises.size() , baises.size() ));
                Eigen::MatrixXd b( baises.asDiagonal() );
                new_traj.trajectory_ += ( ( costs[i] * m ) - b) * deltas.row(i).transpose();
                // delta += deltas.row(i).norm();
            }
            else
            {
                costs[i] = 0.;
            }
        }
        // new_traj.trajectory_ /= double(nb_valid_samples);

        double eta =  1. / ( costs.array().abs().sum() * ( 1. + 1 / (0.8 * std::sqrt( double(nb_valid_samples) ) ) ) );


        new_traj.trajectory_ *= ( PlanEnv->getDouble(PlanParam::lamp_eta) * eta );

        // cout << "delta : " << delta << endl;
        // Set update
        group_trajectory_.trajectory_ -= new_traj.trajectory_;

        getTrajectoryCost( group_trajectory_, true ) ; //- group_trajectory_cost;
    }
}

void LampSimpleLoop::normalize_update( const Eigen::MatrixXd& deltas, const std::vector<bool>& valid, Eigen::VectorXd& update ) const
{
    Eigen::MatrixXd deltas_tmp = deltas;

    // 1. Get only valid samples
    for( int i=0; i<deltas.rows(); i++ )
    {
        if( !valid[i] )
        {
            deltas_tmp.row(i) = Eigen::VectorXd::Zero( update.size() );
        }
    }

    double eta = 1.;

    // 2. For each time step
    for( int i=0; i<deltas_tmp.cols(); i++ )
    {
        // 3, Compute standard dev at time step i
        double delta_mean   = deltas_tmp.col(i).mean();
        double delta_sq_sum =  deltas_tmp.col(i).transpose() * deltas_tmp.col(i);
        double delta_stddev  = std::sqrt( (delta_sq_sum / double(deltas_tmp.col(i).size())) - (delta_mean * delta_mean) );

        // 4. Factor of the Stddev
        delta_stddev /= 2;

        // 5. Compute neta to not exceedes stddev
        if( std::fabs( eta * update[i] ) > delta_stddev )
        {
            eta = delta_stddev / std::fabs( update[i] );
        }
    }

    cout << "neta : " << eta << endl;

    update *= eta;
}

void LampSimpleLoop::animateEndeffector()
{
    robot_model_->setCurrentMove3DTraj( group_trajectory_.getMove3DTrajectory() );
    robot_model_->getCurrentMove3DTraj().replaceP3dTraj();

    // cout << "traj : " << group_trajectory_.trajectory_.transpose() << endl;

    g3d_draw_allwin_active();
}

double LampSimpleLoop::getTrajectoryCost( Move3D::LampTrajectory& traj, bool check_joint_limits )
{
    double coll_cost =  getCollisionCost( traj, check_joint_limits );
    double cont_cost =  getSmoothnessCost( traj );

    traj.cost_ = coll_cost + cont_cost;

    return traj.cost_;
}

double LampSimpleLoop::getCollisionCost( Move3D::LampTrajectory& traj, bool check_joint_limits )
{
//    double collision_cost = 0.;
//    for( int i=0; i<traj.getNumPoints(); i++)
//    {
//        collision_cost += traj.getMove3DConfiguration( i )->cost();
//    }

    Eigen::VectorXd costs(Eigen::VectorXd::Zero(traj.getNumPoints()));

    // Computes costs and joint limits
    cost_computer_.getCost( traj, costs, iteration_, check_joint_limits, false, false );

    // Get out of bounds
    traj.out_of_bounds_ = !cost_computer_.getJointLimitViolationSuccess();

    return costs.sum();
}

double LampSimpleLoop::getSmoothnessCost(const Move3D::LampTrajectory& traj)
{
    // return PlanEnv->getDouble(PlanParam::lamp_control_cost) * traj.trajectory_.transpose() * sampler_.control_ * traj.trajectory_;

    const std::vector<Eigen::VectorXd>& control_costs = cost_computer_.getControlCosts();

    double cost = 0.;
    for (size_t d=0; d<control_costs.size(); ++d)
        cost += control_costs[d].sum();

    return cost;
}

double LampSimpleLoop::getTrajectoryCost()
{
    return getTrajectoryCost( group_trajectory_ );
}

double LampSimpleLoop::getSmoothnessCost()
{
    return getSmoothnessCost( group_trajectory_ );
}

double LampSimpleLoop::getCollisionCost()
{
    return getCollisionCost( group_trajectory_ );
}

Eigen::VectorXd LampSimpleLoop::generateSamples()
{
    std::vector<LampTrajectory>  samples = sampler_.sampleTrajectories( num_rollouts_gen_, group_trajectory_ );

    Eigen::VectorXd costs( num_rollouts_ );
    for( int i=0; i<samples.size(); i++ )
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

        for( int r=0; r<reused_rollouts_.size(); ++r )
            samples.push_back( reused_rollouts_[r] );

        // figure out which rollouts to reuse
        std::vector< std::pair<double,int> > rollout_cost_sorter;
        for (int r=0; r<samples.size(); ++r)
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
                costs[i] = samples[i].cost_ ; //- group_trajectory_cost;
        }
    }

    samples_ = samples;

    return costs;
}

#include "lampSimpleLoop.hpp"
#include "lampComputeCost.hpp"
#include "lampStomp.hpp"

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

    // Move3D::LampSimpleLoop optimizer( robot );
    Move3D::Stomp optimizer( robot );


    optimizer.runDeformation( 50, 0 );

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
        VectorTrajectory new_traj( group_trajectory_ );
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

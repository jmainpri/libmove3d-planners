#include "lampStomp.hpp"
#include "lampComputeCost.hpp"
#include "API/project.hpp"
#include "planner/planEnvironment.hpp"

#include <libmove3d/p3d/env.hpp>
#include <libmove3d/include/Graphic-pkg.h>

using namespace Move3D;
using std::cout;
using std::endl;

void stomp_simple_loop()
{
    Move3D::Robot* robot = global_Project->getActiveScene()->getActiveRobot();
    Move3D::Stomp optimizer( robot );
    optimizer.runDeformation( 50, 0 );
}

bool Stomp::initialize(bool single_rollout, double discretization)
{
    probabilities_.resize( num_rollouts_ );
    precompute_projection_matrices();
    return true;
}

void Stomp::end()
{
    cost_computer_.print_time();
}

//! Precomputes the projection matrices M
//! using the inverse of the control cost matrix
//! Each column of the
bool Stomp::precompute_projection_matrices()
{
    projection_matrix_ = sampler_.covariance_;

    for (int p=0; p<sampler_.covariance_.cols(); ++p)
    {
        double column_max = sampler_.covariance_.col(p).maxCoeff();
        projection_matrix_.col(p) *= (1.0/(sampler_.covariance_.rows()*column_max));
    }

    move3d_save_matrix_to_file(projection_matrix_,"../matlab/m_mat.txt");

    return true;
}

void Stomp::run_single_iteration()
{
    std::vector<bool> succeeded( samples_.size(), false );
    Eigen::VectorXd costs( generateSamples() );
    std::vector<Move3D::VectorTrajectory> deltas( samples_.size() );

    int nb_valid_samples= 0;
//    double group_trajectory_cost =
            getTrajectoryCost();

    // Compute cost
    for( size_t i=0; i<samples_.size(); i++)
    {
        deltas[i] = Move3D::VectorTrajectory( samples_[i] );
        deltas[i].trajectory_ -= group_trajectory_.trajectory_;

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

    // cout << "nb_valid_samples : " << nb_valid_samples << endl;

    if( nb_valid_samples > 0 )
    {
        rollout_probabilities();

        Eigen::VectorXd parameter_updates( Eigen::VectorXd::Zero( group_trajectory_.trajectory_.size() ) );
        compute_parameter_updates( deltas, parameter_updates );

        // cout << "delta : " << delta << endl;
        // Set update
        group_trajectory_.trajectory_ += parameter_updates;

        getTrajectoryCost( group_trajectory_, true ) ; //- group_trajectory_cost;
    }
}

bool Stomp::rollout_probabilities()
{
    if( samples_.empty() )
        return false;

    int nb_dofs             = samples_[0].getNumDofs();
    int nb_free_points      = samples_[0].getNumFreePoints();

    // Initialize the probabilities
    for (int r=0; r<num_rollouts_; ++r)
    {
        probabilities_[r] = Eigen::VectorXd::Zero( nb_free_points * nb_dofs );
    }

    // TODO
    for (int d=0; d<nb_dofs; ++d)
    {
        // cout  << "dimension " << d << ", Cumulative costs " << rollouts_[0].cumulative_costs_[d] << endl;
        // tmp_min_cost_ = rollout_cumulative_costs_[d].colwise().minCoeff().transpose();
        // tmp_max_cost_ = rollout_cumulative_costs_[d].colwise().maxCoeff().transpose();
        // tmp_max_minus_min_cost_ = tmp_max_cost_ - tmp_min_cost_;

        for (int t=0; t<nb_free_points; t++)
        {
            // find min and max cost over all rollouts
//            double min_cost = samples_[0].dof_cost(t,d);
            double min_cost = samples_[0].state_costs_(t,d);
            double max_cost = min_cost;
            for (int r=1; r<num_rollouts_; ++r)
            {
                // double c = samples_[r].dof_cost(t,d);
                double c = samples_[r].state_costs_(t);
                if( c < min_cost )
                    min_cost = c;
                if( c > max_cost )
                    max_cost = c;
            }

            double denom = max_cost - min_cost;

            // prevent divide by zero:
            if( denom < 1e-8 )
                denom = 1e-8;

            double p_sum = 0.0;

            // the -10.0 here is taken from the paper
            for (int r=0; r<num_rollouts_; ++r)
            {
                int index = samples_[r].getVectorIndex( t, d );
                probabilities_[r](index) = exp(-10.0*(samples_[r].state_costs_(t) - min_cost)/denom);
                p_sum += probabilities_[r](index);
            }

            for (int r=0; r<num_rollouts_; ++r)
            {
                int index = samples_[r].getVectorIndex( t, d );
                probabilities_[r](index) /= p_sum;
            }
        }
    }

    return true;
}

bool Stomp::compute_parameter_updates(
        const std::vector<Move3D::VectorTrajectory> & deltas,
        Eigen::VectorXd& parameter_updates )
{
    // const bool draw_update = true;

    for( int r=0; r<num_rollouts_; ++r )
    {
        if( !samples_[r].out_of_bounds_ )
        {
            parameter_updates +=
                    deltas[r].trajectory_.cwiseProduct( probabilities_[r]);
        }
        else
        {
            // cout << "joint limits out of bounds" << endl;
        }
    }

    parameter_updates =  projection_matrix_ * parameter_updates;

//    if( draw_update )
//        addParmametersToDraw( parameters, 33 ); // 33 -> Orange

    return true;
}

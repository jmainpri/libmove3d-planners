#include "HRICS_legibility.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

Predictability::Predictability()
{

}

void Predictability::addGoal( const Eigen::VectorXd& g )
{
    goals_.push_back( g );
}

void Predictability::clearGoals()
{
    goals_.clear();
}

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

Legibility::Legibility() : Predictability()
{
    length_ = 1.0;
}

void Legibility::setTrajectory( const Eigen::MatrixXd& t )
{
    traj_ = t.transpose();
    q_source_ = traj_.col(0);

    // Compute all straight lines to goal cost
    c_g_.resize( goals_.size() );
    for(int i=0;i<int(goals_.size());i++)
    {
        //cout << "goals_[" << i << "] : " << goals_[i].transpose() << endl;
        straight_line_ = getInterpolatedTrajectory( q_source_, goals_[i], traj_.cols() - 2*(diff_rule_length_-1) );
        //cout << "straight_line_ : " << endl << straight_line_ << endl;
        c_g_[i] = cost( straight_line_ );
    }
}

double Legibility::legibilityCost( int ith )
{
    Eigen::VectorXd q_cur = traj_.col(ith);
    double t = double(ith) / double(traj_.cols()-1);

    int rows = traj_.rows();
    int cols = ith+1;

    Eigen::MatrixXd sub_traj_1 = resample( traj_.block( 0, 0, rows, cols ), traj_.cols() - 2*(diff_rule_length_-1) );
    Eigen::MatrixXd sub_traj_2 = getInterpolatedTrajectory( q_cur, goals_[0], traj_.cols() - 2*(diff_rule_length_-1) );

    double c_1 = cost( sub_traj_1 );
    double c_2 = cost( sub_traj_2 );

    double z=0.0;
    for( int i=0; i<int(goals_.size()); i++ )
    {
        straight_line_ = getInterpolatedTrajectory( q_cur, goals_[i], traj_.cols() - 2*(diff_rule_length_-1) );
        z += exp( c_g_[i] - cost( straight_line_ ) );
    }

    // Main cost function
    double prob = exp( -c_1 - c_2 ) / ( z * exp( -c_g_[0] ) );
    double cost = prob * ( length_ - t );

    return cost;
}

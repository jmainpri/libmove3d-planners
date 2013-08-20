#include "HRICS_Legibility.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

static const int DIFF_RULE_LENGTH = 7;
static const int NUM_DIFF_RULES = 3;

// the differentiation rules (centered at the center)
static const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {0, 0, -2/6.0, -3/6.0, 6/6.0, -1/6.0, 0},                   // velocity
    {0, -1/12.0, 16/12.0, -30/12.0, 16/12.0, -1/12.0, 0},       // acceleration
    {0, 1/12.0, -17/12.0, 46/12.0, -46/12.0, 17/12.0, -1/12.0}  // jerk
};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

Predictability::Predictability()
{

}

double Predictability::costPredict( const Eigen::MatrixXd& traj )
{
    // num of collums is the dimension of state space
    // num of rows is the length of the trajectory
    std::vector<Eigen::VectorXd> control_costs( traj.rows() );
    const double weight = 1.0;

    // this measures the velocity and squares them
    for ( int d=0; d<traj.rows(); ++d )
    {
        Eigen::VectorXd params_all = traj.row(d);
        Eigen::VectorXd acc_all    = Eigen::VectorXd::Zero( traj.cols() );

        for (int i=0; i<params_all.size(); i++)
        {
            // TODO change to velocity
            for (int j=-DIFF_RULE_LENGTH/2; j<=DIFF_RULE_LENGTH/2; j++)
            {
                int index = i+j;

                if (index < 0)
                    continue;
                if (index >= traj.cols())
                    continue;

                acc_all[i] += (params_all[index]*DIFF_RULES[1][j+DIFF_RULE_LENGTH/2]);
            }
        }

        control_costs[d] = weight * ( acc_all.cwise()*acc_all );
    }

    double cost=0.0;
    for ( int d=0; d<int(control_costs.size()); ++d )
        cost += control_costs[d].sum();

    return cost;
}

Eigen::MatrixXd Predictability::getInterpolatedTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, int nb_points )
{
    if( a.size() != b.size() )
        cout << "Error in getInterpolatedTrajectory" << endl;

    Eigen::MatrixXd traj( nb_points, a.size() );

    double delta = 1 / double(nb_points-1);
    double s = 0.0;

    for( int i=0;i<traj.cols();i++)
    {
        traj.col(i) = interpolate( a, b, s );
        s += delta;
    }

    return traj;
}

//! Interpolates linearly two configurations
//! u = 0 -> a
//! u = 1 -> b
Eigen::VectorXd Predictability::interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const
{
    Eigen::VectorXd out;
    if( a.size() != b.size() )
    {
        cout << "Error in interpolate" << endl;
        return out;
    }

    out = a;
    for( int i=0;i<int(out.size());i++)
    {
        out[i] += u*(b[i]-a[i]);
    }
    return out;
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

}

void setTrajectory( const Eigen::MatrixXd& t )
{

}

double Legibility::cost( int i )
{
    Eigen::VectorXd q_cur = traj_.col(i);
    Eigen::VectorXd q_source = traj_.col(0);
    Eigen::VectorXd q_target = traj_.col(traj_.cols()-1);

    int rows = traj_.rows();
    int cols = traj_.cols() - i;

    Eigen::MatrixXd sub_traj_0 = getInterpolatedTrajectory( q_source, q_target, traj_.cols() );
    Eigen::MatrixXd sub_traj_1 = traj_.block(rows,cols,0,i);
    Eigen::MatrixXd sub_traj_2 = getInterpolatedTrajectory( q_source, q_cur, i );

    double c_0 = costPredict( sub_traj_0 );
    double c_1 = costPredict( sub_traj_1 );
    double c_2 = costPredict( sub_traj_2 );

    double t = double(i) / double(traj_.cols());

    // Main cost function
    double prob =  exp( -c_1 - c_2 ) / exp( -c_0 );
    double cost = prob * ( length_ - t );
    return cost;
}

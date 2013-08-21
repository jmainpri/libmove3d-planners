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
    if( traj.cols() == 0 || traj.rows() == 0 )
    {
        return 0.0;
    }
//    cout << "traj.cols() : " << traj.cols() << endl;
//    cout << "traj.rows() : " << traj.rows() << endl;
//    cout << "traj : " << endl << traj << endl;

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

                // 0 should be velocity
                acc_all[i] += (params_all[index]*DIFF_RULES[0][j+DIFF_RULE_LENGTH/2]);
            }
        }

//        cout << "params_all.size() : " << params_all.size() << endl;
//        cout << "params_all : " << params_all.transpose() << endl;
//        cout << "acc_all : " << acc_all.transpose() << endl;

        control_costs[d] = weight * ( acc_all.cwise()*acc_all );
    }

    double cost=0.0;
    for ( int d=0; d<int(control_costs.size()); ++d )
    {
        Eigen::VectorXd cost_vect =  control_costs[d].segment( DIFF_RULE_LENGTH, control_costs[d].size() - 2*(DIFF_RULE_LENGTH-1));
//        cout << "cost_vect : " << cost_vect.transpose() << endl;
//        cout << "cost_vect.size() : " << cost_vect.size() << endl;
        cost += cost_vect.sum();
    }

    return cost / 100; // scaling
}

void Predictability::fillTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& traj )
{
    // figure out the num_points_:
    // we need diff_rule_length-1 extra points on either side:

    // now copy the trajectories over:
    for (int i=0; i<DIFF_RULE_LENGTH-1; i++)
    {
        traj.col(i) = a;
    }
    for (int i=(traj.cols()-1-DIFF_RULE_LENGTH-1); i<traj.cols(); i++)
    {
        traj.col(i) = b;
    }
}

Eigen::MatrixXd Predictability::getInterpolatedTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, int nb_points )
{
    if( nb_points == 0)
        return Eigen::MatrixXd();

    if( a.size() != b.size() )
        cout << "Error in getInterpolatedTrajectory" << endl;

    Eigen::MatrixXd traj( a.size(), nb_points + 2*(DIFF_RULE_LENGTH-1) );

    fillTrajectory( a, b, traj );

    double delta = 1 / double(nb_points-1);
    double s = 0.0;

    for( int i=(DIFF_RULE_LENGTH-1);i<traj.cols()-(DIFF_RULE_LENGTH-1);i++)
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
    length_ = 1.0;
}

void Legibility::setTrajectory( const Eigen::MatrixXd& t )
{
    traj_ = t.transpose();
    q_source_ = traj_.col(0);
    q_target_ = traj_.col(traj_.cols()-1);
    straight_line_ = getInterpolatedTrajectory( q_source_, q_target_, traj_.cols() - 2*(DIFF_RULE_LENGTH-1) );
    c_0_ = costPredict( straight_line_ );
//    cout << "traj : " << endl << traj_ << endl;
//    cout << "q_source : " << q_source_ << endl;
//    cout << "q_target : " << q_target_ << endl;
//    cout << "straight_line_ : " << endl << straight_line_ << endl;
}

double Legibility::cost( int i )
{
//    cout << "i : " << i << endl;

    Eigen::VectorXd q_cur = traj_.col(i);

    int rows = traj_.rows();
    int cols = traj_.cols() - i;

    Eigen::MatrixXd sub_traj_1 = traj_.block( rows, cols, 0 , i );
    Eigen::MatrixXd sub_traj_2 = getInterpolatedTrajectory( q_source_, q_cur, i );

//    cout << "straight_line_ : " << straight_line_ << endl;
//    cout << "sub_traj_1 : " << endl << sub_traj_1 << endl;
//    cout << "sub_traj_2 : " << endl << sub_traj_2 << endl;

    double c_1 = costPredict( sub_traj_1 );
    double c_2 = costPredict( sub_traj_2 );

//    cout << "c_0 : " << c_0_ << endl;
//    cout << "c_1 : " << c_1 << endl;
//    cout << "c_2 : " << c_2 << endl;

    double t = double(i) / double(traj_.cols());

    // Main cost function
    double prob = exp( -c_1 - c_2 ) / exp( -c_0_ );
    double cost = prob * ( length_ - t );

//    cout << "prob : " << prob << endl;
//    cout << "cost : " << cost << endl;

    return cost;
}

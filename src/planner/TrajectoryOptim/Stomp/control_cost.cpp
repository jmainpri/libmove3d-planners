#include "control_cost.hpp"

#include <iostream>

using std::cout;
using std::endl;

static const int DIFF_RULE_LENGTH = 7;
static const int NUM_DIFF_RULES = 3;

// the differentiation rules (centered at the center)
const double DIFF_RULES[NUM_DIFF_RULES][DIFF_RULE_LENGTH] = {
    {0, 0, -2/6.0, -3/6.0, 6/6.0, -1/6.0, 0},                   // velocity
    {0, -1/12.0, 16/12.0, -30/12.0, 16/12.0, -1/12.0, 0},       // acceleration
    {0, 1/12.0, -17/12.0, 46/12.0, -46/12.0, 17/12.0, -1/12.0}  // jerk
};

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

ControlCost::ControlCost()
{
    diff_rule_length_ = DIFF_RULE_LENGTH;
    type_ = vel;
    scaling_ = 100;
}

int ControlCost::getDiffRuleLength()
{
    return diff_rule_length_;
}

double ControlCost::cost( const Eigen::MatrixXd& traj )
{
    std::vector<Eigen::VectorXd> control_costs = getSquaredQuantities( traj );

    double cost=0.0;
    for ( int d=0; d<int(control_costs.size()); ++d )
    {
        Eigen::VectorXd cost_vect =  control_costs[d].segment( diff_rule_length_, control_costs[d].size() - 2*(diff_rule_length_-1));
        //        cout << "cost_vect : " << cost_vect.transpose() << endl;
        //        cout << "cost_vect.size() : " << cost_vect.size() << endl;
        cost += cost_vect.sum();
    }

    return cost / scaling_; // scaling
}


std::vector<Eigen::VectorXd> ControlCost::getSquaredQuantities( const Eigen::MatrixXd& traj )
{
    // num of collums is the dimension of state space
    // num of rows is the length of the trajectory
    std::vector<Eigen::VectorXd> control_costs( traj.rows() );

    if( traj.cols() == 0 || traj.rows() == 0 || traj.cols() == 1 )
    {
        return control_costs;
    }

    // cout << "traj.cols() : " << traj.cols() << endl;
    // cout << "traj.rows() : " << traj.rows() << endl;
    // cout << "traj : " << endl << traj << endl;

    const double weight = 1.0;

    int type = type_;

    // this measures the velocity and squares them
    for ( int d=0; d<traj.rows(); ++d )
    {
        Eigen::VectorXd params_all = traj.row(d);
        Eigen::VectorXd acc_all    = Eigen::VectorXd::Zero( traj.cols() );

        for (int i=0; i<params_all.size(); i++)
        {
            // TODO change to velocity
            for (int j=-diff_rule_length_/2; j<=diff_rule_length_/2; j++)
            {
                int index = i+j;

                if ( index < 0 )
                    continue;
                if ( index >= traj.cols() )
                    continue;

                // 0 should be velocity
                acc_all[i] += (params_all[index]*DIFF_RULES[type][j+diff_rule_length_/2]);
            }
        }

        // cout << "params_all.size() : " << params_all.size() << endl;
        // cout << "params_all : " << params_all.transpose() << endl;
        // cout << "acc_all : " << acc_all.transpose() << endl;

        control_costs[d] = weight * ( acc_all.cwise()*acc_all );

        cout << "control_costs[" << d << "] : " << control_costs[d].transpose() << endl;
    }

    return control_costs; // scaling
}
void ControlCost::fillTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& traj )
{
    // we need diff_rule_length-1 extra points on either side
    // Copy on the left side
    for (int i=0; i<diff_rule_length_-1; i++)
    {
        traj.col(i) = a;
    }
    // Copy on the right side
    for (int i=(traj.cols()-1-diff_rule_length_-1); i<traj.cols(); i++)
    {
        traj.col(i) = b;
    }
}

Eigen::MatrixXd ControlCost::resample( const Eigen::MatrixXd& m, int nb_points ) const
{
    if( m.cols() == 0 || m.cols() == 1 )
        return m;

    double d1 = 1/double(m.cols());
    double d2 = 1/double(nb_points);

    Eigen::MatrixXd traj( m.rows(), nb_points );

    double s=0.0;

    traj.col(0) = m.col(0);
    int q_id_prev=0;

    for( int i=1; i<nb_points; i++ )
    {
        s += d2;
        int q_id = s / d1;
        traj.col(i) = interpolate( m.col(q_id_prev), m.col(q_id), s - (q_id*d1) );
        q_id_prev = q_id;
    }

    return traj;
}

Eigen::MatrixXd ControlCost::getInterpolatedTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, int nb_points )
{
    if( nb_points == 0)
        return Eigen::MatrixXd();

    if( a.size() != b.size() )
        cout << "Error in getInterpolatedTrajectory" << endl;

    Eigen::MatrixXd traj( a.size(), nb_points + 2*(diff_rule_length_-1) );

    fillTrajectory( a, b, traj );

    double delta = 1 / double(nb_points-1);
    double s = 0.0;

    for( int i=(diff_rule_length_-1);i<traj.cols()-(diff_rule_length_-1);i++)
    {
        traj.col(i) = interpolate( a, b, s );
        s += delta;
    }

    return traj;
}

//! Interpolates linearly two configurations
//! u = 0 -> a
//! u = 1 -> b
Eigen::VectorXd ControlCost::interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const
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

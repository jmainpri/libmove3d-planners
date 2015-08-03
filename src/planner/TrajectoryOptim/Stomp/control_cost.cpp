/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "control_cost.hpp"

#include "utils/misc_functions.hpp"
#include "chompUtils.hpp"

#include <iostream>
#include <iomanip>

#include <libmove3d/include/Util-pkg.h>

using std::cout;
using std::endl;

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

ControlCost::ControlCost()
{
    diff_rule_length_ = DIFF_RULE_LENGTH;
    type_ = vel; // Becareful to match covariant_trajectory_policy
    scaling_ = 100;
    check_trajectory_consistency_ = true;
    resetInnerSegmentPadding();
}

void ControlCost::setInnerSegmentPadding( int padding )
{
    inner_segment_padding_left_ = padding;
}

void ControlCost::resetInnerSegmentPadding()
{
    inner_segment_padding_left_ = std::floor(double(diff_rule_length_)/double(2));
    inner_segment_padding_right_ = inner_segment_padding_left_ ;

//    inner_segment_padding_left_+= 1;
//    inner_segment_padding_right_-= 1;
//    cout << "inner_segment_padding_left_ : " << inner_segment_padding_left_ << endl;
}

void ControlCost::setType(int type)
{
    type_ = cost_type(type);
}

int ControlCost::getDiffRuleLength() const
{
    return diff_rule_length_;
}

int ControlCost::getInnerSegmentSize( const Eigen::VectorXd& control ) const
{
    // return control.size() - 2*(diff_rule_length_-1);
    return control.size() - ( 2*diff_rule_length_ +
                              inner_segment_padding_left_ +
                              inner_segment_padding_right_);
}

Eigen::VectorXd ControlCost::getInnerSegment( const Eigen::VectorXd& control ) const
{
    // return control.segment( diff_rule_length_-1, control.size() - 2*(diff_rule_length_-1));;
    int size = getInnerSegmentSize(control);
    return control.segment( diff_rule_length_ + inner_segment_padding_left_, size );
}

double ControlCost::cost( const std::vector<Eigen::VectorXd>& control_costs, double dt )
{
    double cost=0.0;
    double time_step = dt == 0.0 ? 1.0 : dt;

    for ( int d=0; d<int(control_costs.size()); ++d )
    {
        Eigen::VectorXd cost_vect = time_step * getInnerSegment( control_costs[d] );
//        cout.precision(2);
//        cout << "cost_vect : " << cost_vect.transpose() << endl;
//        cout << "cost_vect.size() : " << cost_vect.size() << endl;
        cost += cost_vect.sum();
    }

    return cost; // scaling
}

double ControlCost::cost( const Eigen::MatrixXd& traj )
{
    std::vector<Eigen::VectorXd> control_costs = getSquaredQuantities( traj );
    return cost( control_costs ) / scaling_; // scaling
}


std::vector<Eigen::VectorXd> ControlCost::getSquaredQuantities( const Eigen::MatrixXd& traj, double dt ) const
{
    // num of collums is the dimension of state space
    // num of rows is the length of the trajectory
    std::vector<Eigen::VectorXd> control_costs( traj.rows() );

    if( traj.cols() == 0 || traj.rows() == 0 || traj.cols() == 1 )
    {
        return control_costs;
    }

    double factor = 1.;

    switch( type_ )
    {
    case vel:
        factor = 2.;
        break;
    case acc:
        factor = 3.;
        break;
    case jerk:
        factor = 6.;
        break;
    }

    // cout << "traj.cols() : " << traj.cols() << endl;
    // cout << "traj.rows() : " << traj.rows() << endl;
    // cout << "traj : " << endl << traj << endl;

    const double weight = 1.0;

    // this measures the velocity and squares them (for each DoF)
    for ( int d=0; d<traj.rows(); ++d )
    {
        Eigen::VectorXd params_all = traj.row(d);
        Eigen::VectorXd acc_all    = Eigen::VectorXd::Zero( traj.cols() );

        for (int i=0; i<params_all.size(); i++)
        {
             // Check for jumps on circular joints (MAY ACTIVATE FOR TRANSLATION JOINTS (diff > 3.14)
            if( check_trajectory_consistency_ && ( i < params_all.size()-1 ) )
            {
                double diff = params_all[i] - params_all[i+1];
                double error = std::fabs( diff_angle( params_all[i+1] , params_all[i] ) - diff );
                if( error > 1e-6 ){
                    cout << "control cost breaks at row "  << d << " and col  " << i << " by " << error << " in " << __PRETTY_FUNCTION__ << endl;
                }
            }

            // TODO change to velocity
            for (int j=-diff_rule_length_/2; j<=diff_rule_length_/2; j++)
            {
                int index = i+j;

                if ( index < 0 )
                    continue;
                if ( index >= traj.cols() )
                    continue;

                // 0 should be velocity
                acc_all[i] +=
                        (params_all[index]*DIFF_RULES2[type_][j+diff_rule_length_/2]);
            }

            if( dt != 0.0 )
            {
                acc_all[i] /= ( factor * std::pow( dt, double(int(type_)+1) ) );
            }
        }

        // cout << "params_all.size() : " << params_all.size() << endl;
        // cout << "params_all : " << params_all.transpose() << endl;
        // cout << "acc_all : " << acc_all.transpose() << endl;

        control_costs[d] = weight * ( acc_all.cwise()*acc_all );

//        cout << "control_costs[" << d << "] : " << endl << control_costs[d].transpose() << endl;
    }

    return control_costs; // scaling
}

void ControlCost::fillTrajectoryWithBuffer( const Eigen::VectorXd& b, Eigen::MatrixXd& traj ) const
{
    // set the start and end of the trajectory
    for (int i=0; i<diff_rule_length_-1; ++i)
    {
        traj.col(i) = buffer_[i];
        traj.col(traj.cols()-1-i) = b;
    }
}

void ControlCost::fillTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& traj) const
{
    // set the start and end of the trajectory
    // cout  << "diff_rule_length_ : " << diff_rule_length_ << endl;
    for (int i=0; i<diff_rule_length_-1; ++i)
    {
        traj.col(i) = a;
        traj.col(traj.cols()-1-i) = b;
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

void ControlCost::saveProfiles( const Eigen::MatrixXd& traj, std::string foldername, double dt ) const
{
//    cost_type tmp = type_;
//    type_ = vel;

//    cout << __PRETTY_FUNCTION__ << endl;

    std::vector<Eigen::VectorXd> control_costs = getSquaredQuantities( traj, dt );

    for (int d=0; d<int(control_costs.size()); ++d)
    {
//        Eigen::VectorXd cost_vect = control_costs[d].segment( diff_rule_length_-1, control_costs[d].size() - 2*(diff_rule_length_-1));

        double time_step = dt == 0.0 ? 1.0 : dt;
        Eigen::VectorXd cost_vect = time_step * getInnerSegment( control_costs[d] );


        // Get abs values
//        cost_vect = time_step * Eigen::ArrayXd( cost_vect ).sqrt();

//        cout << "cost_vect.size() : " << cost_vect.size() << endl;

        std::stringstream ss;
        ss << "stomp_vel_"  << std::setw(3) << std::setfill( '0' ) << d << ".txt";
        move3d_save_matrix_to_csv_file( cost_vect.transpose(), foldername + ss.str() );
    }

//    type_ = tmp;
}

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

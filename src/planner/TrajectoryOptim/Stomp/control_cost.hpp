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
#ifndef CONTROL_COST_HPP
#define CONTROL_COST_HPP


#define EIGEN2_SUPPORT_STAGE10_FULL_EIGEN2_API

#include <vector>

#include <Eigen/Core>

class ControlCost
{
public:
    ControlCost();

    //! Set type of quantity
    void setType(int type);

    //! Returns the diff rule between
    int getDiffRuleLength();

    //! Returns segment of active controls
    Eigen::VectorXd getInnerSegment( const Eigen::VectorXd& control ) const;

    //! Returns the cost of a given trajectory
    double cost( const Eigen::MatrixXd& t );

    //! Returns the cost of vel, acc or jerk profile
    double cost( const std::vector<Eigen::VectorXd>& control_costs );

    //! Returns the squared vel, acc or jerk
    std::vector<Eigen::VectorXd> getSquaredQuantities( const Eigen::MatrixXd& traj, double dt=0.0 );

    //! sets the start and end points in the trajectory
    void fillTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, Eigen::MatrixXd& traj );

    //! set buffer with previous trajectory
    void fillTrajectoryWithBuffer( const Eigen::VectorXd& b, Eigen::MatrixXd& traj );

    //! resample the matrix rows
    Eigen::MatrixXd resample( const Eigen::MatrixXd& m, int nb_points ) const;

    // Get a discretized interpolated trajectory
    Eigen::MatrixXd getInterpolatedTrajectory( const Eigen::VectorXd& a, const Eigen::VectorXd& b, int nb_points );

    //! interpolate between configurations
    Eigen::VectorXd interpolate( const Eigen::VectorXd& a, const Eigen::VectorXd& b, double u ) const;

    //! Set buffer with initial trajectory
    void setBuffer( const std::vector<Eigen::VectorXd>& buffer ) { buffer_ = buffer; }

    //! Save velocity cost to file
    void saveProfiles( const Eigen::MatrixXd& traj, std::string foldername, double dt=0.0 );

protected:
    enum cost_type { vel=0, acc=1, jerk=2 } type_;
    int diff_rule_length_;
    double scaling_;
    std::vector<Eigen::VectorXd> buffer_;

};

#endif // CONTROL_COST_HPP

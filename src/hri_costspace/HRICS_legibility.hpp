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
#ifndef HRICS_LEGIBILITY_HPP
#define HRICS_LEGIBILITY_HPP

#include "API/ConfigSpace/configuration.hpp"
#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"
#include <Eigen/Core>
#include <vector>

/**
 @ingroup HRICS

 These classes implement the legibility criterion of
 Anca Dragan and Siddartha Srinivasa

 */
namespace HRICS
{

class Predictability : public ControlCost
{
public:
    Predictability();

    //! Add a goal to the predictability cost function
    void addGoal( const Eigen::VectorXd& g );

    //! Clear the goal set
    void clearGoals();

protected:
    std::vector<Eigen::VectorXd> goals_;
};

class Legibility : public Predictability
{
public:
    Legibility();

    //! Set the current trajectory
    void setTrajectory( const Eigen::MatrixXd& t );

    //! Set the current trajectory length
    void setLength(double l);

    //! Get the cost of the ith configuration
    double legibilityCost( int i );

private:
    double length_;
    Eigen::MatrixXd traj_;
    Eigen::MatrixXd straight_line_;
    Eigen::VectorXd q_source_;
    std::vector<double> c_g_;
};

}

#endif // HRICS_LEGIBILITY_HPP

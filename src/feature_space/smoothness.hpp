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

#ifndef SMOOTHNESS_HPP
#define SMOOTHNESS_HPP

#include "features.hpp"
#include "planner/TrajectoryOptim/Stomp/control_cost.hpp"

namespace Move3D
{

////////////////////////////////////////
class TrajectorySmoothness : public Feature
{
public:
    TrajectorySmoothness();

    //! Returns a smoothness cost for the trajectory
    FeatureVect getFeatures(const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0));
    FeatureVect getFeatureCount(const Move3D::Trajectory& t);

    void setWeights(const WeightVect &w);

    //! Prints the control cost along the trajectory
    void printControlCosts( const std::vector<Eigen::VectorXd>& control_cost  );

    //! get smoothed trajectory as matrix
    Eigen::MatrixXd getSmoothedTrajectory( const Move3D::Trajectory& t );

    //! Set Buffer
    void setBuffer(const Eigen::MatrixXd& buffer) { control_cost_.setBuffer(buffer); buffer_is_filled_=true; }
    void clearBuffer() { buffer_is_filled_=false; }

protected:
    ControlCost control_cost_;

private:
    double computeControlCost( const Eigen::MatrixXd& traj );
    bool buffer_is_filled_;

};

////////////////////////////////////////
class LengthFeature : public TrajectorySmoothness
{
public:
    LengthFeature();
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    double scaling_;
};

////////////////////////////////////////
class VelocitySmoothness : public TrajectorySmoothness
{
public:
    VelocitySmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class AccelerationSmoothness : public TrajectorySmoothness
{
public:
    AccelerationSmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class JerkSmoothness : public TrajectorySmoothness
{
public:
    JerkSmoothness();
    void setWeights(const WeightVect &w);
};

////////////////////////////////////////
class SmoothnessFeature : public Move3D::Feature
{
public:
    SmoothnessFeature();
    Move3D::FeatureVect getFeatureCount( const Move3D::Trajectory& t );
    Move3D::FeatureVect getFeatures( const Move3D::Configuration& q, std::vector<int> active_dofs = std::vector<int>(0) );
    void setActiveDoFs( const std::vector<int>& active_dofs );
    void setWeights(const WeightVect &w);

    //! Set Buffer
    void setBuffer(const Eigen::MatrixXd& buffer);
    void clearBuffer();

private:
    LengthFeature length_;
    VelocitySmoothness velocity_;
    AccelerationSmoothness acceleration_;
    JerkSmoothness jerk_;
};

}

#endif // SMOOTHNESS_HPP
